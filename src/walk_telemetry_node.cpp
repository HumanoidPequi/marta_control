/*
 * walk_telemetry_node.cpp
 *
 * Funde o PLANO do gerador de marcha com a VERDADE DE SOLO do Gazebo e publica
 * as grandezas do 3D-LIPM de Kajita 2001 em /marta/walk/telemetry.
 *
 * Por que um no separado, e em C++
 * --------------------------------
 * Separado porque quem gera nao deve ser quem mede: o controller_node continua
 * com um unico trabalho, e esta instrumentacao pode ser iterada sem tocar no
 * laco de controle.
 *
 * Em C++ porque /gazebo/link_states sai na taxa da fisica (1 kHz) com dezenas
 * de links por mensagem - sao ~700 KB/s so desse topico. Ja medimos nesta mesma simulacao que um no Python
 * martelando topicos nessa taxa derruba o fator de tempo real do Gazebo pela
 * metade - foi a causa raiz do problema do real_to_sim_topics.py. Aqui a
 * assinatura usa queue_size 1 e o trabalho acontece num timer de 100 Hz, sempre
 * sobre a ultima mensagem recebida.
 *
 * E por AsyncSpinner com varias threads, nao ros::spin(): com spinner de thread
 * unica, a enxurrada de link_states satura a fila de callbacks e OS OUTROS
 * CALLBACKS NUNCA RODAM. Foi exatamente o que aconteceu na primeira versao -
 * /marta/walk/reference aparecia corretamente assinado no rosnode info e mesmo
 * assim o callback jamais era chamado, deixando as grandezas do LIPM zeradas.
 *
 * O CoM e a soma massa x posicao sobre os links do robo, com as massas lidas do
 * URDF em /robot_description - mesmo padrao que marta_bringup/tf_to_cm.cpp ja
 * usa, mas aqui no frame do MUNDO e com a velocidade vindo do twist de cada
 * link, sem derivada numerica.
 *
 * Parametros (~):
 *   rate_hz        (double, 100.0)  taxa de publicacao
 *   model_name     (string, "marta") prefixo dos links em /gazebo/link_states
 *   world_frame    (string, "odom")
 *   publish_world_tf (bool, true) TF world_frame -> base_link pela verdade de
 *                  solo, para visualizacao (Foxglove/RViz): sem ela o
 *                  robot_state_publisher so da as juntas relativas ao tronco e
 *                  o robo "anda" parado no lugar.
 *
 * Visualizacao (alem da telemetria):
 *   /robot_description  std_msgs/String latched, copia do parametro, para o
 *                       URDF entrar nos bags.
 *   /marta/viz/rumo     visualization_msgs/MarkerArray: seta do rumo atual
 *                       (yaw verdadeiro) e do rumo desejado da malha de rumo.
 *   /marta/viz/rumo_graus  Float64MultiArray [yaw_verdadeiro, rumo_desejado,
 *                       erro] em graus, para plotar direto (o Plot do Foxglove
 *                       nao subtrai series).
 */

#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <urdf/model.h>
#include <visualization_msgs/MarkerArray.h>

#include <marta_msgs/WalkReference.h>
#include <marta_msgs/WalkTelemetry.h>

#include <cmath>
#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace {

const double GRAVITY = 9.80665;

/** Converte um quaternio para roll e pitch, na convencao do projeto. */
void quatToRollPitch(const geometry_msgs::Quaternion &q, double &roll, double &pitch) {
    const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    roll = std::atan2(sinr, cosr);
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    sinp = std::max(-1.0, std::min(1.0, sinp));
    pitch = std::asin(sinp);
}

/** Yaw (Z da sequencia ZYX) de um quaternio, em [-pi, pi]. */
double quatToYaw(const geometry_msgs::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/** Aplica um deslocamento expresso no frame do pe a uma pose de pe. */
geometry_msgs::Point applyFootOffset(const geometry_msgs::Pose &pose,
                                     double ox, double oy, double oz) {
    const geometry_msgs::Quaternion &q = pose.orientation;
    // rotacao do vetor pelo quaternio: v' = q * v * q^-1
    const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    geometry_msgs::Point p;
    p.x = pose.position.x + (1 - 2 * (yy + zz)) * ox + 2 * (xy - wz) * oy + 2 * (xz + wy) * oz;
    p.y = pose.position.y + 2 * (xy + wz) * ox + (1 - 2 * (xx + zz)) * oy + 2 * (yz - wx) * oz;
    p.z = pose.position.z + 2 * (xz - wy) * ox + 2 * (yz + wx) * oy + (1 - 2 * (xx + yy)) * oz;
    return p;
}

}  // namespace

class WalkTelemetryNode {
public:
    explicit WalkTelemetryNode(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;
    ros::Subscriber link_sub_, ref_sub_, attitude_sub_, debug_sub_;
    ros::Publisher pub_, urdf_pub_, rumo_pub_, rumo_graus_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    bool publish_world_tf_;
    double last_tf_stamp_;
    ros::Timer timer_;

    std::string model_name_, world_frame_;
    double foot_off_x_, foot_off_y_, foot_off_z_;
    bool use_link_states_;
    std::map<std::string, double> link_mass_;   ///< nome curto do link -> massa
    double total_mass_;

    std::mutex mutex_;   ///< protege o estado abaixo: varias threads de spinner
    gazebo_msgs::LinkStates::ConstPtr links_;
    marta_msgs::WalkReference::ConstPtr ref_;
    double pitch_, pitch_rate_, roll_, roll_rate_;
    bool has_attitude_;
    double heading_error_;   ///< erro da malha de rumo (yaw estimado - referencia)
    bool has_heading_;

    // Estado anterior, para a aceleracao do CoM usada no ZMP estimado.
    bool has_prev_;
    double prev_vx_, prev_vy_, prev_stamp_;

    void loadMasses();
    void linkCb(const gazebo_msgs::LinkStates::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        links_ = msg;
    }
    void refCb(const marta_msgs::WalkReference::ConstPtr &msg) {
        ROS_WARN_ONCE("[WalkTelemetry] referencia de marcha conectada "
                      "(z_com=%.4f, t_c=%.4f).", msg->z_com, msg->t_c);
        std::lock_guard<std::mutex> lock(mutex_);
        ref_ = msg;
    }
    void attitudeCb(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void debugCb(const std_msgs::Float64MultiArray::ConstPtr &msg);
    void publishViz(const geometry_msgs::Pose &base_pose, double yaw_true,
                    const ros::Time &stamp);
    void update(const ros::TimerEvent &);
};

WalkTelemetryNode::WalkTelemetryNode(ros::NodeHandle &nh)
    : nh_(nh), publish_world_tf_(true), last_tf_stamp_(-1.0), total_mass_(0.0), pitch_(0.0), pitch_rate_(0.0), roll_(0.0),
      roll_rate_(0.0), has_attitude_(false), heading_error_(0.0),
      has_heading_(false), has_prev_(false), prev_vx_(0.0),
      prev_vy_(0.0), prev_stamp_(0.0) {
    ros::NodeHandle pnh("~");
    double rate_hz;
    pnh.param<double>("rate_hz", rate_hz, 100.0);
    pnh.param<std::string>("model_name", model_name_, "marta");
    pnh.param<std::string>("world_frame", world_frame_, "odom");

    // A origem do link *_feet_1 nao e o ponto de contato da sola. O
    // marta_bringup/tf_to_cm.cpp ja usa este mesmo deslocamento para publicar
    // os frames l_feet_point / r_feet_point; repetimos o valor aqui para que o
    // frame do LIPM tenha origem no contato, que e onde o ZMP mora.
    pnh.param<double>("foot_contact_offset_x", foot_off_x_, 0.02445);
    pnh.param<double>("foot_contact_offset_y", foot_off_y_, 0.0);
    pnh.param<double>("foot_contact_offset_z", foot_off_z_, -0.037);

    // Assinar /gazebo/link_states obriga o gzserver a montar e serializar 37
    // link states na taxa da fisica. Este parametro existe para medir esse
    // custo: com false o no sobe e nao assina, isolando o efeito.
    pnh.param<bool>("use_link_states", use_link_states_, true);
    pnh.param<bool>("publish_world_tf", publish_world_tf_, true);

    pub_ = nh_.advertise<marta_msgs::WalkTelemetry>("/marta/walk/telemetry", 10);
    urdf_pub_ = nh_.advertise<std_msgs::String>("/robot_description", 1, true);
    rumo_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/marta/viz/rumo", 1);
    rumo_graus_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/marta/viz/rumo_graus", 10);

    loadMasses();

    // queue_size 1 em tudo: sempre a amostra mais recente, nunca uma fila
    // acumulada que atrasaria a telemetria e consumiria CPU a toa.
    if (use_link_states_) {
        link_sub_ = nh_.subscribe("/gazebo/link_states", 1,
                                  &WalkTelemetryNode::linkCb, this);
    } else {
        ROS_WARN("[WalkTelemetry] use_link_states=false: sem CoM, so medindo custo.");
    }
    ref_sub_ = nh_.subscribe("/marta/walk/reference", 1,
                             &WalkTelemetryNode::refCb, this);
    attitude_sub_ = nh_.subscribe("/marta/torso_attitude", 1,
                                  &WalkTelemetryNode::attitudeCb, this);
    debug_sub_ = nh_.subscribe("/marta/stabilizer_debug", 1,
                               &WalkTelemetryNode::debugCb, this);

    timer_ = nh_.createTimer(ros::Duration(1.0 / rate_hz),
                             &WalkTelemetryNode::update, this);

    ROS_INFO("[WalkTelemetry] %zu links com massa, total %.4f kg, publicando a %.0f Hz",
             link_mass_.size(), total_mass_, rate_hz);
}

/**
 * Le as massas do URDF em /robot_description. Links sem inercia ou com massa
 * zero ficam de fora: nao contribuem para o CoM e o Gazebo costuma nem os
 * expor em /gazebo/link_states (juntas fixas sao absorvidas no pai).
 */
void WalkTelemetryNode::loadMasses() {
    std::string xml;
    urdf::Model model;
    if (!nh_.getParam("robot_description", xml) || !model.initString(xml)) {
        ROS_ERROR("[WalkTelemetry] nao consegui ler /robot_description; "
                  "o CoM nao pode ser calculado.");
        return;
    }
    // Copia latched do URDF: o parametro nao vai para o bag, o topico vai.
    std_msgs::String urdf_msg;
    urdf_msg.data = xml;
    urdf_pub_.publish(urdf_msg);

    for (const auto &kv : model.links_) {
        const urdf::LinkConstSharedPtr &link = kv.second;
        if (!link->inertial || link->inertial->mass <= 0.0) {
            continue;
        }
        link_mass_[kv.first] = link->inertial->mass;
        total_mass_ += link->inertial->mass;
    }
    if (total_mass_ <= 0.0) {
        ROS_ERROR("[WalkTelemetry] massa total zero: URDF sem tags <inertial>?");
    }
}

void WalkTelemetryNode::attitudeCb(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    if (msg->data.size() < 4) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    pitch_ = msg->data[0];
    pitch_rate_ = msg->data[1];
    roll_ = msg->data[2];
    roll_rate_ = msg->data[3];
    has_attitude_ = true;
}

/** /marta/stabilizer_debug: [..., heading_error (7), heading_correction (8)]. */
void WalkTelemetryNode::debugCb(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    if (msg->data.size() < 9) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    heading_error_ = msg->data[7];
    has_heading_ = true;
}

/**
 * TF do mundo e setas de rumo. O rumo desejado no frame da verdade e
 * yaw_true - heading_error: a malha guarda a referencia no yaw ESTIMADO, e o
 * erro dela e o mesmo nos dois frames enquanto o estimador nao deriva (mesma
 * conta do experiments/plot_rumo.py). Parado, heading_error = 0 e as duas setas
 * coincidem.
 */
void WalkTelemetryNode::publishViz(const geometry_msgs::Pose &base_pose, double yaw_true,
                                   const ros::Time &stamp) {
    // Com use_sim_time o timer pode disparar duas vezes no mesmo instante
    // simulado; TF repetida gera aviso de TF_REPEATED_DATA no tf2.
    if (publish_world_tf_ && stamp.toSec() > last_tf_stamp_) {
        geometry_msgs::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = world_frame_;
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = base_pose.position.x;
        tf.transform.translation.y = base_pose.position.y;
        tf.transform.translation.z = base_pose.position.z;
        tf.transform.rotation = base_pose.orientation;
        tf_broadcaster_.sendTransform(tf);
        last_tf_stamp_ = stamp.toSec();
    }

    double heading_error;
    bool has_heading;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        heading_error = heading_error_;
        has_heading = has_heading_;
    }

    visualization_msgs::MarkerArray arr;
    const char *nomes[2] = {"rumo_atual", "rumo_desejado"};
    const double yaws[2] = {yaw_true, yaw_true - (has_heading ? heading_error : 0.0)};
    // Atual: azul; desejado: laranja (paleta das figuras).
    const double cores[2][3] = {{0.165, 0.471, 0.839}, {0.922, 0.408, 0.204}};
    for (int k = 0; k < 2; ++k) {
        visualization_msgs::Marker m;
        m.header.stamp = stamp;
        m.header.frame_id = world_frame_;
        m.ns = nomes[k];
        m.id = k;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point a, b;
        a.x = base_pose.position.x;
        a.y = base_pose.position.y;
        a.z = base_pose.position.z + 0.12 + 0.01 * k;   // acima da cabeca
        b = a;
        b.x += 0.3 * std::cos(yaws[k]);
        b.y += 0.3 * std::sin(yaws[k]);
        m.points.push_back(a);
        m.points.push_back(b);
        m.scale.x = 0.012;   // diametro da haste
        m.scale.y = 0.03;    // diametro da ponta
        m.scale.z = 0.05;    // comprimento da ponta
        m.color.r = cores[k][0];
        m.color.g = cores[k][1];
        m.color.b = cores[k][2];
        m.color.a = 1.0;
        m.pose.orientation.w = 1.0;
        arr.markers.push_back(m);
    }
    rumo_pub_.publish(arr);

    // Em graus e com o rumo desejado levado para perto do atual (sem salto de
    // 360 graus quando um dos dois cruza +-180).
    const double rad2deg = 180.0 / M_PI;
    const double err = std::atan2(std::sin(yaws[0] - yaws[1]), std::cos(yaws[0] - yaws[1]));
    std_msgs::Float64MultiArray g;
    g.layout.dim.resize(1);
    g.layout.dim[0].label = "yaw_verdadeiro,rumo_desejado,erro";
    g.layout.dim[0].size = 3;
    g.layout.dim[0].stride = 3;
    g.data = {yaws[0] * rad2deg, (yaws[0] - err) * rad2deg, err * rad2deg};
    rumo_graus_pub_.publish(g);
}

void WalkTelemetryNode::update(const ros::TimerEvent &) {
    // Copia o estado sob lock e solta o mutex antes de calcular: o somatorio do
    // CoM nao precisa bloquear os callbacks.
    gazebo_msgs::LinkStates::ConstPtr links;
    marta_msgs::WalkReference::ConstPtr ref;
    double pitch, pitch_rate, roll, roll_rate;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        links = links_;
        ref = ref_;
        pitch = pitch_; pitch_rate = pitch_rate_;
        roll = roll_; roll_rate = roll_rate_;
    }

    if (!links || total_mass_ <= 0.0) {
        ROS_WARN_THROTTLE(5.0, "[WalkTelemetry] aguardando /gazebo/link_states "
                          "e as massas do URDF.");
        return;
    }

    // ---- CoM por soma massa x posicao, no frame do mundo -------------------
    // /gazebo/link_states traz tambem os links do campo e da bola; filtra pelo
    // prefixo do modelo.
    const std::string prefix = model_name_ + "::";
    double mx = 0.0, my = 0.0, mz = 0.0, mvx = 0.0, mvy = 0.0, mvz = 0.0, m_seen = 0.0;
    geometry_msgs::Pose base_pose;
    geometry_msgs::Pose left_foot_pose, right_foot_pose;
    double base_wz = 0.0, left_wz = 0.0, right_wz = 0.0;
    bool has_base = false, has_left = false, has_right = false;

    for (size_t i = 0; i < links->name.size(); ++i) {
        const std::string &full = links->name[i];
        if (full.compare(0, prefix.size(), prefix) != 0) {
            continue;
        }
        const std::string shortname = full.substr(prefix.size());

        // O twist do link_states e no frame do mundo, entao angular.z ja e a
        // taxa de yaw.
        if (shortname == "base_link") {
            base_pose = links->pose[i]; base_wz = links->twist[i].angular.z; has_base = true;
        } else if (shortname == "l_feet_1") {
            left_foot_pose = links->pose[i]; left_wz = links->twist[i].angular.z; has_left = true;
        } else if (shortname == "r_feet_1") {
            right_foot_pose = links->pose[i]; right_wz = links->twist[i].angular.z; has_right = true;
        }

        auto it = link_mass_.find(shortname);
        if (it == link_mass_.end()) {
            continue;
        }
        const double m = it->second;
        m_seen += m;
        mx += m * links->pose[i].position.x;
        my += m * links->pose[i].position.y;
        mz += m * links->pose[i].position.z;
        mvx += m * links->twist[i].linear.x;
        mvy += m * links->twist[i].linear.y;
        mvz += m * links->twist[i].linear.z;
    }

    if (m_seen <= 0.0) {
        ROS_WARN_THROTTLE(5.0, "[WalkTelemetry] nenhum link de '%s' encontrado em "
                          "/gazebo/link_states.", model_name_.c_str());
        return;
    }

    marta_msgs::WalkTelemetry out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = world_frame_;

    out.com.x = mx / m_seen;
    out.com.y = my / m_seen;
    out.com.z = mz / m_seen;
    out.com_vel.x = mvx / m_seen;
    out.com_vel.y = mvy / m_seen;
    out.com_vel.z = mvz / m_seen;
    out.total_mass = m_seen;
    out.height = has_base ? base_pose.position.z : out.com.z;

    if (has_base) {
        quatToRollPitch(base_pose.orientation, out.roll_true, out.pitch_true);
        out.yaw_true = quatToYaw(base_pose.orientation);
        out.yaw_rate_true = base_wz;
        publishViz(base_pose, out.yaw_true, out.header.stamp);
    }
    if (has_left) {
        out.left_foot_yaw = quatToYaw(left_foot_pose.orientation);
        out.left_foot_yaw_rate = left_wz;
    }
    if (has_right) {
        out.right_foot_yaw = quatToYaw(right_foot_pose.orientation);
        out.right_foot_yaw_rate = right_wz;
    }
    out.pitch = pitch;
    out.pitch_rate = pitch_rate;
    out.roll = roll;
    out.roll_rate = roll_rate;

    // ---- estado de apoio, vindo do plano -----------------------------------
    uint8_t support = marta_msgs::WalkTelemetry::SUPPORT_DOUBLE;
    double z_com = 0.0, t_c = 0.0;
    if (!ref) {
        // Esperado nos ~10 s iniciais: o controller_node so entra no laco
        // principal (onde publica a referencia) depois da rampa do Agacha().
        ROS_WARN_THROTTLE(5.0, "[WalkTelemetry] ainda sem /marta/walk/reference - "
                          "normal durante a rampa inicial; grandezas do LIPM "
                          "zeradas ate ela chegar.");
    }
    if (ref) {
        support = ref->support_state;
        out.phase = ref->phase;
        z_com = ref->z_com;
        t_c = ref->t_c;
    }
    out.support_state = support;

    // Em apoio duplo o pe de apoio nao esta definido; usa o ponto medio, que e
    // a escolha natural para o frame de referencia nessa fase.
    if (has_left && has_right) {
        const geometry_msgs::Point lc =
                applyFootOffset(left_foot_pose, foot_off_x_, foot_off_y_, foot_off_z_);
        const geometry_msgs::Point rc =
                applyFootOffset(right_foot_pose, foot_off_x_, foot_off_y_, foot_off_z_);
        out.left_foot_contact = lc;
        out.right_foot_contact = rc;
        if (support == marta_msgs::WalkTelemetry::SUPPORT_LEFT) {
            out.support_foot = lc;
            out.swing_foot = rc;
        } else if (support == marta_msgs::WalkTelemetry::SUPPORT_RIGHT) {
            out.support_foot = rc;
            out.swing_foot = lc;
        } else {
            // Apoio duplo: o pe de apoio nao esta definido; o ponto medio e a
            // escolha natural de origem para o frame nessa fase.
            out.support_foot.x = 0.5 * (lc.x + rc.x);
            out.support_foot.y = 0.5 * (lc.y + rc.y);
            out.support_foot.z = 0.5 * (lc.z + rc.z);
            out.swing_foot = out.support_foot;
        }
    }

    // ---- CoM no frame do pe de apoio ---------------------------------------
    // E o frame do paper: a origem no pe que sustenta. E por isso que as curvas
    // da figura 5 saltam a cada troca de apoio.
    const double x = out.com.x - out.support_foot.x;
    const double y = out.com.y - out.support_foot.y;
    out.com_support.x = x;
    out.com_support.y = y;
    out.com_support.z = out.com.z - out.support_foot.z;
    out.z_com_measured = out.com_support.z;
    out.com_vel_support = out.com_vel;   // pe de apoio parado: velocidade igual

    if (ref) {
        // A referencia do gerador ja vem relativa ao torso; invertendo o sinal
        // do pe de apoio obtem-se o torso relativo ao pe, no mesmo frame.
        const geometry_msgs::Point &foot =
                (support == marta_msgs::WalkTelemetry::SUPPORT_RIGHT)
                        ? ref->right_foot : ref->left_foot;
        out.com_ref_support.x = -foot.x;
        out.com_ref_support.y = -foot.y;
        out.com_ref_support.z = (support == marta_msgs::WalkTelemetry::SUPPORT_RIGHT)
                                        ? ref->z_right : ref->z_left;
        out.com_error.x = out.com_ref_support.x - out.com_support.x;
        out.com_error.y = out.com_ref_support.y - out.com_support.y;
        out.com_error.z = out.com_ref_support.z - out.com_support.z;
    }

    // ---- grandezas do 3D-LIPM ----------------------------------------------
    if (z_com > 0.0) {
        const double w2 = GRAVITY / z_com;
        const double vx = out.com_vel.x, vy = out.com_vel.y;

        // eq. (29) e (28): invariantes sob o LIPM.
        out.orbital_energy_x = -0.5 * w2 * x * x + 0.5 * vx * vx;
        out.orbital_energy_y = -0.5 * w2 * y * y + 0.5 * vy * vy;

        // Ponto de captura, com o MESMO t_c que o gerador usou.
        out.dcm_x = x + t_c * vx;
        out.dcm_y = y + t_c * vy;

        // ZMP estimado do CoM. Aqui a derivada numerica e inevitavel: nao ha
        // sensor de forca nos pes. Ignora momento angular - ver o .msg.
        const double now = out.header.stamp.toSec();
        if (has_prev_) {
            const double dt = now - prev_stamp_;
            if (dt > 1e-4 && dt < 0.1) {
                out.zmp_est_x = x - (vx - prev_vx_) / dt / w2;
                out.zmp_est_y = y - (vy - prev_vy_) / dt / w2;
            }
        }
        prev_vx_ = vx;
        prev_vy_ = vy;
        prev_stamp_ = now;
        has_prev_ = true;
    }

    pub_.publish(out);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "walk_telemetry_node");
    ros::NodeHandle nh;
    WalkTelemetryNode node(nh);
    // 3 threads: uma absorve a enxurrada de link_states enquanto as outras
    // mantem referencia, atitude e o timer respondendo.
    ros::AsyncSpinner spinner(3);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
