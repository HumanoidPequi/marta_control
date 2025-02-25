#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point  # Usado para receber as velocidades (vx, vy, vψ)


def normalize_angle(angle):
    """Normaliza o ângulo para o intervalo [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def pose_relative(pose_global, pose_origin):
    """
    Converte uma pose global em uma pose relativa ao frame 'pose_origin'.
    Cada pose é um dicionário com 'x', 'y' (posição) e 'z' (rotação, em radianos).
    """
    theta = pose_origin['z']
    dx = pose_global['x'] - pose_origin['x']
    dy = pose_global['y'] - pose_origin['y']
    rel_x = math.cos(theta) * dx + math.sin(theta) * dy
    rel_y = -math.sin(theta) * dx + math.cos(theta) * dy
    rel_z = normalize_angle(pose_global['z'] - pose_origin['z'])
    return {'x': rel_x, 'y': rel_y, 'z': rel_z}

def pose_global(pose_relative, pose_origin):
    """
    Converte uma pose relativa (definida no frame do 'pose_origin') em pose global.
    """
    theta = pose_origin['z']
    x = pose_origin['x'] + math.cos(theta) * pose_relative['x'] - math.sin(theta) * pose_relative['y']
    y = pose_origin['y'] + math.sin(theta) * pose_relative['x'] + math.cos(theta) * pose_relative['y']
    z = normalize_angle(pose_origin['z'] + pose_relative['z'])
    return {'x': x, 'y': y, 'z': z}


class WalkingZMP:
    def __init__(self):
        rospy.init_node('walk_node')

        self.publisher = rospy.Publisher('/coordinates_input', JointState, queue_size=10)
        self.subscriber = rospy.Subscriber('/velocity_input', Point, self.xyz_callback)

        self.rate = rospy.Rate(100)  # 10 Hz
        self.start_time = rospy.Time.now()

        # Velocidades iniciais
        self.v_x = 0.0  # vx
        self.v_y = 0.0  # vy
        self.v_z = 0.0  # vψ

        # Flag para alternar o pé em balanço (para fins de simulação)
        self.left_is_swing_foot = True

        self.walking_params = self.load_walking_parameters()
        self.physical_parameters = self.load_physical_parameters()

        # Inicializa poses (representadas como dicionários com x, y, z)
        # Aqui, 'z' é o ângulo (yaw).
        self.torso_begin = {'x': 0.0, 'y': -self.walking_params['ySeparation'], 'z': 0.0}
        self.torso_end = {'x': 0.0, 'y': -self.walking_params['ySeparation'], 'z': 0.0}
        self.swing_foot_begin = {'x': 0.0, 'y': self.torso_begin['y'] - self.walking_params['ySeparation'], 'z': 0.0}
        self.swing_foot_end = {'x': 0.0, 'y': self.torso_end['y'] - self.walking_params['ySeparation'], 'z': 0.0}
        self.previous_torso = self.torso_end.copy()
        self.left_is_swing_foot = True


######################### FUNCOES SUPORTE DO ROS ###########################
    def load_walking_parameters(self):
        params = {
            'period': rospy.get_param('/walking_params/period', 0.4),
            'zStep': rospy.get_param('/walking_params/zStep', 0.04),
            'doubleSupportRatio': rospy.get_param('/walking_params/doubleSupportRatio', 0.2),
            'zCom': rospy.get_param('/walking_params/zCom', 0.28),
            'zStep': rospy.get_param('/walking_params/zStep', 0.04),
            'armsCompensationFactor': rospy.get_param('/walking_params/armsCompensationFactor', 1.0),
            'comSwingFactor': rospy.get_param('/walking_params/comSwingFactor', 1.0),
            'ySeparation': rospy.get_param('/walking_params/ySeparation', 0.0875),
            'GRAVITY': 9.80665
        }
        params['tZMP'] = math.sqrt(params['zCom'] * params['comSwingFactor'] / params['GRAVITY'])
        rospy.loginfo("Loaded walking parameters: %s", params)
        return params

    def load_physical_parameters(self):
        params = {
            'upper_leg_length': rospy.get_param('/walking_params/upper_leg_length', 0.1),
            'lower_leg_length': rospy.get_param('/walking_params/lower_leg_length', 0.1),
            'hip_pitch0': rospy.get_param('/walking_params/hip_pitch0', 0.0),
            'knee_pitch0': rospy.get_param('/walking_params/knee_pitch0', 0.0),
            'ankle_pitch0': rospy.get_param('/walking_params/ankle_pitch0', 0.0),
            'torso_to_left_hip': rospy.get_param('/walking_params/torso_to_left_hip', [0.007, 0.040, -0.103]),
            'left_ankle_to_left_foot': rospy.get_param('/walking_params/left_ankle_to_left_foot', [0.001, -0.001, -0.038])
        }
        rospy.loginfo("Loaded physical parameters: %s", params)
        return params

    def xyz_callback(self, msg):
        self.v_x = msg.x
        self.v_y = msg.y
        self.v_z = msg.z
        rospy.loginfo("Received velocities: vx=%.2f, vy=%.2f, vψ=%.2f", self.v_x, self.v_y, self.v_z)

    def compute_torso_displacement(self, T):
        """
        Calcula o deslocamento do torso para um passo de duração T.
        Se v_z (vψ) for quase zero, usa integração linear.
        Caso contrário, utiliza:
          a = 2*(vx/vψ)*sin(vψ*T/2)
          b = 2*(vy/vψ)*sin(vψ*T/2)
          disp_x = a*cos(vψ*T/2) - b*sin(vψ*T/2)
          disp_y = a*sin(vψ*T/2) + b*cos(vψ*T/2)
          disp_z = T*vψ
        """
        if abs(self.v_z) < 1e-6:
            disp_x = self.v_x * T
            disp_y = self.v_y * T
            disp_z = 0.0
        else:
            a = 2.0 * (self.v_x / self.v_z) * math.sin(self.v_z * T / 2.0)
            b = 2.0 * (self.v_y / self.v_z) * math.sin(self.v_z * T / 2.0)
            disp_x = a * math.cos(self.v_z * T / 2.0) - b * math.sin(self.v_z * T / 2.0)
            disp_y = a * math.sin(self.v_z * T / 2.0) + b * math.cos(self.v_z * T / 2.0)
            disp_z = T * self.v_z
        return disp_x, disp_y, disp_z

############################ FUNCOES DE CALCULO DE POSE DA CAMINHADA #################

    def get_desired_torso_displacement(self, torso_begin):
        """
        Calcula o deslocamento desejado do torso para um passo de duração T.
        Se v_z for quase zero, usa integração linear.
        Caso contrário, aplica:
          a = 2*(vx/vψ)*sin(vψ*T/2)
          b = 2*(vy/vψ)*sin(vψ*T/2)
          disp_x = a*cos(torso_begin.z + vψ*T/2) - b*sin(torso_begin.z + vψ*T/2)
          disp_y = a*sin(torso_begin.z + vψ*T/2) + b*cos(torso_begin.z + vψ*T/2)
          disp_z = T*vψ
        """
        T = self.walking_params['period']
        if abs(self.v_z) < 1e-6:
            return {'x': self.v_x * T, 'y': self.v_y * T, 'z': 0.0}
        else:
            a = 2.0 * (self.v_x / self.v_z) * math.sin(self.v_z * T / 2.0)
            b = 2.0 * (self.v_y / self.v_z) * math.sin(self.v_z * T / 2.0)
            angle = torso_begin['z'] + self.v_z * T / 2.0
            disp_x = a * math.cos(angle) - b * math.sin(angle)
            disp_y = a * math.sin(angle) + b * math.cos(angle)
            disp_z = T * self.v_z
            return {'x': disp_x, 'y': disp_y, 'z': disp_z}

    def limit_torso(self, torso_begin, torso_disp):
        """
        Limita o deslocamento do torso conforme a separação desejada entre os pés.
        Se o pé de balanço for o esquerdo, garante que o y seja pelo menos ySeparation,
        caso contrário, o máximo é -ySeparation; para o ângulo (z), força 0.
        """
        torso_end = {
            'x': torso_begin['x'] + torso_disp['x'],
            'y': torso_begin['y'] + torso_disp['y'],
            'z': torso_begin['z'] + torso_disp['z']
        }
        y_sep = self.walking_params['ySeparation']
        if self.left_is_swing_foot:
            torso_end['y'] = max(y_sep, torso_end['y'])
            torso_end['z'] = max(0.0, torso_end['z'])
        else:
            torso_end['y'] = min(-y_sep, torso_end['y'])
            torso_end['z'] = min(0.0, torso_end['z'])
        return torso_end

    def switch_swing_foot(self):
        """Alterna o pé de balanço."""
        self.left_is_swing_foot = not self.left_is_swing_foot

    def compute_new_step(self):
        """
        Implementa a lógica para calcular os novos alvos (poses) do torso e do pé de balanço
        para o próximo passo, baseada na abordagem ZMP/3D-LIPM.
        """
        # Para simplificar, ignoramos os estados de parada (stoppingState)
        

        # Define torso_begin como a pose relativa entre torso_end e swing_foot_end
        torso_begin = pose_relative(self.torso_end, self.swing_foot_end)
        # Definimos support_foot como a origem (0,0,0)
        support_foot = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        swing_foot_begin = pose_relative(support_foot, self.swing_foot_end)
        self.previous_torso = pose_relative(self.previous_torso, self.swing_foot_end)

        # Calcula o deslocamento desejado do torso e limita para obter torso_end
        torso_disp = self.get_desired_torso_displacement(torso_begin)
        torso_end = self.limit_torso(torso_begin, torso_disp)

        # Calcula a pose do torso para dois passos adiante
        disp_next = self.get_desired_torso_displacement(torso_end)
        torso_two_steps = {
            'x': torso_end['x'] + disp_next['x'],
            'y': torso_end['y'] + disp_next['y'],
            'z': torso_end['z'] + disp_next['z']
        }

        # Define flags para escolha de referências (oY e oPsi)
        oY = (torso_disp['y'] >= 0 and self.left_is_swing_foot) or (torso_disp['y'] < 0 and not self.left_is_swing_foot)
        oPsi = (torso_disp['z'] >= 0 and self.left_is_swing_foot) or (torso_disp['z'] < 0 and not self.left_is_swing_foot)

        # Seleciona a referência p
        p = torso_two_steps if oPsi else torso_end

        # Garante que o pé de balanço tenha z = 0
        self.swing_foot_end['z'] = 0.0

        # Calcula as poses relativas do torso em relação a p
        torso_end_rel_p = pose_relative(torso_end, p)
        torso_two_steps_rel_p = pose_relative(torso_two_steps, p)

        # Define o deslocamento vertical do pé de balanço com base na separação desejada
        swing_disp_y = self.walking_params['ySeparation'] if self.left_is_swing_foot else -self.walking_params['ySeparation']
        if oY:
            self.swing_foot_end['y'] = torso_two_steps_rel_p['y'] + swing_disp_y
        else:
            self.swing_foot_end['y'] = torso_end_rel_p['y'] + swing_disp_y

        # Calcula dx com base na velocidade e na duração do passo
        T = self.walking_params['period']
        if abs(self.v_z) < 1e-6:
            dx = T * self.v_x
        else:
            a = 2.0 * (self.v_x / self.v_z) * math.sin(self.v_z * T / 2.0)
            b = 2.0 * (self.v_y / self.v_z) * math.sin(self.v_z * T / 2.0)
            cosine = math.cos(self.v_z * T / 2.0)
            sine = math.sin(self.v_z * T / 2.0)
            dx = a * cosine + b * sine if oPsi else a * cosine - b * sine

        self.swing_foot_end['x'] = torso_end_rel_p['x'] + dx / 2.0

        self.swing_foot_begin = swing_foot_begin
        # Atualiza a pose global do pé de balanço a partir da pose relativa
        self.swing_foot_end = pose_global(self.swing_foot_end, p)

        # Atualiza o torso_end para o próximo ciclo
        self.torso_end = torso_end
        self.torso_begin = torso_begin
        self.switch_swing_foot()

        # Para depuração, pode-se imprimir os novos alvos:
        rospy.loginfo("New step computed:")
        rospy.loginfo("  torso_begin: %s", self.torso_begin)
        rospy.loginfo("  torso_end: %s", self.torso_end)
        rospy.loginfo("  swing_foot_end: %s", self.swing_foot_end)


    def solve_zmp(self, zs, z1, z2, x1, x2, period, double_support_ratio, tZMP):
        """
        Calcula os parâmetros aP e aN para a solução do problema ZMP.

        Parâmetros:
            zs: altura desejada da CoM (zCom) durante a etapa
            z1: posição z inicial (por exemplo, do torso ou pé) no início do passo
            z2: posição z final no final do passo
            x1: posição x inicial
            x2: posição x final
            period: período do passo (T)
            double_support_ratio: razão de duplo suporte (rds)
            tZMP: parâmetro tZMP calculado (sqrt(zCom * comSwingFactor / GRAVITY))

        Retorna:
            (aP, aN) - constantes derivadas da solução da equação do modelo 3D-LIPM.
        """
        T1 = period * double_support_ratio / 2.0
        T2 = period - T1

        m1 = (zs - z1) / T1
        m2 = -(zs - z2) / (period - T2)

        # Note que sinh(-T1/tZMP) = -sinh(T1/tZMP)
        c1 = x1 - z1 + tZMP * m1 * math.sinh(-T1 / tZMP)
        c2 = x2 - z2 + tZMP * m2 * math.sinh((period - T2) / tZMP)
        expTStep = math.exp(period / tZMP)

        denominator = expTStep - 1/expTStep
        aP = (c2 - c1 / expTStep) / denominator
        aN = (c1 * expTStep - c2) / denominator

        return aP, aN

    def get_com(self, t, aP, aN, zs, z1, z2, period, double_support_ratio, tZMP):
        """
        Calcula a posição do centro de massa (CoM) ao longo do passo para um dado instante.

        Parâmetros:
            t: instante normalizado (entre 0 e 1) durante o passo
            aP, aN: parâmetros obtidos de solve_zmp
            zs: altura desejada da CoM (zCom)
            z1: posição z inicial
            z2: posição z final
            period: duração total do passo (T)
            double_support_ratio: razão de duplo suporte (rds)
            tZMP: parâmetro tZMP
            
        Retorna:
            com: valor escalar representando a posição (no eixo x, por exemplo) da CoM, 
                após compensações adicionais em função do tempo.
        """
        t1 = double_support_ratio / 2.0
        t2 = 1.0 - t1

        expT = math.exp(period * t / tZMP)
        com = aP * expT + aN / expT

        m1 = (zs - z1) / (t1 * period)
        m2 = -(zs - z2) / (period - t2 * period)

        if t < t1:
            com += z1 + m1 * period * t - tZMP * m1 * math.sinh(period * (t - t1) / tZMP)
        elif t < t2:
            com += zs
        else:
            com += zs + m2 * period * (t - t2) - tZMP * m2 * math.sinh(period * (t - t2) / tZMP)
        return com

    def compute_x_frac(self, t, double_support_ratio):
        """
        Calcula a fração de deslocamento no eixo X durante o passo.
        
        t: instante normalizado (0 <= t <= 1) dentro do período do passo.
        double_support_ratio: razão de duplo suporte.
        """
        t1 = double_support_ratio / 2.0
        t2 = 1.0 - t1
        if t < t1:
            return 0.0
        elif t < t2:
            return 0.5 * (1.0 - math.cos(math.pi * (t - t1) / (t2 - t1)))
        else:
            return 1.0

    def compute_y_frac(self, t, double_support_ratio):
        """
        Calcula a fração de deslocamento no eixo Y durante o passo.
        
        t: instante normalizado (0 <= t <= 1) dentro do período do passo.
        double_support_ratio: razão de duplo suporte.
        """
        t1 = double_support_ratio / 2.0
        t2 = 1.0 - t1
        if t < t1:
            return 0.0
        elif t < t2:
            return 0.5 * (1.0 - math.cos(math.pi * (t - t1) / (t2 - t1)))
        else:
            return 1.0

    def compute_z_frac(self, t, double_support_ratio):
        """
        Calcula a fração de deslocamento no eixo Z (vertical) durante o passo.
        
        t: instante normalizado (0 <= t <= 1) dentro do período do passo.
        double_support_ratio: razão de duplo suporte.
        
        Nota: Essa função normalmente retorna valor diferente de zero apenas
            durante a fase de suporte simples.
        """
        t1 = double_support_ratio / 2.0
        t2 = 1.0 - t1
        if t > t1 and t < t2:
            return 0.5 * (1.0 - math.cos(2.0 * math.pi * (t - t1) / (t2 - t1)))
        else:
            return 0.0

    def compute_theta_frac(self, t, double_support_ratio):
        """
        Calcula a fração de interpolação para a rotação (θ) durante o passo.
        
        t: instante normalizado (0 <= t <= 1) durante o período do passo.
        double_support_ratio: razão de duplo suporte.
        
        Retorna:
            Um valor entre 0 e 1 que representa a fração da rotação desejada ao longo do passo.
        """
        t1 = double_support_ratio / 2.0
        t2 = 1.0 - t1
        if t < t1:
            return 0.0
        elif t < t2:
            return 0.5 * (1.0 - math.cos(math.pi * (t - t1) / (t2 - t1)))
        else:
            return 1.0


    def run(self):
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed_time = (current_time - self.start_time).to_sec()

            # Se o tempo do passo for alcançado, reinicia o ciclo e alterna o pé em balanço
            if elapsed_time >= self.walking_params['period']:
                self.start_time = rospy.Time.now()
                self.compute_new_step()
                rospy.loginfo("Step completed. Switching swing foot: left_is_swing_foot = %s", self.left_is_swing_foot)
                elapsed_time = 0.0

            # Calcula a fração do passo decorrido (entre 0 e 1)
            t_norm = elapsed_time / self.walking_params['period']

            # resolve os parametros para o ZMP
            aPx, aNx = self.solve_zmp(0.0, self.torso_begin['x'], self.torso_end['x'], self.torso_begin['x'], self.torso_end['x'], 
                self.walking_params['period'], self.walking_params['doubleSupportRatio'], self.walking_params['tZMP'])
            aPy, aNy = self.solve_zmp(0.0, self.torso_begin['y'], self.torso_end['y'], self.torso_begin['y'], self.torso_end['y'], 
                self.walking_params['period'], self.walking_params['doubleSupportRatio'], self.walking_params['tZMP'])

            comX = self.get_com(t_norm, aPx, aNx, 0.0, self.torso_begin['x'], self.torso_end['x'],
                self.walking_params['period'], self.walking_params['doubleSupportRatio'], self.walking_params['tZMP'])
            comY = self.get_com(t_norm, aPy, aNy, 0.0, self.torso_begin['y'], self.torso_end['y'],
                self.walking_params['period'], self.walking_params['doubleSupportRatio'], self.walking_params['tZMP'])

            # rospy.loginfo("CoM: (%.3f, %.3f, %.3f)", comX, comY, self.walking_params['zCom'])

            xFrac = self.compute_x_frac(t_norm, self.walking_params['doubleSupportRatio'])
            yFrac = self.compute_y_frac(t_norm, self.walking_params['doubleSupportRatio'])
            zFrac = self.compute_z_frac(t_norm, self.walking_params['doubleSupportRatio'])
            thetaFrac = self.compute_theta_frac(t_norm, self.walking_params['doubleSupportRatio'])

            # rospy.loginfo("Interpolated fractions: X=%.3f, Y=%.3f, Z=%.3f, theta=Z=%.3f", x_frac, y_frac, z_frac, thetaFrac)

            thetaTorso = self.torso_begin['z'] + (self.torso_end['z'] - self.torso_begin['z']) * thetaFrac

            torso = {'x': comX, 'y': comY, 'z': thetaTorso}
            # rospy.loginfo("torso: X=%.3f, Y=%.3f, Z=%.3f", comX, comY, thetaTorso)

            # Calcula a posição global (no caso, global eh com referencia ao pe de suporte)
            x_swing_global = self.swing_foot_begin['x'] + (self.swing_foot_end['x'] - self.swing_foot_begin['x']) * xFrac
            y_swing_global = self.swing_foot_begin['y'] + (self.swing_foot_end['y'] - self.swing_foot_begin['y']) * yFrac
            theta_swing_global = self.swing_foot_begin['z'] + (self.swing_foot_end['z'] - self.swing_foot_begin['z']) * thetaFrac

            swing_global = {'x': x_swing_global, 'y': y_swing_global, 'z': theta_swing_global}
            # rospy.loginfo("swing_global: X=%.3f, Y=%.3f, Z=%.3f", swing_global['x'], swing_global['y'], swing_global['z'])
            support_global = {'x': 0.0, 'y': 0.0, 'z': 0.0}

            # posicao do pe de balanco e de suporte relativos ao torso
            swing_foot = pose_relative(swing_global, torso)
            support_foot = pose_relative(support_global, torso)

            # alturas (coordenada z) do pe de balanco e de suporte
            z_swing = -self.walking_params['zCom'] + zFrac * self.walking_params['zStep']
            z_support = -self.walking_params['zCom']

            rospy.loginfo("swing_foot: X=%.3f, Y=%.3f, Z=%.3f || support_foot: X=%.3f, Y=%.3f, Z=%.3f", swing_foot['x'], swing_foot['y'], z_swing, support_foot['x'], support_foot['y'], z_support)
            

            #TODO: Usar a TF do CoM para calcular as posicoes
            # passa a posicao dos pes para a cinematica inversa
            joint_state = JointState()
            if self.left_is_swing_foot:
                joint_state.position = [
                    swing_foot['x'],  # posição x do swing foot
                    -swing_foot['y'],  # posição y do swing foot
                    z_swing,          # altura do swing foot
                    swing_foot['z'],  # rotação (yaw) do swing foot
                    support_foot['x'],# posição x do support foot
                    -support_foot['y'],# posição y do support foot
                    z_support,        # altura do support foot
                    support_foot['x'],# (ou outro valor desejado, conforme sua definição)
                    0, 0, 0, 0, 0, 0  # demais elementos zerados
                ]
            else:
                joint_state.position = [
                    support_foot['x'],# posição x do support foot
                    -support_foot['y'],# posição y do support foot
                    z_support,        # altura do support foot
                    support_foot['z'],# rotação (yaw) do support foot
                    swing_foot['x'],  # posição x do swing foot
                    -swing_foot['y'],  # posição y do swing foot
                    z_swing,          # altura do swing foot
                    swing_foot['x'],  # (ou outro valor desejado)
                    0, 0, 0, 0, 0, 0  # demais elementos zerados
                ]

            # publica a posicao para os pes
            joint_state.header.stamp = rospy.Time.now()
            self.publisher.publish(joint_state)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = WalkingZMP()
        node.run()
    except rospy.ROSInterruptException:
        pass
