/*
 * LegGravityCompensation.cpp -- ver LegGravityCompensation.h.
 */
#include "LegGravityCompensation.h"

#include <std_msgs/Float64MultiArray.h>
#include <urdf/model.h>

#include <algorithm>
#include <sstream>

#include "RobotModelLoader.h"

namespace control {
namespace stabilization {

using representations::Joints;

LegGravityCompensation::LegGravityCompensation(ros::NodeHandle &nh) : nh_(nh) {
    const struct { double Joints::*f; const char *n; const char *k; bool l; } tab[] = {
        {&Joints::leftHipYaw, "l_hip_yaw_link", "hip_yaw", true},
        {&Joints::leftHipRoll, "l_hip_roll_link", "hip_roll", true},
        {&Joints::leftHipPitch, "l_hip_pitch_link", "hip_pitch", true},
        {&Joints::leftKneePitch, "l_knee_link", "knee", true},
        {&Joints::leftAnklePitch, "l_ank_pitch_link", "ank_pitch", true},
        {&Joints::leftAnkleRoll, "l_ank_roll_link", "ank_roll", true},
        {&Joints::rightHipYaw, "r_hip_yaw_link", "hip_yaw", false},
        {&Joints::rightHipRoll, "r_hip_roll_link", "hip_roll", false},
        {&Joints::rightHipPitch, "r_hip_pitch_link", "hip_pitch", false},
        {&Joints::rightKneePitch, "r_knee_link", "knee", false},
        {&Joints::rightAnklePitch, "r_ank_pitch_link", "ank_pitch", false},
        {&Joints::rightAnkleRoll, "r_ank_roll_link", "ank_roll", false},
    };
    for (const auto &t : tab) {
        LegJoint j;
        j.field = t.f;
        j.urdf = t.n;
        j.key = t.k;
        j.left = t.l;
        legs_.push_back(j);
    }
    // Bracos: entram na cinematica (massa e CoM), nao sao compensados.
    others_ = {{&Joints::leftShoulderPitch, "l_sho_pitch_link"},
               {&Joints::leftShoulderRoll, "l_sho_roll_link"},
               {&Joints::rightShoulderPitch, "r_sho_pitch_link"},
               {&Joints::rightShoulderRoll, "r_sho_roll_link"}};
}

bool LegGravityCompensation::init(bool isReal) {
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/marta/gravity_comp", 10);
    refreshParams();
    if (isReal) {
        ROS_WARN_COND(enabled_, "[GravityComp] isReal=true: compensacao DESLIGADA "
                      "(o Kp do servo real nao e o do Gazebo).");
        return false;
    }
    urdf::Model urdf;
    if (!urdf.initParam("robot_description")) {
        ROS_ERROR("[GravityComp] sem robot_description; compensacao indisponivel.");
        return false;
    }
    RigidBodyModel model;
    std::string err;
    if (!buildRigidBodyModel(urdf, model, err) || !comp_.setModel(model)) {
        ROS_ERROR("[GravityComp] modelo invalido: %s", err.c_str());
        return false;
    }
    leftFoot_ = model.linkIndex("l_feet_1");
    rightFoot_ = model.linkIndex("r_feet_1");
    double kpDefault = 50.0;
    nh_.param<double>("servo_kp", kpDefault, kpDefault);
    for (auto &j : legs_) {
        j.index = model.jointIndex(j.urdf);
        if (j.index < 0 || leftFoot_ < 0 || rightFoot_ < 0) {
            ROS_ERROR("[GravityComp] junta ou pe ausente no URDF (%s).", j.urdf.c_str());
            return false;
        }
        // O Kp que interessa e o do controlador que de fato roda.
        const std::string ctrl = j.urdf.substr(0, j.urdf.size() - 5);   // tira "_link"
        if (!ros::param::get("/marta/" + ctrl + "_position/pid/p", j.kp)) {
            j.kp = kpDefault;
            ROS_WARN("[GravityComp] sem /marta/%s_position/pid/p; usando servo_kp=%.2f",
                     ctrl.c_str(), kpDefault);
        }
        auto uj = urdf.getJoint(j.urdf);
        j.effort = (uj && uj->limits) ? uj->limits->effort : 1e9;
    }
    for (auto &o : others_)
        o.index = model.jointIndex(o.urdf);
    ready_ = true;
    ROS_INFO("[GravityComp] modelo de %.3f kg; Kp joelho %.2f, quadril roll %.2f N.m/rad; "
             "limite %.2f N.m; %s, juntas [%s], ganho %.2f, max %.2f rad, balanco %s",
             comp_.totalMass(), legs_[3].kp, legs_[1].kp, legs_[3].effort,
             enabled_ ? "LIGADA" : "desligada", mask_.c_str(), gain_, maxDelta_,
             swing_ ? "sim" : "nao");
    return true;
}

void LegGravityCompensation::parseMask(const std::string &mask) {
    mask_ = mask;
    std::vector<std::string> keys;
    std::stringstream ss(mask);
    std::string k;
    while (std::getline(ss, k, ','))
        keys.push_back(k);
    for (auto &j : legs_)
        j.active = std::find(keys.begin(), keys.end(), j.key) != keys.end()
                   || std::find(keys.begin(), keys.end(), "all") != keys.end();
}

void LegGravityCompensation::refreshParams() {
    bool en = enabled_;
    nh_.getParamCached("gravity_comp_enabled", en);
    if (en && !enabled_)
        ramp_ = 0.0;          // liga com rampa
    enabled_ = en;
    nh_.getParamCached("gravity_comp_gain", gain_);
    nh_.getParamCached("max_gravity_comp", maxDelta_);
    nh_.getParamCached("gravity_comp_swing", swing_);
    std::string mask = mask_.empty() ? "hip_roll,knee" : mask_;
    nh_.getParamCached("gravity_comp_joints", mask);
    if (mask != mask_)
        parseMask(mask);
}

void LegGravityCompensation::apply(Joints &joints, double wLeft, double wRight, double dt) {
    if (!enabled())
        return;
    ramp_ = std::min(1.0, ramp_ + dt / RAMP_TIME);

    std::vector<double> q(comp_.model().joints.size(), 0.0);
    for (const auto &j : legs_)
        q[j.index] = joints.*(j.field);
    for (const auto &o : others_)
        if (o.index >= 0)
            q[o.index] = joints.*(o.field);

    const auto supL = comp_.legTorques(q, leftFoot_, true);
    const auto supR = comp_.legTorques(q, rightFoot_, true);
    std::map<int, double> hangL, hangR;
    if (swing_) {
        hangL = comp_.legTorques(q, leftFoot_, false);
        hangR = comp_.legTorques(q, rightFoot_, false);
    }

    std_msgs::Float64MultiArray msg;
    msg.data = {wLeft, wRight, ramp_};
    std::vector<double> taus;
    for (const auto &j : legs_) {
        const double w = j.left ? wLeft : wRight;
        const auto &sup = j.left ? supL : supR;
        const auto &hang = j.left ? hangL : hangR;
        double tau = w * sup.at(j.index);
        if (swing_)
            tau += (1.0 - w) * hang.at(j.index);
        // O feedforward nao pede torque que o servo nao entrega.
        tau = std::max(-j.effort, std::min(j.effort, gain_ * tau));
        double delta = 0.0;
        if (j.active && j.kp > 0.0)
            delta = ramp_ * std::max(-maxDelta_, std::min(maxDelta_, tau / j.kp));
        joints.*(j.field) += delta;
        msg.data.push_back(delta);
        taus.push_back(tau);
    }
    msg.data.insert(msg.data.end(), taus.begin(), taus.end());
    pub_.publish(msg);
}

}  // namespace stabilization
}  // namespace control
