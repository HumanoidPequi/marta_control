/*
 * squat_controller.cpp
 *
 * Standalone ROS node that makes Marta squat down and stand up repeatedly,
 * using inverse kinematics to compute joint angles from a desired CoM height
 * trajectory, and a PD controller to compensate forward/backward tilt
 * measured by the IMU.
 *
 * Control loop overview:
 *
 *   Trajectory (sine) ──► desired CoM height (Z)
 *   IMU acc_x         ──► estimated tilt angle ──► PD ──► CoM X correction
 *   (desired CoM X, Z) ──► Inverse Kinematics ──► joint angles ──► motors
 *
 * Phases:
 *   1. Ramp-up : slowly scales joint commands from 0 → 1 (avoids sudden jerk).
 *   2. Squat   : continuous sinusoidal CoM height + tilt PD correction.
 *
 * Parameters (ROS private, ~):
 *   isReal (bool, default: false) – if true, publish on multi-array topics
 *                                   for the real robot; otherwise individual
 *                                   topics for Gazebo.
 *   kp_tilt (double, default: 0.05) – proportional gain for tilt PD.
 *   kd_tilt (double, default: 0.01) – derivative gain for tilt PD.
 *
 * Build:
 *   Add to CMakeLists.txt (see instructions at the bottom of this file).
 *
 * Run:
 *   rosrun marta_controller squat_controller_node
 */

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <cmath>
#include <deque>
#include <numeric>
#include <unistd.h>

#include "Joints.h"
#include "RobotPhysicalParameters.h"
#include "InverseKinematics.h"
#include "MathUtils.h"
#include "PDController.h"

using namespace representations;
using namespace control::walking;
using itandroids_lib::math::MathUtils;

// ─── Tunable parameters ──────────────────────────────────────────────────────
//
// COM_STAND_HEIGHT  : nominal CoM height when standing [m].
//                    Comes from OmnidirectionalWalkZMPParams::zCom.
// COM_SQUAT_HEIGHT  : lowest CoM height during squat [m].
//                    Decrease carefully – deep squats may exceed servo limits.
// SQUAT_PERIOD      : duration of one full squat cycle [s].
// LOOP_DT           : control loop period [s] (100 Hz).
// GRAVITY           : used to convert IMU acc → tilt angle.
// RAMP_STEPS        : number of steps in the ramp-up phase.
// RAMP_STEP_US      : microseconds per ramp step (50 ms → 5 s total ramp).
// IMU_BUFFER_SIZE   : moving average window for IMU filter.
// HIP_PITCH_OFFSET  : correction applied after IK, same as walk engine [-5°].
//
static const double COM_STAND_HEIGHT  = 0.2975;
static const double COM_SQUAT_HEIGHT  = 0.27;
static const double SQUAT_PERIOD      = 4.0;
static const double LOOP_DT           = 0.01;
static const double GRAVITY           = 9.80665;
static const int    RAMP_STEPS        = 100;
static const int    RAMP_STEP_US      = 50000;   // 50 ms
static const int    IMU_BUFFER_SIZE   = 20;
static const double HIP_PITCH_OFFSET  = -5.0 * M_PI / 180.0;

// ─────────────────────────────────────────────────────────────────────────────

class SquatControllerNode {
public:
    SquatControllerNode();
    ~SquatControllerNode();

    /** Blocking main loop. Call after construction. */
    void run();

private:
    // ── ROS ────────────────────────────────────────────────────────────────
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;

    // Individual joint publishers (used in simulation, isReal = false)
    ros::Publisher pubPeEsquerdoPitch_;
    ros::Publisher pubPeEsquerdoRoll_;
    ros::Publisher pubQuadrilEsquerdoPitch_;
    ros::Publisher pubQuadrilEsquerdoRoll_;
    ros::Publisher pubQuadrilEsquerdoYaw_;
    ros::Publisher pubJoelhoEsquerdoPitch_;
    ros::Publisher pubPeDireitoPitch_;
    ros::Publisher pubPeDireitoRoll_;
    ros::Publisher pubQuadrilDireitoPitch_;
    ros::Publisher pubQuadrilDireitoRoll_;
    ros::Publisher pubQuadrilDireitoYaw_;
    ros::Publisher pubJoelhoDireitoPitch_;
    ros::Publisher pubOmbroEsquerdoPitch_;
    ros::Publisher pubOmbroEsquerdoRoll_;
    ros::Publisher pubOmbroDireitoPitch_;
    ros::Publisher pubOmbroDireitoRoll_;

    // Multi-array publishers (used on real robot, isReal = true)
    ros::Publisher pub_right_leg_;
    ros::Publisher pub_left_leg_;
    ros::Publisher pub_right_arm_;
    ros::Publisher pub_left_arm_head_;

    // ── IMU state ──────────────────────────────────────────────────────────
    std::deque<double> imu_buffer_;   // ring buffer for moving average
    double imu_filtered_;             // filtered forward acceleration [m/s²]

    // ── Controllers ────────────────────────────────────────────────────────
    //
    // tilt_pd_ stabilises the robot around the upright posture during the
    // squat by adjusting the CoM X position fed to the inverse kinematics.
    //
    // Convention used here:
    //   - imu_filtered_ > 0  →  robot leaning forward
    //   - tilt_angle = imu_filtered_ / GRAVITY  (small-angle approximation)
    //   - desired tilt = 0  (stay upright)
    //   - PD error = 0 - tilt_angle  (negative when leaning forward)
    //   - com_x_correction = -pd_output  (flip sign: forward lean → positive
    //     com_x so the IK shifts the joint configuration backward)
    //
    PDController tilt_pd_;

    // ── Robot model ────────────────────────────────────────────────────────
    RobotPhysicalParameters physical_params_;
    InverseKinematics*       ik_;
    bool                     is_real_;

    // ── Private methods ────────────────────────────────────────────────────

    /** Store latest IMU forward acceleration (moving-average filtered). */
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * Compute joint angles for a static, symmetric stance given the desired
     * CoM position in the sagittal (X) and vertical (Z) directions.
     *
     * @param com_x   Forward CoM position [m] (0 = centred over feet).
     * @param com_z   CoM height above ground [m].
     * @param joints  Output joint angles.
     */
    void computeJointsFromCoM(double com_x, double com_z, Joints& joints);

    /**
     * Publish joint commands, scaling every angle by `scalar`.
     * scalar = 0 → motors receive zero command; scalar = 1 → full command.
     */
    void publishJoints(const Joints& joints, double scalar);

    /**
     * Ramp-up phase: hold standing pose and slowly scale from 0 to 1 so
     * the robot does not jerk when the node starts.
     */
    void rampUpToStanding(Joints& joints);

    /**
     * Sinusoidal CoM height trajectory.
     * Returns standing height at t=0, squat height at t=SQUAT_PERIOD/2,
     * and back to standing at t=SQUAT_PERIOD.
     */
    double desiredComHeight(double t) const;
};

// ─── Implementation ───────────────────────────────────────────────────────────

SquatControllerNode::SquatControllerNode()
    : nh_("~"),
      imu_filtered_(0.0),
      tilt_pd_(0.0, 0.0),   // gains loaded from params below
      ik_(nullptr)
{
    // ── Load parameters ────────────────────────────────────────────────────
    nh_.param<bool>("isReal", is_real_, false);

    double kp_tilt, kd_tilt;
    nh_.param<double>("kp_tilt", kp_tilt, 0.05);
    nh_.param<double>("kd_tilt", kd_tilt, 0.01);
    tilt_pd_.setGains(kp_tilt, kd_tilt);

    ROS_INFO("[SquatController] isReal=%s  kp=%.4f  kd=%.4f",
             is_real_ ? "true" : "false", kp_tilt, kd_tilt);

    // ── Robot model ────────────────────────────────────────────────────────
    physical_params_ = RobotPhysicalParameters::getMartaPhysicalParameters();
    ik_ = new InverseKinematics(physical_params_);

    // ── Publishers ─────────────────────────────────────────────────────────
    //   Use Joints::StandardNames() only to get the correct topic strings;
    //   the Joints struct itself is not stored here.
    Joints j;
    j.StandardNames();

    if (is_real_) {
        pub_left_leg_      = nh_.advertise<std_msgs::Float64MultiArray>(j.left_leg,      10);
        pub_right_leg_     = nh_.advertise<std_msgs::Float64MultiArray>(j.right_leg,     10);
        pub_right_arm_     = nh_.advertise<std_msgs::Float64MultiArray>(j.arm_r,         10);
        pub_left_arm_head_ = nh_.advertise<std_msgs::Float64MultiArray>(j.arm_l_head,    10);
    } else {
        pubPeEsquerdoPitch_      = nh_.advertise<std_msgs::Float64>(j.PeEsquerdoPitchTopic,      10);
        pubPeEsquerdoRoll_       = nh_.advertise<std_msgs::Float64>(j.PeEsquerdoRollTopic,       10);
        pubQuadrilEsquerdoPitch_ = nh_.advertise<std_msgs::Float64>(j.QuadrilEsquerdoPitchTopic, 10);
        pubQuadrilEsquerdoRoll_  = nh_.advertise<std_msgs::Float64>(j.QuadrilEsquerdoRollTopic,  10);
        pubQuadrilEsquerdoYaw_   = nh_.advertise<std_msgs::Float64>(j.QuadrilEsquerdoYawTopic,   10);
        pubJoelhoEsquerdoPitch_  = nh_.advertise<std_msgs::Float64>(j.JoelhoEsquerdoPitchTopic,  10);
        pubPeDireitoPitch_       = nh_.advertise<std_msgs::Float64>(j.PeDireitoPitchTopic,       10);
        pubPeDireitoRoll_        = nh_.advertise<std_msgs::Float64>(j.PeDireitoRollTopic,        10);
        pubQuadrilDireitoPitch_  = nh_.advertise<std_msgs::Float64>(j.QuadrilDireitoPitchTopic,  10);
        pubQuadrilDireitoRoll_   = nh_.advertise<std_msgs::Float64>(j.QuadrilDireitoRollTopic,   10);
        pubQuadrilDireitoYaw_    = nh_.advertise<std_msgs::Float64>(j.QuadrilDireitoYawTopic,    10);
        pubJoelhoDireitoPitch_   = nh_.advertise<std_msgs::Float64>(j.JoelhoDireitoPitchTopic,   10);
        pubOmbroEsquerdoPitch_   = nh_.advertise<std_msgs::Float64>(j.OmbroEsquerdoPitchTopic,   10);
        pubOmbroEsquerdoRoll_    = nh_.advertise<std_msgs::Float64>(j.OmbroEsquerdoRollTopic,    10);
        pubOmbroDireitoPitch_    = nh_.advertise<std_msgs::Float64>(j.OmbroDireitoPitchTopic,    10);
        pubOmbroDireitoRoll_     = nh_.advertise<std_msgs::Float64>(j.OmbroDireitoRollTopic,     10);
    }

    // ── Subscribers ────────────────────────────────────────────────────────
    imu_sub_ = nh_.subscribe("/marta/imu", 400,
                             &SquatControllerNode::imuCallback, this);
}

SquatControllerNode::~SquatControllerNode() {
    delete ik_;
}

// ─────────────────────────────────────────────────────────────────────────────

void SquatControllerNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Forward (sagittal) acceleration from the accelerometer.
    // When the robot leans forward by small angle θ:
    //   acc_x ≈ g * sin(θ) ≈ g * θ
    imu_buffer_.push_back(msg->linear_acceleration.x);
    if (static_cast<int>(imu_buffer_.size()) > IMU_BUFFER_SIZE) {
        imu_buffer_.pop_front();
    }
    imu_filtered_ = std::accumulate(imu_buffer_.begin(), imu_buffer_.end(), 0.0)
                    / static_cast<double>(imu_buffer_.size());
}

// ─────────────────────────────────────────────────────────────────────────────

void SquatControllerNode::computeJointsFromCoM(double com_x, double com_z,
                                               Joints& joints) {
    // For a symmetric, static stance:
    //   - Both feet are at the same X (adjusted for balance) and Z height.
    //   - Y is 0 in torso frame; the IK internally applies the hip Y offset
    //     (torsoToLeftHip.y) to both legs.
    //   - Theta (yaw) is 0 for both feet.
    //
    // com_z is the CoM height above the ground.  The IK receives it as the
    // distance from the CoM to the foot, with Z offsets for the hip and ankle
    // links applied internally inside computeJointsTargets.
    float xscale = 1.0f;
    ik_->computeJointsTargets(
        com_x, 0.0, com_z, 0.0,   // left  foot: x, y, z, theta
        com_x, 0.0, com_z, 0.0,   // right foot: x, y, z, theta
        joints, xscale);

    // Identical correction used by the walk engine to reduce hip overload.
    joints.leftHipPitch  -= 5.0 * M_PI / 180.0;
    joints.rightHipPitch -= 5.0 * M_PI / 180.0;

    // Arms: neutral position (not moving, so just set a safe resting pose).
    joints.leftShoulderPitch  = 0.0;
    joints.rightShoulderPitch = 0.0;
    joints.leftShoulderRoll   = MathUtils::degreesToRadians(-50.0);
    joints.rightShoulderRoll  = MathUtils::degreesToRadians(50.0);
}

// ─────────────────────────────────────────────────────────────────────────────

void SquatControllerNode::publishJoints(const Joints& joints, double scalar) {
    if (is_real_) {
        std_msgs::Float64MultiArray msg;

        // Real robot: grouped multi-array topics per limb.
        // head_pan (-10) and head_tilt (-10) are passed as placeholders
        // (same convention as controller.cpp) so the firmware ignores them.
        msg.data = {-10,
                    -10,
                    joints.leftShoulderPitch * scalar,
                    joints.leftShoulderRoll  * scalar,
                    0};
        pub_left_arm_head_.publish(msg);

        msg.data = {joints.rightShoulderPitch * scalar,
                    joints.rightShoulderRoll  * scalar,
                    0};
        pub_right_arm_.publish(msg);

        msg.data = {joints.leftHipYaw   * scalar,
                    joints.leftHipRoll  * scalar,
                    joints.leftHipPitch * scalar,
                    joints.leftKneePitch    * scalar,
                    joints.leftAnklePitch   * scalar,
                    joints.leftAnkleRoll    * scalar};
        pub_left_leg_.publish(msg);

        msg.data = {joints.rightHipYaw   * scalar,
                    joints.rightHipRoll  * scalar,
                    joints.rightHipPitch * scalar,
                    joints.rightKneePitch    * scalar,
                    joints.rightAnklePitch   * scalar,
                    joints.rightAnkleRoll    * scalar};
        pub_right_leg_.publish(msg);

    } else {
        // Simulation: one Float64 per joint.
        std_msgs::Float64 msg;

        msg.data = joints.leftShoulderPitch  * scalar; pubOmbroEsquerdoPitch_.publish(msg);
        msg.data = joints.leftShoulderRoll   * scalar; pubOmbroEsquerdoRoll_.publish(msg);
        msg.data = joints.rightShoulderPitch * scalar; pubOmbroDireitoPitch_.publish(msg);
        msg.data = joints.rightShoulderRoll  * scalar; pubOmbroDireitoRoll_.publish(msg);

        msg.data = joints.leftHipYaw    * scalar; pubQuadrilEsquerdoYaw_.publish(msg);
        msg.data = joints.leftHipRoll   * scalar; pubQuadrilEsquerdoRoll_.publish(msg);
        msg.data = joints.leftHipPitch  * scalar; pubQuadrilEsquerdoPitch_.publish(msg);
        msg.data = joints.leftKneePitch * scalar; pubJoelhoEsquerdoPitch_.publish(msg);
        msg.data = joints.leftAnklePitch* scalar; pubPeEsquerdoPitch_.publish(msg);
        msg.data = joints.leftAnkleRoll * scalar; pubPeEsquerdoRoll_.publish(msg);

        msg.data = joints.rightHipYaw    * scalar; pubQuadrilDireitoYaw_.publish(msg);
        msg.data = joints.rightHipRoll   * scalar; pubQuadrilDireitoRoll_.publish(msg);
        msg.data = joints.rightHipPitch  * scalar; pubQuadrilDireitoPitch_.publish(msg);
        msg.data = joints.rightKneePitch * scalar; pubJoelhoDireitoPitch_.publish(msg);
        msg.data = joints.rightAnklePitch* scalar; pubPeDireitoPitch_.publish(msg);
        msg.data = joints.rightAnkleRoll * scalar; pubPeDireitoRoll_.publish(msg);
    }
}

// ─────────────────────────────────────────────────────────────────────────────

void SquatControllerNode::rampUpToStanding(Joints& joints) {
    ROS_INFO("[SquatController] Ramping up to standing pose over %.1f s ...",
             (RAMP_STEPS * RAMP_STEP_US) / 1e6);

    computeJointsFromCoM(0.0, COM_STAND_HEIGHT, joints);

    for (int i = 1; i <= RAMP_STEPS; ++i) {
        publishJoints(joints, static_cast<double>(i) / RAMP_STEPS);
        ros::spinOnce();
        usleep(RAMP_STEP_US);
    }

    ROS_INFO("[SquatController] Ramp-up complete. Starting squat loop.");
}

// ─────────────────────────────────────────────────────────────────────────────

double SquatControllerNode::desiredComHeight(double t) const {
    // Cosine from standing (t=0) down to squat (t=T/2) and back up (t=T).
    double mid = (COM_STAND_HEIGHT + COM_SQUAT_HEIGHT) / 2.0;
    double amp = (COM_STAND_HEIGHT - COM_SQUAT_HEIGHT) / 2.0;
    return mid + amp * std::cos(2.0 * M_PI * t / SQUAT_PERIOD);
}

// ─────────────────────────────────────────────────────────────────────────────

void SquatControllerNode::run() {
    Joints joints;
    joints.StandardNames();

    // ── Phase 1: ramp-up ───────────────────────────────────────────────────
    // Wait briefly for IMU to start publishing before entering the loop,
    // then ramp up from zero torque to the full standing pose.
    ros::Duration(1.0).sleep();
    rampUpToStanding(joints);

    // ── Phase 2: squat loop ────────────────────────────────────────────────
    const useconds_t loop_us = static_cast<useconds_t>(LOOP_DT * 1e6);
    ros::Time start = ros::Time::now();

    while (ros::ok()) {
        double elapsed = (ros::Time::now() - start).toSec();

        // ── Desired CoM height from trajectory ──────────────────────────
        double desired_z = desiredComHeight(elapsed);

        // ── Tilt estimation and PD correction ───────────────────────────
        //
        // imu_filtered_ ≈ g * sin(tilt) ≈ g * tilt  (small-angle approx.)
        // Positive imu_filtered_ → robot leaning forward.
        //
        // We want the robot to stay upright (desired tilt = 0).
        // The PD output is:  Kp*(0 - tilt) + Kd*(-d_tilt/dt)
        //
        // The sign inversion below maps the PD output to a CoM X correction:
        //   forward lean (tilt > 0) → pd_output < 0 → com_x > 0
        // A positive com_x in the IK shifts the joint configuration so the
        // ankles compensate and bring the body back toward upright.
        //
        double tilt_angle   = imu_filtered_ / GRAVITY;
        double pd_output    = tilt_pd_.compute(0.0, tilt_angle, LOOP_DT);
        double com_x        = -pd_output;   // sign convention, see comment above

        // ── Compute joints ───────────────────────────────────────────────
        computeJointsFromCoM(com_x, desired_z, joints);

        // ── Log every 100 iterations (1 s at 100 Hz) ────────────────────
        static int log_counter = 0;
        if (++log_counter >= 100) {
            log_counter = 0;
            ROS_INFO("[SquatController] t=%.2f s  CoM Z=%.4f m  tilt=%.4f rad  com_x=%.4f m",
                     elapsed, desired_z, tilt_angle, com_x);
        }

        // ── Publish ──────────────────────────────────────────────────────
        publishJoints(joints, 1.0);

        ros::spinOnce();
        usleep(loop_us);
    }
}

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    ros::init(argc, argv, "squat_controller_node");

    SquatControllerNode node;
    node.run();

    return 0;
}
