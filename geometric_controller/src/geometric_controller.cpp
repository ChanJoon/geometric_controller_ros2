/****************************************************************************
 *
 *   Copyright (c) 2024 Chanjoon Park. All rights reserved.
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author
 * - Jaeyoung Lim <jalim@ethz.ch>
 * - Chanjoon Park <chanjoon.park@kaist.ac.kr>
 */

#include "geometric_controller/geometric_controller.h"
#include "geometric_controller/jerk_tracking_control.h"
#include "geometric_controller/nonlinear_attitude_control.h"
#include "geometric_controller/nonlinear_geometric_control.h"

using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl() : Node("geometric_controller"), last_request_(this->get_clock()->now()), reference_request_now_(this->get_clock()->now()), reference_request_last_(this->get_clock()->now()) {
  referenceSub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("reference/setpoint", 1, std::bind(&geometricCtrl::targetCallback, this, std::placeholders::_1));
  flatreferenceSub_ = this->create_subscription<controller_msgs::msg::FlatTarget>("reference/flatsetpoint", 1, std::bind(&geometricCtrl::flattargetCallback, this, std::placeholders::_1));
  yawreferenceSub_ = this->create_subscription<std_msgs::msg::Float32>("reference/yaw", 1, std::bind(&geometricCtrl::yawtargetCallback, this, std::placeholders::_1));
  multiDOFJointSub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("command/trajectory", 1, std::bind(&geometricCtrl::multiDOFJointCallback, this, std::placeholders::_1));
  mavstateSub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos_profile, std::bind(&geometricCtrl::mavstateCallback, this, std::placeholders::_1));
  mavposSub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos_profile, std::bind(&geometricCtrl::mavposeCallback, this, std::placeholders::_1));
  mavattSub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude", qos_profile, std::bind(&geometricCtrl::mavattCallback, this, std::placeholders::_1));
  // mavodomSub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", qos_profile, std::bind(&geometricCtrl::mavodomCallback, this, std::placeholders::_1));
  cmdloop_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&geometricCtrl::cmdloopCallback, this));

  angularVelPub_ = this->create_publisher<px4_msgs::msg::VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", qos_profile);
  referencePosePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("reference/pose", qos_profile);
  target_pose_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);
  posehistoryPub_ = this->create_publisher<nav_msgs::msg::Path>("geometric_controller/path", qos_profile);
  vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
  offb_ctrl_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);
  land_service_ = this->create_service<std_srvs::srv::SetBool>("land", std::bind(&geometricCtrl::landCallback, this, std::placeholders::_1, std::placeholders::_2));

  currentPosePub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current/pose", qos_profile);

  // Declare parameters
  this->declare_parameter<std::string>("mavname", "iris");
  this->declare_parameter<int>("ctrl_mode", ERROR_QUATERNION);
  this->declare_parameter<bool>("enable_sim", true);
  this->declare_parameter<bool>("velocity_yaw", false);
  this->declare_parameter<double>("max_acc", 9.0);
  this->declare_parameter<double>("yaw_heading", 0.0);
  this->declare_parameter<double>("drag_dx", 0.0);
  this->declare_parameter<double>("drag_dy", 0.0);
  this->declare_parameter<double>("drag_dz", 0.0);
  this->declare_parameter<double>("attctrl_constant", 0.1);
  this->declare_parameter<double>("normalizedthrust_constant", 0.05);
  this->declare_parameter<double>("normalizedthrust_offset", 0.1);
  this->declare_parameter<double>("Kp_x", 8.0);
  this->declare_parameter<double>("Kp_y", 8.0);
  this->declare_parameter<double>("Kp_z", 10.0);
  this->declare_parameter<double>("Kv_x", 1.5);
  this->declare_parameter<double>("Kv_y", 1.5);
  this->declare_parameter<double>("Kv_z", 3.3);
  this->declare_parameter<int>("posehistory_window", 200);
  this->declare_parameter<double>("init_pos_x", 0.0);
  this->declare_parameter<double>("init_pos_y", 0.0);
  this->declare_parameter<double>("init_pos_z", 2.0);

  // Retrieve parameter values
  this->get_parameter("mavname", mav_name_);
  this->get_parameter("ctrl_mode", ctrl_mode_);
  this->get_parameter("enable_sim", sim_enable_);
  this->get_parameter("velocity_yaw", velocity_yaw_);
  this->get_parameter("max_acc", max_fb_acc_);
  this->get_parameter("yaw_heading", mavYaw_);

  double dx, dy, dz;
  this->get_parameter("drag_dx", dx);
  this->get_parameter("drag_dy", dy);
  this->get_parameter("drag_dz", dz);
  D_ << dx, dy, dz;

  double attctrl_tau;
  this->get_parameter("attctrl_constant", attctrl_tau);
  this->get_parameter("normalizedthrust_constant", norm_thrust_const_);
  this->get_parameter("normalizedthrust_offset", norm_thrust_offset_);
  this->get_parameter("Kp_x", Kpos_x_);
  this->get_parameter("Kp_y", Kpos_y_);
  this->get_parameter("Kp_z", Kpos_z_);
  this->get_parameter("Kv_x", Kvel_x_);
  this->get_parameter("Kv_y", Kvel_y_);
  this->get_parameter("Kv_z", Kvel_z_);
  this->get_parameter("posehistory_window", posehistory_window_);
  this->get_parameter("init_pos_x", initTargetPos_x_);
  this->get_parameter("init_pos_y", initTargetPos_y_);
  this->get_parameter("init_pos_z", initTargetPos_z_);

  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;

  bool jerk_enabled = false;
  if (!jerk_enabled) {
    if (ctrl_mode_ == ERROR_GEOMETRIC) {
      controller_ = std::make_shared<NonlinearGeometricControl>(attctrl_tau);
    } else {
      controller_ = std::make_shared<NonlinearAttitudeControl>(attctrl_tau);
    }
  } else {
    controller_ = std::make_shared<JerkTrackingControl>();
  }

  auto status_callback = [this]() -> void {
    if (sim_enable_ && received_home_pose) {
      if (offb_counter_ == 10) {
        this->pubVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        this->arm();
      }

      pubOffbControlMode();

      if (offb_counter_ < 11) {
        offb_counter_++;
      }
    }
  };
  statusloop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), status_callback);
}
geometricCtrl::~geometricCtrl() {
  // Destructor
}

void geometricCtrl::arm()
{
	pubVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void geometricCtrl::pubVehicleCommand(uint16_t command, float param1, float param2)
{
	px4_msgs::msg::VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_pub_->publish(msg);
}

void geometricCtrl::pubOffbControlMode()
{
	px4_msgs::msg::OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offb_ctrl_mode_pub_->publish(msg);
}

void geometricCtrl::targetCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = this->get_clock()->now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).seconds();

  targetPos_ = toEigen(msg->twist.angular);
  targetVel_ = toEigen(msg->twist.linear);

  if (reference_request_dt_ > 0)
    targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
  else
    targetAcc_ = Eigen::Vector3d::Zero();

  reference_request_last_ = reference_request_now_;
}

void geometricCtrl::flattargetCallback(const controller_msgs::msg::FlatTarget::SharedPtr msg) {
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  targetPos_ = toEigen(msg->position);
  targetVel_ = toEigen(msg->velocity);

  if (msg->type_mask == 1) {
    targetAcc_ = toEigen(msg->acceleration);
    targetJerk_ = toEigen(msg->jerk);
    targetSnap_ = Eigen::Vector3d::Zero();

  } else if (msg->type_mask == 2) {
    targetAcc_ = toEigen(msg->acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

  } else if (msg->type_mask == 4) {
    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

  } else {
    targetAcc_ = toEigen(msg->acceleration);
    targetJerk_ = toEigen(msg->jerk);
    targetSnap_ = toEigen(msg->snap);
  }
}

void geometricCtrl::yawtargetCallback(const std_msgs::msg::Float32::SharedPtr msg) {
  if (!velocity_yaw_) mavYaw_ = double(msg->data);
}

void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg) {
  auto pt = msg->points[0];

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = this->get_clock()->now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).seconds();

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();

  if (!velocity_yaw_) {
    Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
                         pt.transforms[0].rotation.z);
    Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
    mavYaw_ = rpy(2);
  }

  reference_request_last_ = reference_request_now_;
}

void geometricCtrl::mavodomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
  if (!received_home_pose) {
    received_home_pose = true;
    geometry_msgs::msg::Pose tmp_pose;

    // Convert position from NED to ENU
    Eigen::Vector3d ned_position(msg->position[0], msg->position[1], msg->position[2]);
    Eigen::Vector3d enu_position = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned_position);
    tmp_pose.position.x = enu_position.x();
    tmp_pose.position.y = enu_position.y();
    tmp_pose.position.z = enu_position.z();

    // Convert orientation from NED to ENU
    Eigen::Quaterniond ned_orientation(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    Eigen::Quaterniond enu_orientation = px4_ros_com::frame_transforms::px4_to_ros_orientation(ned_orientation);
    tmp_pose.orientation.w = enu_orientation.w();
    tmp_pose.orientation.x = enu_orientation.x();
    tmp_pose.orientation.y = enu_orientation.y();
    tmp_pose.orientation.z = enu_orientation.z();

    home_pose_ = tmp_pose;

    RCLCPP_INFO(this->get_logger(), "Home pose initialized to: [x: %f, y: %f, z: %f]",
                enu_position.x(), enu_position.y(), enu_position.z());
  }

  // Update MAV position, velocity, and orientation in ENU frame
  Eigen::Vector3d ned_pos(msg->position[0], msg->position[1], msg->position[2]);
  mavPos_ = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned_pos);

  Eigen::Vector3d ned_vel(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
  mavVel_ = px4_ros_com::frame_transforms::ned_to_enu_local_frame(ned_vel);

  Eigen::Quaterniond ned_att(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
  Eigen::Quaterniond enu_att = px4_ros_com::frame_transforms::px4_to_ros_orientation(ned_att);
  mavAtt_ << enu_att.w(), enu_att.x(), enu_att.y(), enu_att.z();
}

void geometricCtrl::mavposeCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  if (!received_home_pose) {
    received_home_pose = true;
    geometry_msgs::msg::Pose tmp_pose;
    tmp_pose.position.x = msg->x;
    tmp_pose.position.y = -msg->y;
    tmp_pose.position.z = -msg->z;

    home_pose_ = tmp_pose;

    RCLCPP_INFO(this->get_logger(), "Home pose initialized to: [x: %f, y: %f, z: %f]",
                home_pose_.position.x, home_pose_.position.y, home_pose_.position.z);
  }
  mavPos_ << msg->x, -msg->y, -msg->z;
  mavVel_ << msg->vx, -msg->vy, -msg->vz;
}

void geometricCtrl::mavattCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
  Eigen::Quaterniond q(msg->q[0], msg->q[1], -msg->q[2], -msg->q[3]);
  mavAtt_ << q.w(), q.x(), q.y(), q.z();
}

bool geometricCtrl::landCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Land requested");
  node_state = LANDING;
  return true;
}

void geometricCtrl::cmdloopCallback() {
  switch (node_state) {
    case WAITING_FOR_HOME_POSE:
      if (received_home_pose) {
        RCLCPP_INFO(this->get_logger(), "Got pose! Drone Ready to be armed.");
        node_state = MISSION_EXECUTION;
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for home pose...");
      }
      break;

    case MISSION_EXECUTION: {
      // pubOffbControlMode();
      if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        Eigen::Vector3d desired_acc;
        if (feedthrough_enable_) {
          desired_acc = targetAcc_;
        } else {
          desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
        }
        computeBodyRateCmd(cmdBodyRate_, desired_acc);
        pubReferencePose(targetPos_, q_des);
        pubRateCommands(cmdBodyRate_, q_des);
        appendPoseHistory();
        pubPoseHistory();
        pubCurrentPose(mavPos_, mavAtt_);
      }
      break;
    }

    case LANDING: {
      px4_msgs::msg::TrajectorySetpoint landingmsg;
      landingmsg.timestamp = this->get_clock()->now().seconds();
      landingmsg.position[0] = home_pose_.position.x;
      landingmsg.position[1] = home_pose_.position.y;
      landingmsg.position[2] = home_pose_.position.z + 1.0;
      target_pose_pub_->publish(landingmsg);
      node_state = LANDED;
      break;
    }
    case LANDED:
      RCLCPP_INFO(this->get_logger(), "Landed. Please set to position control and disarm.");
      break;
  }
}

void geometricCtrl::mavstateCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  nav_state_ = msg->nav_state;
  arming_state_ = msg->arming_state;
  // RCLCPP_INFO(this->get_logger(), "NavState: %d", nav_state_);
  // RCLCPP_INFO(this->get_logger(), "    - offboard status: %s", (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) ? "true" : "false");
}

void geometricCtrl::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude) {
  geometry_msgs::msg::PoseStamped msg;

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = target_position(0);
  msg.pose.position.y = target_position(1);
  msg.pose.position.z = target_position(2);
  msg.pose.orientation.w = target_attitude(0);
  msg.pose.orientation.x = target_attitude(1);
  msg.pose.orientation.y = target_attitude(2);
  msg.pose.orientation.z = target_attitude(3);
  referencePosePub_->publish(msg);

  // px4_msgs::msg::TrajectorySetpoint targetmsg;
  // targetmsg.timestamp = this->get_clock()->now().nanoseconds() / 1000; // Timestamp in microseconds
  // Eigen::Vector3d enu_position = px4_ros_com::frame_transforms::enu_to_ned_local_frame(target_position);
  // targetmsg.position[0] = enu_position(0);
  // targetmsg.position[1] = enu_position(1);
  // targetmsg.position[2] = enu_position(2);
  // target_pose_pub_->publish(targetmsg);
}

void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude) {
  Eigen::Vector3d ned_rates(cmd(0), -cmd(1), -cmd(2));

  // Convert thrust (ENU z-axis to NED z-axis)
  double ned_thrust = -cmd(3); // Thrust is inverted due to axis flipping

  // Prepare and publish the message
  px4_msgs::msg::VehicleRatesSetpoint msg;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000; // Timestamp in microseconds
  msg.roll = ned_rates.x();  // NED roll rate
  msg.pitch = ned_rates.y(); // NED pitch rate
  msg.yaw = ned_rates.z();   // NED yaw rate
  msg.thrust_body[2] = ned_thrust;

  angularVelPub_->publish(msg);
}

void geometricCtrl::pubPoseHistory() {
  nav_msgs::msg::Path msg;

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";
  msg.poses = posehistory_vector_;

  posehistoryPub_->publish(msg);
}

void geometricCtrl::appendPoseHistory() {
  posehistory_vector_.insert(posehistory_vector_.begin(), vector3d2PoseStampedMsg(mavPos_, mavAtt_));
  if (posehistory_vector_.size() > posehistory_window_) {
    posehistory_vector_.pop_back();
  }
}

void geometricCtrl::pubCurrentPose(Eigen::Vector3d &position, Eigen::Vector4d &orientation) {
  currentPosePub_->publish(vector3d2PoseStampedMsg(position, orientation));
}

geometry_msgs::msg::PoseStamped geometricCtrl::vector3d2PoseStampedMsg(Eigen::Vector3d &position,
                                                                  Eigen::Vector4d &orientation) {
  geometry_msgs::msg::PoseStamped encode_msg;
  encode_msg.header.stamp = this->get_clock()->now();
  encode_msg.header.frame_id = "map";
  encode_msg.pose.orientation.w = orientation(0);
  encode_msg.pose.orientation.x = orientation(1);
  encode_msg.pose.orientation.y = orientation(2);
  encode_msg.pose.orientation.z = orientation(3);
  encode_msg.pose.position.x = position(0);
  encode_msg.pose.position.y = position(1);
  encode_msg.pose.position.z = position(2);
  return encode_msg;
}

Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc) {
  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3d a_ref = target_acc;
  if (velocity_yaw_) {
    mavYaw_ = getVelocityYaw(mavVel_);
  }

  const Eigen::Vector4d q_ref = acc2quaternion(a_ref - gravity_, mavYaw_);
  const Eigen::Matrix3d R_ref = quat2RotMatrix(q_ref);

  const Eigen::Vector3d pos_error = mavPos_ - target_pos;
  const Eigen::Vector3d vel_error = mavVel_ - target_vel;

  // Position Controller
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

  // Rotor Drag compensation
  const Eigen::Vector3d a_rd = R_ref * D_.asDiagonal() * R_ref.transpose() * target_vel;  // Rotor drag

  // Reference acceleration
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - gravity_;

  return a_des;
}

void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
  // Reference attitude
  q_des = acc2quaternion(a_des, mavYaw_);

  controller_->Update(mavAtt_, q_des, a_des, targetJerk_);  // Calculate BodyRate
  bodyrate_cmd.head(3) = controller_->getDesiredRate();
  double thrust_command = controller_->getDesiredThrust().z();
  bodyrate_cmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * thrust_command +
                                      norm_thrust_offset_));  // Calculate thrustcontroller_->getDesiredThrust()(3);
}

Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error

  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  return a_fb;
}

Eigen::Vector4d geometricCtrl::acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
  Eigen::Vector4d quat;
  Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
  Eigen::Matrix3d rotmat;

  proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

  zb_des = vector_acc / vector_acc.norm();
  yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
  xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

  rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
  quat = rot2Quaternion(rotmat);
  return quat;
}

