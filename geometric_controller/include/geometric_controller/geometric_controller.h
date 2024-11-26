/****************************************************************************
 *
 *   Copyright (c) 2024 Chanjoon Park. All rights reserved.
 *   Copyright (c) 2018-2022 Jaeyoung Lim. All rights reserved.
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

#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float32.hpp>
#include <Eigen/Dense>

#include <controller_msgs/msg/flat_target.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_ros_com/frame_transforms.h>

#include "geometric_controller/common.h"
#include "geometric_controller/control.h"

#define ERROR_QUATERNION 1
#define ERROR_GEOMETRIC 2

using namespace std;
using namespace Eigen;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

class geometricCtrl : public rclcpp::Node {
 private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr referenceSub_;
  rclcpp::Subscription<controller_msgs::msg::FlatTarget>::SharedPtr flatreferenceSub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr multiDOFJointSub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr mavstateSub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr mavodomSub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr mavposSub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr mavattSub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yawreferenceSub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr angularVelPub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr referencePosePub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr posehistoryPub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offb_ctrl_mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr currentPosePub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr land_service_;
  rclcpp::TimerBase::SharedPtr cmdloop_timer_, statusloop_timer_;
  rclcpp::Time last_request_, reference_request_now_, reference_request_last_;
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                                    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                                    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
                                    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

  string mav_name_;
  bool fail_detec_{false};
  bool feedthrough_enable_{false};
  bool ctrl_enable_{true};
  int ctrl_mode_;
  bool landing_commanded_{false};
  bool sim_enable_;
  bool velocity_yaw_;
  double kp_rot_, kd_rot_;
  double reference_request_dt_;
  double norm_thrust_const_, norm_thrust_offset_;
  double max_fb_acc_;

  int nav_state_;
  int arming_state_;
  std::vector<geometry_msgs::msg::PoseStamped> posehistory_vector_;
  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;

  double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
  Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_;
  Eigen::Vector3d mavPos_, mavVel_, mavRate_;
  double mavYaw_;
  Eigen::Vector3d gravity_{Eigen::Vector3d(0.0, 0.0, -9.8)};
  Eigen::Vector4d mavAtt_, q_des;
  Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}
  Eigen::Vector3d Kpos_, Kvel_, D_;
  double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_;
  int posehistory_window_;
  int offb_counter_ = 0;

  void arm();
  void pubVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0);
  void pubOffbControlMode();
  void pubMotorCommands();
  void pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);
  void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);
  void pubPoseHistory();
  void appendPoseHistory();
  void targetCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void flattargetCallback(const controller_msgs::msg::FlatTarget::SharedPtr msg);
  void yawtargetCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void multiDOFJointCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg);
  void cmdloopCallback();
  void mavstateCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void mavodomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void mavposeCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void mavattCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  bool landCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  geometry_msgs::msg::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);
  void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_acc);
  Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                  const Eigen::Vector3d &target_acc);
  Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);
  Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                Eigen::Vector4d &curr_att);

  enum FlightState { WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state;
  geometry_msgs::msg::Pose home_pose_;
  bool received_home_pose = false;
  std::shared_ptr<Control> controller_;

 public:
  geometricCtrl();
  virtual ~geometricCtrl();
  void getStates(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel) {
    pos = mavPos_;
    att = mavAtt_;
    vel = mavVel_;
    angvel = mavRate_;
  };
  void getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel) {
    pos = mavPos_ - targetPos_;
    vel = mavVel_ - targetVel_;
  };
  void setBodyRateCommand(Eigen::Vector4d bodyrate_command) { cmdBodyRate_ = bodyrate_command; };
  void setFeedthrough(bool feed_through) { feedthrough_enable_ = feed_through; };
  void setDesiredAcceleration(Eigen::Vector3d &acceleration) { targetAcc_ = acceleration; };
  static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
  static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };
};

#endif
