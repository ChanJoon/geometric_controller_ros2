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
 * @brief Trajectory Publisher
 *
 * @author
 * - Jaeyoung Lim <jalim@ethz.ch>
 * - Chanjoon Park <chanjoon.park@kaist.ac.kr>
 */

#ifndef TRAJECTORYPUBLISHER_H
#define TRAJECTORYPUBLISHER_H

#include <stdio.h>
#include <Eigen/Dense>
#include <cstdlib>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "controller_msgs/msg/flat_target.hpp"
#include "trajectory_publisher/polynomialtrajectory.h"
#include "trajectory_publisher/shapetrajectory.h"
#include "trajectory_publisher/trajectory.h"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <px4_ros_com/frame_transforms.h>

#define REF_TWIST 8
#define REF_HOVER 16

using namespace std;
using namespace Eigen;
class trajectoryPublisher : public rclcpp::Node{
 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectoryPub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr referencePub_;
  rclcpp::Publisher<controller_msgs::msg::FlatTarget>::SharedPtr flatreferencePub_;
  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr> primitivePub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr motionselectorSub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr mavposeSub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr mavstate_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr trajtriggerServ_;
  rclcpp::TimerBase::SharedPtr trajloop_timer_;
  rclcpp::TimerBase::SharedPtr refloop_timer_;
  rclcpp::Time start_time_, curr_time_;
  rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                                    .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                                    .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
                                    .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);                              

  nav_msgs::msg::Path refTrajectory_;
  nav_msgs::msg::Path primTrajectory_;
  px4_msgs::msg::VehicleStatus current_state_;

  int trajectory_type_;
  Eigen::Vector3d p_targ, v_targ, a_targ;
  Eigen::Vector3d p_mav_, v_mav_;
  Eigen::Vector3d shape_origin_, shape_axis_;
  double shape_omega_ = 0;
  double theta_ = 0.0;
  double controlUpdate_dt_;
  double primitive_duration_;
  double trigger_time_;
  double init_pos_x_, init_pos_y_, init_pos_z_;
  double max_jerk_;
  int pubreference_type_;
  int num_primitives_;
  int motion_selector_;

  std::vector<std::shared_ptr<trajectory>> motionPrimitives_;
  std::vector<Eigen::Vector3d> inputs_;

 public:
  trajectoryPublisher();
  void updateReference();
  void pubrefTrajectory(int selector);
  void pubprimitiveTrajectory();
  void pubrefState();
  void pubrefhoverState();
  void pubflatrefState();
  void pubrefSetpointRaw();
  void pubrefSetpointRawGlobal();
  void initializePrimitives(int type);
  void updatePrimitives();
  void loopCallback();
  void refCallback();
  bool triggerCallback(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res);
  void motionselectorCallback(const std_msgs::msg::Int32::SharedPtr selector);
  void mavposeCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
  void mavstateCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
};

#endif
