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

#include "trajectory_publisher/trajectoryPublisher.h"

using namespace std;
using namespace Eigen;
trajectoryPublisher::trajectoryPublisher()
    : Node("trajectory_publisher"), motion_selector_(0), start_time_(this->get_clock()->now()) {
  trajectoryPub_ = this->create_publisher<nav_msgs::msg::Path>("trajectory_publisher/trajectory", 10);
  referencePub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("reference/setpoint", 10);
  flatreferencePub_ = this->create_publisher<controller_msgs::msg::FlatTarget>("reference/flatsetpoint", 10);
  motionselectorSub_ = this->create_subscription<std_msgs::msg::Int32>("trajectory_publisher/motionselector", 10, std::bind(&trajectoryPublisher::motionselectorCallback, this, std::placeholders::_1));
  mavposeSub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos_profile, std::bind(&trajectoryPublisher::mavposeCallback, this, std::placeholders::_1));
  mavstate_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", qos_profile, std::bind(&trajectoryPublisher::mavstateCallback, this, std::placeholders::_1));

  trajloop_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&trajectoryPublisher::loopCallback, this));
  refloop_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&trajectoryPublisher::refCallback, this));

  trajtriggerServ_ = this->create_service<std_srvs::srv::SetBool>("start", std::bind(&trajectoryPublisher::triggerCallback, this, std::placeholders::_1, std::placeholders::_2));

  this->declare_parameter<double>("initpos_x", 0.0);
  this->declare_parameter<double>("initpos_y", 0.0);
  this->declare_parameter<double>("initpos_z", 1.0);
  this->declare_parameter<double>("updaterate", 0.01);
  this->declare_parameter<double>("horizon", 1.0);
  this->declare_parameter<double>("maxjerk", 10.0);
  this->declare_parameter<double>("shape_omega", 1.5);
  this->declare_parameter<int>("trajectory_type", 0);
  this->declare_parameter<int>("number_of_primitives", 7);
  this->declare_parameter<int>("reference_type", 2);

  this->get_parameter("initpos_x", init_pos_x_);
  this->get_parameter("initpos_y", init_pos_y_);
  this->get_parameter("initpos_z", init_pos_z_);
  this->get_parameter("updaterate", controlUpdate_dt_);
  this->get_parameter("horizon", primitive_duration_);
  this->get_parameter("maxjerk", max_jerk_);
  this->get_parameter("shape_omega", shape_omega_);
  this->get_parameter("trajectory_type", trajectory_type_);
  this->get_parameter("number_of_primitives", num_primitives_);
  this->get_parameter("reference_type", pubreference_type_);

  inputs_.resize(num_primitives_);

  if (trajectory_type_ == 0) {  // Polynomial Trajectory

    if (num_primitives_ == 7) {
      inputs_.at(0) << 0.0, 0.0, 0.0;  // Constant jerk inputs for minimim time trajectories
      inputs_.at(1) << 1.0, 0.0, 0.0;
      inputs_.at(2) << -1.0, 0.0, 0.0;
      inputs_.at(3) << 0.0, 1.0, 0.0;
      inputs_.at(4) << 0.0, -1.0, 0.0;
      inputs_.at(5) << 0.0, 0.0, 1.0;
      inputs_.at(6) << 0.0, 0.0, -1.0;
    }

    for (int i = 0; i < num_primitives_; i++) {
      motionPrimitives_.emplace_back(std::make_shared<polynomialtrajectory>());
      primitivePub_.push_back(
          this->create_publisher<nav_msgs::msg::Path>("trajectory_publisher/primitiveset" + std::to_string(i), 1));
      inputs_.at(i) = inputs_.at(i) * max_jerk_;
    }
  } else {  // Shape trajectories

    num_primitives_ = 1;
    motionPrimitives_.emplace_back(std::make_shared<shapetrajectory>(trajectory_type_));
    primitivePub_.push_back(this->create_publisher<nav_msgs::msg::Path>("trajectory_publisher/primitiveset", 1));
  }

  p_targ << init_pos_x_, init_pos_y_, init_pos_z_;
  v_targ << 0.0, 0.0, 0.0;
  shape_origin_ << init_pos_x_, init_pos_y_, init_pos_z_;
  shape_axis_ << 0.0, 0.0, 1.0;
  motion_selector_ = 0;

  initializePrimitives(trajectory_type_);
}

void trajectoryPublisher::updateReference() {
  curr_time_ = this->get_clock()->now();
  if (current_state_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {  /// Reset start_time_ when not in offboard
    start_time_ = this->get_clock()->now();
  }
  trigger_time_ = (curr_time_ - start_time_).seconds();

  p_targ = motionPrimitives_.at(motion_selector_)->getPosition(trigger_time_);
  v_targ = motionPrimitives_.at(motion_selector_)->getVelocity(trigger_time_);
  if (pubreference_type_ != 0) a_targ = motionPrimitives_.at(motion_selector_)->getAcceleration(trigger_time_);
}

void trajectoryPublisher::initializePrimitives(int type) {
  if (type == 0) {
    for (int i = 0; i < motionPrimitives_.size(); i++)
      motionPrimitives_.at(i)->generatePrimitives(p_mav_, v_mav_, inputs_.at(i));
  } else {
    for (int i = 0; i < motionPrimitives_.size(); i++)
      motionPrimitives_.at(i)->initPrimitives(shape_origin_, shape_axis_, shape_omega_);
    // TODO: Pass in parameters for primitive trajectories
  }
}

void trajectoryPublisher::updatePrimitives() {
  for (int i = 0; i < motionPrimitives_.size(); i++) motionPrimitives_.at(i)->generatePrimitives(p_mav_, v_mav_);
}

void trajectoryPublisher::pubrefTrajectory(int selector) {
  // Publish current trajectory the publisher is publishing
  refTrajectory_ = motionPrimitives_.at(selector)->getSegment(this->get_clock());
  refTrajectory_.header.stamp = this->get_clock()->now();
  refTrajectory_.header.frame_id = "map";
  trajectoryPub_->publish(refTrajectory_);
}

void trajectoryPublisher::pubprimitiveTrajectory() {
  for (int i = 0; i < motionPrimitives_.size(); i++) {
    primTrajectory_ = motionPrimitives_.at(i)->getSegment(this->get_clock());
    primTrajectory_.header.stamp = this->get_clock()->now();
    primTrajectory_.header.frame_id = "map";
    primitivePub_.at(i)->publish(primTrajectory_);
  }
}

void trajectoryPublisher::pubrefState() {
  geometry_msgs::msg::TwistStamped msg;

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";
  msg.twist.angular.x = p_targ(0);
  msg.twist.angular.y = p_targ(1);
  msg.twist.angular.z = p_targ(2);
  msg.twist.linear.x = v_targ(0);
  msg.twist.linear.y = v_targ(1);
  msg.twist.linear.z = v_targ(2);
  referencePub_->publish(msg);
}

void trajectoryPublisher::pubrefhoverState() {
  controller_msgs::msg::FlatTarget msg;

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";
  msg.type_mask = 4;
  msg.position.x = 0.0;
  msg.position.y = 0.0;
  msg.position.z = 2.0;
  msg.velocity.x = 0.0;
  msg.velocity.y = 0.0;
  msg.velocity.z = 0.0;
  msg.acceleration.x = 0.0;
  msg.acceleration.y = 0.0;
  msg.acceleration.z = 0.0;
  flatreferencePub_->publish(msg);
}

void trajectoryPublisher::pubflatrefState() {
  controller_msgs::msg::FlatTarget msg;

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "map";
  msg.type_mask = pubreference_type_;
  msg.position.x = p_targ(0);
  msg.position.y = p_targ(1);
  msg.position.z = p_targ(2);
  msg.velocity.x = v_targ(0);
  msg.velocity.y = v_targ(1);
  msg.velocity.z = v_targ(2);
  msg.acceleration.x = a_targ(0);
  msg.acceleration.y = a_targ(1);
  msg.acceleration.z = a_targ(2);
  flatreferencePub_->publish(msg);
}

void trajectoryPublisher::mavstateCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  current_state_ = *msg;
  // RCLCPP_INFO(this->get_logger(), "NavState: %d", current_state_.nav_state);
  // RCLCPP_INFO(this->get_logger(), "    - offboard status: %s", (current_state_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) ? "true" : "false");
}


void trajectoryPublisher::loopCallback() {
  // Slow Loop publishing trajectory information
  pubrefTrajectory(motion_selector_);
  pubprimitiveTrajectory();
}

void trajectoryPublisher::refCallback() {
  // Fast Loop publishing reference states
  updateReference();
  switch (pubreference_type_) {
    case REF_TWIST:
      pubrefState();
      break;
    case REF_HOVER:
      pubrefhoverState();
      break;
    default:
      pubflatrefState();
      break;
  }
}

bool trajectoryPublisher::triggerCallback(const std_srvs::srv::SetBool::Request::SharedPtr req, const std_srvs::srv::SetBool::Response::SharedPtr res) {
  unsigned char mode = req->data;

  start_time_ = this->get_clock()->now();
  res->success = true;
  res->message = "trajectory triggered";

  return true;
}

void trajectoryPublisher::motionselectorCallback(const std_msgs::msg::Int32::SharedPtr selector_msg) {
  motion_selector_ = selector_msg->data;
  updatePrimitives();
  start_time_ = this->get_clock()->now();
}

void trajectoryPublisher::mavposeCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
  p_mav_ << msg->x, msg->y, msg->z;
  v_mav_ << msg->vx, msg->vy, msg->vz;

  updatePrimitives();
}