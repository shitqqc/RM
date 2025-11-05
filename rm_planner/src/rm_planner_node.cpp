// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_planner/rm_planner_node.hpp"

// std
#include <memory>
#include <vector>

namespace rm_auto_aim {
RMPlannerNode::RMPlannerNode(const rclcpp::NodeOptions &options)
: Node("rm_planner", options), planner_(nullptr) {

  RCLCPP_INFO(this->get_logger(), "Starting RMPlannerNode!");

  debug_mode_ = this->declare_parameter("debug", true);
  // enable_ = this->declare_parameter("planner_enable",true);


  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  target_sub_.subscribe(this, "tracker/target", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(target_sub_,
                                             *tf2_buffer_,
                                             target_frame_,
                                             10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when
  // transforms are available
  tf2_filter_->registerCallback(&RMPlannerNode::targetCallback, this);

  gimbal_pub_ = this->create_publisher<auto_aim_interfaces::msg::GimbalCmd>("armor_planner/cmd_gimbal",
                                                                      rclcpp::SensorDataQoS());
  // Timer 250 Hz
  pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                       std::bind(&RMPlannerNode::timerCallback, this));
  armor_target_.header.frame_id = "";

  if (debug_mode_) {
    initMarkers();
  }

  // Heartbeat
  // heartbeat_ = HeartBeatPublisher::create(this);
}

void RMPlannerNode::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr target_msg)
{
    // Lazy initialize planner owing to weak_from_this() can't be called in constructor
  if (planner_ == nullptr) {
    planner_ = std::make_unique<Planner>();
    init_planner();
  }

  armor_target_.tracking = false;
  armor_target_ = *target_msg;
  
}
void RMPlannerNode::timerCallback() {
  if (planner_ == nullptr) {
    return;
  }

  // if (!enable_) {
  //   return;
  // }

  // Init message
  auto_aim_interfaces::msg::GimbalCmd control_msg;

  // If target never detected
  if (armor_target_.header.frame_id.empty()) {
    control_msg.fire = false;
    control_msg.target_yaw = 0;
    control_msg.target_pitch = 0;
    control_msg.yaw = 0;
    control_msg.yaw_vel = 0;
    control_msg.yaw_acc = 0;
    control_msg.pitch = 0;
    control_msg.pitch_vel = 0;
    control_msg.pitch_acc = 0;    
    gimbal_pub_->publish(control_msg);
    return;
  }

  if (armor_target_.tracking) {
    try {
          planner_->prediction_delay_ = get_parameter("planner.prediction_delay").as_double();
      control_msg = planner_->plan(armor_target_, 20, this->now(), tf2_buffer_);
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Something went wrong in planner!");
      control_msg.fire = false;
    }
  } else {
    control_msg.fire = false;
  }
  gimbal_pub_->publish(control_msg);

  if (debug_mode_) {
    publishMarkers(armor_target_, control_msg);
  }

}

void RMPlannerNode::initMarkers() noexcept {
  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armors_marker_.ns = "filtered_armors";
  armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armors_marker_.scale.x = 0.03;
  armors_marker_.scale.z = 0.125;
  armors_marker_.color.a = 1.0;
  armors_marker_.color.b = 1.0;
  // selection_marker_.ns = "selection";
  // selection_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  // selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.1;
  // selection_marker_.color.a = 1.0;
  // selection_marker_.color.g = 1.0;
  // selection_marker_.color.r = 1.0;
  trajectory_marker_.ns = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::POINTS;
  trajectory_marker_.scale.x = 0.01;
  trajectory_marker_.scale.y = 0.01;
  trajectory_marker_.color.a = 1.0;
  trajectory_marker_.color.r = 1.0;
  trajectory_marker_.color.g = 0.75;
  trajectory_marker_.color.b = 0.79;
  trajectory_marker_.points.clear();

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_planner/marker", 10);
}

void RMPlannerNode::publishMarkers(const auto_aim_interfaces::msg::Target &target_msg,
                                     const auto_aim_interfaces::msg::GimbalCmd &gimbal_cmd) noexcept {
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armors_marker_.header = target_msg.header;
  selection_marker_.header = target_msg.header;
  trajectory_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;

  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r = target_msg.radius, dr = target_msg.d_radius;
    double xc = target_msg.position.x, yc = target_msg.position.y, zc = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = zc;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armors_marker_.action = visualization_msgs::msg::Marker::ADD;
    armors_marker_.scale.y = target_msg.id == "1" ? 0.23 : 0.135;
    // Draw armors
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r_ = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r_ = is_current_pair ? r : r + dr;
        p_a.z = zc + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r_ = r;
        p_a.z = zc;
      }
      p_a.x = xc - r_ * cos(tmp_yaw);
      p_a.y = yc - r_ * sin(tmp_yaw);

      armors_marker_.id = i;
      armors_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.2618 : 0.2618, tmp_yaw);
      armors_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armors_marker_);
    }

    // selection_marker_.action = visualization_msgs::msg::Marker::ADD;
    // selection_marker_.points.clear();
    // selection_marker_.pose.position.y = gimbal_cmd.distance * sin(gimbal_cmd.yaw * M_PI / 180);
    // selection_marker_.pose.position.x = gimbal_cmd.distance * cos(gimbal_cmd.yaw * M_PI / 180);
    // selection_marker_.pose.position.z = gimbal_cmd.distance * sin(gimbal_cmd.pitch * M_PI / 180);

    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();
    trajectory_marker_.header.frame_id = "gimbal_link";
    for (const auto &point : planner_->getTrajectory()) {
      geometry_msgs::msg::Point p;
      p.x = point.first;
      p.z = point.second;
      trajectory_marker_.points.emplace_back(p);
    }
    if (gimbal_cmd.fire) {
      trajectory_marker_.color.r = 0;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 0;
    } else {
      trajectory_marker_.color.r = 1;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 1;
    }

  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    armors_marker_.action = visualization_msgs::msg::Marker::DELETE;
    trajectory_marker_.action = visualization_msgs::msg::Marker::DELETE;
    // selection_marker_.action = visualization_msgs::msg::Marker::DELETE;
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(trajectory_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_array.markers.emplace_back(armors_marker_);
  // marker_array.markers.emplace_back(selection_marker_);
  marker_pub_->publish(marker_array);
}

void RMPlannerNode::init_planner()
{

  planner_->prediction_delay_ = this->declare_parameter("planner.prediction_delay", 0.0);
  auto compenstator_type = this->declare_parameter("planner.compensator_type", "ideal");
  planner_->trajectory_compensator_ = rm_tools::CompensatorFactory::createCompensator(compenstator_type);
  planner_->trajectory_compensator_->iteration_times = this->declare_parameter("planner.iteration_times", 20);
  planner_->trajectory_compensator_->velocity = this->declare_parameter("planner.bullet_speed", 20.0);
  planner_->trajectory_compensator_->gravity = this->declare_parameter("planner.gravity", 9.8);
  planner_->trajectory_compensator_->resistance = this->declare_parameter("planner.resistance", 0.001);

  planner_->max_yaw_acc = this->declare_parameter("gimbal.max_yaw_acc", 50);
  planner_->max_pitch_acc = this->declare_parameter("gimbal.max_pitch_acc", 100);

  planner_->fire_thresh_ = this->declare_parameter("fire_thresh", 0.003);

  planner_->Q_yaw = this->declare_parameter("gimbal.Q_yaw", std::vector<double>{9e6, 0});
  planner_->R_yaw = this->declare_parameter("gimbal.R_yaw", std::vector<double>{1});

  planner_->Q_pitch = this->declare_parameter("gimbal.Q_pitch", std::vector<double>{9e6, 0});
  planner_->R_pitch = this->declare_parameter("gimbal.R_pitch", std::vector<double>{1});

  planner_->manual_compensator_ = std::make_unique<rm_tools::ManualCompensator>();
  auto angle_offset = this->declare_parameter("planner.angle_offset", std::vector<std::string>{});
  if(!planner_->manual_compensator_->updateMapFlow(angle_offset)) {
    RCLCPP_WARN(this->get_logger(), "Manual compensator update failed!");
  }
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RMPlannerNode)
