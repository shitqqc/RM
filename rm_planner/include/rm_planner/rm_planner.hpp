#ifndef RM_PLANNER_RM_PLANNER_HPP_
#define RM_PLANNER_RM_PLANNER_HPP_

#include <Eigen/Dense>
#include <list>
#include <optional>

#include "tinympc/tiny_api.hpp"
#include "rm_planner/target.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/gimbal_cmd.hpp"
#include "rm_tools/trajectory_compensator.hpp"
#include "rm_tools/manual_compensator.hpp"
//#include "compensator_factory.hpp" 

#include <tf2_ros/buffer.h>
#include <angles/angles.h>
#include <rclcpp/time.hpp>
#include "rclcpp/rclcpp.hpp" 
namespace rm_auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  Planner();
  void init_planner();
  auto_aim_interfaces::msg::GimbalCmd plan(const auto_aim_interfaces::msg::Target &target, double bullet_speed, const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);
 // Plan plan(std::optional<Target> target, double bullet_speed);
  std::vector<std::pair<double, double>> getTrajectory() const noexcept;
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double prediction_delay_;
  std::vector<double> Q_yaw, Q_pitch;
  std::vector<double> R_yaw, R_pitch;
  double max_yaw_acc, max_pitch_acc;

  std::unique_ptr<rm_tools::TrajectoryCompensator> trajectory_compensator_;
  std::unique_ptr<rm_tools::ManualCompensator> manual_compensator_;

  Target tracking_target;

private:

  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

  std::array<double, 3> rpy_;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;



  void setup_yaw_solver();
  void setup_pitch_solver();

  void calcYawAndPitch(const Eigen::Vector3d &p,
                        const std::array<double, 3> rpy,
                        double &yaw,
                        double &pitch) const noexcept;

  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);

};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP