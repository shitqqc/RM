#include "rm_planner/rm_planner.hpp"
#include "rm_planner/rm_planner_node.hpp"
#include <vector>

#include "rm_tools/math.hpp"
#include "rm_tools/trajectory_compensator.hpp"
#include "rm_tools/manual_compensator.hpp"

using namespace std::chrono_literals;

namespace rm_auto_aim
{
Planner::Planner()
{

}
void Planner::init_planner()
{
  setup_yaw_solver();
  setup_pitch_solver();
}
auto_aim_interfaces::msg::GimbalCmd Planner:: plan(const auto_aim_interfaces::msg::Target &target, double bullet_speed, const rclcpp::Time &current_time,
                                            std::shared_ptr<tf2_ros::Buffer> tf2_buffer_)
{
  auto_aim_interfaces::msg::GimbalCmd cmd;
  // 0. Check bullet speed
  if (bullet_speed < 10 || bullet_speed > 25) {
    bullet_speed = 22;
  }

  // Get current roll, yaw and pitch of gimbal
  try {
    auto gimbal_tf =
      tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
    rpy_[1] = -rpy_[1];
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(rclcpp::get_logger("armor_planner"), "%s", ex.what());
    throw ex;
  }

  // 1. Select armor and predict fly_time
  // Use flying time to approximately predict the position of target
  Eigen::Vector3d target_position(target.position.x, target.position.y, target.position.z);
  double target_yaw = target.yaw;
  double flying_time = trajectory_compensator_->getFlyingTime(target_position);
  double dt =
    (current_time - rclcpp::Time(target.header.stamp)).seconds() + flying_time + prediction_delay_;
  target_position.x() += dt * target.velocity.x;
  target_position.y() += dt * target.velocity.y;
  target_position.z() += dt * target.velocity.z;
  target_yaw += dt * target.v_yaw;
    
  // Compensate angle by angle_offset_map
  auto angle_offset = manual_compensator_->angleHardCorrect(target_position.head(2).norm(), target_position.z());
  pitch_offset_ = angle_offset[0] * M_PI / 180;
  yaw_offset_ = angle_offset[1] * M_PI / 180;

  //Update tracking_target_info
  tracking_target.xc = target_position.x();
  tracking_target.yc = target_position.y();
  tracking_target.zc = target_position.z();
  tracking_target.target_center << tracking_target.xc, tracking_target.yc, tracking_target.zc;
  tracking_target.vx = target.velocity.x;
  tracking_target.vy = target.velocity.y;
  tracking_target.vz = target.velocity.z;
  tracking_target.yaw = target_yaw;
  tracking_target.vyaw = target.v_yaw;
  tracking_target.r = target.radius;
  tracking_target.dr = target.d_radius;
  tracking_target.dz = target.dz;
  tracking_target.armors_num = target.armors_num;

  // Choose the best armor to shoot
  std::vector<Eigen::Vector3d> armor_positions = tracking_target.getArmorPositions();
  int idx = tracking_target.selectBestArmor();
  auto chosen_armor_position = armor_positions.at(idx);
      if (chosen_armor_position.norm() < 0.1) {
    throw std::runtime_error("No valid armor to shoot");
  }

  // 2. Get trajectory
  double yaw0;
  Trajectory traj;
  try {
    yaw0 = std::atan2(chosen_armor_position[1], chosen_armor_position[0]) + yaw_offset_;
    traj = get_trajectory(tracking_target, yaw0, bullet_speed);
  } catch (const std::exception & e) {
    RCLCPP_INFO(rclcpp::get_logger("rm_planner"), "Target solve fail!");
    cmd.fire = false;
    return cmd;
  }

  // 3. Solve yaw
  Eigen::VectorXd x0(2);
  x0 << traj(0, 0), traj(1, 0);//yaw yaw_vel
  tiny_set_x0(yaw_solver_, x0);

  yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);//提取参考值
  tiny_solve(yaw_solver_);

  // 4. Solve pitch
  x0 << traj(2, 0), traj(3, 0);
  tiny_set_x0(pitch_solver_, x0);

  pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
  tiny_solve(pitch_solver_);

  Plan plan;
  plan.control = true;

  plan.target_yaw = rm_tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
  plan.target_pitch = traj(2, HALF_HORIZON);

  plan.yaw = rm_tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
  plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
  plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

  plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
  plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
  plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

  auto shoot_offset_ = 2;
  plan.fire =
    std::hypot(   //根号
      traj(0, HALF_HORIZON + shoot_offset_) - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
      traj(2, HALF_HORIZON + shoot_offset_) -
        pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_)) < fire_thresh_;
  cmd.control = plan.control;
  cmd.fire = plan.fire;
  cmd.target_yaw = plan.target_yaw;
  cmd.target_pitch = -plan.target_pitch;
  cmd.yaw = plan.yaw;
  cmd.yaw_vel = plan.yaw_vel;
  cmd.yaw_acc = plan.yaw_acc;
  cmd.pitch = plan.pitch;
  cmd.pitch_vel = plan.pitch_vel;
  cmd.pitch_acc = plan.pitch_acc;
  return cmd;
}

// Plan Planner::plan(std::optional<Target> target, double bullet_speed)
// {
//   if (!target.has_value()) return {false};

//   double delay_time =
//     std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

//   auto future = std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

//   target->predict(future);

//   return plan(*target, bullet_speed);
// }

void Planner::setup_yaw_solver()
{
  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q;
  Q << Q_yaw[0], Q_yaw[1];
  Eigen::Matrix<double, 1, 1> R;
  R << R_yaw[0];
  tiny_setup(&yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
  tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

  yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver()
{


  Eigen::MatrixXd A{{1, DT}, {0, 1}};
  Eigen::MatrixXd B{{0}, {DT}};
  Eigen::VectorXd f{{0, 0}};
  Eigen::Matrix<double, 2, 1> Q;
  Q << Q_pitch[0], Q_pitch[1];
  Eigen::Matrix<double, 1, 1> R;
  R << R_pitch[0];
  tiny_setup(&pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1, HORIZON, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
  tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

  pitch_solver_->settings->max_iter = 10;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target & target, double bullet_speed)
{
  std::vector<Eigen::Vector3d> armor_positions = target.getArmorPositions();
  int idx = target.selectBestArmor();
  auto chosen_armor_position = armor_positions.at(idx);

  double pitch,yaw;
  calcYawAndPitch(chosen_armor_position, rpy_, yaw, pitch);

  return {rm_tools::limit_rad(yaw + yaw_offset_), pitch + pitch_offset_};
}

Trajectory Planner::get_trajectory(Target & target, double yaw0, double bullet_speed)
{
  Trajectory traj;

  target.predict(-DT * (HALF_HORIZON + 1));
  auto yaw_pitch_last = aim(target, bullet_speed);

  target.predict(DT);  // [0] = -HALF_HORIZON * DT -> [HHALF_HORIZON] = 0
  auto yaw_pitch = aim(target, bullet_speed);

  for (int i = 0; i < HORIZON; i++) {
    target.predict(DT);
    auto yaw_pitch_next = aim(target, bullet_speed);

    auto yaw_vel = rm_tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
    auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

    traj.col(i) << rm_tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch = yaw_pitch_next;
  }

  return traj;
}

void Planner::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}

std::vector<std::pair<double, double>> Planner::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  // Rotate
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

}  // namespace auto_aim