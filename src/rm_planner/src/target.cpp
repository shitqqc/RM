#include "rm_planner/target.hpp"

namespace rm_auto_aim
{

std::vector<Eigen::Vector3d> Target::getArmorPositions()const noexcept{
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());

  // Calculate the position of each armor
  bool is_current_pair = true;
  double r_ = 0, target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    double temp_yaw = yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) {
      r_ = is_current_pair ? r : r + dr;
      target_dz = (is_current_pair ? 0 : dz);
      is_current_pair = !is_current_pair;
    } else {
      r_ = r;
      target_dz = dz;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r_ * cos(temp_yaw), -r_ * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

int Target::selectBestArmor() const noexcept {
  // Angle between the car's center and the X-axis
  double alpha = std::atan2(yc, xc);
  // Angle between the front of observed armor and the X-axis
  double beta = yaw;

  // clang-format off
  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  R_odom2center << std::cos(alpha), std::sin(alpha), 
                  -std::sin(alpha), std::cos(alpha);
  R_odom2armor << std::cos(beta), std::sin(beta), 
                 -std::sin(beta), std::cos(beta);
  // clang-format on
  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  // Equal to (alpha - beta) in most cases
  double decision_angle = -std::asin(R_center2armor(0, 1));

  // Angle thresh of the armor jump
  double side_angle_ = 20;
  double theta = (vyaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

  // Avoid the frequent switch between two armor
  double min_switching_v_yaw_ = 1.0;
  if (std::abs(vyaw) < min_switching_v_yaw_) {
    theta = 0;
  }

  double temp_angle = decision_angle + M_PI / armors_num - theta;

  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
  return selected_id;
}

void Target::predict(double dt)
{
    xc += vx*dt;
    yc += vy*dt;
    zc += vz*dt;
    yaw += vyaw*dt;
    yaw = rm_tools::limit_rad(yaw);
    //update center position
    target_center << xc, yc, zc;
}

}
