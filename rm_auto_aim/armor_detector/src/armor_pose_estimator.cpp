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

#include "armor_detector/armor_pose_estimator.hpp"
#include <rclcpp/logger.hpp>
#include "armor_detector/armor.hpp"
#include "rm_tools/math.hpp"

#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace rm_auto_aim {
ArmorPoseEstimator::ArmorPoseEstimator(
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
  // Setup pnp solver
  pnp_solver_ = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
  pnp_solver_->setObjectPoints(
      "small", Armor::buildObjectPoints<cv::Point3f>(SMALL_ARMOR_WIDTH,
                                                     SMALL_ARMOR_HEIGHT));
  pnp_solver_->setObjectPoints(
      "large", Armor::buildObjectPoints<cv::Point3f>(LARGE_ARMOR_WIDTH,
                                                     LARGE_ARMOR_HEIGHT));

  // BA solver
  ba_solver_ = std::make_unique<BaSolver>(camera_info->k, camera_info->d);

  //相机坐标系转换方向
  //TODO 标定这个参数
  R_gimbal_camera_ = Eigen::Matrix3d::Identity();
  R_gimbal_camera_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;
}

std::vector<auto_aim_interfaces::msg::Armor>
ArmorPoseEstimator::extractArmorPoses(std::vector<Armor> &armors, Eigen::Matrix3d R_imu_camera) {

  std::vector<auto_aim_interfaces::msg::Armor> armors_msg;
  for (auto &armor : armors) {

    std::vector<cv::Mat> rvecs, tvecs;

    // Use PnP to get the initial pose information
    if (pnp_solver_->solvePnPGeneric(
            armor.points, rvecs, tvecs,
            (armor.type == ArmorType::small ? "small" : "large"))) {
      sortPnPResult(armor, rvecs, tvecs);
      cv::Mat rmat;
      cv::Rodrigues(rvecs[0], rmat);
      Eigen::Matrix3d R = rm_tools::cvToEigen(rmat);
      Eigen::Vector3d t = rm_tools::cvToEigen(tvecs[0]);

      double armor_roll = rotationMatrixToRPY(R_gimbal_camera_ * R)[0] * 180 / M_PI;
      double armor_yaw = rotationMatrixToRPY(R_gimbal_camera_ * R)[2] * 180 / M_PI;
      double armor_pitch = rotationMatrixToRPY(R_gimbal_camera_ * R)[1] * 180 / M_PI;

      Eigen::Vector3d ypr_in_world(armor_yaw, armor_pitch, armor_roll);

      Eigen::Vector3d xyz_in_camera;
      cv::cv2eigen(tvecs[0], xyz_in_camera);
      armor.ypr_in_world = ypr_in_world;
      armor.yaw_raw = armor_yaw* M_PI / 180;//maybe have problem
      //armor.ypd_in_world = rm_tools::xyz2ypd(armor.xyz_in_world);

      //Eigen::Vector3d gimbal_ypr = rm_tools::eulers(R_imu_camera, 2, 1, 0);

      if (use_ba_ && armor_roll < 15) {
        R = ba_solver_->solveBa(armor, t, R, R_imu_camera);
      }
     
      Eigen::Quaterniond q(R);
      Eigen::Vector3d eulerAngles = q.toRotationMatrix().eulerAngles(0, 1, 2);
      double yaw = eulerAngles[2];  // XYZ 顺序，yaw 在最后
      // Fill the armor message
      auto_aim_interfaces::msg::Armor armor_msg;

      // Fill basic info
      armor_msg.type = ARMOR_TYPES[static_cast<int>(armor.type)];
      armor_msg.number = ARMOR_NAMES[static_cast<int>(armor.name)];
      armor_msg.color = COLORS[static_cast<int>(armor.color)];
      armor_msg.prob = armor.confidence;
      armor_msg.yaw_raw = armor.yaw_raw;
      //double yaw = armor.ypr_in_world[0];
      armor_msg.yaw = yaw;
      // auto dyaw = yaw - armor.yaw_raw;
      // armor_msg.dyaw = (yaw - armor.yaw_raw);

      // Fill pose
      armor_msg.pose.position.x = t(0);
      armor_msg.pose.position.y = t(1);
      armor_msg.pose.position.z = t(2);
      armor_msg.pose.orientation.x = q.x();
      armor_msg.pose.orientation.y = q.y();
      armor_msg.pose.orientation.z = q.z();
      armor_msg.pose.orientation.w = q.w();
      
      // Fill the distance to image center
      armor_msg.distance_to_image_center =
          pnp_solver_->calculateDistanceToCenter(armor.center);

                      // Fill keypoints
      armor_msg.kpts.clear();
        for (const auto & pt :armor.points) {
          geometry_msgs::msg::Point point;
          point.x = pt.x;
          point.y = pt.y;
          armor_msg.kpts.emplace_back(point);
          }  

      armors_msg.emplace_back(std::move(armor_msg));
    } else {
      RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "PnP Failed!");
    }
  }

  return armors_msg;
}

Eigen::Vector3d ArmorPoseEstimator::rotationMatrixToRPY(const Eigen::Matrix3d &R) {
  // Transform to camera frame
  Eigen::Quaterniond q(R);
  // Get armor yaw
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  Eigen::Vector3d rpy;
  tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
  return rpy;
}

void ArmorPoseEstimator::sortPnPResult(const Armor &armor,
                                    std::vector<cv::Mat> &rvecs,
                                    std::vector<cv::Mat> &tvecs) const {
  constexpr float PROJECT_ERR_THRES = 3.0;

  // 获取这两个解
  cv::Mat &rvec1 = rvecs.at(0);
  cv::Mat &tvec1 = tvecs.at(0);
  cv::Mat &rvec2 = rvecs.at(1);
  cv::Mat &tvec2 = tvecs.at(1);

  // 将旋转向量转换为旋转矩阵
  cv::Mat R1_cv, R2_cv;
  cv::Rodrigues(rvec1, R1_cv);
  cv::Rodrigues(rvec2, R2_cv);

  // 转换为Eigen矩阵
  Eigen::Matrix3d R1 = rm_tools::cvToEigen(R1_cv);
  Eigen::Matrix3d R2 = rm_tools::cvToEigen(R2_cv);

  // 计算云台系下装甲板的RPY角
  auto rpy1 = rotationMatrixToRPY(R_gimbal_camera_ * R1);
  auto rpy2 = rotationMatrixToRPY(R_gimbal_camera_ * R2);

  std::string coord_frame_name =
      (armor.type == ArmorType::small ? "small" : "large");
  double error1 = pnp_solver_->calculateReprojectionError(
      armor.points, rvec1, tvec1, coord_frame_name);
  double error2 = pnp_solver_->calculateReprojectionError(
      armor.points, rvec2, tvec2, coord_frame_name);
  // 两个解的重投影误差差距较大或者roll角度较大时，不做选择
  if ((error2 / error1 > PROJECT_ERR_THRES) || (rpy1[0] > 10 * 180 / M_PI) ||
      (rpy2[0] > 10 * 180 / M_PI)) {
      //RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "PnP derror too large or roll is too large!");
    return;
  }

  // 计算灯条在图像中的倾斜角度
  double l_angle =
      std::atan2((armor.points[3] - armor.points[0]).y, (armor.points[3] - armor.points[0]).x) * 180 / M_PI;
  double r_angle =
      std::atan2((armor.points[2] - armor.points[1]).y, (armor.points[2] - armor.points[1]).x) * 180 /
      M_PI;
  double angle = (l_angle + r_angle) / 2;
  angle += 90.0;

  if (armor.name == ArmorName::outpost) angle = -angle;

  // 根据倾斜角度选择解
  // 如果装甲板左倾（angle > 0），选择Yaw为负的解
  // 如果装甲板右倾（angle < 0），选择Yaw为正的解
  if ((angle > 0 && rpy1[2] > 0 && rpy2[2] < 0) ||
      (angle < 0 && rpy1[2] < 0 && rpy2[2] > 0)) {
    std::swap(rvec1, rvec2);
    std::swap(tvec1, tvec2);
    RCLCPP_DEBUG(rclcpp::get_logger("armor_detector"), "PnP Solution 2 Selected");
  }
}

// Eigen::Matrix3d ArmorPoseEstimator::optimize_yaw(Armor & armor, Eigen::Matrix3d R_imu_camera)const {

//   Eigen::Vector3d gimbal_ypr = rm_tools::eulers(R_imu_camera, 2, 1, 0);
//   Sophus::SO3d R_camera_imu = Sophus::SO3d(R_imu_camera.transpose());

//   std::string coord_frame_name =
//     (armor.type == ArmorType::small ? "small" : "large");

//   constexpr double SEARCH_RANGE = 140;  // degree
//   auto yaw0 = rm_tools::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);
//   auto pitch = (armor.name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
//     Sophus::SO3d R_pitch = Sophus::SO3d::exp(Eigen::Vector3d(0, pitch, 0));
//   auto min_error = 1e10;
//   auto best_yaw = armor.ypr_in_world[0];

//   //Eigen::Matrix3d best_R;

//   for (int i = 0; i < SEARCH_RANGE; i++) {
//     double yaw = rm_tools::limit_rad(yaw0 + i * CV_PI / 180.0);
//       auto sin_yaw = std::sin(yaw);
//       auto cos_yaw = std::cos(yaw);

//       auto sin_pitch = std::sin(pitch);
//       auto cos_pitch = std::cos(pitch);

//       // clang-format off
//       const Eigen::Matrix3d R_armor2world {
//         {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
//         {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
//         {         -sin_pitch,        0,           cos_pitch}
//       };
//       // clang-format on

//       // get R_armor2camera t_armor2camera
//       const Eigen::Vector3d & t_armor2world = armor.xyz_in_world;
//       Eigen::Matrix3d R_armor2camera =
//         R_gimbal_camera_.transpose() * R_imu_camera.transpose() * R_armor2world;
//       Eigen::Vector3d t_armor2camera =
//         R_gimbal_camera_.transpose() * (R_imu_camera.transpose() * t_armor2world);

//       // get rvec tvec
//       cv::Vec3d rvec;
//       cv::Mat R_armor2camera_cv;
//       cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
//       cv::Rodrigues(R_armor2camera_cv, rvec);
//       cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);
//       //cv::Mat rvec_mat = cv::Mat(rvec);
//       //cv::Mat tvec_mat = cv::Mat(tvec);
//         auto error = pnp_solver_->calculateReprojectionError(armor.points, rvec, tvec, coord_frame_name);
//         //std::cout<<"error"<<i<<": "<<error<<std::endl;
//         if (error < min_error) {
//           min_error = error;
//           best_yaw = yaw;
//           //bset_R = R_armor2camera;
//         }
//       }

//   armor.yaw_raw = armor.ypr_in_world[0];
//   armor.ypr_in_world[0] = best_yaw;
//   Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, best_yaw));
//   //Eigen::Quaterniond q_total = q_yaw * q_pitch;
//   return (R_camera_imu * R_yaw * R_pitch).matrix();
// }
} // namespace fyt::auto_aim
