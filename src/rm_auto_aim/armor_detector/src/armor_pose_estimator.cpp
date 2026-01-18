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

      // Eigen::Vector3d xyz_in_camera;
      // cv::cv2eigen(tvecs[0], xyz_in_camera);
      armor.ypr_in_world = ypr_in_world;
      armor.yaw_raw = armor_yaw* M_PI / 180;//maybe have problem
      armor.xyz_in_gimbal = R_gimbal_camera_ * t ;
      armor.xyz_in_world = R_imu_camera * armor.xyz_in_gimbal;
      //Eigen::Vector3d gimbal_rpy = rm_tools::eulers(R_imu_camera, 2, 1, 0);

      if (use_ba_ && armor_roll < 15) {
        //R = ba_solver_->solveBa(armor, t, R, R_imu_camera);
        R = optimize_yaw(armor, t, R, R_imu_camera);
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

Eigen::Matrix3d ArmorPoseEstimator::optimize_yaw(
    const Armor & armor, 
    const Eigen::Vector3d &t_camera_armor,
    const Eigen::Matrix3d &R_camera_armor, 
    Eigen::Matrix3d R_imu_camera) const {
    
  std::string coord_frame_name =
    (armor.type == ArmorType::small ? "small" : "large");

  constexpr double SEARCH_RANGE = 140;  // degree
  
  // 从当前姿态估计初始yaw
  Eigen::Matrix3d R_gimbal_armor = R_gimbal_camera_ * R_camera_armor;
  Eigen::Vector3d gimbal_rpy = rotationMatrixToRPY(R_gimbal_armor);
  
  auto yaw0 = rm_tools::limit_rad(gimbal_rpy[2] - SEARCH_RANGE / 2 * CV_PI / 180.0);
  auto pitch = (armor.name == ArmorName::outpost) ? 
               -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  
  auto min_error = 1e10;
  double best_yaw = gimbal_rpy[2];

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = rm_tools::limit_rad(yaw0 + i * CV_PI / 180.0);
    auto sin_yaw = std::sin(yaw);
    auto cos_yaw = std::cos(yaw);
    auto sin_pitch = std::sin(pitch);
    auto cos_pitch = std::cos(pitch);

    // 构建云台系下的装甲板姿态(ZYX欧拉角)
    const Eigen::Matrix3d R_gimbal_armor_new {
      {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
      {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
      {         -sin_pitch,        0,           cos_pitch}
    };

    // 转换到相机系
    Eigen::Matrix3d R_camera_armor_new = 
        R_gimbal_camera_.transpose() * R_gimbal_armor_new;

    // 位置保持PnP的结果(关键!)
    Eigen::Vector3d t_camera_armor_use = t_camera_armor;

    // 转换为OpenCV格式
    cv::Mat R_camera_armor_cv;
    cv::eigen2cv(R_camera_armor_new, R_camera_armor_cv);
    
    cv::Vec3d rvec;
    cv::Rodrigues(R_camera_armor_cv, rvec);
    cv::Vec3d tvec(t_camera_armor_use[0], 
                   t_camera_armor_use[1], 
                   t_camera_armor_use[2]);

    // 计算重投影误差
    auto error = pnp_solver_->calculateReprojectionError(
        armor.points, rvec, tvec, coord_frame_name);
    
    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }
  //std::cout<<"min_error:"<<min_error<<std::endl;
//   // 返回优化后的旋转矩阵(相机系)
  auto sin_yaw = std::sin(best_yaw);
  auto cos_yaw = std::cos(best_yaw);
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);
  
  const Eigen::Matrix3d R_gimbal_armor_best {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };

  return R_gimbal_camera_.transpose() * R_gimbal_armor_best;
}

// Eigen::Matrix3d ArmorPoseEstimator::optimize_yaw(
//     const Armor & armor, 
//     const Eigen::Vector3d &t_camera_armor,
//     const Eigen::Matrix3d &R_camera_armor, 
//     Eigen::Matrix3d R_imu_camera) const {
    
//   std::string coord_frame_name =
//     (armor.type == ArmorType::small ? "small" : "large");
  
//   // 从当前姿态估计初始yaw
//   Eigen::Matrix3d R_gimbal_armor = R_gimbal_camera_ * R_camera_armor;
//   Eigen::Vector3d gimbal_rpy = rotationMatrixToRPY(R_gimbal_armor);
  
//   double pitch = (armor.name == ArmorName::outpost) ? 
//                  -15.0 * M_PI / 180.0 : 15.0 * M_PI / 180.0;
  
//   // Lambda函数: 计算像素重投影误差
//   auto getPixelCost = [&](double yaw) -> double {
//     auto sin_yaw = std::sin(yaw);
//     auto cos_yaw = std::cos(yaw);
//     auto sin_pitch = std::sin(pitch);
//     auto cos_pitch = std::cos(pitch);
    
//     const Eigen::Matrix3d R_gimbal_armor_new {
//       {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
//       {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
//       {         -sin_pitch,        0,           cos_pitch}
//     };
    
//     Eigen::Matrix3d R_camera_armor_new = 
//         R_gimbal_camera_.transpose() * R_gimbal_armor_new;
    
//     cv::Mat R_camera_armor_cv;
//     cv::eigen2cv(R_camera_armor_new, R_camera_armor_cv);
    
//     cv::Vec3d rvec;
//     cv::Rodrigues(R_camera_armor_cv, rvec);
//     cv::Vec3d tvec(t_camera_armor[0], t_camera_armor[1], t_camera_armor[2]);
    
//     return pnp_solver_->calculateReprojectionError(
//         armor.points, rvec, tvec, coord_frame_name);
//   };
  
//   // Lambda函数: 计算角度误差
//   auto getAngleCost = [&](double yaw) -> double {
//     auto sin_yaw = std::sin(yaw);
//     auto cos_yaw = std::cos(yaw);
//     auto sin_pitch = std::sin(pitch);
//     auto cos_pitch = std::cos(pitch);
    
//     const Eigen::Matrix3d R_gimbal_armor_new {
//       {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
//       {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
//       {         -sin_pitch,        0,           cos_pitch}
//     };
    
//     Eigen::Matrix3d R_camera_armor_new = 
//         R_gimbal_camera_.transpose() * R_gimbal_armor_new;
    
//     cv::Mat R_camera_armor_cv;
//     cv::eigen2cv(R_camera_armor_new, R_camera_armor_cv);
//     cv::Vec3d rvec;
//     cv::Rodrigues(R_camera_armor_cv, rvec);
    
//     // 获取投影点
//     auto object_points = (armor.type == ArmorType::small) ? 
//         Armor::buildObjectPoints<cv::Point3f>(SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT) :
//         Armor::buildObjectPoints<cv::Point3f>(LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT);
    
//     std::vector<cv::Point2f> projected_points;
//     cv::projectPoints(object_points, rvec, 
//                      cv::Vec3d(t_camera_armor[0], t_camera_armor[1], t_camera_armor[2]),
//                      pnp_solver_->camera_matrix_, pnp_solver_->distortion_coefficients_,
//                      projected_points);
    
//     // 计算角度误差 (4条边的角度差异)
//     int map[4] = {0, 1, 3, 2};
//     double cost = 0.0;
    
//     for (int i = 0; i < 4; i++) {
//       int idx_this = map[i];
//       int idx_next = map[(i + 1) % 4];
      
//       cv::Point2f pixel_vec = armor.points[idx_next] - armor.points[idx_this];
//       cv::Point2f project_vec = projected_points[idx_next] - projected_points[idx_this];
      
//       double pixel_norm = std::sqrt(pixel_vec.x * pixel_vec.x + pixel_vec.y * pixel_vec.y);
//       double project_norm = std::sqrt(project_vec.x * project_vec.x + project_vec.y * project_vec.y);
      
//       if (pixel_norm > 1e-6 && project_norm > 1e-6) {
//         double cos_angle = (pixel_vec.x * project_vec.x + pixel_vec.y * project_vec.y) / 
//                           (pixel_norm * project_norm);
//         cos_angle = std::clamp(cos_angle, -1.0, 1.0);
//         cost += std::fabs(std::acos(cos_angle));
//       }
//     }
//     return cost;
//   };
  
//   // 三分搜索 - 像素误差
//   double left = gimbal_rpy[2] - 70.0 * M_PI / 180.0;
//   double right = gimbal_rpy[2] + 70.0 * M_PI / 180.0;
//   double epsilon = 0.03;
  
//   while (right - left > epsilon) {
//     double mid1 = left + (right - left) / 3.0;
//     double mid2 = right - (right - left) / 3.0;
    
//     if (getPixelCost(mid1) < getPixelCost(mid2)) {
//       right = mid2;
//     } else {
//       left = mid1;
//     }
//   }
//   double pixel_yaw = (left + right) / 2.0;
  
//   // 三分搜索 - 角度误差
//   left = gimbal_rpy[2] - 70.0 * M_PI / 180.0;
//   right = gimbal_rpy[2] + 70.0 * M_PI / 180.0;
  
//   while (right - left > epsilon) {
//     double mid1 = left + (right - left) / 3.0;
//     double mid2 = right - (right - left) / 3.0;
    
//     if (getAngleCost(mid1) < getAngleCost(mid2)) {
//       right = mid2;
//     } else {
//       left = mid1;
//     }
//   }
//   double angle_yaw = (left + right) / 2.0;
  
//   // 混合策略: 根据yaw偏差大小动态混合
//   double abs_pixel_yaw = std::fabs(pixel_yaw - gimbal_rpy[2]);
//   double mid = 0.3;
//   double len = 0.1;
//   double best_yaw;
  
//   if ((abs_pixel_yaw > (mid - len / 2)) && (abs_pixel_yaw < (mid + len / 2))) {
//     double ratio = 0.5 + 0.5 * std::sin(M_PI * (abs_pixel_yaw - mid) / len);
//     best_yaw = ratio * pixel_yaw + (1 - ratio) * angle_yaw;
//   } else if (abs_pixel_yaw <= (mid - len / 2)) {
//     best_yaw = angle_yaw;  // 小偏差用角度优化
//   } else {
//     best_yaw = pixel_yaw;  // 大偏差用像素优化
//   }
  
//   // 打印调试信息
//   double final_error = getPixelCost(best_yaw);
//   std::cout << "Pixel yaw: " << pixel_yaw * 180 / M_PI 
//             << ", Angle yaw: " << angle_yaw * 180 / M_PI
//             << ", Best yaw: " << best_yaw * 180 / M_PI
//             << ", Final error: " << final_error << std::endl;
  
//   // 构建优化后的旋转矩阵
//   auto sin_yaw = std::sin(best_yaw);
//   auto cos_yaw = std::cos(best_yaw);
//   auto sin_pitch = std::sin(pitch);
//   auto cos_pitch = std::cos(pitch);
  
//   const Eigen::Matrix3d R_gimbal_armor_best {
//     {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
//     {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
//     {         -sin_pitch,        0,           cos_pitch}
//   };

//   return R_gimbal_camera_.transpose() * R_gimbal_armor_best;
// }
} // namespace fyt::auto_aim
