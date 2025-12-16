#include "buff_detector/pnp_solver.hpp"
#include <opencv2/calib3d.hpp>
#include <vector>
//r=39cm
namespace rm_buff {
PnPSolver::PnPSolver(const std::array<double, 9> &camera_matrix,
                     const std::vector<double> &dist_coeffs)
    : camera_matrix_(
          cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data()))
              .clone()),
      dist_coeffs_(
          cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data()))
              .clone()) {
  // Unit: m
  constexpr double half_R = BLADE_R / 2 / 1000;

 // constexpr double radius = BLADE_RADIUS / 1000;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  blade_points_.emplace_back(cv::Point3f(0, 0, -half_R));
  blade_points_.emplace_back(cv::Point3f(0, 0, half_R));
  blade_points_.emplace_back(cv::Point3f(0, -half_R, 0));
  blade_points_.emplace_back(cv::Point3f(0, half_R, 0));
  //blade_points_.emplace_back(cv::Point3f(0, 0, 0));
  //blade_points_.emplace_back(cv::Point3f(0, 0, -radius));
  //square
  //z forward x right y up
  // blade_points_.emplace_back(cv::Point3f(-half_R, half_R, 0));//tl
  // blade_points_.emplace_back(cv::Point3f(half_R, half_R, 0));//tr
  // blade_points_.emplace_back(cv::Point3f(half_R, -half_R, 0));//br
  // blade_points_.emplace_back(cv::Point3f(-half_R, -half_R, 0)); //bl

}

bool PnPSolver::solvePnP(const Blade & blade, cv::Mat &rvec, cv::Mat &tvec) {
  std::vector<cv::Point2f> image_blade_points;
  //ippe || itreative
  cv::Point2f blade_top = blade.m_top;
  cv::Point2f blade_bottom = blade.m_bottom;
  cv::Point2f blade_left = blade.m_left;
  cv::Point2f blade_right = blade.m_right;
  //cv::Point2f blade_center = blade.m_center;
  //cv::Point2f blade_r = blade.m_r;

  //suqare
  // cv::Point2f bl(blade.m_left.x , blade.m_bottom.y);
  // cv::Point2f br(blade.m_right.x , blade.m_bottom.y);
  // cv::Point2f tr(blade.m_right.x , blade.m_top.y);
  // cv::Point2f tl(blade.m_left.x , blade.m_top.y);

  // Fill in image points
  //ippe || itreative
  image_blade_points.emplace_back(blade_bottom);
  image_blade_points.emplace_back(blade_top);
  image_blade_points.emplace_back(blade_right);
  image_blade_points.emplace_back(blade_left);
  //image_blade_points.emplace_back(blade_center);
  //image_blade_points.emplace_back(blade_r);

 //square
  // image_blade_points.emplace_back(tl);
  // image_blade_points.emplace_back(tr);
  // image_blade_points.emplace_back(br);
  // image_blade_points.emplace_back(bl);

  // Solve pnp
  return cv::solvePnP(blade_points_, image_blade_points, camera_matrix_,
                      dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_IPPE);
}


float PnPSolver::calculateDistanceToCenter(const cv::Point2f &image_point) {
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_buff