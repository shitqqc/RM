#include <geometry_msgs/msg/point.hpp>
#include <opencv2/core.hpp>

// STD
#include <array>
#include <vector>

#include "buff_detector/buff.hpp"

namespace rm_buff
{
class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // Get 3d position
  bool solvePnP(const Blade & blade, cv::Mat & rvec, cv::Mat & tvec);

  // Calculate the distance between armor center and image center
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Unit: mm
  static constexpr float BLADE_R = 310.00;
 // static constexpr float BLADE_RADIUS = 700.00;

  // Four vertices of armor in 3d
  std::vector<cv::Point3f> blade_points_;
};

}  // namespace rm_buff
