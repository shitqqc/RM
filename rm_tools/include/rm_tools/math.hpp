#ifndef RM_TOOLS__MATH_HPP
#define RM_TOOLS__MATH_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
namespace rm_tools
{
// 将弧度值限制在(-pi, pi]
double limit_rad(double angle);

// 四元数转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
// 参考：https://github.com/evbernardes/quaternion_to_euler
Eigen::Vector3d eulers(
  Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false);

// 旋转矩阵转欧拉角
// x = 0, y = 1, z = 2
// e.g. 先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转：axis0=2, axis1=1, axis2=0
Eigen::Vector3d eulers(Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

// 欧拉角转旋转矩阵
// zyx:先绕z轴旋转，再绕y轴旋转，最后绕x轴旋转
Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d & ypr);

// 直角坐标系转球坐标系
// ypd为yaw、pitch、distance的缩写
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz);

// 直角坐标系转球坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d & xyz);

// 球坐标系转直角坐标系
Eigen::Vector3d ypd2xyz(const Eigen::Vector3d & ypd);

// 球坐标系转直角坐标系转换函数对xyz的雅可比矩阵
Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d & ypd);

// 计算时间差a - b，单位：s
double delta_time(
  const std::chrono::steady_clock::time_point & a, const std::chrono::steady_clock::time_point & b);

// 向量夹角 总是返回 0 ~ pi 来自SJTU
double get_abs_angle(const Eigen::Vector2d & vec1, const Eigen::Vector2d & vec2);

// 返回输入值的平方
template <typename T>
T square(T const & a)
{
  return a * a;
};

double limit_min_max(double input, double min, double max);


inline Eigen::Vector3d getRPY(const Eigen::Matrix3d &R) {
  double yaw = atan2(R(0, 1), R(0, 0));
  double c2 = Eigen::Vector2d(R(2, 2), R(1, 2)).norm();
  double pitch = atan2(-R(0, 2), c2);

  double s1 = sin(yaw);
  double c1 = cos(yaw);
  double roll = atan2(s1 * R(2, 0) - c1 * R(2, 1), c1 * R(1, 1) - s1 * R(1, 0));

  return -Eigen::Vector3d(roll, pitch, yaw);
}

template <typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
cv::Mat eigenToCv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols> &eigen_mat) {
  cv::Mat cv_mat;
  cv::eigen2cv(eigen_mat, cv_mat);
  return cv_mat;
}

inline Eigen::MatrixXd cvToEigen(const cv::Mat &cv_mat) noexcept {
  Eigen::MatrixXd eigen_mat = Eigen::MatrixXd::Zero(cv_mat.rows, cv_mat.cols);
  cv::cv2eigen(cv_mat, eigen_mat);
  return eigen_mat;
}



}  // namespace tools

#endif  // TOOLS__MATH_TOOLS_HPP