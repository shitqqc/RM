#ifndef BUFF_DETECTOR_YOLO_HPP
#define BUFF_DETECTOR_YOLO_HPP

#include <opencv2/opencv.hpp>

#include "buff.hpp"

namespace rm_buff
{
class YOLOBase
{
public:
  virtual std::vector<Blade> detect(const cv::Mat & img) = 0;

  virtual std::vector<Blade> postprocess(
    double scale, const cv::Mat & output, const cv::Mat & bgr_img) = 0;
};

class YOLO
{
public:
  YOLO(const std::string & model_path, double conf_threshold, const std::string & device);

  std::vector<Blade> detect(const cv::Mat & img);

  std::vector<Blade> postprocess(
    double scale, const cv::Mat & output, const cv::Mat & bgr_img);

private:
  std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP