#include "buff_detector/yolo.hpp"

#include "buff_detector/yolov8.hpp"

namespace rm_buff
{

YOLO::YOLO(const std::string & model_path, double conf_threshold, const std::string & device)
{
    yolo_ = std::make_unique<YOLOV8>(model_path, conf_threshold, device);
}
std::vector<Blade> YOLO::detect(const cv::Mat & img)
{
  return yolo_->detect(img);
}

std::vector<Blade> YOLO::postprocess(
  double scale, const cv::Mat & output, const cv::Mat & bgr_img)
{
  return yolo_->postprocess(scale, output, bgr_img);
}

}  // namespace auto_aim