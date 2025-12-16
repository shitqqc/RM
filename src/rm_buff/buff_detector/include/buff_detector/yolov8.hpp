#ifndef BUFF_DETECTOR_YOLOV8_HPP
#define BUFF_DETECTOR_YOLOV8_HPP

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <string>
#include <vector>
#include "thread_safe_quene.hpp"
#include "buff.hpp"
#include "yolo.hpp"

namespace rm_buff
{
class YOLOV8 : public YOLOBase
{
public:
  YOLOV8(const std::string & model_path, double conf_threshold, const std::string & device);

  std::vector<Blade> detect(const cv::Mat & bgr_img) override;

  std::vector<Blade> postprocess(
    double scale, const cv::Mat & output, const cv::Mat & bgr_img) override;

  void push(cv::Mat img, int64_t timestamp_nanosec);
  std::tuple<std::vector<Blade>,int64_t> pop();
  std::tuple<cv::Mat, std::vector<Blade>,int64_t> debug_pop();

  ThreadSafeQueue<std::tuple<cv::Mat, int64_t, ov::InferRequest>>
  queue_{16, [] { std::cout<<"[MultiThreadDetector] queue is full!"<<std::endl; }};

private:
  std::string device_, model_path_;
  bool debug_, use_traditional_;
  bool use_roi_ = false;
  const float nms_threshold_ = 0.3;
  float score_threshold_ = 0.7;
  double min_confidence_;

  ov::Core core_;
  ov::CompiledModel compiled_model_;
  std::mutex mtx_;
  cv::Rect roi_;
  cv::Point2f offset_;

  std::vector<Blade> parse(double scale, const cv::Mat & output, const cv::Mat & bgr_img);

  double sigmoid(double x);


};

}  // namespace auto_aim

#endif  //AUTO_AIM__YOLOV8_HPP