#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

#include <chrono>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <tuple>
#include "armor_detector/armor.hpp"
#include "armor_detector/thread_safe_quene.hpp"
#include "rm_tools/pnp_solver.hpp"


namespace rm_auto_aim
{


class MultiThreadDetector
{
public:
  MultiThreadDetector(const std::string & model_path, const std::string & device, bool debug = false);

  void push(cv::Mat img);
  cv::Mat draw_result(const std::vector<Armor> armor, cv::Mat  img);
  inline double sigmoid(double x) {
      return 1.0 / (1.0 + std::exp(-x));
  }
  std::vector<Armor> pop();
  std::tuple<cv::Mat, std::vector<Armor>> debug_pop();
  std::vector<Armor> parse(double scale, cv::Mat & output, const cv::Mat & bgr_img);

  bool traditional_correct(Armor &armor, const cv::Mat & img);
  cv::Mat get_all_binary_img(std::vector<Armor> armors);
  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;
  bool check_geometry(const Lightbar & lightbar) const;
  cv::Point2f get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const;
  void lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const;
  void zy_corrector(Lightbar & lightbar) const;

    double score_threshold_;
    double nms_threshold_;
    double min_confidence_;
    std::string device_;
    std::string model_path_;
    cv::Mat result_img;
    int binary_thres;
    bool use_traditional_;
    double max_angle_error_;
    double min_lightbar_ratio_, max_lightbar_ratio_;
    double min_l2l_ratio_;
    double min_lightbar_length_;
    double min_armor_ratio_, max_armor_ratio_;
    double max_side_ratio_;
    double max_rectangular_error_;
    bool use_pca;
    double tolerate;

private:
 
  cv::Mat tmp_img_;
  ov::Core core_;
  ov::CompiledModel compiled_model_;

  rm_auto_aim::ThreadSafeQueue<
  std::tuple<cv::Mat, ov::InferRequest>>
  queue_{16, [] { std::cout<<"[MultiThreadDetector] queue is full!"<<std::endl; }};
};


}  // namespace auto_aim

#endif