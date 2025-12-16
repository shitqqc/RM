#include "buff_detector/yolov8.hpp"
#include <rclcpp/rclcpp.hpp> 
#include <rclcpp/logger.hpp>

namespace rm_buff
{
YOLOV8::YOLOV8(const std::string & model_path, double conf_threshold, const std::string & device)
{
  auto model = core_.read_model(model_path);
  ov::preprocess::PrePostProcessor ppp(model);
  auto & input = ppp.input();

  input.tensor()
    .set_element_type(ov::element::u8)
    .set_shape({1, 640, 640, 3})  // TODO
    .set_layout("NHWC")
    .set_color_format(ov::preprocess::ColorFormat::RGB);

  input.model().set_layout("NCHW");

  input.preprocess()
    .convert_element_type(ov::element::f32)
    .convert_color(ov::preprocess::ColorFormat::BGR)
    // .resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR)
    .scale(255.0);
  device_ = device;
  model = ppp.build();
  compiled_model_ = core_.compile_model(
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));//THROUGHPUT
  score_threshold_ = conf_threshold;
  RCLCPP_INFO(rclcpp::get_logger("buff_detector"), "yolo initialized !");
}

std::vector<Blade> YOLOV8::detect(const cv::Mat & raw_img)
{
    std::lock_guard<std::mutex> lock(mtx_);
    auto total_start = std::chrono::high_resolution_clock::now();
    auto preprocess_start = total_start;
    auto inference_start = total_start;
    auto postprocess_start = total_start;
    
    if (raw_img.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("blade_detector"), "Empty img!, camera drop!");
        return std::vector<Blade>();
    }

    cv::Mat bgr_img;
    
    // 预处理阶段计时开始
    preprocess_start = std::chrono::high_resolution_clock::now();
    
    if (use_roi_) {
        if (roi_.width == -1) {
            roi_.width = raw_img.cols;
        }
        if (roi_.height == -1) {
            roi_.height = raw_img.rows;
        }
        bgr_img = raw_img(roi_);
    } else {
        bgr_img = raw_img;
    }

    auto x_scale = static_cast<double>(480) / bgr_img.rows;
    auto y_scale = static_cast<double>(640) / bgr_img.cols;
    auto scale = std::min(x_scale, y_scale);
    auto h = static_cast<int>(bgr_img.rows * scale);
    auto w = static_cast<int>(bgr_img.cols * scale);

    // preprocess
    auto input = cv::Mat(640, 480, CV_8UC3, cv::Scalar(0, 0, 0));
    auto roi = cv::Rect(0, 0, w, h);
    cv::resize(bgr_img, input(roi), {w, h});
    ov::Tensor input_tensor(ov::element::u8, {1, 640, 480, 3}, input.data);

    // 推理阶段计时开始
    inference_start = std::chrono::high_resolution_clock::now();
    
    // infer
    auto infer_request = compiled_model_.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    // 后处理阶段计时开始
    postprocess_start = std::chrono::high_resolution_clock::now();
    
    // postprocess
    auto output_tensor = infer_request.get_output_tensor();
    auto output_shape = output_tensor.get_shape();
    cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
    auto result = parse(scale, output, raw_img);

    // 计算各阶段时间
    auto total_end = std::chrono::high_resolution_clock::now();
    
    auto preprocess_time = std::chrono::duration_cast<std::chrono::microseconds>(
        inference_start - preprocess_start);
    auto inference_time = std::chrono::duration_cast<std::chrono::microseconds>(
        postprocess_start - inference_start);
    auto postprocess_time = std::chrono::duration_cast<std::chrono::microseconds>(
        total_end - postprocess_start);
    auto total_time = std::chrono::duration_cast<std::chrono::microseconds>(
        total_end - total_start);

    static int log_counter = 0;
    if (++log_counter % 50 == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("yolo_latency"),
                   " Preprocess: %.2fms, Inference: %.2fms, Postprocess: %.2fms, Total: %.2fms, Blades: %zu",
                   preprocess_time.count() / 1000.0,
                   inference_time.count() / 1000.0,
                   postprocess_time.count() / 1000.0,
                   total_time.count() / 1000.0,
                   result.size());
        log_counter = 0;
    }
    
    if (total_time > std::chrono::milliseconds(30)) { 
        RCLCPP_WARN(rclcpp::get_logger("yolo_latency"),
                   "Too SLOW! Total time %.2fms > 30ms",
                   total_time.count() / 1000.0);
    }

    return result;
}

void YOLOV8::push(cv::Mat img, int64_t t)
{
  auto x_scale = static_cast<double>(480) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  auto infer_request = compiled_model_.create_infer_request();
  auto input_tensor = infer_request.get_input_tensor();

  cv::Mat wrapper(640, 480, CV_8UC3, input_tensor.data<uint8_t>());

  wrapper = cv::Scalar(0, 0, 0); 

  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(img, wrapper(roi), {w, h});
  infer_request.start_async();
  queue_.push({img.clone(), t, std::move(infer_request)});
  
}

std::tuple<std::vector<Blade>, int64_t> YOLOV8::pop()
{
  auto [img, t, infer_request] = queue_.pop();
  infer_request.wait();
  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(480) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto blades = parse(scale, output, img);

  return {blades, t};
}

std::tuple<cv::Mat, std::vector<Blade>, int64_t>
YOLOV8::debug_pop()
{
  auto [img, t, infer_request] = queue_.pop();
  infer_request.wait();
  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(480) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto blades = parse(scale, output, img);
  return {img, std::move(blades), t};
}


std::vector<Blade> YOLOV8::parse(
  double scale, const cv::Mat & output, const cv::Mat & bgr_img)
{
  std::vector<int> color_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> blades_key_points;

    // data layout:
    // [0-3]: cx, cy, w, h
    // [4-5]: class_scores (R, B)
    // [6-20]: keypoints (x, y, visibility) * 5
  for (int r = 0; r < output.rows; r++) {
    double red_score = output.at<float>(r, 4);
    double blue_score = output.at<float>(r, 5);
    double score = std::max(red_score, blue_score);
    int _color_id = red_score > blue_score ? 0 : 1;
    score = sigmoid(score);
    if (score < score_threshold_) continue;

    std::vector<cv::Point2f> blade_key_points;
    //R bottom left top right
    blade_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    blade_key_points.push_back(
      cv::Point2f(output.at<float>(r, 9) / scale, output.at<float>(r, 10) / scale));
    blade_key_points.push_back(
      cv::Point2f(output.at<float>(r, 12) / scale, output.at<float>(r, 13) / scale));
    blade_key_points.push_back(
      cv::Point2f(output.at<float>(r, 15) / scale, output.at<float>(r, 16) / scale));
    blade_key_points.push_back(
      cv::Point2f(output.at<float>(r, 18) / scale, output.at<float>(r, 19) / scale));

    float min_x = blade_key_points[1].x;
    float max_x = blade_key_points[1].x;
    float min_y = blade_key_points[1].y;
    float max_y = blade_key_points[1].y;

    for (size_t i = 1; i < blade_key_points.size(); i++) {
      if (blade_key_points[i].x < min_x) min_x = blade_key_points[i].x;
      if (blade_key_points[i].x > max_x) max_x = blade_key_points[i].x;
      if (blade_key_points[i].y < min_y) min_y = blade_key_points[i].y;
      if (blade_key_points[i].y > max_y) max_y = blade_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    blades_key_points.emplace_back(blade_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::vector<Blade> blades;
  for (const auto & i : indices) {
    Blade blade;
    std::vector<cv::Point2f> blade_key_points = blades_key_points[i];
    blade.m_r = blade_key_points[0];
    blade.m_bottom = blade_key_points[1];
    blade.m_left = blade_key_points[2];
    blade.m_top = blade_key_points[3];
    blade.m_right = blade_key_points[4];
    blade.prob = confidences[i];
    blade.color = color_ids[i];
    blades.emplace_back(blade);
  }
  return blades;
}

double YOLOV8::sigmoid(double x)
{
  if (x > 0)
    return 1.0 / (1.0 + exp(-x));
  else
    return exp(x) / (1.0 + exp(x));
}

std::vector<Blade> YOLOV8::postprocess(
  double scale,const cv::Mat & output, const cv::Mat & bgr_img)
{
  return parse(scale, output, bgr_img);
}

}  // namespace 