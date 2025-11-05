#include "armor_detector/detector.hpp"
#include <rclcpp/rclcpp.hpp> 
#include <rclcpp/logger.hpp>

namespace rm_auto_aim
{

MultiThreadDetector::MultiThreadDetector(const std::string & model_path, const std::string & device, bool debug)
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
    model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::THROUGHPUT));

  RCLCPP_INFO(rclcpp::get_logger("armor_detector"), "[MultiThreadDetector] initialized !");
}

void MultiThreadDetector::push(cv::Mat img)
{
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto h = static_cast<int>(img.rows * scale);
  auto w = static_cast<int>(img.cols * scale);

  // preproces
  auto input = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  auto roi = cv::Rect(0, 0, w, h);
  cv::resize(img, input(roi), {w, h});

  auto input_port = compiled_model_.input();
  auto infer_request = compiled_model_.create_infer_request();
  ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, input.data);

  infer_request.set_input_tensor(input_tensor);
  infer_request.start_async();
  queue_.push({img.clone(), std::move(infer_request)});
}

std::vector<Armor> MultiThreadDetector::pop()
{
  auto [img, infer_request] = queue_.pop();
  infer_request.wait();
  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = parse(scale, output, img);

  return ((armors));
}

std::tuple<cv::Mat, std::vector<Armor>>
MultiThreadDetector::debug_pop()
{
  auto [img, infer_request] = queue_.pop();
  infer_request.wait();
  // postprocess
  auto output_tensor = infer_request.get_output_tensor();
  auto output_shape = output_tensor.get_shape();
  cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
  auto x_scale = static_cast<double>(640) / img.rows;
  auto y_scale = static_cast<double>(640) / img.cols;
  auto scale = std::min(x_scale, y_scale);
  auto armors = parse(scale, output, img);
  return {img, std::move(armors)};
}

std::vector<Armor> MultiThreadDetector::parse(double scale, cv::Mat & output, const cv::Mat & bgr_img)
{
   // for each row: xywh + classess
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armors_key_points;
  for (int r = 0; r < output.rows; r++) {
    double score = output.at<float>(r, 8);
    score = sigmoid(score);

    if (score < score_threshold_) continue;

    std::vector<cv::Point2f> armor_key_points;

    //颜色和类别独热向量
    cv::Mat color_scores = output.row(r).colRange(9, 13);     //color
    cv::Mat classes_scores = output.row(r).colRange(13, 22);  //num
    cv::Point class_id, color_id;
    int _class_id, _color_id;
    double score_color, score_num;
    cv::minMaxLoc(classes_scores, NULL, &score_num, NULL, &class_id);
    cv::minMaxLoc(color_scores, NULL, &score_color, NULL, &color_id);
    _class_id = class_id.x;
    _color_id = color_id.x;

    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 0) / scale, output.at<float>(r, 1) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 6) / scale, output.at<float>(r, 7) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 4) / scale, output.at<float>(r, 5) / scale));
    armor_key_points.push_back(
      cv::Point2f(output.at<float>(r, 2) / scale, output.at<float>(r, 3) / scale));

    float min_x = armor_key_points[0].x;
    float max_x = armor_key_points[0].x;
    float min_y = armor_key_points[0].y;
    float max_y = armor_key_points[0].y;

  for (int i = 1; i < static_cast<int>(armor_key_points.size()); i++) {
      if (armor_key_points[i].x < min_x) min_x = armor_key_points[i].x;
      if (armor_key_points[i].x > max_x) max_x = armor_key_points[i].x;
      if (armor_key_points[i].y < min_y) min_y = armor_key_points[i].y;
      if (armor_key_points[i].y > max_y) max_y = armor_key_points[i].y;
    }

    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);

    color_ids.emplace_back(_color_id);
    num_ids.emplace_back(_class_id);
    boxes.emplace_back(rect);
    confidences.emplace_back(score);
    armors_key_points.emplace_back(armor_key_points);
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::vector<Armor> armors;
  for (const auto & i : indices) {
      armors.emplace_back(color_ids[i], num_ids[i], confidences[i], boxes[i], armors_key_points[i]);    
  }

  tmp_img_ = bgr_img;
  for (auto it = armors.begin(); it != armors.end();) {
    if (!check_name(*it)) {
      it = armors.erase(it);
      continue;
    }

    if (!check_type(*it)) {
      it = armors.erase(it);
      continue;
    }
    // 使用传统方法二次矫正角点
    if (use_traditional_) traditional_correct(*it, bgr_img);

    it->center_norm = get_center_norm(bgr_img, it->center);
    ++it;
  }

  //if (debug_) draw_detections(bgr_img, armors);

  return armors;
}


cv::Mat MultiThreadDetector::draw_result(const std::vector<Armor> armors, cv::Mat img)
{
  for(auto & armor : armors)
  {
    cv::line(img, armor.points[0], armor.points[2], cv::Scalar(0, 255, 0), 1);
    cv::line(img, armor.points[1], armor.points[3], cv::Scalar(0, 255, 0), 1);
      cv::putText(
      img, armor.rfs, armor.points[0], cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
        for(const auto& armor : armors)
      for (const auto& pt : armor.net_points) {           
            cv::circle(img, pt, 1, cv::Scalar(125, 125, 125), -1);
    }
  }
  return img;
}

bool MultiThreadDetector::traditional_correct(Armor & armor , const cv::Mat & img)
{
  // 取得四个角点
  auto tl = armor.points[0];
  auto tr = armor.points[1];
  auto br = armor.points[2];
  auto bl = armor.points[3];
  // 计算向量和调整后的点
  auto lt2b = bl - tl;
  auto rt2b = br - tr;
  //扩大1.5倍
  auto tl1 = (tl + bl) / 2 - lt2b;
  auto bl1 = (tl + bl) / 2 + lt2b;
  auto br1 = (tr + br) / 2 + rt2b;
  auto tr1 = (tr + br) / 2 - rt2b;
  //计算宽度
  auto tl2tr = tr1 - tl1;
  auto bl2br = br1 - bl1;

  //扩大
  auto tl2 = (tl1 + tr) / 2 - 0.75 * tl2tr;
  auto tr2 = (tl1 + tr) / 2 + 0.75 * tl2tr;
  auto bl2 = (bl1 + br) / 2 - 0.75 * bl2br;
  auto br2 = (bl1 + br) / 2 + 0.75 * bl2br;
  // 构造新的四个角点
  std::vector<cv::Point> points = {tl2, tr2, br2, bl2};
  auto armor_rotaterect = cv::minAreaRect(points);
  cv::Rect boundingBox = armor_rotaterect.boundingRect();
  // 检查boundingBox是否超出图像边界
  if (
    boundingBox.x < 0 || boundingBox.y < 0 || boundingBox.x + boundingBox.width > img.cols ||
    boundingBox.y + boundingBox.height > img.rows) {
    return false;
  }

  // 在图像上裁剪出这个矩形区域（ROI）
  cv::Mat armor_roi = img(boundingBox);
  if (armor_roi.empty()) {
    return false;
  }

  // 彩色图转灰度图
  cv::Mat gray_img;
  cv::cvtColor(armor_roi, gray_img, cv::COLOR_BGR2GRAY);
  // 进行二值化
  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);
  armor.pattern = binary_img;
  // 获取轮廓点
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  // 获取灯条
  std::size_t lightbar_id = 0;
  std::list<Lightbar> lightbars;
  for (const auto & contour : contours) {
    auto rotated_rect = cv::minAreaRect(contour);
    auto lightbar = Lightbar(rotated_rect, lightbar_id);

    if (!check_geometry(lightbar)) continue;

    //lightbar.color = get_color(bgr_img, contour);
    // lightbar_points_corrector(lightbar, gray_img); //关闭PCA
    lightbars.emplace_back(lightbar);
    lightbar_id += 1;
  }

  if (lightbars.size() < 2) return false;

  // 将灯条从左到右排序
  lightbars.sort([](const Lightbar & a, const Lightbar & b) { return a.center.x < b.center.x; });

  // 计算与 tl_roi, bl_roi 和 br_roi, tr_roi 距离最近的灯条
  Lightbar * closest_left_lightbar = nullptr;
  Lightbar * closest_right_lightbar = nullptr;
  float min_distance_tl_bl = std::numeric_limits<float>::max();
  float min_distance_br_tr = std::numeric_limits<float>::max();
  for (auto & lightbar : lightbars) {
    float distance_tl_bl =
      cv::norm(tl - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(bl - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (distance_tl_bl < min_distance_tl_bl) {
      min_distance_tl_bl = distance_tl_bl;
      closest_left_lightbar = &lightbar;
    }
    float distance_br_tr =
      cv::norm(br - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y))) +
      cv::norm(tr - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y)));
    if (distance_br_tr < min_distance_br_tr) {
      min_distance_br_tr = distance_br_tr;
      closest_right_lightbar = &lightbar;
    }
  }

  auto dis = (min_distance_br_tr + min_distance_tl_bl);
  RCLCPP_DEBUG(rclcpp::get_logger("armor_detector"), "min_distance_br_tr + min_distance_tl_bl is: %f", dis);
  if (
    closest_left_lightbar && closest_right_lightbar &&
    min_distance_br_tr + min_distance_tl_bl < tolerate) {
    // 将四个点从armor_roi坐标系转换到原始图像坐标系
    armor.points[0] = closest_left_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[1] = closest_right_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[2] = closest_right_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
    armor.points[3] = closest_left_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
    return true;
  }
  RCLCPP_WARN(rclcpp::get_logger("armor_detector"), "min_distance_br_tr + min_distance_tl_bl too large: %f", dis);
  return false;
}

bool MultiThreadDetector::check_geometry(const Lightbar & lightbar) const
{
  auto angle_ok = lightbar.angle_error < max_angle_error_;
  auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
  auto length_ok = lightbar.length > min_lightbar_length_;
  return angle_ok && ratio_ok && length_ok;
}


bool MultiThreadDetector::check_name(const Armor & armor) const
{
  auto name_ok = armor.name != ArmorName::not_armor;
  auto confidence_ok = armor.confidence > min_confidence_;

  return name_ok && confidence_ok;
}

bool MultiThreadDetector::check_type(const Armor & armor) const
{
  auto name_ok = (armor.type == ArmorType::small)
                   ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
                   : (armor.name == ArmorName::one && armor.name == ArmorName::base);

  return name_ok;
}


cv::Point2f MultiThreadDetector::get_center_norm(const cv::Mat & bgr_img, const cv::Point2f & center) const
{
  auto h = bgr_img.rows;
  auto w = bgr_img.cols;
  return {center.x / w, center.y / h};
}

cv::Mat MultiThreadDetector::get_all_binary_img(std::vector<Armor> armors)
{
  if (armors.empty()) {
    cv::Mat black_image = cv::Mat::zeros(cv::Size(250, 60), CV_8UC1);
    cv::putText(black_image, "PIONEER", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(255),2,8);                             
    return black_image;
  } else {
    int max_width = 0;
    int max_height = 0;
    std::vector<cv::Mat> binary_imgs;
    std::vector<cv::Mat> padd_imgs;
    binary_imgs.reserve(armors.size());
    padd_imgs.reserve(armors.size());
    for (auto & armor : armors) {
      if(!armor.pattern.empty())
      binary_imgs.emplace_back(armor.pattern);
    }
    for (const auto& img : binary_imgs) {
      max_width = std::max(max_width, img.cols);
      max_height = std::max(max_height, img.rows);
    }
    if( max_width == 0 || binary_imgs.size() == 0 || max_height == 0)
      return cv::Mat(cv::Size(20, 28), CV_8UC1);
    for (auto& img : binary_imgs) {
        cv::Mat padded_img(max_height, max_width, img.type(), cv::Scalar(0, 0, 0));
        auto x_offset = (max_width - img.cols) / 2;
        auto y_offset = (max_height - img.rows) / 2;
        img.copyTo(padded_img(cv::Rect(x_offset, y_offset, img.cols, img.rows)));

        img = padded_img;
        padd_imgs.emplace_back(img);
    }
    cv::Mat all_binary_img;
    cv::vconcat(padd_imgs, all_binary_img);
    return all_binary_img;
  }
}
}  // namespace auto_aim
