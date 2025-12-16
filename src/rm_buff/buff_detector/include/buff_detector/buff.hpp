#ifndef BUFF_DETECTOR__BUFF_HPP_
#define BUFF_DETECTOR__BUFF_HPP_

#include <opencv2/core.hpp>

namespace rm_buff {
    const int RED = 0;
    const int BLUE = 1;
/**
 * @brief 装甲板
 */
struct Blade {
    Blade() = default;
    cv::Rect rect;
    float prob;
    int color;
    cv::Point2f m_top;
    cv::Point2f m_bottom;
    cv::Point2f m_left;
    cv::Point2f m_right;
    cv::Point2f m_center;  // 扇叶中心点
    cv::Point2f m_r;
    double m_x;
    double m_y;
};


}  // namespace rm_buff

#endif  // BUFF_DETECTOR__BLADE_HPP_