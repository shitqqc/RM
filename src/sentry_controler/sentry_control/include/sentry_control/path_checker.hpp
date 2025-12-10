#pragma once

#include "nav_msgs/msg/path.hpp"

struct bumpy_area
{
    double x_max;
    double x_min;
    double y_max;
    double y_min;
};


namespace sentry_control
{
    class path_checker
    {
    private:
        bool in_bumpy_area_;
        bool have_bumpy_area_;

        bumpy_area friend_bumpy_area_;
        bumpy_area enermy_bumpy_area_;

        nav_msgs::msg::Path::SharedPtr path_;
    public:
        path_checker(const bool have_bumpy_area = false);
        void get_path(const nav_msgs::msg::Path::SharedPtr &path);
        void set_bumpy_area(const bumpy_area &friend_area, const bumpy_area &enermy_area);
        bool is_in_bumpy_area();
        bool bumpy_area_check(const geometry_msgs::msg::Pose &pose, const bumpy_area &friend_area, const bumpy_area &enermy_area);
    };

} // namespace sentry_control