#include "sentry_control/path_checker.hpp"

namespace sentry_control
{
    path_checker::path_checker(const bool have_bumpy_area) : have_bumpy_area_(have_bumpy_area) {}

    void path_checker::get_path(const nav_msgs::msg::Path::SharedPtr &path)
    {
        this->path_ = path;
        this->in_bumpy_area_ = false;
        if(this->path_->poses.size() < 10)
        {
            for(size_t i = 0; i < this->path_->poses.size(); i++)
            {
                in_bumpy_area_ = bumpy_area_check(this->path_->poses[i].pose, this->friend_bumpy_area_, this->enermy_bumpy_area_);
                if(in_bumpy_area_) break;
            }
        }
        else
        {
            for(size_t i = 0; i < 10; i++)
            {
                in_bumpy_area_ = bumpy_area_check(this->path_->poses[i].pose, this->friend_bumpy_area_, this->enermy_bumpy_area_);
                if(in_bumpy_area_) break;
            }
        }
    }

    void path_checker::set_bumpy_area(const bumpy_area &friend_area, const bumpy_area &enermy_area)
    {
        this->friend_bumpy_area_ = friend_area;
        this->enermy_bumpy_area_ = enermy_area;
    }

    bool path_checker::is_in_bumpy_area()
    {
        if(!have_bumpy_area_)
            return false;
        else
            return this->in_bumpy_area_;
    }

    bool path_checker::bumpy_area_check(const geometry_msgs::msg::Pose &pose, const bumpy_area &friend_area, const bumpy_area &enermy_area)
    {
        return (pose.position.x <= friend_area.x_max &&
                pose.position.x >= friend_area.x_min &&
                pose.position.y <= friend_area.y_max &&
                pose.position.y >= friend_area.y_min) ||
               (pose.position.x <= enermy_area.x_max &&
                pose.position.x >= enermy_area.x_min &&
                pose.position.y <= enermy_area.y_max &&
                pose.position.y >= enermy_area.y_min);
    }
}