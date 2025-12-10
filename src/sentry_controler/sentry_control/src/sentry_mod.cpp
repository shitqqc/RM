#include "sentry_control/sentry_mod.hpp"

namespace sentry_control
{
    SentryMod::SentryMod(){}

    void SentryMod::inBumpyArea(const bool in_bumpy_area)
    {
        in_bumpy_area_ = in_bumpy_area;
    }

    void SentryMod::useSpin(const bool use_spin)
    {
        this->use_spin_ = use_spin;
    }

    void SentryMod::getChassisMod(control_interface::msg::ChassisMod::SharedPtr & chassis_mod)
    {
        this->chassis_mod_ = chassis_mod;
        if(in_bumpy_area_)
        {
            chassis_mod_->type = control_interface::msg::ChassisMod::AIMANGLE;
        }else if (!in_bumpy_area_ && use_spin_)
        {
            chassis_mod_->type = control_interface::msg::ChassisMod::AIMSPEED;
            chassis_mod_->aim_speed = 0.5;
        }else
        {
            chassis_mod_->type = control_interface::msg::ChassisMod::AIMSPEED;
            chassis_mod_->aim_speed = 0.0;
        }
    }
}