#pragma once

#include "control_interface/msg/chassis_mod.hpp"

namespace sentry_control
{
    class SentryMod
    {
    private:
        bool in_bumpy_area_;
        bool use_spin_ = true;

        control_interface::msg::ChassisMod::SharedPtr chassis_mod_;

    public:
        SentryMod();

        void inBumpyArea(const bool in_bumpy_area);
        void useSpin(const bool use_spin);
        void getChassisMod(control_interface::msg::ChassisMod::SharedPtr &chassis_mod);
    };
}