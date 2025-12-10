#ifndef RM_PLANNER_TARGET_HPP_
#define RM_PLANNER_TARGET_HPP_
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "rm_tools/math.hpp"
namespace rm_auto_aim
{
    class Target
    {
    public:
        std::vector<Eigen::Vector3d> getArmorPositions() const noexcept;
        int selectBestArmor() const noexcept;
        void predict(double dt);
        double xc,yc,zc,vx,vy,vz,yaw,vyaw,r,dr,dz;
        double armors_num;
        Eigen::Vector3d target_center;
    };
}
#endif