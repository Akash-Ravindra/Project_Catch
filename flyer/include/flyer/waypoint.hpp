#pragma once
#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

class Waypoints{
    public:
    static const std::pair<Eigen::Vector3d, Eigen::Vector3d> TAKEOFF;
    static const std::pair<Eigen::Vector3d, Eigen::Vector3d> LAND;
    static const std::pair<Eigen::Vector3d, Eigen::Vector3d> HOME;
    static std::pair<Eigen::Vector3d, Eigen::Vector3d> MOVE;
    static const double maxX;
    static const double maxY;
    static const double maxZ;
    static const double minX;
    static const double minY;
    static const double minZ;
    static const double tolerance;
    static const double timeout;
};
enum class FlyerCommand
{
    TAKEOFF=1,
    LAND,
    HOME,
    MOVE,
    SET_MODE,
    ARM,
    DISARM
};
#endif