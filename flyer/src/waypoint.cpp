#include "../include/flyer/waypoint.hpp"

const std::pair<Eigen::Vector3d, Eigen::Vector3d> Waypoints::TAKEOFF = std::make_pair(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0));
const std::pair<Eigen::Vector3d, Eigen::Vector3d> Waypoints::LAND = std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
const std::pair<Eigen::Vector3d, Eigen::Vector3d> Waypoints::HOME = std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
std::pair<Eigen::Vector3d, Eigen::Vector3d> Waypoints::MOVE = std::make_pair(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0));
constexpr double Waypoints::maxX = 2.0;  // meters
constexpr double Waypoints::maxY = 2.0;   // meters
constexpr double Waypoints::maxZ = 4.0;   // meters
constexpr double Waypoints::minX = -2.0;    // meters
constexpr double Waypoints::minY = -2.0;    // meters
constexpr double Waypoints::minZ = 0;        // meters
constexpr double Waypoints::tolerance = 0.2; // meters
constexpr double Waypoints::timeout = 5.0; // seconds