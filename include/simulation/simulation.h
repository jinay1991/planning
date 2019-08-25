///
/// @file
///
#ifndef SIMULATION_SIMULATION_H_
#define SIMULATION_SIMULATION_H_

#include <spline.h>
#include <uWS/uWS.h>
#include <Eigen/Core>
#include <Eigen/QR>
#include <fstream>
#include <iostream>
#include <json.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace sim
{
struct WayPoint
{
    double x;
    double y;
    double s;
    double dx;
    double dy;
};

class Simulation
{
  public:
    using json = nlohmann::json;

    explicit Simulation(const std::string& map_file_);
    ~Simulation();

    void Run();

  private:
    /// @brief Checks if the SocketIO event has JSON data.
    ///        If there is data the JSON object in string format will be returned,
    ///        else the empty string "" will be returned.
    std::string HasData(std::string s) const;

    /// @brief For converting back and forth between radians and degrees.
    constexpr inline double PI() const { return M_PI; }
    constexpr inline double DegToRad(double x) const { return x * PI() / 180.0; }
    constexpr inline double RadToDeg(double x) const { return x * 180.0 / PI(); }

    /// @brief Calculate distance between two points
    double GetDistance(double x1, double y1, double x2, double y2) const;

    /// @brief Transform from Frenet s,d coordinates to Cartesian x,y
    std::vector<double> GetXY(double s, double d, const std::vector<WayPoint>& maps_waypoints);

    /// @brief Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    std::vector<double> GetFrenet(double x, double y, double theta, const std::vector<WayPoint>& maps_waypoints);

    /// @brief Returns next waypoint of the closest waypoint
    int NextWaypoint(double x, double y, double theta, const std::vector<WayPoint>& maps_waypoints);

    /// @brief Calculate closest waypoint to current x, y position
    int ClosestWaypoint(double x, double y, const std::vector<WayPoint>& maps_waypoints);

    uWS::Hub h_;
    std::vector<WayPoint> map_waypoints_;
    std::string map_file_;

    std::int32_t lane{1};
    double ref_vel{0.0};
};
}  // namespace sim

#endif  /// SIMULATION_SIMULATION_H_
