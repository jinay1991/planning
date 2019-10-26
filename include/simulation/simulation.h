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
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <motion_planning/motion_planning.h>
#include <motion_planning/roadmodel_data_source.h>

namespace sim
{
class Simulation
{
  public:
    using json = nlohmann::json;

    explicit Simulation(const std::string& map_file);
    ~Simulation();

    void Run();

  private:
    void InitializeMap();
    void UpdateDataSource(const json& msg);
    static const motion_planning::FrenetCoordinates GetPreviousPathEnd(const json& msg);
    static const motion_planning::PreviousPathGlobal GetPreviousPathGlobal(const json& msg);
    static const motion_planning::VehicleDynamics GetVehicleDynamics(const json& msg);
    static const motion_planning::SensorFusion GetSensorFusion(const json& msg);

    /// @brief Checks if the SocketIO event has JSON data.
    ///        If there is data the JSON object in string format will be returned,
    ///        else the empty string "" will be returned.
    static const std::string HasData(const std::string& s);

    /// @brief For converting back and forth between radians and degrees.
    static constexpr inline double PI() { return M_PI; }
    static constexpr inline double DegToRad(double x) { return x * PI() / 180.0; }
    static constexpr inline double RadToDeg(double x) { return x * 180.0 / PI(); }
    static constexpr motion_planning::LaneInformation::GlobalLaneId GetGlobalLaneId(
        const motion_planning::FrenetCoordinates& coords);

    uWS::Hub h_;
    std::vector<motion_planning::MapCoordinates> map_waypoints_;
    std::string map_file_;

    std::int32_t lane{1};
    motion_planning::LaneInformation::GlobalLaneId current_global_lane_id{
        motion_planning::LaneInformation::GlobalLaneId::kCenter};
    double ref_vel{0.0};

    std::shared_ptr<motion_planning::IDataSource> data_source_;
    std::unique_ptr<motion_planning::MotionPlanning> motion_planning_;
};
}  // namespace sim

#endif  /// SIMULATION_SIMULATION_H_
