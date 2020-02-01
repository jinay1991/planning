///
/// @file udacity_simulator.h
/// @brief Contains Simulation Client Interface for Udacity Simulator
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef SIMULATOR_UDACITY_SIMULATOR_H_
#define SIMULATOR_UDACITY_SIMULATOR_H_

#include <math.h>
// #include <spline.h>
// #include <Eigen/Core>
// #include <Eigen/QR>
#include <fstream>
#include <iomanip>
#include <json.hpp>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "argument_parser/i_argument_parser.h"
#include "motion_planning/motion_planning.h"
#include "motion_planning/roadmodel_data_source.h"
#include "simulator/i_simulator.h"

namespace sim
{
/// @brief Simulator Client
class UdacitySimulator : public ISimulator
{
  public:
    using json = nlohmann::json;

    /// @brief Constructor. Initializes Map Points from map_file
    explicit UdacitySimulator(const std::string& map_file);

    /// @brief Initialize and Register Callbacks for Connect, Receive and Disconnect
    virtual void Init() override;

    /// @brief Listen to WebSocket Port
    virtual void Listen() override;

  protected:
    /// @brief Connect callback for WebSocket
    virtual void ConnectCallback(uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) override;

    /// @brief Disconnect callback for WebSocket
    virtual void DisconnectCallback(uWS::WebSocket<uWS::SERVER> ws, std::int32_t code, char* message,
                                    size_t length) override;

    /// @brief Receive callback for WebSocket
    virtual void ReceiveCallback(uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
                                 uWS::OpCode op_code) override;

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

#endif  /// SIMULATOR_UDACITY_SIMULATOR_H_
