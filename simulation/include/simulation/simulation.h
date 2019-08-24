///
/// @file
///
#ifndef SIMULATION_SIMULATION_H_
#define SIMULATION_SIMULATION_H_

#include <uWS/uWS.h>
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

    void Run();

  private:
    /// @brief Checks if the SocketIO event has JSON data.
    ///        If there is data the JSON object in string format will be returned,
    ///        else the empty string "" will be returned.
    std::string HasData(std::string s) const;
    void OnMessageCallback(uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode);
    void OnConnectionCallback(uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req);
    void OnDisconnectionCallback(uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length);

    uWS::Hub h_;
    std::vector<WayPoint> map_waypoints_;
    std::string map_file_;
};
}  // namespace sim

#endif  /// SIMULATION_SIMULATION_H_
