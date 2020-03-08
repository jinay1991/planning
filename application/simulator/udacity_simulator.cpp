///
/// @file udacity_simulator.cpp
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include "application/simulator/udacity_simulator.h"
#include "planning/common/logging/logging.h"

namespace sim
{
namespace internal
{
/// @brief Checks if the SocketIO event has JSON data.
///        If there is data the JSON object in string format will be returned,
///        else the empty string "" will be returned.
const std::string HasData(const std::string& s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != std::string::npos)
    {
        return "";
    }
    else if (b1 != std::string::npos && b2 != std::string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

/// @brief Extract Previous Path End from WebSocket Msg (json)
const planning::FrenetCoordinates DecodePreviousPathEnd(const json& msg)
{
    const auto end_path_s = msg["end_path_s"].get<double>();
    const auto end_path_d = msg["end_path_d"].get<double>();
    const auto previous_path_end = planning::FrenetCoordinates{end_path_s, end_path_d};
    return previous_path_end;
}

/// @brief Extract Previous Path Points in Global Coordinates from WebSocket Msg (json)
const planning::PreviousPathGlobal DecodePreviousPathGlobal(const json& msg)
{
    const auto previous_path_x = msg["previous_path_x"];
    const auto previous_path_y = msg["previous_path_y"];
    std::vector<planning::GlobalCoordinates> previous_path_global;
    for (auto idx = 0U; idx < previous_path_x.size(); ++idx)
    {
        previous_path_global.push_back(planning::GlobalCoordinates{previous_path_x[idx], previous_path_y[idx]});
    }
    return previous_path_global;
}

/// @brief Extract Vehicle Dynamics from WebSocket Msg (json)
const planning::VehicleDynamics DecodeVehicleDynamics(const json& msg)
{
    planning::VehicleDynamics vehicle_dynamics;
    vehicle_dynamics.global_coords.x = msg["x"].get<double>();
    vehicle_dynamics.global_coords.y = msg["y"].get<double>();
    vehicle_dynamics.frenet_coords.s = msg["s"].get<double>();
    vehicle_dynamics.frenet_coords.d = msg["d"].get<double>();
    vehicle_dynamics.yaw = units::angle::degree_t{msg["yaw"].get<double>()};
    vehicle_dynamics.velocity = units::velocity::miles_per_hour_t{msg["speed"].get<double>()};

    return vehicle_dynamics;
}

/// @brief Extract Sensor Fusion from WebSocket Msg (json)
const planning::SensorFusion DecodeSensorFusion(const json& msg)
{
    const auto sensor_fusion = msg["sensor_fusion"];
    planning::SensorFusion sf;
    for (auto idx = 0U; idx < sensor_fusion.size(); ++idx)
    {
        const auto data = sensor_fusion[idx];
        const auto id = data[0].get<std::int32_t>();
        const auto x = data[1].get<double>();
        const auto y = data[2].get<double>();
        const auto vx = data[3].get<double>();
        const auto vy = data[4].get<double>();
        const auto s = data[5].get<double>();
        const auto d = data[6].get<double>();

        const auto global_coords = planning::GlobalCoordinates{x, y};
        const auto frenet_coords = planning::FrenetCoordinates{s, d, 0.0, 0.0};
        const auto velocity = units::velocity::meters_per_second_t{sqrt((vx * vx) + (vy * vy))};

        sf.objs.push_back(planning::ObjectFusion{id, global_coords, frenet_coords, velocity});
    }

    return sf;
}
}  // namespace internal

UdacitySimulator::UdacitySimulator(const std::string& map_file)
    : map_file_{map_file},
      data_source_{std::make_shared<planning::RoadModelDataSource>()},
      motion_planning_{std::make_unique<planning::MotionPlanning>(data_source_)}
{
}

void UdacitySimulator::Init()
{
    InitializeMap();

    h_.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode op_code) {
        ReceiveCallback(ws, data, length, op_code);
    });

    h_.onConnection([&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { ConnectCallback(ws, req); });

    h_.onDisconnection([&](uWS::WebSocket<uWS::SERVER> ws, std::int32_t code, char* message, size_t length) {
        DisconnectCallback(ws, code, message, length);
    });
}

void UdacitySimulator::InitializeMap()
{
    LOG(INFO) << "Using " << map_file_;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    std::string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        planning::MapCoordinates wp;
        iss >> wp.global_coords.x;
        iss >> wp.global_coords.y;
        iss >> wp.frenet_coords.s;
        iss >> wp.frenet_coords.dx;
        iss >> wp.frenet_coords.dy;
        map_waypoints_.push_back(wp);
    }

    LOG(INFO) << "Read " << map_waypoints_.size() << " map points";
}

void UdacitySimulator::ReceiveCallback(uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode op_code)
{
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data != nullptr && data[0] == '4' && data[1] == '2')
    {
        const auto s = internal::HasData(data);
        if (s != "")
        {
            const auto j = json::parse(s);

            const auto event = j[0].get<std::string>();

            if (event == "telemetry")
            {
                json msgJson;

                std::vector<double> next_x_vals;
                std::vector<double> next_y_vals;

                // ##############################################################
                UpdateDataSource(j[1]);

                const auto start = std::chrono::system_clock::now();
                motion_planning_->GenerateTrajectories();
                const auto elapsed_time =
                    std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start)
                        .count();
                LOG(DEBUG) << "Time taken by GenerateTrajectories() is " << elapsed_time << "usec." << std::endl;

                const auto trajectory = motion_planning_->GetSelectedTrajectory();
                for (const auto& wp : trajectory.waypoints)
                {
                    next_x_vals.push_back(wp.x);
                    next_y_vals.push_back(wp.y);
                }
                // ##############################################################
                // sequentially every .02 seconds
                msgJson["next_x"] = next_x_vals;
                msgJson["next_y"] = next_y_vals;

                const auto msg = "42[\"control\"," + msgJson.dump() + "]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
        else
        {
            // Manual driving
            std::string msg = "42[\"manual\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
    }
}

void UdacitySimulator::ConnectCallback(uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
{
    LOG(INFO) << "Connected";
}

void UdacitySimulator::Listen()
{
    std::int32_t port = 4567;
    if (h_.listen(port))
    {
        LOG(INFO) << "Listening to port " << port;
    }
    else
    {
        LOG(ERROR) << "Failed to listen to port";
        return;
    }
    h_.run();
}

void UdacitySimulator::DisconnectCallback(uWS::WebSocket<uWS::SERVER> ws, std::int32_t code, char* message,
                                          size_t length)
{
    ws.close();
    LOG(INFO) << "Disconnected";
}

void UdacitySimulator::UpdateDataSource(const json& msg)
{
    data_source_->SetMapCoordinates(map_waypoints_);
    data_source_->SetSensorFusion(internal::DecodeSensorFusion(msg));
    data_source_->SetPreviousPath(internal::DecodePreviousPathGlobal(msg));
    data_source_->SetPreviousPathEnd(internal::DecodePreviousPathEnd(msg));
    auto vehicle_dynamics = internal::DecodeVehicleDynamics(msg);
    const auto previous_path_global = data_source_->GetPreviousPathInGlobalCoords();
    if (!previous_path_global.empty())
    {
        vehicle_dynamics.frenet_coords.s = data_source_->GetPreviousPathEnd().s;
    }
    data_source_->SetVehicleDynamics(vehicle_dynamics);
    data_source_->SetSpeedLimit(units::velocity::miles_per_hour_t{49.5});
}

}  // namespace sim
