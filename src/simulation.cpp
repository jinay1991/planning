///
/// @file simulation.cpp
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include "simulation/simulation.h"
#include "logging/logging.h"

namespace sim
{
Simulation::Simulation(const std::string& map_file)
    : map_file_{map_file},
      data_source_{std::make_shared<motion_planning::RoadModelDataSource>()},
      motion_planning_{std::make_unique<motion_planning::MotionPlanning>(data_source_)}
{
    InitializeMap();

    h_.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data != nullptr && data[0] == '4' && data[1] == '2')
        {
            const auto s = HasData(data);
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
                    LOG(DEBUG) << "Time taken by GenerateTrajectories(): " << elapsed_time << "usec";

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
    });

    h_.onConnection([&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { LOG(INFO) << "Connected"; });

    h_.onDisconnection([&](uWS::WebSocket<uWS::SERVER> ws, std::int32_t code, char* message, size_t length) {
        ws.close();
        LOG(INFO) << "Disconnected";
    });
}

Simulation::~Simulation() { LOG(INFO) << "Destructed!!"; }

void Simulation::InitializeMap()
{
    LOG(INFO) << "Using " << map_file_;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    std::string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        motion_planning::MapCoordinates wp;
        iss >> wp.global_coords.x;
        iss >> wp.global_coords.y;
        iss >> wp.frenet_coords.s;
        iss >> wp.frenet_coords.dx;
        iss >> wp.frenet_coords.dy;
        map_waypoints_.push_back(wp);
    }
}

const std::string Simulation::HasData(const std::string& s)
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

void Simulation::UpdateDataSource(const json& msg)
{
    data_source_->SetMapCoordinates(map_waypoints_);
    data_source_->SetSensorFusion(GetSensorFusion(msg));
    data_source_->SetVehicleDynamics(GetVehicleDynamics(msg));
    data_source_->SetPreviousPath(GetPreviousPathGlobal(msg));
    data_source_->SetPreviousPathEnd(GetPreviousPathEnd(msg));
    data_source_->SetSpeedLimit(units::velocity::miles_per_hour_t{49.5});
}

const motion_planning::FrenetCoordinates Simulation::GetPreviousPathEnd(const json& msg)
{
    const auto end_path_s = msg["end_path_s"].get<double>();
    const auto end_path_d = msg["end_path_d"].get<double>();
    const auto previous_path_end = motion_planning::FrenetCoordinates{end_path_s, end_path_d};
    return previous_path_end;
}

const motion_planning::PreviousPathGlobal Simulation::GetPreviousPathGlobal(const json& msg)
{
    const auto previous_path_x = msg["previous_path_x"];
    const auto previous_path_y = msg["previous_path_y"];
    std::vector<motion_planning::GlobalCoordinates> previous_path_global;
    for (auto idx = 0U; idx < previous_path_x.size(); ++idx)
    {
        previous_path_global.push_back(motion_planning::GlobalCoordinates{previous_path_x[idx], previous_path_y[idx]});
    }
    return previous_path_global;
}

const motion_planning::VehicleDynamics Simulation::GetVehicleDynamics(const json& msg)
{
    motion_planning::VehicleDynamics vehicle_dynamics;
    vehicle_dynamics.global_coords.x = msg["x"].get<double>();
    vehicle_dynamics.global_coords.y = msg["y"].get<double>();
    vehicle_dynamics.frenet_coords.s = msg["s"].get<double>();
    vehicle_dynamics.frenet_coords.d = msg["d"].get<double>();
    vehicle_dynamics.yaw = units::angle::degree_t{msg["yaw"].get<double>()};
    vehicle_dynamics.velocity = units::velocity::miles_per_hour_t{msg["speed"].get<double>()};

    return vehicle_dynamics;
}

const motion_planning::SensorFusion Simulation::GetSensorFusion(const json& msg)
{
    const auto sensor_fusion = msg["sensor_fusion"];
    motion_planning::SensorFusion sf;
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

        const auto global_coords = motion_planning::GlobalCoordinates{x, y};
        const auto frenet_coords = motion_planning::FrenetCoordinates{s, d, 0.0, 0.0};
        const auto velocity = units::velocity::meters_per_second_t{sqrt((vx * vx) + (vy * vy))};

        sf.objs.push_back(motion_planning::ObjectFusion{id, global_coords, frenet_coords, velocity});
    }

    return sf;
}

void Simulation::Run()
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

}  // namespace sim
