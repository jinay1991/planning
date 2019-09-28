///
/// @file
///

#include <math.h>
#include <simulation/simulation.h>
#include <iomanip>
#include <string>

namespace sim
{
Simulation::Simulation(const std::string& map_file)
    : map_file_{map_file},
      data_source_{std::make_shared<motion_planning::RoadModelDataSource>()},
      motion_planning_{std::make_unique<motion_planning::MotionPlanning>(data_source_)}
{
    // The max s value before wrapping around the track back to 0
    // double max_s = 6945.554;

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

    h_.onMessage([&](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data != nullptr && data[0] == '4' && data[1] == '2')
        {
            auto s = HasData(data);
            if (s != "")
            {
                auto j = json::parse(s);

                std::string event = j[0].get<std::string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    // // Main car's localization Data
                    // double car_x = j[1]["x"];
                    // double car_y = j[1]["y"];
                    // double car_s = j[1]["s"];
                    // double car_d = j[1]["d"];
                    // double car_yaw = j[1]["yaw"];
                    // double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];

                    // Previous path's end s and d values
                    auto end_path_s = j[1]["end_path_s"];
                    auto end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of
                    // the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    std::vector<double> next_x_vals;
                    std::vector<double> next_y_vals;
                    // ##############################################################
                    data_source_->SetMapCoordinates(map_waypoints_);
                    // data_source_->SetGlobalLaneId(GetGlobalLaneId(motion_planning::FrenetCoordinates{
                    //     j[1]["end_path_s"].get<double>(), j[1]["end_path_d"].get<double>()}));

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
                    data_source_->SetSensorFusion(sf);

                    motion_planning::VehicleDynamics vehicle_dynamics;
                    vehicle_dynamics.global_coords.x = j[1]["x"];
                    vehicle_dynamics.global_coords.y = j[1]["y"];
                    vehicle_dynamics.frenet_coords.s = j[1]["s"];
                    vehicle_dynamics.frenet_coords.d = j[1]["d"];
                    vehicle_dynamics.yaw = units::angle::degree_t{j[1]["yaw"]};
                    vehicle_dynamics.velocity = units::velocity::meters_per_second_t{j[1]["speed"]};
                    data_source_->SetVehicleDynamics(vehicle_dynamics);

                    std::vector<motion_planning::GlobalCoordinates> previous_path_global;
                    std::vector<motion_planning::FrenetCoordinates> previous_path_frenet;
                    for (auto idx = 0U; idx < previous_path_x.size(); ++idx)
                    {
                        previous_path_global.push_back(
                            motion_planning::GlobalCoordinates{previous_path_x[idx], previous_path_y[idx]});
                    }
                    data_source_->SetPreviousPath(previous_path_global);
                    data_source_->SetPreviousPathEnd(motion_planning::FrenetCoordinates{
                        j[1]["end_path_s"].get<double>(), j[1]["end_path_d"].get<double>()});

                    motion_planning_->GenerateTrajectories();
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

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    // this_thread::sleep_for(chrono::milliseconds(1000));
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

    h_.onConnection(
        [&](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

    h_.onDisconnection([&](uWS::WebSocket<uWS::SERVER> ws, std::int32_t code, char* message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
}

Simulation::~Simulation() { std::cout << "Destructed!!" << std::endl; }

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

void Simulation::Run()
{
    std::int32_t port = 4567;
    if (h_.listen(port))
    {
        std::cout << "Listening to port " << port << std::endl;
    }
    else
    {
        std::cerr << "Failed to listen to port" << std::endl;
        return;
    }
    h_.run();
}

}  // namespace sim
