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

<<<<<<< HEAD
<<<<<<< HEAD
const std::string Simulation::HasData(const std::string& s)
=======
constexpr std::string Simulation::HasData(std::string s)
>>>>>>> Fix cppcheck analysis errors
=======
const std::string Simulation::HasData(const std::string& s)
>>>>>>> Fix Builds
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> Fix Builds
=======
constexpr motion_planning::LaneInformation::GlobalLaneId Simulation::GetGlobalLaneId(
    const motion_planning::FrenetCoordinates& coords)
{
    if (coords.d > 0 && coords.d < 4)  // left lane (near to double solid lane marking)
    {
        return motion_planning::LaneInformation::GlobalLaneId::kLeft;
    }
    else if (coords.d > 4 && coords.d < 8)  // center lane
    {
        return motion_planning::LaneInformation::GlobalLaneId::kCenter;
    }
    else if (coords.d > 8 && coords.d < 12)  // right lane (near to the edge of the road)
    {
        return motion_planning::LaneInformation::GlobalLaneId::kRight;
    }
    else
    {
        return motion_planning::LaneInformation::GlobalLaneId::kInvalid;
    }
}

>>>>>>> Refactor
// /// @brief Calculate distance between two points
// constexpr double Simulation::GetDistance(double x1, double y1, double x2, double y2)
// {
//     return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
// }

// std::vector<double> Simulation::GetXY(double s, double d,
//                                       const std::vector<motion_planning::MapCoordinates>& map_waypoints)
// {
//     std::int32_t prev_wp = -1;

//     while (s > map_waypoints[prev_wp + 1].frenet_coords.s && (prev_wp < (std::int32_t)(map_waypoints.size() -
//     1)))
//     {
//         ++prev_wp;
//     }

//     std::int32_t wp2 = (prev_wp + 1) % map_waypoints.size();

//     double heading = atan2((map_waypoints[wp2].global_coords.y - map_waypoints[prev_wp].global_coords.y),
//                            (map_waypoints[wp2].global_coords.x - map_waypoints[prev_wp].global_coords.x));
//     // the x,y,s along the segment
//     double seg_s = (s - map_waypoints[prev_wp].frenet_coords.s);

//     double seg_x = map_waypoints[prev_wp].global_coords.x + seg_s * cos(heading);
//     double seg_y = map_waypoints[prev_wp].global_coords.y + seg_s * sin(heading);

//     double perp_heading = heading - PI() / 2;

//     double x = seg_x + d * cos(perp_heading);
//     double y = seg_y + d * sin(perp_heading);

//     return {x, y};
// }
// std::vector<double> Simulation::GetFrenet(double x, double y, double theta,
//                                           const std::vector<motion_planning::MapCoordinates>& map_waypoints)
// {
//     std::int32_t next_wp = NextWaypoint(x, y, theta, map_waypoints);

//     std::int32_t prev_wp;
//     prev_wp = next_wp - 1;
//     if (next_wp == 0)
//     {
//         prev_wp = map_waypoints.size() - 1;
//     }

//     double n_x = map_waypoints[next_wp].global_coords.x - map_waypoints[prev_wp].global_coords.x;
//     double n_y = map_waypoints[next_wp].global_coords.y - map_waypoints[prev_wp].global_coords.y;
//     double x_x = x - map_waypoints[prev_wp].global_coords.x;
//     double x_y = y - map_waypoints[prev_wp].global_coords.y;

//     // find the projection of x onto n
//     double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
//     double proj_x = proj_norm * n_x;
//     double proj_y = proj_norm * n_y;

//     double frenet_d = GetDistance(x_x, x_y, proj_x, proj_y);

//     // see if d value is positive or negative by comparing it to a center point
//     double center_x = 1000 - map_waypoints[prev_wp].global_coords.x;
//     double center_y = 2000 - map_waypoints[prev_wp].global_coords.y;
//     double centerToPos = GetDistance(center_x, center_y, x_x, x_y);
//     double centerToRef = GetDistance(center_x, center_y, proj_x, proj_y);

//     if (centerToPos <= centerToRef)
//     {
//         frenet_d *= -1;
//     }

//     // calculate s value
//     double frenet_s = 0;
//     for (auto i = 0; i < prev_wp; ++i)
//     {
//         frenet_s += GetDistance(map_waypoints[i].global_coords.x, map_waypoints[i].global_coords.y,
//                                 map_waypoints[i + 1].global_coords.x, map_waypoints[i + 1].global_coords.y);
//     }

//     frenet_s += GetDistance(0, 0, proj_x, proj_y);

//     return {frenet_s, frenet_d};
// }
// std::int32_t Simulation::NextWaypoint(double x, double y, double theta,
//                                       const std::vector<motion_planning::MapCoordinates>& map_waypoints)
// {
//     std::int32_t closestWaypoint = ClosestWaypoint(x, y, map_waypoints);

//     double map_x = map_waypoints[closestWaypoint].global_coords.x;
//     double map_y = map_waypoints[closestWaypoint].global_coords.y;

//     double heading = atan2((map_y - y), (map_x - x));

//     double angle = fabs(theta - heading);
//     angle = std::min(2 * PI() - angle, angle);

//     if (angle > PI() / 2)
//     {
//         ++closestWaypoint;
//         if (closestWaypoint == static_cast<std::int32_t>(map_waypoints.size()))
//         {
//             closestWaypoint = 0;
//         }
//     }

//     return closestWaypoint;
// }
// std::int32_t Simulation::ClosestWaypoint(double x, double y,
//                                          const std::vector<motion_planning::MapCoordinates>& map_waypoints)
// {
//     double closestLen = 100000;  // large number
//     std::int32_t closestWaypoint = 0;

//     for (auto i = 0U; i < map_waypoints.size(); ++i)
//     {
//         double map_x = map_waypoints[i].global_coords.x;
//         double map_y = map_waypoints[i].global_coords.y;
//         double dist = GetDistance(x, y, map_x, map_y);
//         if (dist < closestLen)
//         {
//             closestLen = dist;
//             closestWaypoint = i;
//         }
//     }

//     return closestWaypoint;
// }
<<<<<<< HEAD
=======
/// @brief Calculate distance between two points
constexpr double Simulation::GetDistance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

std::vector<double> Simulation::GetXY(double s, double d,
                                      const std::vector<motion_planning::MapCoordinates>& map_waypoints)
{
    int prev_wp = -1;

    while (s > map_waypoints[prev_wp + 1].frenet_coords.s && (prev_wp < (int)(map_waypoints.size() - 1)))
    {
        ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % map_waypoints.size();

    double heading = atan2((map_waypoints[wp2].global_coords.y - map_waypoints[prev_wp].global_coords.y),
                           (map_waypoints[wp2].global_coords.x - map_waypoints[prev_wp].global_coords.x));
    // the x,y,s along the segment
    double seg_s = (s - map_waypoints[prev_wp].frenet_coords.s);

    double seg_x = map_waypoints[prev_wp].global_coords.x + seg_s * cos(heading);
    double seg_y = map_waypoints[prev_wp].global_coords.y + seg_s * sin(heading);

    double perp_heading = heading - PI() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}
std::vector<double> Simulation::GetFrenet(double x, double y, double theta,
                                          const std::vector<motion_planning::MapCoordinates>& map_waypoints)
{
    int next_wp = NextWaypoint(x, y, theta, map_waypoints);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = map_waypoints.size() - 1;
    }

    double n_x = map_waypoints[next_wp].global_coords.x - map_waypoints[prev_wp].global_coords.x;
    double n_y = map_waypoints[next_wp].global_coords.y - map_waypoints[prev_wp].global_coords.y;
    double x_x = x - map_waypoints[prev_wp].global_coords.x;
    double x_y = y - map_waypoints[prev_wp].global_coords.y;

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = GetDistance(x_x, x_y, proj_x, proj_y);

    // see if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - map_waypoints[prev_wp].global_coords.x;
    double center_y = 2000 - map_waypoints[prev_wp].global_coords.y;
    double centerToPos = GetDistance(center_x, center_y, x_x, x_y);
    double centerToRef = GetDistance(center_x, center_y, proj_x, proj_y);

    if (centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = 0;
    for (int i = 0; i < prev_wp; ++i)
    {
        frenet_s += GetDistance(map_waypoints[i].global_coords.x, map_waypoints[i].global_coords.y,
                                map_waypoints[i + 1].global_coords.x, map_waypoints[i + 1].global_coords.y);
    }

    frenet_s += GetDistance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}
int Simulation::NextWaypoint(double x, double y, double theta,
                             const std::vector<motion_planning::MapCoordinates>& map_waypoints)
{
    int closestWaypoint = ClosestWaypoint(x, y, map_waypoints);

    double map_x = map_waypoints[closestWaypoint].global_coords.x;
    double map_y = map_waypoints[closestWaypoint].global_coords.y;

    double heading = atan2((map_y - y), (map_x - x));

    double angle = fabs(theta - heading);
    angle = std::min(2 * PI() - angle, angle);

    if (angle > PI() / 2)
    {
        ++closestWaypoint;
        if (closestWaypoint == map_waypoints.size())
        {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}
int Simulation::ClosestWaypoint(double x, double y, const std::vector<motion_planning::MapCoordinates>& map_waypoints)
{
    double closestLen = 100000;  // large number
    int closestWaypoint = 0;

    for (int i = 0; i < map_waypoints.size(); ++i)
    {
        double map_x = map_waypoints[i].global_coords.x;
        double map_y = map_waypoints[i].global_coords.y;
        double dist = GetDistance(x, y, map_x, map_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}
>>>>>>> Fix cppcheck analysis errors
=======
>>>>>>> Fix Builds
}  // namespace sim
