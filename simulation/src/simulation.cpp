///
/// @file
///

#include <simulation/simulation.h>

namespace sim
{
Simulation::Simulation(const std::string& map_file) : map_file_{map_file}
{
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

    std::string line;
    while (getline(in_map_, line))
    {
        std::istringstream iss(line);
        WayPoint wp;
        iss >> wp.x;
        iss >> wp.y;
        iss >> wp.s;
        iss >> wp.dx;
        iss >> wp.dy;
        map_waypoints_.push_back(wp);
    }

    h_.onMessage([this](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = this->HasData(data);
            if (s != "")
            {
                auto j = json::parse(s);

                std::string event = j[0].get<std::string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];

                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];

                    // Sensor Fusion Data, a list of all other cars on the same side of
                    // the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];

                    json msgJson;

                    std::vector<double> next_x_vals;
                    std::vector<double> next_y_vals;
                    // ##############################################################

                    int prev_size = previous_path_x.size();

                    if (prev_size > 0)
                    {
                        car_s = end_path_s;
                    }

                    bool car_in_front = false;
                    bool car_to_left = false;
                    bool car_to_right = false;

                    for (int i = 0; i < sensor_fusion.size(); i++)
                    {
                        int car_lane = -1;
                        float d = sensor_fusion[i][6];
                        if (d > 0 && d < 4)  // left lane (near to double solid lane marking)
                            car_lane = 0;
                        else if (d > 4 && d < 8)  // center lane
                            car_lane = 1;
                        else if (d > 8 && d < 12)  // right lane (near to the edge of the road)
                            car_lane = 2;
                        else
                            continue;

                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_speed = sqrt(vx * vx + vy * vy);
                        double check_car_s = sensor_fusion[i][5];

                        check_car_s += ((double)(prev_size * 0.02 * check_speed));

                        if (car_lane == lane)
                        {
                            car_in_front |= (check_car_s > car_s) && ((check_car_s - car_s) < 30);
                        }
                        else if (car_lane == lane - 1)
                        {
                            car_to_left |= ((car_s - 30) < check_car_s) && ((car_s + 30) > check_car_s);
                        }
                        else if (car_lane == lane + 1)
                        {
                            car_to_right |= ((car_s - 30) < check_car_s) && ((car_s + 30) > check_car_s);
                        }
                    }

                    if (car_in_front)  // blocked by vehicle in ego lane
                    {
                        if (!car_to_right && lane != 2)
                            lane++;  // change lane to RIGHT
                        else if (!car_to_left && lane > 0)
                            lane--;  // change lane to LEFT
                        else
                            ref_vel -= 0.224;  // 5 meters per seconds
                    }
                    else
                    {
                        if (ref_vel < 49.5) ref_vel += 0.224;
                    }

                    std::vector<double> ptsx;
                    std::vector<double> ptsy;

                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = DegToRad(car_yaw);

                    if (prev_size < 2)
                    {
                        double prev_car_x = car_x - cos(car_yaw);
                        double prev_car_y = car_y - sin(car_yaw);

                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);

                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    }
                    else
                    {
                        ref_x = previous_path_x[prev_size - 1];
                        ref_y = previous_path_y[prev_size - 1];

                        double ref_x_prev = previous_path_x[prev_size - 2];
                        double ref_y_prev = previous_path_y[prev_size - 2];
                        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);

                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }

                    std::vector<double> next_wp0 = GetXY(car_s + 30, (2 + (4 * lane)), map_waypoints_);
                    std::vector<double> next_wp1 = GetXY(car_s + 60, (2 + (4 * lane)), map_waypoints_);
                    std::vector<double> next_wp2 = GetXY(car_s + 90, (2 + (4 * lane)), map_waypoints_);

                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);

                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);

                    for (int i = 0; i < ptsx.size(); i++)
                    {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;

                        ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
                        ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
                    }

                    tk::spline s;

                    s.set_points(ptsx, ptsy);

                    for (int i = 0; i < previous_path_x.size(); i++)
                    {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }

                    double target_x = 30.0;
                    double target_y = s(target_x);
                    double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

                    double x_add_on = 0;

                    for (int i = 1; i <= 50 - prev_size; i++)
                    {
                        double N = (target_dist / (0.02f * ref_vel / 2.24f));
                        double x_point = x_add_on + (target_x / N);
                        double y_point = s(x_point);

                        x_add_on = x_point;

                        double x_ref = x_point;
                        double y_ref = y_point;

                        x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
                        y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

                        x_point += ref_x;
                        y_point += ref_y;

                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
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

    h_.onDisconnection([&](uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
}

Simulation::~Simulation() { std::cout << "Destructed!!" << std::endl; }

std::string Simulation::HasData(std::string s) const
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
    int port = 4567;
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

/// @brief Calculate distance between two points
double Simulation::GetDistance(double x1, double y1, double x2, double y2) const
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

std::vector<double> Simulation::GetXY(double s, double d, const std::vector<WayPoint>& map_waypoints)
{
    int prev_wp = -1;

    while (s > map_waypoints[prev_wp + 1].s && (prev_wp < (int)(map_waypoints.size() - 1)))
    {
        ++prev_wp;
    }

    int wp2 = (prev_wp + 1) % map_waypoints.size();

    double heading =
        atan2((map_waypoints[wp2].y - map_waypoints[prev_wp].y), (map_waypoints[wp2].x - map_waypoints[prev_wp].x));
    // the x,y,s along the segment
    double seg_s = (s - map_waypoints[prev_wp].s);

    double seg_x = map_waypoints[prev_wp].x + seg_s * cos(heading);
    double seg_y = map_waypoints[prev_wp].y + seg_s * sin(heading);

    double perp_heading = heading - PI() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};
}
std::vector<double> Simulation::GetFrenet(double x, double y, double theta, const std::vector<WayPoint>& map_waypoints)
{
    int next_wp = NextWaypoint(x, y, theta, map_waypoints);

    int prev_wp;
    prev_wp = next_wp - 1;
    if (next_wp == 0)
    {
        prev_wp = map_waypoints.size() - 1;
    }

    double n_x = map_waypoints[next_wp].x - map_waypoints[prev_wp].x;
    double n_y = map_waypoints[next_wp].y - map_waypoints[prev_wp].y;
    double x_x = x - map_waypoints[prev_wp].x;
    double x_y = y - map_waypoints[prev_wp].y;

    // find the projection of x onto n
    double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
    double proj_x = proj_norm * n_x;
    double proj_y = proj_norm * n_y;

    double frenet_d = GetDistance(x_x, x_y, proj_x, proj_y);

    // see if d value is positive or negative by comparing it to a center point
    double center_x = 1000 - map_waypoints[prev_wp].x;
    double center_y = 2000 - map_waypoints[prev_wp].y;
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
        frenet_s += GetDistance(map_waypoints[i].x, map_waypoints[i].y, map_waypoints[i + 1].x, map_waypoints[i + 1].y);
    }

    frenet_s += GetDistance(0, 0, proj_x, proj_y);

    return {frenet_s, frenet_d};
}
int Simulation::NextWaypoint(double x, double y, double theta, const std::vector<WayPoint>& map_waypoints)
{
    int closestWaypoint = ClosestWaypoint(x, y, map_waypoints);

    double map_x = map_waypoints[closestWaypoint].x;
    double map_y = map_waypoints[closestWaypoint].y;

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
int Simulation::ClosestWaypoint(double x, double y, const std::vector<WayPoint>& map_waypoints)
{
    double closestLen = 100000;  // large number
    int closestWaypoint = 0;

    for (int i = 0; i < map_waypoints.size(); ++i)
    {
        double map_x = map_waypoints[i].x;
        double map_y = map_waypoints[i].y;
        double dist = GetDistance(x, y, map_x, map_y);
        if (dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}
}  // namespace sim
