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

                    /**
                     * TODO: define a path made up of (x,y) points that the car will visit
                     *   sequentially every .02 seconds
                     */

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

void Simulation::OnMessageCallback(uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode opCode)
{
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

                /**
                 * TODO: define a path made up of (x,y) points that the car will visit
                 *   sequentially every .02 seconds
                 */

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
}

void Simulation::OnConnectionCallback(uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
{
    std::cout << "Connected!!!" << std::endl;
}

void Simulation::OnDisconnectionCallback(uWS::WebSocket<uWS::SERVER> ws, int code, char* message, size_t length)
{
    ws.close();
    std::cout << "Disconnected" << std::endl;
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
}  // namespace sim
