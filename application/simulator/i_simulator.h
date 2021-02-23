///
/// @file
/// @brief Contains Interface for communicating to Simulator
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#ifndef SIMULATOR_I_SIMULATOR_H_
#define SIMULATOR_I_SIMULATOR_H_

#include <uWS.h>

namespace sim
{
/// @brief Interface for Simulator communications
class ISimulator
{
  public:
    /// @brief Destructor
    virtual ~ISimulator() = default;

    /// @brief Initialize and Register Callbacks for Connect, Receive and Disconnect
    virtual void Init() = 0;

    /// @brief Listen to WebSocket Port
    virtual void Listen() = 0;

  protected:
    /// @brief Connect callback for WebSocket
    virtual void ConnectCallback(uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) = 0;

    /// @brief Disconnect callback for WebSocket
    virtual void DisconnectCallback(uWS::WebSocket<uWS::SERVER> ws, std::int32_t code, char* message,
                                    size_t length) = 0;

    /// @brief Receive callback for WebSocket
    virtual void ReceiveCallback(uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length, uWS::OpCode op_code) = 0;
};

}  // namespace sim
#endif  /// SIMULATOR_I_SIMULATOR_H_
