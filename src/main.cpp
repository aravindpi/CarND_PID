#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid_steering;
  PID pid_throttle;
  
  // Initialize the pid variables.
  // Attempt #1: start with gains from the video
  pid_steering.Init(0.2, 0.004, 3.0); // This failed miserably

  // Attempt #2: Reduce Ki gain by a factor
  pid_steering.Init(0.2, 0.0002, 3.0); 

  // Attempt #3: Update Kp & Kd of steering based on twiddle.
  //             Make Ki smaller. Seems to have no effect
  //             Add pid for throttle and stay speed at 30 mph
  pid_steering.Init(0.242, 0.00022, 3.427); 
  pid_throttle.Init(0.2, 0.00004, 0.2);  

  // Attempt #4: Update steering gains based on twiddel
  //             update throttle gains based on twiddle
  pid_steering.Init(0.27143, 0.000242, 3.767); 
  pid_throttle.Init(0.2243, 0.0, 0.022);

  // Attempt #5: Adjust gains based on twiddle
  //             Increase speed to 55 mph
  pid_steering.Init(0.135, 0.00026, 3.12); 
  pid_throttle.Init(0.24795, 0.0, 0.023);

  // Attempt #6: Adjust gains based on twiddle
  //             Stay speed at 55 mph
  pid_steering.Init(0.14497, 0.00027, 3.3715); 
  pid_throttle.Init(0.3588, 0.0, 0.0253);
  

  h.onMessage([&pid_steering, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;
          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
	  pid_steering.UpdateError(cte);
          steer_value = -pid_steering.TotalError();

	  pid_throttle.UpdateError(fabs(cte));
	  throttle_value = 0.3; //0.55 - pid_throttle.TotalError();
	  
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
	  //std::cout << "CTE: " << cte << " Throttle Value: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
