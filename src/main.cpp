#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <Eigen/Core>
#include <Eigen/QR>
#include "MPC.h"
#include "json.hpp"
#include "tools.hpp"

// for convenience
using json = nlohmann::json;

const int N_ref = 12; // number of reference points for reference trajectory
const double wp_delta = 6.0; // delta between waypoints in x direction
const double latency_s = 0.01; // latency in seconds
const double Lf = 2.67;

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc(0.436332, -0.6, 1.0, 1000);

  // save steering angle and throttle values in case optimizer failed
  double steer_value = 0.0;
  double throttle_value = 0.8;

  h.onMessage([&mpc, &steer_value, &throttle_value](uWS::WebSocket<uWS::SERVER> ws,
                                                    char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> waypts_x = j[1]["ptsx"];
          vector<double> waypts_y = j[1]["ptsy"];
          double px               = j[1]["x"];
          double py               = j[1]["y"];
          double psi              = j[1]["psi"];
          double v                = j[1]["speed"];

          //Display the waypoints/reference line
          vector<double> next_x_vals(N_ref);
          vector<double> next_y_vals(N_ref);
          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // Transform waypoints from global coordinate system into the local car coordinate system
          std::vector<double> waypts_x_trans;
          std::vector<double> waypts_y_trans;
          Tranform2d(waypts_x_trans, waypts_y_trans, waypts_x, waypts_y, px, py, -psi);
          Eigen::Map<Eigen::VectorXd> wp_x(waypts_x_trans.data(), waypts_x_trans.size());
          Eigen::Map<Eigen::VectorXd> wp_y(waypts_y_trans.data(), waypts_y_trans.size());

          // Fit a polynomial of order 3 to the above x and y coordinates
          auto coeffs = polyfit(wp_x, wp_y, 3);

          for (int x = 0; x < (int) next_x_vals.size(); x++) {
            next_x_vals[x] = wp_delta*x;
            next_y_vals[x] = polyeval(coeffs, next_x_vals[x]);
          }

          bool success = false;

          // Calculate the cross-track-error
          // current CTE is fitted polynomial (road curve) evaluated at px = 0.0
          // f = K[3] * px0 * px0 + px0 + K[2] * px0 * px0 + K[1] * px0 + K[0];
          double cte = coeffs[0];

          // current heading error epsi is the tangent to the road curve at px = 0.0
          // epsi = arctan(f') where f' is the derivative of the fitted polynomial
          // derivative of coeffs[0] + coeffs[1] * x  ... -> coeffs[1]
          double epsi = -atan(coeffs[1]);

          // In initial state for trajectory planning x, y, and psi are always 0!
          px = 0.0;
          py = 0.0;
          psi = 0.0;
          if  (latency_s > 0.0) {
            px   += v * cos(psi) * latency_s;
            py   += v * sin(psi) * latency_s;
            psi  += v * -steer_value / Lf * latency_s;
            cte  += v * sin(epsi) * latency_s;
            epsi += v * -steer_value / Lf * latency_s;
            v    += throttle_value * latency_s;
          }

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;

          auto solution = mpc.Solve(state, coeffs, mpc_x_vals, mpc_y_vals, success);

          // Calculate steering angle and throttle using MPC.
          // Both are in between [-1, 1].
          // But only update values if solver succeeded, otherwise keep old values!
          if (success) {
            // Note:
            // - If steering is positive we rotate counter-clockwise, or turn left.
            //   In the simulator however, a positive value implies a right turn and a negative value implies a left turn.
            //   Therefore we change directions here.
            // - Remember to divide by deg2rad(25) before you send the steering value back.
            //   Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
            steer_value    = -solution[0] / deg2rad(25);
            throttle_value = solution[1];
          } else {
            std::cout << "Optimizer failed to find solution!" << std::endl;
          }

          json msgJson;

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds((int) (latency_s*1e3)));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
