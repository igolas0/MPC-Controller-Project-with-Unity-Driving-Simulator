#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          //taking actuator latency into account by making prediction after 100ms:
          double latency = 0.1;
          //define constant with length of from to center of gravitiy of the car.
          const double Lf = 2.67;
          //prediction of car variables in Global coordinates  after latency @100ms
          //mph to m/s factor = 0.44704
          double predicted_px = px + 0.44704 * v * cos(psi) * latency;
          double predicted_py = py + 0.44704 * v * sin(psi) * latency;
          double predicted_psi = psi - 0.44704 * v * steer_value / Lf * latency;
          //use throttle value as proxy for acc.
          double predicted_v = v + throttle_value * latency;

          //transform global coordinates into car centered coordinate system
          // x axis points to heading direction
          Eigen::VectorXd waypoints_x(ptsx.size());
          Eigen::VectorXd waypoints_y(ptsx.size());

          for (size_t i = 0; i < ptsx.size(); i++)
          {
             //shift car reference angle to 90 degrees
             double transf_x = ptsx[i] - predicted_px;
             double transf_y = ptsy[i] - predicted_py;

             waypoints_x[i] = transf_x*cos(-predicted_psi)-transf_y*sin(-predicted_psi);
             waypoints_y[i] = transf_x *sin(-predicted_psi)+transf_y*cos(-predicted_psi);
          }

          //fit points to polynomial curve of 3rd order
          auto coeffs = polyfit(waypoints_x, waypoints_y, 3);


          //Determining Cross Track Error= determining polynomial f(x) at x and substract y.
          double cte = polyeval(coeffs, 0);
          //compute at px=0 and with psi=0: 
          //epsi = psi -atan(coeffs[1] + 2 * px * coeffs[2] + 3*coeffs[3] * pow(px,2));
          double epsi = -atan(coeffs[1]);
 
          //Define State vector in Car Coordinates and call Solve method of mpc Class
          Eigen::VectorXd state(6);
          state << 0, 0, 0, predicted_v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);

          json msgJson;
          //send steering and throttle values solutions out of solver back to the simulator
          //division by deg2rad(25) to keep values between [-1, 1]
          msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf);
          msgJson["throttle"] = vars[1];

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (size_t i = 2; i < vars.size(); i++)
          {
             if(i%2 == 0)
             {
                mpc_x_vals.push_back(vars[i]);
             }
             else
             {
                mpc_y_vals.push_back(vars[i]);
             }
          }
          //Adding (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //Plotting 20 points by increments of 3
          double inc = 3;
          int num_pts = 20;

          for (int i = 1; i < num_pts; i++)
          {
             next_x_vals.push_back(inc*i);
             next_y_vals.push_back(polyeval(coeffs, inc*i));
          }
          //adding (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(100));
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
