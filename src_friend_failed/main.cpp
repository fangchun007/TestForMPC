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
string hasData(string s)
{
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
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);
  for (int i = 0; i < xvals.size(); i++)
  {
    A(i, 0) = 1.0;
  }
  for (int j = 0; j < xvals.size(); j++)
  {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }
  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Transfer from map/global coordinate system to vehicle/local coordinate system.
// In the vehicle/local system, the position px = py = 0, and orientation psi = 0,
// which can simplify future calculations.
void globalToLocal(vector<double> &ptsx, vector<double> &ptsy, double px, double py,
                   double psi, Eigen::VectorXd &xvals, Eigen::VectorXd &yvals)
{
  for (size_t i=0; i<ptsx.size(); i++)
  {
    double dx = ptsx[i] - px;
    double dy = ptsy[i] - py;
    xvals[i] = dx * cos(-psi) - dy * sin(-psi);
    yvals[i] = dx * sin(-psi) + dy * cos(-psi);
  }
}

int main() {
  uWS::Hub h;
  // This is the length from front to CoG that has a similar radius.
  double Lf = 2.67;
  double dt = 0.1;
  // steps
  int N = 10;
  // MPC is initialized here!
  MPC mpc(N,dt,Lf);
    
  // Set a variable to save the previous time stamp.
  // This is used to estimate the latency
  std::chrono::time_point<std::chrono::system_clock> time_pre;
  
  bool latency_init = true;
  
  h.onMessage([&mpc, &Lf, &N, &time_pre, &latency_init](uWS::WebSocket<uWS::SERVER> ws,
                                                        char *data, size_t length,
                                                        uWS::OpCode opCode)
              {
                // "42" at the start of the message means there's a websocket message event.
                // The 4 signifies a websocket message
                // The 2 signifies a websocket event
                string sdata = string(data).substr(0, length);
                //cout << sdata << endl;
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
                      
                      //std::cout << "speed: " << v << std::endl;
                      // Convert speed from mph to m/s
                      v *= 0.44704;
                      
                      // Obtain current actuator values [delta, a] = ["steering_angle","throttle"],
                      // which will be used in calibration of latency
                      double delta0 = j[1]["steering_angle"];
                      
                      //std::cout << "steering angle: " << delta0 << std::endl;
                      
                      double a0     = j[1]["throttle"];
                      
                      std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - time_pre;
                      double latency_pre = elapsed_seconds.count();
                      std::cout << "real latency: " << latency_pre << std::endl;
                      
                      time_pre = std::chrono::system_clock::now();
                      
                      // Transform from global map system to local vehicle system so that the life is easier.
                      size_t n_pts = ptsx.size();
                      Eigen::VectorXd xvals(n_pts);
                      Eigen::VectorXd yvals(n_pts);
                      globalToLocal(ptsx, ptsy, px, py, psi, xvals, yvals);
                      // Fit for the reference line
                      Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);
                      // Calculate the cross track error in vehicle's coordinate system.
                      double cte = polyeval(coeffs, 0);
                      // Calculate the epsi in vehicle's coordinate system
                      double epsi = -atan(coeffs[1]);
                      // Calculate the new start state after the latency
                      // Recall in the local vehicle system, we have px = py = psi = 0, v=v
                      // We use the same equations of the model to obtain the new state:
                      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
                      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
                      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
                      // v_[t+1] = v[t] + a[t] * dt
                      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
                      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
                      //double latency = 0.1 + 0.05; // TUNE LATER
                      // Handle latency
                      double latency;
                      if (!latency_init)
                      {
                        latency = latency_pre;
                      }
                      else
                      {
                        std::cout << "Latency initialization!" << std::endl;
                        latency = 0.15;
                        latency_init = false;
                      }
                      if (latency >= 0.25) { latency = 0.25; }
                      std::cout << "latency used: " << latency << std::endl;
                      
                      double x0      = v * latency;
                      double y0      = 0;
                      double psi0    = - v / Lf * delta0 * latency;
                      double v0      = v + a0 * latency;
                      double cte0    = cte + v * sin(epsi) * latency;
                      double epsi0   = epsi - v / Lf * delta0 * latency;
                      Eigen::VectorXd state(6);
                      state << x0, y0, psi0, v0, cte0, epsi0;
                      // Use MPC to obtain a decent steering angle and throttle.
                      // Both are in between [-1, 1].
                      vector<double> pred_info = mpc.Solve(state, coeffs);
                      // Recall the first two components contain actuation values [steer_value, throttle_value],
                      // followed with N
                      double steer_value    = pred_info[0];
                      double throttle_value = pred_info[1];
                      
                      json msgJson;
                      // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                      // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                      msgJson["steering_angle"] = steer_value/deg2rad(25);
                      //std::cout << "Steering Angle: " << steer_value << std::endl;
                      msgJson["throttle"] = throttle_value;
                      
                      
                      //Display the MPC predicted trajectory
                      //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                      // the points in the simulator are connected by a Green line
                      vector<double> mpc_x_vals(N);
                      vector<double> mpc_y_vals(N);
                      /*
                       for (size_t t=0; t<N; t++)
                       {
                       mpc_x_vals[t] = pred_info[t+2];
                       mpc_y_vals[t] = pred_info[t+N+2];
                       }*/
                      
                      for (size_t t=2; t<pred_info.size(); t+=2)
                      {
                        mpc_x_vals.push_back(pred_info[t]);
                        mpc_y_vals.push_back(pred_info[t+1]);
                      }
                      
                      msgJson["mpc_x"] = mpc_x_vals;
                      msgJson["mpc_y"] = mpc_y_vals;
                      //Display the waypoints/reference line
                      //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                      // the points in the simulator are connected by a Yellow line
                      vector<double> next_x_vals(n_pts);
                      vector<double> next_y_vals(n_pts);
                      for (size_t t=0; t<n_pts; t++)
                      {
                        next_x_vals[t] = xvals[t];
                        next_y_vals[t] = yvals[t];
                      }
                      
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
