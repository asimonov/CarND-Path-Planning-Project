#include <fstream>
#include <uWS/uWS.h>
#include <iomanip>
#include <thread>
#include <vector>

#include "json.hpp"

#include "log_utils.h"
#include "coordinate_utils.h"

#include "Route.h"
#include "Car.h"
#include "PathPlanner.h"

using namespace std;
using json = nlohmann::json;


// global variable. initialised in main. used in onMessage
Route route;


// process telemetery event
void onMessage(uWS::WebSocket<uWS::SERVER> ws,
               char *data,
               size_t length,
               uWS::OpCode opCode)
{
  // "42" at the start of the message means there's a websocket message event.
  // The 4 signifies a websocket message
  // The 2 signifies a websocket event
  if (length && length > 2 && data[0] == '4' && data[1] == '2') {

    // Checks if the SocketIO event has JSON data.
    string data_s = string(data);
    string s = "";
    auto found_null = data_s.find("null");
    auto b1 = data_s.find_first_of("[");
    auto b2 = data_s.find_first_of("}");
    if (b1 != string::npos && b2 != string::npos) {
      s = data_s.substr(b1, b2 - b1 + 2);
    }
    if (s != "") {
      auto j = json::parse(s);

      string event = j[0].get<string>();

      if (event == "telemetry") {
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

        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        auto sensor_fusion = j[1]["sensor_fusion"];
        //["sensor_fusion"] A 2d vector of cars and then that car's
        // [car's unique ID,
        //  car's x position in map coordinates,
        //  car's y position in map coordinates,
        //  car's x velocity in m/s,
        //  car's y velocity in m/s,
        //  car's s position in frenet coordinates,
        //  car's d position in frenet coordinates.]

        // log event
        cout << ts_ms_str() << "IN  n="<<setw(5)<<previous_path_x.size()
             <<" x  ="<<setw(8)<<car_x  <<" y  ="<<setw(8)<<car_y
             <<" s="<<car_s<<" d="<<car_d
             <<" yaw="<<car_yaw <<" v="<<car_speed
             << endl;

        // plan trajectory
        double dt_s = 0.02; // discretisation time length, in seconds
        double time_horizon_s = 2.0; // planning time horizon, in seconds
        double target_speed_ms = mph2ms(47.0); // target speed in meter/second
        double max_acceleration = 10.0; // maximum acceleration, in m/s2
        double max_jerk = 10.0; // maximum jerk, in m/s3
        int planned_n = previous_path_x.size();
        Trajectory tr; // final trajectory

        if (planned_n == 0)
        {
          // there is no previous path. use current car state to start planning from scratch
          Car c(car_x, car_y, car_yaw, car_speed, Trajectory());

          Trajectory tr_left_lane_frenet;

          auto fr = route.get_frenet(c.getX(), c.getY(), c.getYaw());
          fr[1] += 2.0; // be in middle of left lane
          tr_left_lane_frenet.add(fr[0], fr[1]);
          double s_prev = fr[0];
          double s_prev_dot = 0;
          double s_prev_ddot = 0;
          double d_prev = fr[1];
          double d_prev_dot = 0;
          double d_prev_ddot = 0;
          double t_total = 0;

          Trajectory waypoints = route.get_next_segments(c.getX(), c.getY(), c.getYaw(), 15);
          int n_wp = waypoints.getX().size();
          for (int i=0; i<n_wp; i++) {
            fr = route.get_frenet(waypoints.getX()[i], waypoints.getY()[i], c.getYaw());
            fr[1] += 2.0; // be in middle of left lane
            double s_new = fr[0];
            double d_new = fr[1];

            double s_dist = s_new - s_prev;
            assert(s_dist>0);
            double delta_v = target_speed_ms - s_prev_dot;
            double used_acceleration = max_acceleration/2.0;
            double T = abs(delta_v) / used_acceleration; // time horizon over which we can get to desired speed
//            if (T>time_horizon_s) {
//              T = time_horizon_s;
//            }
            double s_new_dot = s_prev_dot + T*used_acceleration;
            double s_new_ddot = 0.0;
            JerkMinimizingPolynomial jmt_s({s_prev, s_prev_dot, s_prev_ddot}, {s_new, s_new_dot, s_new_ddot}, T);

            int n_steps = T/dt_s;
            for (int j=0;j<n_steps; j++)
            {
              double s = jmt_s.eval(j*dt_s);
              tr_left_lane_frenet.add(s, d_new);
            }

            s_prev = s_new;
            s_prev_dot = s_new_dot;
            s_prev_ddot = s_new_ddot;
            d_prev = d_new;
            d_prev_dot = 0;
            d_prev_ddot = 0;
            t_total += T;
            if (t_total > time_horizon_s)
              break;
          }

          n_wp = tr_left_lane_frenet.getX().size();
          for (int i=0; i<n_wp; i++) {
            auto xy = route.get_XY(tr_left_lane_frenet.getX()[i], tr_left_lane_frenet.getY()[i]);
            tr.add(xy[0],xy[1]);
          }
        } else
        {
          // there is already trajectory that car follows. lets plan from its end
          Trajectory prev_tr(previous_path_x, previous_path_y);
          Car c(car_x, car_y, car_yaw, car_speed, prev_tr);
          tr = prev_tr;
        }

//        Trajectory tr = route.get_next_segments(c., 15);
//
//        if (planned_n < time_horizon_s/dt_s) {
//          Trajectory tr_d;
//          for (int i=0; i<tr.getX().size(); i++) {
//            auto fr = route.get_frenet(tr.getX()[i], tr.getY()[i], c.getYaw());
//            fr[1] += 2.0; // be in middle of left lane
//            auto xy = route.get_XY(fr[0], fr[1]);
//            tr_d.add(xy[0], xy[1]);
//          }
//          tr_d.respace_at_constant_speed(dt_s, target_speed_ms);
//          tr = tr_d;
//        }
//        else
//          tr = prev_tr;

        // send control message back to the simulator
        json msgJson;
        msgJson["next_x"] = tr.getX();
        msgJson["next_y"] = tr.getY();
        auto msg = "42[\"control\"," + msgJson.dump() + "]";

        // sleep for 100 ms, to match real cars latency
        this_thread::sleep_for(chrono::milliseconds(100));

        int n = tr.getY().size();
        cout << ts_ms_str() << "OUT n="<<setw(5)<<n
             << " x_s="<<setw(8)<<tr.getX()[0] <<" y_s="<<setw(8)<<tr.getY()[0]
             << " x_f="<<tr.getX()[n-1] << " y_f="<<tr.getY()[n-1]
             << endl;
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } else {
      // Manual driving
      std::string msg = "42[\"manual\",{}]";
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  }
}




// main.
// just setup webserver listening and provide handlers to do the job. then start listen/reply loop
int main() {
  uWS::Hub h;

  // read waypoints data from file
  route.read_data("../data/highway_map.csv");

  h.onMessage(onMessage);

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
















































































