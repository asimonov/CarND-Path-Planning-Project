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
#include "BehaviourPlanner.h"
#include "PathPlanner.h"

using namespace std;
using json = nlohmann::json;


// global variables.
// Route (set of waypoints and code to spline them) is initialised in main. used in onMessage later.
Route                    g_route;
// planned state and final planned s coordinate are saved in onMessage after each planning cycle
std::pair<Maneuvre, int> g_planned_state;
std::vector<double>      g_final_frenet = {-1000000,0};


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
        double car_s_sim = j[1]["s"];
        double car_d_sim = j[1]["d"];
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


        // log inputs
        auto fr = g_route.get_frenet(car_x, car_y, car_yaw);
        // replace frenet with my own frenet estimates
        double car_s_mine = fr[0];
        double car_d_mine = fr[1];
        cout << ts_ms_str() << "IN  n="<<setw(5)<<previous_path_x.size()
             <<" x  ="<<setw(8)<<car_x  <<" y  ="<<setw(8)<<car_y
             <<" s="<<car_s_sim<<"(mine: "<<car_s_mine<<")"<<" d="<<car_d_sim<<"(mine:"<<car_d_mine<<")"
             <<" yaw="<<car_yaw <<" v="<<car_speed
             << endl;


        // planning constants
        const double dt_s = 0.02; // discretisation time length, in seconds
        const int    num_lanes = 3; // number of lanes we have
        const double lane_width = 4.0; // highway lane width, in meters
        const double max_speed = mph2ms(50.0); // max speed in meter/second
        const double target_speed = mph2ms(44.3); // target speed in meter/second
        const double max_acceleration = 10.0; // maximum acceleration, in m/s2
        const double max_jerk = 10.0; // maximum jerk, in m/s3

        // define existing trajectory
        Trajectory in_traj(previous_path_x, previous_path_y, dt_s);
        if (g_final_frenet[0]>-100)
          in_traj.storeFinalFrenet(g_final_frenet[0], g_final_frenet[1]); // hack to avoid jerks from xy->frenet->xy imperfect conversion
        else
          in_traj.storeFinalFrenet(car_s_sim, car_d_sim); // just use what came from simulator for first iteration

#ifdef DEBUG
        // save debug trajectory info to a file
        std::stringstream ss;
        ss << "trace_" << std::setw(6) << std::setfill('0') << ts_ms() << "_0_in_traj";
        in_traj.dump_to_file(ss.str());
#endif

        // define ego car as of where simulator is now
        double car_acceleration = 0.0;
        int car_lane = car_d_mine / lane_width;
        Car ego_sim(Car::getEgoID(),
                car_x, car_y, deg2rad(car_yaw), car_s_mine, car_d_mine,
                car_lane, car_speed, car_acceleration,
                target_speed, max_speed, max_acceleration);

        // define ego car object, as of end of retained trajectory
        if (in_traj.getSize())
        {
          auto xy = in_traj.getFinalXY();
          car_x = xy[0];
          car_y = xy[1];
          car_yaw = in_traj.getFinalYaw();
          car_speed = in_traj.getFinalSpeed();
          car_acceleration = in_traj.getFinalAcceleration();
          // what it should be. BUT we have saved it, so just restore
//          auto fr = g_route.get_frenet(car_x, car_y, car_yaw);
//          car_s = fr[0];
//          car_d = fr[1];
          car_s_mine = g_final_frenet[0];
          car_d_mine = g_final_frenet[1];
        }
        car_lane = car_d_mine / lane_width;
        Car ego(Car::getEgoID(),
                car_x, car_y, deg2rad(car_yaw), car_s_mine, car_d_mine,
                car_lane, car_speed, car_acceleration,
                target_speed, max_speed, max_acceleration);
        double ego_time = in_traj.getTotalT();

        // process sensor fusion and define all other cars on the road
        vector<Car> other_cars;
        for (int i = 0; i < sensor_fusion.size(); i++) {
          int id = sensor_fusion[i][0];
          assert(id!=Car::getEgoID());
          double x = sensor_fusion[i][1];
          double y = sensor_fusion[i][2];

          double vx = sensor_fusion[i][3];
          double vy = sensor_fusion[i][4];
          double v = sqrt(vx*vx+vy*vy); // assuming speed magnitude is all in direction of s, none in d
          double yaw = atan2(vy,vx);

          double s = sensor_fusion[i][5];
          double d = sensor_fusion[i][6];
          auto fr = g_route.get_frenet(x, y, yaw);
          s = fr[0];
          d = fr[1];
          int lane = d / lane_width;
          if (lane < 0 || lane>5)
            continue; // ignore cars outside of driveable area
          double a = 0.0;
          double v_target = v;
          Car other_car_at_zero(id, x, y, yaw, s, d, lane, v, a, v_target, max_speed, max_acceleration);
          // now create state of other cars as of end time of ego trajectory
          other_cars.push_back(other_car_at_zero.advance(ego_time));
        }

        // see if we want to plan ahead on this message
        auto fr_now =     g_route.get_frenet(ego_sim.getX(), ego_sim.getY(), ego_sim.getYaw());
        auto fr_planned = g_route.get_frenet(ego.getX(),     ego.getY(),     ego.getYaw());
        double re_planning_s_horizon = 30.0; // when do we extend the planned route?
        Trajectory out_tr = in_traj;

        if ( fr_planned[0] - fr_now[0] < re_planning_s_horizon) {
          // plan maneuvre using behaviour planner
          BehaviourPlanner bp(num_lanes, lane_width, ego, other_cars);
          const double time_horizon_s = 5.0; // max planning time horizon, in seconds
          Car ego_plan = bp.plan(time_horizon_s);
          auto new_state = ego_plan.getState();
          double planning_time = ego_plan.get_target_time();
          double cost = bp.calculate_cost(ego_plan);

          cout << "Old State: " << g_planned_state.first << ", target lane = " << g_planned_state.second << endl;
          cout << "behaviour cost: " << cost << endl;
          cout << "New State: " << new_state.first << ", target lane = " << new_state.second << ", target time = "
               << planning_time << " secs" << endl;

#ifdef DEBUG
          // dump sensor fusion trace
          ss.str("");
          ss << "trace_" << std::setw(6) << std::setfill('0') << ts_ms() << "_1_sf";
          for (auto c : other_cars)
          {
            c.dumpToStream(ss.str()); // sensor fusion as of end of planned trajectory, so far
          }
          // dump planned ego
          ss.str("");
          ss << "trace_" << std::setw(6) << std::setfill('0') << ts_ms() << "_2_ego";
          ego_plan.dumpToStream(ss.str());
#endif


          // plan final trajectory (x,y points spaced at dt_s) using the maneuvre
          JMTPlanner planner;
          out_tr = planner.extendTrajectory(ego_plan, in_traj, g_route, planning_time, lane_width, target_speed, max_speed, max_acceleration, max_jerk);
          // wrap final planned s around the track
          fr = out_tr.getFinalFrenet();
          if (fr[0] > g_route.get_max_s())
            fr[0] -= g_route.get_max_s();
          out_tr.storeFinalFrenet(fr[0], fr[1]);
          // store final frenet coordinates for next round of planning
          g_final_frenet = out_tr.getFinalFrenet();
          cout << "New Planned s: "<<g_final_frenet[0] << endl;

#ifdef DEBUG
          ss.str("");
          ss << "trace_" << std::setw(6) << std::setfill('0') << ts_ms() << "_3_out_traj";
          out_tr.dump_to_file(ss.str());
#endif

          g_planned_state = new_state;

          // sleep for 100 ms, to match real cars latency
          this_thread::sleep_for(chrono::milliseconds(100));

          int n = out_tr.getY().size();
          cout << ts_ms_str() << "OUT n="<<setw(5)<<n
               << " x_s="<<setw(8)<<out_tr.getX()[0] <<" y_s="<<setw(8)<<out_tr.getY()[0]
               << " x_f="<<out_tr.getX()[n-1] << " y_f="<<out_tr.getY()[n-1]
               << endl;
        }





        // send control message back to the simulator
        json msgJson;
        msgJson["next_x"] = out_tr.getX();
        msgJson["next_y"] = out_tr.getY();
        auto msg = "42[\"control\"," + msgJson.dump() + "]";

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
  g_route.read_data("../data/highway_map.csv");
  // debug output
  ofstream f("track_debug.csv", ofstream::out);
  double d = 6.0;
  auto prev_xy = g_route.get_XY(0.0, d);
  double ds = 0.5;
  for (double s=ds; s<g_route.get_max_s()+2.; s=s+ds)
  {
    auto xy = g_route.get_XY(s, d);
    double yaw = angle(prev_xy[0], prev_xy[1], xy[0], xy[1]);
    auto fr = g_route.get_frenet(xy[0], xy[1],yaw);
    auto xy2 = g_route.get_XY(fr[0],fr[1]);
    f<<s<<" "<<xy[0]<<" "<<xy[1]<<" "<<rad2deg(yaw)<<" "<<fr[0]<<" "<<fr[1]<<" "<<xy2[0]<<" "<<xy2[1]<<endl;
    prev_xy = xy;
  }
  f.close();

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
















































































