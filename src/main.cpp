#include <fstream>
#include <uWS/uWS.h>
#include <iomanip>
#include <thread>
#include <vector>

#include "json.hpp"

#include "coordinate_utils.h"
#include "log_utils.h"
#include "Route.h"

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
  //auto sdata = string(data).substr(0, length);
  //cout << sdata << endl;
  if (length && length > 2 && data[0] == '4' && data[1] == '2') {

    // Checks if the SocketIO event has JSON data.
    // If there is data the JSON object in string format will be returned,
    // else the empty string "" will be returned.
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



        // log event
        cout << ts_ms_str() << "IN s="<<car_s<<" d="<<car_d<< " yaw="<<car_yaw <<" v="<<car_speed << endl;



        // plan path
        vector<double> next_x_vals;
        vector<double> next_y_vals;


        // straight line
//            double dist_inc = 0.5;
//            for (int i = 0; i < 50; i++) {
//              next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
//              next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
//            }

        // constant angle and velosity
        double pos_x;
        double pos_y;
        double angle;
        int path_size = previous_path_x.size();

        for(int i = 0; i < path_size; i++)
        {
          next_x_vals.push_back(previous_path_x[i]);
          next_y_vals.push_back(previous_path_y[i]);
        }

        if(path_size == 0)
        {
          pos_x = car_x;
          pos_y = car_y;
          angle = deg2rad(car_yaw);
        }
        else
        {
          pos_x = previous_path_x[path_size-1];
          pos_y = previous_path_y[path_size-1];

          double pos_x2 = previous_path_x[path_size-2];
          double pos_y2 = previous_path_y[path_size-2];
          angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
        }

        double dist_inc = 1.5;
        for(int i = 0; i < 50-path_size; i++)
        {
          next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi() / 100)));
          next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi() / 100)));
          pos_x += (dist_inc)*cos(angle+(i+1)*(pi() / 100));
          pos_y += (dist_inc)*sin(angle+(i+1)*(pi() / 100));
        }




        // send control message back to the simulator
        json msgJson;
        msgJson["next_x"] = next_x_vals;
        msgJson["next_y"] = next_y_vals;
        auto msg = "42[\"control\"," + msgJson.dump() + "]";

        // sleep for 100 ms, to match real cars latency
        this_thread::sleep_for(chrono::milliseconds(100));

        cout << ts_ms_str() << "OUT" << endl;
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
















































































