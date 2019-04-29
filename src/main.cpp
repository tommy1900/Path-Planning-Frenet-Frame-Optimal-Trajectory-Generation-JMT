#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <utility>      // std::pair, std::make_pair
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
// #include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "trajectory.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
// move the hleper functions here


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double max_s = 6945.554;

// Global
tk::spline splineX;
tk::spline splineY;
tk::spline splinedX;
tk::spline splinedY;

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}
void setupGetXY( const vector<double> &maps_x,
                 const vector<double> &maps_y,
                 const vector<double> &maps_s,
                 const vector<double> &maps_dx,
                 const vector<double> &maps_dy){


  splineX.set_points(maps_s,maps_x);
  splineY.set_points(maps_s,maps_y);
  splinedX.set_points(maps_s,maps_dx);
  splinedY.set_points(maps_s,maps_dy);

}

vector<double> getXY(double s, double d) {
  s = fmod(s, max_s);
  double x = splineX(s) + d * splinedX(s);
  double y = splineY(s) + d * splinedY(s);
  return {x,y};
}





bool collisionCheck(Traj& s_traj, Traj& d_traj, int predict_horizon, vector<vector<double>> sensor_fusion){

  double Tf = s_traj.duration; // since s movement has to >= d

  for(double t = 0; t < Tf; t+= 0.02){
    // sensor_fusion results at time t
    vector<double> ego_xy = getXY(s_traj.getDist(t),d_traj.getDist(t));
    // check the acceleration and jerk the same time
    if(abs(d_traj.getAcel(t)) > 10.0 || abs(d_traj.getJerk(t)) > 10.0 || abs(s_traj.getAcel(t)) > 10.0 || abs(s_traj.getJerk(t)) > 10.0) return false;

    for(int i = 0; i < sensor_fusion.size(); ++i){
      double near_x = sensor_fusion[i][1] + sensor_fusion[i][3] * (t + predict_horizon*0.02);
      double near_y = sensor_fusion[i][2] + sensor_fusion[i][4] * (t + predict_horizon*0.02);

      if(abs(near_x - ego_xy[0]) <= 3 && abs(near_y - ego_xy[1]) <= 1){
        // the box can not be too big, or otherwise won't find solution when too close to other vehicles
        //cout << "ego xy" << ego_xy[0] <<"," << ego_xy[1];
        //cout<< "near cars xy: "<<near_x << "," << near_y;
        //cout<< "collision!" <<endl;
        return false;
      }

    }
  }

  return true;
}


























int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;


  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  // Init some variables
  // pre define some variables
  vector<double> s_BC; // boundary conditions for s
  vector<double> d_BC; // boundary conditions for d
  const double speed_limit = 22; // max speed in m/s
  const double acc_limit = 10.0; // max acceleration in m/s^2
  const double Jerk_limit = 10.0; // max Jerk in m/s^3
  const int path_horizon = 250; // size of path to pass to simulator for each new path
  const int replan_period = 20; // size of path already driven after which a new path must be planned
  const double d_duration_min = 1.5; // sec
  const double s_duration_min = 1.5; // sec
  const double safe_space = 10;

  const double predict_horizon = 20;
  const double delta_t = 0.7;

  double lateral_w = 2, longitudinal_w = 1;
  double prev_plan_size = 0;
  pair<pair<Traj,Traj>,double> bestTraj;
  vector<vector<vector<double>>> lv(3);
  //Traj s_traj,d_traj;

  // Init the vehicle object
  setupGetXY(map_waypoints_x,
              map_waypoints_y,
              map_waypoints_s,
              map_waypoints_dx,
              map_waypoints_dy);

  h.onMessage([&delta_t,&predict_horizon, &safe_space,&s_duration_min,&d_duration_min,&prev_plan_size, &path_horizon,&replan_period,&speed_limit,&s_BC,&d_BC,&bestTraj, &lv, &lateral_w, &longitudinal_w]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

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
          double car_speed = 0.44704*double(j[1]["speed"]);
          // cout<<"current speed: "<< car_speed * 2.23694 <<endl;
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"]; // uniqueID x y vx vy s d
          json msgJson;

          bestTraj.second = 9999999999; // clear the cost for each loop
          int unused_plan_size = previous_path_x.size();
          // get the leading vehicles
          for(auto sf:sensor_fusion){
            if(sf[6] >=0 && sf[6] <13 && sf[5] <= car_s + 40 && sf[5] >= car_s - 40) lv[int(sf[6])/4].push_back(sf); // need to init lv[] first
          }

          // prepare the next path vector
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start the planing
          if(unused_plan_size == 0){
           // set intial s and d conditions
           s_BC.clear();
           s_BC.push_back(car_s);
           s_BC.push_back(0.);
           s_BC.push_back(0.);
           s_BC.push_back(speed_limit);
           s_BC.push_back(0.);


           d_BC.clear();
           d_BC.push_back(car_d);
           d_BC.push_back(0.);
           d_BC.push_back(0.);
           d_BC.push_back(car_d);
           d_BC.push_back(0.);
           d_BC.push_back(0.);

           Traj s_traj = Traj(s_BC, path_horizon*0.02);
           Traj d_traj = Traj(d_BC, path_horizon*0.02);
           double cost = lateral_w * d_traj.getCost() + longitudinal_w * s_traj.getCost();
           bestTraj = {{s_traj,d_traj},cost}; // fist init of the best traj


           for (int i = 0; i < path_horizon; i++)
           {
             //generate next points using selected trajectory with a time pace of 0.02 seconds
             double next_s = bestTraj.first.first.getDist(i*0.02);
             double next_d = bestTraj.first.second.getDist(i*0.02);
             // convert  to  global coordinates
             vector<double> sxy = getXY(next_s, next_d);
             // pass points to simulator
             next_x_vals.push_back(sxy[0]);
             next_y_vals.push_back(sxy[1]);
           }
           prev_plan_size = path_horizon;
         }
         else{
             // CURRENT PATH IS VALID UNTIL NEXT PLAN SO CHECK TIME ELAPSED FROM PREVIOUS PLANNING
            double gone_path = prev_plan_size - unused_plan_size;
            //cout<< "prev_plan_size: "<<prev_plan_size;
            //cout<< " unused_plan_size: " << unused_plan_size;
            //cout<< " gone: " << gone_path<<endl;
            if ( gone_path >= replan_period) { // start the replan, but plan for current + n timestamp for smooth_path purpose
              // replan for the states after a certain predict_horizon
              cout<< "replan !" <<endl;
              prev_plan_size = 0;
              // predict_horizon steps after the gone
              for(int i = 0; i < predict_horizon ; i++)
               {
                 next_x_vals.push_back(previous_path_x[i]);
                 next_y_vals.push_back(previous_path_y[i]);
                 prev_plan_size++;
               }

              // ego car states after timeT(predict_horizon steps after the gone)
               double timeT =(predict_horizon)*0.02 + 0.01;
               car_s = bestTraj.first.first.getDist(timeT); // timeT after now
               car_d = bestTraj.first.second.getDist(timeT);

               double car_s_speed = bestTraj.first.first.getVel(timeT);
               double car_d_speed = bestTraj.first.second.getVel(timeT);
               double car_s_accel = bestTraj.first.first.getAcel(timeT);
               double car_d_accel = bestTraj.first.second.getAcel(timeT);
              // replan based on the states at timeT
               s_BC.clear();
               d_BC.clear();
               double t_f =  (path_horizon - (gone_path+predict_horizon))*0.02;
               vector<double> lanes = {2., 6., 9.5};
               vector<vector<Traj>> lateralTrajs(3);
               // the lateral variables: final lanes and durations
               // control the duration inbetween 2 to 5
               for (int i=0 ; i < 3 ; i++) { // three different lanes
                // laterl durations: 3.7 2.6 1.5
                 for (double t = 1.5; t <= t_f ; t+=delta_t) { // different time durations
                   d_BC = {car_d,car_d_speed,car_d_accel,lanes[i],0.0,0.0};
                   Traj d_traj = Traj(d_BC, t);
                   lateralTrajs[i].push_back(d_traj);
                 }
               }

               // {s_target(T) + delta_s, s_target_dot(T), s_target_dot_dot(T) , T}
               // create s trajectories using different time of manouver and final speed
               vector<vector<Traj>> longitudinalTraj(3);
               // if the longitudinal path are free ended, we only have 5 boundary conditions,
               // the only variable to change will be the durations and final vel based on the duration
               for(int i = 0; i< 3; ++i){ // 3 lane options
                 std::sort(lv[i].begin(),lv[i].end(),[](const vector<double> &a,const vector<double>&b){return a[5] > b[5];}); // compare s position
                 double s_lv_t = lv[i].empty()? 0:lv[i][0][5] + sqrt(lv[i].back()[3]*lv[i].back()[3] + lv[i].back()[4]*lv[i].back()[4])*predict_horizon*0.02; // leading vehicle position at timeT
                 if(lv[i].size() == 0 || s_lv_t + 5 <= car_s){ // no leading vehicle
                   // change the to that lane while trying to reach to the desired speed
                   // control the longitudinal duration  inbetween 1.5 - 5
                   for (double t = 2; t <= t_f ; t+=delta_t) { // three different time duration
                     double s_dot_t =  car_s_speed + (speed_limit - car_s_speed)*(t/t_f);
                     s_BC = {car_s,car_s_speed,car_s_accel,s_dot_t,0};
                     Traj s_traj = Traj(s_BC, t);
                     longitudinalTraj[i].push_back(s_traj);
                   }
                 }else { // 跟车的几个 case 做的不太好
                   // 在有很多车的时候出现了 超车倾向 过大， pass or follow 之间的 decision 不明显
                   // 试一试 调 cost function
                   // 而且还是会有碰撞发生， 说明 collision check 也有问题
                   // 经检查， 碰撞状况的 发生 是因为没有选出来 best traj 导致的， 我们需要一个while loop 来确保 best traj 被选出来
                   // 也许因为计划的 traj 都 远了 ，没有近距离的， 可以通过while loop 来逐渐缩短 duration 进而缩短 travel distance

                    // if leading vehicles ahead in same lane , just follow it
                   // if the leading vehicles at left or right lanes = lane change maybe
                   // pass it or follow it or meger into it if more than one vehicle
                   double vx = lv[i][0][3];
                   double vy = lv[i][0][4];
                   double s_lv_dot_dot = 0;
                   double s_lv_dot = sqrt(vx*vx + vy*vy);
                   for(double t = 2; t <= t_f ; t+=delta_t){
                     //adapt speed:
                     s_BC = {car_s,car_s_speed,car_s_accel,s_lv_dot,0};
                     Traj s_traj = Traj(s_BC, t);
                     longitudinalTraj[i].push_back(s_traj);
                   }
                 }
               }

                         /*
                         double s_lv = lv[i][0][5] +  * (duration + predict_horizon*0.02); // position of the vehicle after t duration
                         // follow
                         s_BC = {car_s,car_speed,0,s_lv - safe_space,s_lv_dot,0};
                         Traj s_traj = Traj(s_BC, duration);
                         longitudinalTraj[i].push_back(s_traj);
                         // pass
                         s_BC = {car_s,car_speed,0,s_lv + safe_space,s_lv_dot,0};
                         s_traj = Traj(s_BC, duration);
                         longitudinalTraj[i].push_back(s_traj);
                         */
                         //  ok 了 事实就是 这样通过sensor fusion 分开来判断 follow pass 的 mode 确实 可以减少大量计算
                         // 但是对 未来估计 和自身 在 frenet frame 位置的不稳定 会导致 很多的 spikes
                         // 同样 即使 减少了计算量， 还是需要collision check 的， 考虑到 body box 和 lv sudden lane changes 等因素
                         // 所以 简单一点实现的方法就是 通过 只 adapt traffic speed 然后 暴力算出所有可能性 再进行 collision check
                         // 牺牲运算时间 换来减少代码复杂性和结果稳定性


                     /*
                     else{ // merge for two or more vehicles
                       double vx = lv[i].back()[3];
                       double vy = lv[i].back()[4];
                       double s_lv_dot_dot = 0;
                       double s_lv_dot = sqrt(vx*vx + vy*vy);
                       double duration_reduction = (t_f - s_duration_min); // in seconds
                       for(int j = 1; j<=3; ++j){
                         double duration = t_f - duration_reduction/3*j;
                         double s_lv = lv[i].back()[5] + s_lv_dot * (duration + predict_horizon*0.02); // position of the vehicle after t duration
                         // merge
                         s_BC = {car_s,car_s_speed,0,s_lv + safe_space,s_lv_dot,0};
                         Traj s_traj = Traj(s_BC, duration);
                         longitudinalTraj[i].push_back(s_traj);
                       }*/





              // 现在 有了 d 和 s 的 JMT coeff 了， 接下来要写 合并 并且取 cost 的 function
              // get the cost-ranking
              // int solutionCnt = 0;
               for(int i = 0; i< 3; ++i){ // three lanes combination
                 for(auto laTraj:lateralTrajs[i]){
                   for(auto loTraj:longitudinalTraj[i]){
                     // if changing lane, and d movement takes longer to finish than s movement, abandon it
                    if(abs(car_d - laTraj.getDist(t_f)) > 2 && laTraj.duration > loTraj.duration) continue;
                    if(!collisionCheck(loTraj,laTraj,predict_horizon,sensor_fusion)) continue;
                    double cost = lateral_w * laTraj.getCost() + longitudinal_w * loTraj.getCost();
                    if(cost < bestTraj.second){
                      bestTraj = {{loTraj,laTraj},cost};
                      cout<< "a solution found" << endl;
                      //++solutionCnt;
                    }
                   }
                 }
               }

               for (int i = 0; i < path_horizon - (predict_horizon + gone_path); i++)
               {
                 //generate next points using selected trajectory with a time pace of 0.02 seconds
                 double next_s = bestTraj.first.first.getDist(i*0.02);
                 double next_d = bestTraj.first.second.getDist(i*0.02);
                 // convert  to  global coordinates
                 vector<double> sxy = getXY(next_s, next_d);

                 // pass points to simulator
                 next_x_vals.push_back(sxy[0]);
                 next_y_vals.push_back(sxy[1]);
                 prev_plan_size++;
               }


             } else {
               // NO PLANNING BECAUSE LAST PATH IS NOT EXPIRED
               for(int i = 0; i < previous_path_x.size(); i++)
               {
                 next_x_vals.push_back(previous_path_x[i]);
                 next_y_vals.push_back(previous_path_y[i]);
               }
             }
           }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          /*
          plt::named_plot("next wps ",s_traj, d_traj,"r--");
          plt::legend();
          plt::show();
          exit(1);
          */


        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
