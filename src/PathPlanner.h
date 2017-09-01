//
// Created by Artur Olszak on 27/08/2017.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>
using namespace std;

class PathPlanner {
  
public:
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  
  void getNextWayPoints(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> previous_path_x, vector<double> previous_path_y, double car_s, double end_path_s, double car_speed, vector<vector<double>> sensor_fusion, double car_x, double car_y, double car_yaw);
  
private:
  const double large_initial_value = 500.0;
  const double car_length = 4.5;
  const double speed_limit = 49.5;
  const double max_speed_safe_distance_head = 30.0;
  const double close_distance = 50.0;
  const double max_speed_safe_distance_turn = 3.0;
  const double planning_time_interval = 0.02;
  const double lane_width = 4.0;
  const double lane_count = 3;
  const double acceleration_rate = 0.4;
  const double deceleration_rate = 0.1;
  
  int current_lane_index = 1;
  double current_velocity = 0.0;
  
  double deg2rad(double x);
  double pi();
  vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
  
  int getLaneFromFrenet(double frenet_d);
  
};


#endif //PATH_PLANNING_PATHPLANNER_H
