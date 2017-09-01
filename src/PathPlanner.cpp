//
// Created by Artur Olszak on 27/08/2017.
//

#include "PathPlanner.h"

#include <math.h>
#include <iostream>
#include "spline.h"

using namespace std;

void PathPlanner::getNextWayPoints(vector<double> &next_x_vals, vector<double> &next_y_vals, vector<double> previous_path_x, vector<double> previous_path_y, double car_s, double end_path_s, double car_speed, vector<vector<double>> sensor_fusion, double car_x, double car_y, double car_yaw) {
  
  //
  // Update s coordinate, if there was previous path
  
  unsigned long previous_path_size = previous_path_x.size();
  if (previous_path_size > 0){
    car_s = end_path_s;
  }
  
  //
  // Higher level assumptions from sensor fusion data
  
  bool current_lane_is_blocked = false;
  double current_lane_distance_to_car = large_initial_value;
  double current_lane_velocity = large_initial_value;
  
  bool left_lane_can_turn = (current_lane_index > 0);
  double left_lane_distance_to_car = large_initial_value;
  double left_lane_velocity = large_initial_value;
  
  bool right_lane_can_turn = (current_lane_index < lane_count - 1);
  double right_lane_distance_to_car = large_initial_value;
  double right_lane_velocity = large_initial_value;
  
  //
  // Sensor fusion data analysis
  
  double safety_gap_ahead = (car_speed / speed_limit) * max_speed_safe_distance_head;
  double safety_gap_turn = (car_speed / speed_limit) * max_speed_safe_distance_turn + car_length;
  
  for (int i = 0; i < sensor_fusion.size(); i++) {
    
    //
    // Single car data

//    double car_id = sensor_fusion[i][0];                                      // Unique car id
//    double coordinate_x = sensor_fusion[i][1];                                // Global map X coordinate
//    double coordinate_y = sensor_fusion[i][2];                                // Global map Y coordinate
    double velocity_x = sensor_fusion[i][3];                                  // Velocity in X axis
    double velocity_y = sensor_fusion[i][4];                                  // Velocity in Y axis
    double frenet_s = sensor_fusion[i][5];                                    // Frenet S coordinate
    double frenet_d = sensor_fusion[i][6];                                    // Frenet D coordinate
    
    double velocity = sqrt(velocity_x*velocity_x+velocity_y*velocity_y);      // Resultant velocity
    double next_frenet_s = frenet_s + previous_path_size * planning_time_interval * velocity;
    double lane = this->getLaneFromFrenet(frenet_d);
    double distance_to_car = next_frenet_s - car_s;
  
    //
    // Analyse car position in relation to the autonomous car
    
    if (lane == current_lane_index) {
  
      //
      // Check is a car blocking current lane
      
      if (distance_to_car < safety_gap_ahead && distance_to_car >= 0.0) {
        
        current_lane_is_blocked = true;
        
        if (distance_to_car < current_lane_distance_to_car) { // Take only data about closest car
          current_lane_distance_to_car = distance_to_car;
          current_lane_velocity = velocity;
        }
        
      }
      
    } else {
  
      //
      // Get distance and velocity of closest cars on the sides
  
      if (distance_to_car > 0.0) {
        
        if (lane == current_lane_index - 1) {
          if (distance_to_car < left_lane_distance_to_car) {
            left_lane_distance_to_car = distance_to_car;
            left_lane_velocity = velocity;
          }
        }
  
        if (lane == current_lane_index + 1) {
          if (distance_to_car < right_lane_distance_to_car) {
            right_lane_distance_to_car = distance_to_car;
            right_lane_velocity = velocity;
          }
        }
        
      }
  
      //
      // Check is a car blocking turning left or right
      
      if ((next_frenet_s > (car_s-safety_gap_turn)) && ((next_frenet_s-(car_s-safety_gap_turn)) < (safety_gap_ahead))) {
  
        if (lane == current_lane_index - 1) {
          left_lane_can_turn = false;
        }
  
        if (lane == current_lane_index + 1) {
          right_lane_can_turn = false;
        }
        
      }
      
    }
  }
  
  //
  // Debug log
  
  cout << endl << "Car current lane: " << current_lane_index << endl
       << "Current: " << (!current_lane_is_blocked) << " - distance: " << current_lane_distance_to_car << endl
       << "Left: " << left_lane_can_turn << endl
       << "Right: " << right_lane_can_turn << endl;
  
  //
  // Behaviour change based on higher level assumptions
  
  if (current_lane_is_blocked) {
  
    if (left_lane_can_turn && right_lane_can_turn) {
      
      //
      // Choose more suitable lane to change
      
      if (right_lane_distance_to_car < close_distance && left_lane_distance_to_car < close_distance) {
        
        //
        // If cars on both sides are close choose the faster one to follow
        
        if (right_lane_velocity > left_lane_velocity) {
  
          current_lane_index++;
          
        } else {
  
          current_lane_index--;
          
        }
        
      } else {
        
        //
        // Choose lane with more free distance
  
        if (right_lane_distance_to_car > left_lane_distance_to_car) {
    
          current_lane_index++;
    
        } else {
    
          current_lane_index--;
    
        }
        
      }
  
    } else if (left_lane_can_turn) {
      
      //
      // Change to left lane
    
      current_lane_index--;
      
    } else if (right_lane_can_turn) {
      
      //
      // Change to right lane
    
      current_lane_index++;
      
    } else {
      
      //
      // Slow down and follow car upfront
      
      if (car_speed > current_lane_velocity or current_lane_distance_to_car < (safety_gap_ahead/2)) {
        
        double velocity_difference_rate = (car_speed - current_lane_velocity) / current_lane_velocity;
        current_velocity -= deceleration_rate * velocity_difference_rate;
        
      }
      
    }
    
  } else if (current_velocity < speed_limit) {
    
    //
    // Drive straight if there is no blocking car
    
    double velocity_difference_rate = (current_lane_velocity - car_speed)/current_lane_velocity;
    double acceleration_change = acceleration_rate * velocity_difference_rate;
    if (current_lane_distance_to_car < close_distance) {
      acceleration_change = acceleration_change * 0.1;
    }
    current_velocity += acceleration_change;
    
  }
  
  //
  // Trajectory calculating
  
  vector<double> points_x;
  vector<double> points_y;
  
  double initial_car_x = car_x;
  double initial_car_y = car_y;
  double initial_yaw = deg2rad(car_yaw);
  
  if (previous_path_size < 2) {
    
    //
    // Estimate points if there is not enough readings
    
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    
    points_x.push_back(prev_car_x);
    points_x.push_back(car_x);
    
    points_y.push_back(prev_car_y);
    points_y.push_back(car_y);
    
  } else {
    
    //
    // Add x and y values to the last points in the vectors previous path vectors
    
    initial_car_x = previous_path_x[previous_path_size-1];
    initial_car_y = previous_path_y[previous_path_size-1];
    
    double ref_x_prev = previous_path_x[previous_path_size-2];
    double ref_y_prev = previous_path_y[previous_path_size-2];
    
    //
    // Reference yaw angles from point sets
    
    initial_yaw = atan2(initial_car_y-ref_y_prev, initial_car_x-ref_x_prev);
    
    points_x.push_back(ref_x_prev);
    points_x.push_back(initial_car_x);
    
    points_y.push_back(ref_y_prev);
    points_y.push_back(initial_car_y);
    
  }
  
  //
  // Get x and y coordinates for 3 points ahead
  
  vector<double> next_wp0 = getXY(car_s+30, (lane_width/2.0 + lane_width * current_lane_index), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60, (lane_width/2.0 + lane_width * current_lane_index), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90, (lane_width/2.0 + lane_width * current_lane_index), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  
  points_x.push_back(next_wp0[0]);
  points_x.push_back(next_wp1[0]);
  points_x.push_back(next_wp2[0]);
  
  points_y.push_back(next_wp0[1]);
  points_y.push_back(next_wp1[1]);
  points_y.push_back(next_wp2[1]);
  
  //
  // Iterate over all the points and shift from map coordinates to the car coordinates
  
  for (int i=0; i<points_x.size(); i++) {
    
    //
    // Relative distances to the vehicle
    
    double shift_x = points_x[i]-initial_car_x;
    double shift_y = points_y[i]-initial_car_y;
    
    //
    // Transform the points to the car xy coordinates
    
    points_x[i] = (shift_x*cos(-initial_yaw) - shift_y*sin(-initial_yaw));
    points_y[i] = (shift_x*sin(-initial_yaw) + shift_y*cos(-initial_yaw));
    
  }
  
  //
  // Spline
  
  tk::spline s;
  s.set_points(points_x,points_y);
  
  //after getting the spline up fro the next 90 m in s-coordinates
  //target a range to immediately follow and draw in the x-axes
  double target_x = 30.0;
  
  //use the spline to get the corresponding y value
  double target_y = s(target_x);
  
  //calculate the distance to the target
  double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));
  
  //
  // Generate values for future coordinates - it should always be 50
  
  for (int i=0; i<previous_path_x.size(); i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  double x_sum = 0.0;
  
  for (int i=0; i <= 50-previous_path_x.size(); i++) {
    
    //
    // Calculate step size
    
    double step_factor = 2.24;
    double calculation_step = (planning_time_interval * current_velocity / step_factor);
    double n = (target_dist / calculation_step);

    //
    // Find X coordinate
    
    double x_point = x_sum + target_x / n;
    
    //
    // Find Y coordinate
    
    double y_point = s(x_point);

    //
    // Update X sum
    
    x_sum = x_point;

    //
    // Store coordinates before transforming
    
    double temp_x_point = x_point;
    double temp_y_point = y_point;
    
    //
    // Transform to map coordinates
    
    x_point = (temp_x_point * cos(initial_yaw) - temp_y_point*sin(initial_yaw));
    y_point = (temp_x_point * sin(initial_yaw) + temp_y_point*cos(initial_yaw));
    
    //
    // Add initial coordinates
    
    x_point += initial_car_x;
    y_point += initial_car_y;
    
    //
    // Push final coordinates
    
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  
}

int PathPlanner::getLaneFromFrenet(double frenet_d) {
  int lane = -1;
  for (int i=0; i<lane_count; i++) {
    if (frenet_d < (lane_width/2.0 + lane_width*i + lane_width/2.0) && frenet_d > (lane_width/2.0 + lane_width*i - lane_width/2.0)) {
      lane = i;
      break;
    }
  }
  return lane;
}

double PathPlanner::pi() { return M_PI; }
double PathPlanner::deg2rad(double x) { return x * pi() / 180; }

vector<double> PathPlanner::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;
  
  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }
  
  unsigned long wp2 = (prev_wp+1)%maps_x.size();
  
  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);
  
  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
  
  double perp_heading = heading-pi()/2;
  
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);
  
  return {x,y};
  
}