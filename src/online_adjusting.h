#include "std_msgs/String.h"
#include <SDL2/SDL.h>
#include <automated_driving_msgs/ClassWithProbability.h>
#include <automated_driving_msgs/ObjectState.h>
#include <automated_driving_msgs/ObjectStateArray.h>
#include <chrono>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <thread>
#include <time.h>


class Transform
{
public:
  Transform();
  inline virtual ~Transform()
  {
    /* Clean up */
    SDL_Quit();
    exit(0);
  };
  void SetVideoMode();
  void keyboard_control();
  void transformation(automated_driving_msgs::ObjectStateArray msg_array_in);
  // void update_state();

private:
  // heading angle
  double heading;
  // double acc;
  // x position of center of mass
  double x;
  double x_der;
  // y position of center of mass
  double y;
  double y_del;
  double v;
  // double time_step;
  double time_counter;

  //the angle which is manually added
  double rotation_angle;
  //x-y translation 
  double dx;
  double dy;

  double dt=0.02;

  //map origin in EPSG:3857
  double ori_x = 5651253.09;
  double ori_y = 676012.40;

  automated_driving_msgs::ObjectState msg;
  automated_driving_msgs::ObjectStateArray msg_array;
  automated_driving_msgs::ClassWithProbability msg_class;

  ros::NodeHandle n_; 
  ros::Publisher msg_pub_;
  ros::Subscriber sub_;
};
