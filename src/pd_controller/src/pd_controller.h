#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


class PD_Controller {
public:

  PD_Controller();
  void publishMessage();
  void turn();
  void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  double wallDistance; 
  double e;            // Difference between desired distance from the wall and actual distance.
  double diffE;        // Derivative element for PD controller;
  double maxSpeed;     
  double P;            // Proportional Constant for PD controller.
  double D;            // Derivative Constant for PD controller.
  double angleMin;     
  double distFront;    

  ros::Publisher pubMessage;  
  ros::NodeHandle nodeHandle;
  ros::Subscriber sub;

};
