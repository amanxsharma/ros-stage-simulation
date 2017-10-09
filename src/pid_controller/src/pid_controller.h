#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


class WallFollower {
public:

  WallFollower();
  void publishMessage();   //function to publish message
  void turn();             //function to turn the robot
  void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

  double wallDistance; 
  double error;        // Difference between desired distance from the wall and actual distance.
  double derivative;   // Derivative element for PID controller;
  double integral;     // Integral element for PID controller;
  double maxSpeed;     
  double Kp;            // Proportional Constant for PID controller.
  double Kd;            // Derivative Constant for PID controller.
  double Ki;            // Integral Constant for PID controller.
  double angleMin;     
  double distFront;    

  ros::Publisher pubMessage;      //Publisher
  ros::NodeHandle nodeHandle;     //Node Handle
  ros::Subscriber sub;            //Subscriber

  
};
