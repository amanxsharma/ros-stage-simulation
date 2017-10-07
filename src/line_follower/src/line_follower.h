#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


class LineFollowing {
public:

  //LineFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an);
  LineFollowing();

 // ~LineFollowing();


  void publishMessage();

  void turn();
  void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

//variables
  int counter;
  double wallDistance; // Desired distance from the wall.
  double e,t0,t1;            // Difference between desired distance from the wall and actual distance.
  double diffE;     // Derivative element for PD controller;
  double maxSpeed;     // Maximum speed of robot.
  double P = 10;            // k_P Constant for PD controller.
  double D = 5;            // k_D Constant for PD controller.
  double angleCoef;    // Coefficient for P controller.
  int direction;      // 1 for wall on the right side of the robot (-1 for the left one).
  double angleMin, am;     // Angle, at which was measured the shortest distance.
  double distFront;    // Distance, measured by ranger in front of robot.
  ros::Publisher pubMessage;  // Object for publishing messages.

  ros::NodeHandle nodeHandle;
  //ros::Publisher pub;
  ros::Subscriber sub;
const static int SUBSCRIBER_BUFFER_SIZE = 1;  // Size of buffer for subscriber.
const static int PUBLISHER_BUFFER_SIZE = 1000;  // Size of buffer for publisher.
const static double WALL_DISTANCE = 0.13;
const static double MAX_SPEED = 0.1;

const static int ANGLE_COEF = 1;    // Proportional constant for angle controller
const static int DIRECTION = 1; // 1 for wall on the left side of the robot (-1 for the right side).

};
