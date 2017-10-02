#include "line_follower.h"
#include <math.h>
#define PI 3.141592


//LineFollowing::LineFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an)
LineFollowing::LineFollowing()
{
ROS_INFO("constructor called");
  wallDistance = 1.0;//wallDist;
  maxSpeed = 0.1;//maxSp;
  //direction = -1;//dir;
  P = 10;//pr;
  D = 5;//di;
  //angleCoef = an;
  e = 0;
  angleMin = 0;  //angle, at which was measured the shortest distance
//  pubMessage = pub;
 sub = nodeHandle.subscribe("base_scan", 1, &LineFollowing::messageCallback, this);
 pubMessage = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);//PUBLISHER_BUFFER_SIZE);

}

int main(int argc, char **argv)
{
  //Initialization of node
  ros::init(argc, argv, "wallFollowing");

  //ros::NodeHandle n;
  //Creating publisher
  

  //Creating object, which stores data from sensors and has methods for
  //publishing and subscribing
//  LineFollowing *lineFollowing = new LineFollowing(pubMessage, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, 1);
  LineFollowing *lineFollowing = new LineFollowing(); //1 for right wall
//LineFollowing lineFollowing; //1 for right wall

  //Creating subscriber and publisher
//  ros::Subscriber sub = n.subscribe("base_scan", 1, &LineFollowing::messageCallback, lineFollowing);
  
  ros::spin();

  return 0;

/*
double P = 10;            // k_P Constant for PD controller.
double D = 5;            // k_D Constant for PD controller.
const static int SUBSCRIBER_BUFFER_SIZE = 1;  // Size of buffer for subscriber.
const static int PUBLISHER_BUFFER_SIZE = 1000;  // Size of buffer for publisher.
const static double WALL_DISTANCE = 0.13;
const static double MAX_SPEED = 0.1;
const static int ANGLE_COEF = 1;    // Proportional constant for angle controller
const static int DIRECTION = 1; // 1 for wall on the left side of the robot (-1 for the right side).
*/
}

/*LineFollowing::~LineFollowing()
{
}
*/
//Publisher
void LineFollowing::publishMessage()
{
  //preparing message
  geometry_msgs::Twist msg;

//double temp = direction*(P*e + D*diffE) + (angleMin - PI*direction/2);
double temp =  (angleMin + PI/2) - (P*e + D*diffE);
//ROS_INFO_STREAM("direc: "<<direction<<" e: "<<e<<" diffE: "<<diffE<<" angleMin: "<<angleMin<<" z: "<<temp);

  msg.angular.z = temp;    //PD controller

  if (distFront < wallDistance){
    msg.linear.x = 0;
  }
  else if (distFront < wallDistance * 2){
    msg.linear.x = 0.5*maxSpeed;
  }
  else if (fabs(angleMin)>1.75){
    msg.linear.x = 0.4*maxSpeed;
  } 
  else {
    msg.linear.x = maxSpeed;
  }

  //publishing message
  pubMessage.publish(msg);
}

//Subscriber
void LineFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("inside callback");
  int size = msg->ranges.size();

  //Variables whith index of highest and lowest value in array.
//  int minIndex = size*(direction+1)/4;	
//  int maxIndex = size*(direction+3)/4;

  int minIndex = 0;//size*(direction+1)/4;	
  int maxIndex = size/2;//size*(direction+3)/4;

  
  //This cycle goes through array and finds minimum
  for(int i = minIndex; i < maxIndex; i++)
  {
    if (msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > 0.0){
      minIndex = i;
    }
  }

  //Calculation of angles from indexes and storing data to class variables.
  angleMin = (minIndex-size/2)*msg->angle_increment;

  double distMin;
  distMin = msg->ranges[minIndex];   //I think this is right Wall

  distFront = msg->ranges[size/2];   //300 CHANGED...originally it was size/2, but my robot is inclined
  ROS_INFO_STREAM("distFront: "<<distFront);
  diffE = (distMin - wallDistance) - e;
  e = distMin - wallDistance;
  //ROS_INFO("now move!");
  //Invoking method for publishing message
  publishMessage();
}
