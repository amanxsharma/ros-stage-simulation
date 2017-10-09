#include "pid_controller.h"
#include <math.h>
#define PI 3.141592

//Constructor
WallFollower::WallFollower()
{
//Initialize variable
  wallDistance = 0.5;
  maxSpeed = 0.05;
  Kp = 10;
  Kd = 5;
  Ki = 0.0001;
  error = 0;
  integral = 0;   
  angleMin = 0;  

 sub = nodeHandle.subscribe("base_scan", 1, &WallFollower::messageCallback, this);   //subscriber
 pubMessage = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);          //publisher

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wallFollowing");   //Initialization of node
  WallFollower *wallFollower = new WallFollower(); 
  ros::spin();

  return 0;
}

//Message publishing function
void WallFollower::publishMessage()
{
  geometry_msgs::Twist msg;

  double PID_value = Kp*error + Kd*derivative + Ki*integral; //PID Controller

  double correction_value = angleMin + PI/2;
  
  msg.angular.z = correction_value - PID_value;  //angular velocity;    
  msg.linear.x = maxSpeed;                       //linear velocity

  if (distFront < wallDistance){
    msg.linear.x = 0;				//stop and turn 
   } 
 
  else if (fabs(angleMin)>1.75){
    msg.linear.x = maxSpeed;			//adjust robot while moving
  } 
 
 pubMessage.publish(msg);
}

//Callback function
void WallFollower::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  
  int size = msg->ranges.size();

  //highest and lowest value for scan.
  int min = 0;	  
  for(int i = min; i < size/2; i++)
  {
    if (msg->ranges[i] < msg->ranges[min]){
      min = i;
    }
  }

  angleMin = (min - size/2)*msg->angle_increment;
  distFront = msg->ranges[size/2];   
  derivative = (msg->ranges[min] - wallDistance) - error;
  error = msg->ranges[min] - wallDistance;
  integral = integral + error;
  publishMessage();
}
