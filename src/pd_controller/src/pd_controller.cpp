#include "pd_controller.h"
#include <math.h>
#define PI 3.141592


PD_Controller::PD_Controller()
{
//Initialize variable
  wallDistance = 0.5;
  maxSpeed = 0.05;
  P = 10;
  D = 5;
  e = 0;
  angleMin = 0;  

 sub = nodeHandle.subscribe("base_scan", 1, &PD_Controller::messageCallback, this);
 pubMessage = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wallFollowing");   //Initialization of node
  PD_Controller *pd_controller = new PD_Controller(); 
  ros::spin();

  return 0;
}


void PD_Controller::publishMessage()
{
  geometry_msgs::Twist msg;

  double temp =  (angleMin + PI/2) - (P*e + D*diffE);  //PD controller

  msg.angular.z = temp;    
  msg.linear.x = maxSpeed;

  if (distFront < wallDistance){
    msg.linear.x = 0;				//stop and turn 
   } 
 
  else if (fabs(angleMin)>1.75){
    msg.linear.x = maxSpeed;			//adjust robot while moving
  } 
 
 pubMessage.publish(msg);
}


void PD_Controller::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
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
  diffE = (msg->ranges[min] - wallDistance) - e;
  e = msg->ranges[min] - wallDistance;

  publishMessage();
}
