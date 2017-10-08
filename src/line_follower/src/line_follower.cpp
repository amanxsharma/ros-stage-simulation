#include "line_follower.h"
#include <math.h>
#define PI 3.141592
//without fabs wala elseif it works...

LineFollowing::LineFollowing()
{
//ROS_INFO("constructor called");
  wallDistance = 0.5;//1.0;//wallDist;
  maxSpeed = 0.5*0.1;//maxSp;

  P = 10;//pr;
  D = 5;//di;

  e = 0;
  angleMin = 0;  //angle, at which was measured the shortest distance

 sub = nodeHandle.subscribe("base_scan", 1, &LineFollowing::messageCallback, this);
 pubMessage = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);//PUBLISHER_BUFFER_SIZE);

}

int main(int argc, char **argv)
{
  //Initialization of node
  ros::init(argc, argv, "wallFollowing");
  LineFollowing *lineFollowing = new LineFollowing(); //1 for right wall  
  ros::spin();

  return 0;
}

//Publisher
void LineFollowing::publishMessage()
{
  //preparing message
  geometry_msgs::Twist msg;

double temp =  (angleMin + PI/2) - (P*e + D*diffE);
//ROS_INFO_STREAM("direc: "<<direction<<" e: "<<e<<" diffE: "<<diffE<<" angleMin: "<<angleMin<<" z: "<<temp);

  msg.angular.z = temp;    //PD controller
  msg.linear.x = maxSpeed;

  if (distFront < wallDistance){
    msg.linear.x = 0;				//start turning
	ROS_INFO("here");  
}
  /*else if (distFront < wallDistance * 2){
    msg.linear.x = 0.5*maxSpeed;		//slow down
  }*/

  else if (fabs(angleMin)>1.75){
    //msg.linear.x = 0.4*maxSpeed;
msg.linear.x = maxSpeed;
	ROS_INFO("here too..........");
  } 
  /*else {
    msg.linear.x = maxSpeed;
  }*/

  //publishing message
  pubMessage.publish(msg);
}

//Subscriber
void LineFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //ROS_INFO("inside callback");
  int size = msg->ranges.size();

  //Variables whith index of highest and lowest value in array.
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

  //double distMin;
  //distMin = msg->ranges[minIndex];   //I think this is right Wall

  distFront = msg->ranges[size/2];   //300 CHANGED...originally it was size/2, but my robot is inclined
  //ROS_INFO_STREAM("distFront: "<<distFront);
  diffE = (msg->ranges[minIndex] - wallDistance) - e;
  e = msg->ranges[minIndex] - wallDistance;
  //Invoking method for publishing message
  publishMessage();
}
