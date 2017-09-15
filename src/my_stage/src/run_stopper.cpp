#include "stopper.h"
int main(int argc, char **argv){
	ros::init(argc, argv, "stopper");
	stopper stopper;
	stopper.startMoving();
	return 0;
}

