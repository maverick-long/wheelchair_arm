#include <ros/ros.h>
#include <jaco_mp_io/jaco_mp_io.h>


int main(int argc, char* argv[]){

	ros::init(argc, argv, "JACO_arm_motion_planning");
	ros::NodeHandle nh;
	
	/* Start motion planning service: */
	jaco::JACOMotionPlannerIO sub("jaco_mp_io");

	ros::spin();
}