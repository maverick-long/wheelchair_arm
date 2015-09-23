#include "ros/ros.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>


void chatterCallback(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
	Eigen::Affine3d transform_baseintag(Eigen::Affine3d::Identity());
	transform_baseintag.translation().x() = 0.135;
	transform_baseintag.translation().y() = -0.22;
	transform_baseintag.translation().z() = 0.145;
	transform_baseintag.linear() = Eigen::Quaterniond(0,0,0,1).toRotationMatrix();

	Eigen::Affine3d transform_tagincamera(Eigen::Affine3d::Identity());
	transform_tagincamera.linear() = Eigen::Quaterniond(msg->markers[0].pose.pose.orientation.w,msg->markers[0].pose.pose.orientation.x,msg->markers[0].pose.pose.orientation.y,msg->markers[0].pose.pose.orientation.z).toRotationMatrix();
	transform_tagincamera.translation().x() = msg->markers[0].pose.pose.position.x;
	transform_tagincamera.translation().y() = msg->markers[0].pose.pose.position.y;
	transform_tagincamera.translation().z() = msg->markers[0].pose.pose.position.z;

	Eigen::Affine3d trasform_camerainbase = (transform_tagincamera*transform_baseintag).inverse();

	Eigen::Quaterniond q = Eigen::Quaterniond(trasform_camerainbase.linear());

	std::cout<<"translation: "<<trasform_camerainbase.translation().x()<<" "<<trasform_camerainbase.translation().y()<<" "<<trasform_camerainbase.translation().z()<<std::endl;
	std::cout<<"orientation: "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "maker_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ar_pose_marker", 1000, chatterCallback);

  ros::spin();

  return 0;
}