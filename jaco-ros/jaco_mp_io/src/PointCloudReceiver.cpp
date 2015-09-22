#include <jaco_mp_io/PointCloudReceiver.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf_conversions/tf_eigen.h>

namespace jaco{

PointCloudReceiver::PointCloudReceiver(ros::NodeHandle &nh, std::string subscribeTo):
currentCloud(new pcl::PCLPointCloud2()),
latestCloud(new sensor_msgs::PointCloud2)
{

	nh_ = nh;

	subscriber = nh_.subscribe(subscribeTo, 1, &PointCloudReceiver::pointCloudCallback, this);

}

PointCloudReceiver::~PointCloudReceiver() {
}

pcl::PCLPointCloud2Ptr PointCloudReceiver::updatePointCloud(){

	ros::spinOnce();
	ros::spinOnce();

	pcl_conversions::toPCL(*latestCloud,*currentCloud);

	if(currentCloud->header.frame_id=="base_link")
		return currentCloud;

	//transform from what ever frame to l_foot if it is not l_foot
	tf::StampedTransform transform;
	try
	{
		listener_.lookupTransform("base_link", currentCloud->header.frame_id, ros::Time().fromNSec(currentCloud->header.stamp*10e2), transform);

	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("Could not find transform!");
	}

	Eigen::Affine3d eigenTr;
	tf::transformTFToEigen(transform, eigenTr);
	sensor_msgs::PointCloud2Ptr transformCloud(new sensor_msgs::PointCloud2);
	pcl_ros::transformPointCloud (eigenTr.cast<float>().matrix(),*latestCloud, *transformCloud);

	pcl_conversions::toPCL(*transformCloud,*currentCloud);

	return currentCloud;

}

pcl::PCLPointCloud2Ptr PointCloudReceiver::updatePointCloud(sensor_msgs::PointCloud2Ptr pc){

	latestCloud = pc;
	return updatePointCloud();

}

void PointCloudReceiver::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr data){
	latestCloud = data;
}

}//namespace jaco