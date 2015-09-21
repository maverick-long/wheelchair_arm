#ifndef POINTCLOUDRECEIVER_H_
#define POINTCLOUDRECEIVER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cloudproc/hacd_interface.hpp>
#include <cloudproc/cloudproc.hpp>
#include <cloudproc/mesh_simplification.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>


#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace jaco{

/**
 * @brief PointCloudReceiver Class
 * used for pulling the latest point cloud from a topic
 */
class PointCloudReceiver {
public:
	/**
	 * @brief PointCloudReceiver constructor
	 * @param nh: nodehandle for PointCloudReceiver to use
	 * @param subscribeTo: topic to subscribe to
	 * @return PointCloudReceiver object
	 */
	PointCloudReceiver(ros::NodeHandle &nh, std::string subscribeTo);
	/**
	 * @brief PointCloudReceiver destructor
	 * @param none
	 * @return none
	 */
	virtual ~PointCloudReceiver();

	/**
	 * @brief pulls the latest point cloud from the topic
	 * @param none
	 * @return pointer to the latest point cloud pulled
	 */
	pcl::PCLPointCloud2Ptr updatePointCloud();
	pcl::PCLPointCloud2Ptr updatePointCloud(sensor_msgs::PointCloud2Ptr pc);

protected:
	void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr data);
	sensor_msgs::PointCloud2ConstPtr latestCloud;
	pcl::PCLPointCloud2Ptr currentCloud;

	ros::NodeHandle nh_;
	ros::Subscriber subscriber;
    tf::TransformListener 									listener_;

};

}//namespace jaco

#endif /* POINTCLOUDRECEIVER_H_ */