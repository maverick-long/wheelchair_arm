#ifndef JointStateReceiver_H_
#define JointStateReceiver_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace jaco{

/**
 * @brief JointStateReceiver class
 * Used for pulling the current robot joints and converting them into a vector
 */
class JointStateReceiver {
public:
	/**
	 * @brief JointStateReceiver class constructor
	 * @param nh: a ros nodehandle for the receiver to use
	 * @param subscribeTo: the topic to pull the joints from
	 * @return JointStateReceiver object
	 */
	JointStateReceiver(ros::NodeHandle &nh, std::string subscribeTo);

	/**
	 * @brief JointStateRecevier destructor
	 * @param none
	 * @return none
	 */
	virtual ~JointStateReceiver(){}

	/**
	 * @brief pulls the most recent joints states
	 * @param none
	 * @return pointer to the last pulled set of joints
	 */
	virtual boost::shared_ptr<std::vector<double>> updateJoints();

	//The idea is that you can pull the pointer to the latest joints once, then just update that pointer repeatedly

protected:

	sensor_msgs::JointStatePtr latestJoints;
	boost::shared_ptr<std::vector<double>> currentJoints;

	ros::NodeHandle nh_;
	ros::Subscriber subscriber;

	void jointMessageCallback(sensor_msgs::JointStatePtr data);
};

}//namespace jaco

#endif /* JointStateReceiver_H_ */