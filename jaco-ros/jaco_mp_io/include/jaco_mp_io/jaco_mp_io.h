#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
#include <arm_motionplanning/jaco_traj.hpp>
#include <jaco_mp_io/ArmMotionPlanningCommand.h>
#include <jaco_mp_io/JointStateReceiver.h>
#include <jaco_mp_io/PointCloudReceiver.h>
#include <jaco_mp_io/GrabCommand.h>
#include <jaco_mp_io/LoadObject.h>
#include <jaco_mp_io/SetProblemParameters.h>

using namespace std;

namespace jaco
{
class JACOMotionPlannerIO{
public:
	JACOMotionPlannerIO(std::string topicPrefix);
	~JACOMotionPlannerIO();
	/* Callback Functions */
	void ComputeTrajectory(const jaco_mp_io::ArmMotionPlanningCommandPtr& command);
	void ComputeTrajectory(Eigen::Affine3d hand_target, bool load_pc, Eigen::Vector3d pos_gains, Eigen::Vector3d rot_gains, Eigen::Vector3d hand_offset);
	void Grab(const jaco_mp_io::GrabCommandPtr& message);
	void LoadObject(const jaco_mp_io::LoadObjectPtr& message);
	void SetProblemParameters(const jaco_mp_io::SetProblemParametersPtr& message);

private:
	/* Mutex */ 
	std::mutex g_i_mutex;

	ros::NodeHandle mNode;
	ros::NodeHandle nh;
	ros::Subscriber ReceiveComputeTrajectoryTarget;
	ros::Subscriber loadObjectTopic;
	ros::Subscriber grabTopic;
	ros::Subscriber SetProblemParametersTopic;
	ros::Publisher	SendPlanResult;
	boost::shared_ptr<jaco_traj::JACOTraj> jacoTrajectory;
	boost::shared_ptr<JointStateReceiver> jsr;
	boost::shared_ptr<PointCloudReceiver> pcr;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr savedcloudPtr;
	bool save_pc;
	bool load_saved_pc;

	control_msgs::FollowJointTrajectoryGoal GenerateTrajMsg(vector< vector<double> >& final_traj);
};

};