#include <jaco_mp_io/jaco_mp_io.h>

using namespace std;

namespace jaco{

JACOMotionPlannerIO::JACOMotionPlannerIO(std::string topicPrefix) : mNode(topicPrefix), jacoTrajectory(new jaco_traj::JACOTraj())
{
	std::lock_guard<std::mutex> lock(g_i_mutex);
	ReceiveComputeTrajectoryTarget = mNode.subscribe("trajectory_target", 1, &JACOMotionPlannerIO::ComputeTrajectory, this);
	loadObjectTopic = mNode.subscribe("load_object", 1, &JACOMotionPlannerIO::LoadObject, this);
	grabTopic = mNode.subscribe("grab", 1, &JACOMotionPlannerIO::Grab, this);
	SetProblemParametersTopic = mNode.subscribe("problem_parameters", 1, &JACOMotionPlannerIO::SetProblemParameters, this);
	SendPlanResult = mNode.advertise<control_msgs::FollowJointTrajectoryGoal>("joint_trajectory", 1);
	jsr = boost::shared_ptr<JointStateReceiver>(new JointStateReceiver(nh, "jaco_arm_driver/out/joint_state"));
	pcr = boost::shared_ptr<PointCloudReceiver>(new PointCloudReceiver(nh, "camera/depth_registered/points"));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr savedcloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
	save_pc = false;
	load_saved_pc = false;
}

JACOMotionPlannerIO::~JACOMotionPlannerIO(){

}

void JACOMotionPlannerIO::ComputeTrajectory(const jaco_mp_io::ArmMotionPlanningCommandPtr& command){ 
	std::lock_guard<std::mutex> lock(g_i_mutex);
	Eigen::Affine3d hand_target;
	hand_target.translation()[0] = command->hand_goal.position.x;
	hand_target.translation()[1] = command->hand_goal.position.y;
	hand_target.translation()[2] = command->hand_goal.position.z;
	Eigen::Quaterniond quat(command->hand_goal.orientation.w, command->hand_goal.orientation.x,command->hand_goal.orientation.y,command->hand_goal.orientation.z);
	hand_target.linear() = quat.toRotationMatrix();

	Eigen::Vector3d pos_gains;
	Eigen::Vector3d rot_gains;
	Eigen::Vector3d hand_offset;
	pos_gains[0] = command->pos_gains[0];
	pos_gains[1] = command->pos_gains[1];
	pos_gains[2] = command->pos_gains[2];

	rot_gains[0] = command->rot_gains[0];
	rot_gains[1] = command->rot_gains[1];
	rot_gains[2] = command->rot_gains[2];

	hand_offset[0] = command->hand_offset[0];
	hand_offset[1] = command->hand_offset[1];
	hand_offset[2] = command->hand_offset[2];

	// jacoTrajectory->SetMode((jaco_traj::TrajoptMode)command->mode);

	JACOMotionPlannerIO::ComputeTrajectory(hand_target, command->load_pc, pos_gains, rot_gains, hand_offset);
}

void JACOMotionPlannerIO::ComputeTrajectory(Eigen::Affine3d hand_target, bool load_pc, Eigen::Vector3d pos_gains, Eigen::Vector3d rot_gains, Eigen::Vector3d hand_offset){
	boost::shared_ptr<std::vector<double>> start_state = jsr->updateJoints();
	jacoTrajectory->LoadGains(pos_gains,rot_gains,hand_offset);
	jacoTrajectory->SeeViewer(true);
	jacoTrajectory->IdleViewer(true);
	jacoTrajectory->SetNumStep(30);

	/**load point cloud**/
	if(load_pc)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr;
		if(load_saved_pc)
		{
			std::cout << "Loading save point cloud" << std::endl;
			cloudPtr = savedcloudPtr;
			load_saved_pc = false;
		}else{
			cloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>()); 

			pcl::fromPCLPointCloud2(*pcr->updatePointCloud(), *cloudPtr);

			if(save_pc){
				std::cout<<"Saving point cloud"<< std::endl;
				savedcloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
				pcl::copyPointCloud(*cloudPtr, *savedcloudPtr);
				std::cout<<"point cloud saved"<< std::endl;
				// pcl::fromPCLPointCloud2(*pcr->updatePointCloud(), *savedcloudPtr);
				save_pc = false;
			}
		}
			Eigen::Affine3d transform(Eigen::Affine3d::Identity());
			transform.linear() = Eigen::Quaterniond(0.70711,0,0,-0.70711).toRotationMatrix();
			// transform.translation().x() = -0.3;
			// transform.translation().y() = 0;
			// transform.translation().z() = -1;

			//////////// get transform matrix between camera and arm ///////////////////
			// OpenRAVE::Transform armPose = robot->GetLink("jaco_link_base")->GetTransform();
			// Eigen::Affine3d transform;
			// transform.linear() = Eigen::Quaterniond(armPose.rot.x,armPose.rot.y,armPose.rot.z,armPose.rot.w).toRotationMatrix();
			// transform.translation().x() = armPose.trans.x;
			// transform.translation().y() = armPose.trans.y;
			// transform.translation().z() = armPose.trans.z;

			jacoTrajectory->PrepPointCloud(cloudPtr, transform);

			jacoTrajectory->LoadPointCloud(*start_state);
		// }
	}

	/**test code for insert object**/
	Eigen::Affine3d affine(Eigen::Affine3d::Identity());
	std::string box = "box";
  	affine.translation() = Eigen::Vector3d(-0.55, 0.2, 0.3);
  	affine.linear() = Eigen::Quaterniond(1,0,0,0).toRotationMatrix();
  	jacoTrajectory->Load(box);
  	jacoTrajectory->TransformObject(box,affine);
	// std::string debris = "debris";
 //  	affine.translation() = Eigen::Vector3d(0.0, -0.35, 0.2);
 //  	affine.linear() = Eigen::Quaterniond(1,0,0,0).toRotationMatrix();
 //  	jacoTrajectory->Load(debris);
 //  	jacoTrajectory->TransformObject(debris,affine);

	jacoTrajectory->ComputeTrajectory(*start_state, hand_target);
	vector< vector<double> > final_traj;
	jacoTrajectory->GetFinalTraj(final_traj);
	control_msgs::FollowJointTrajectoryGoal goal = GenerateTrajMsg(final_traj);
	SendPlanResult.publish(goal);
	jacoTrajectory->ResetEnvironment();
}

control_msgs::FollowJointTrajectoryGoal JACOMotionPlannerIO::GenerateTrajMsg(vector< vector<double> >& final_traj){
	control_msgs::FollowJointTrajectoryGoal goal;
	goal.trajectory.joint_names.resize(6);
	goal.trajectory.joint_names[0] = "jaco_joint_1";
	goal.trajectory.joint_names[1] = "jaco_joint_2";
	goal.trajectory.joint_names[2] = "jaco_joint_3";
	goal.trajectory.joint_names[3] = "jaco_joint_4";
	goal.trajectory.joint_names[4] = "jaco_joint_5";
	goal.trajectory.joint_names[5] = "jaco_joint_6";
	goal.trajectory.points.resize(final_traj.size());
	for(int i=0; i<final_traj.size(); i++){
		goal.trajectory.points[i].positions.resize(final_traj[i].size());
		for(int j=0; j<final_traj[i].size();j++){
			goal.trajectory.points[i].positions[j] = final_traj[i][j];	
		}
	}
	return goal;
}

void JACOMotionPlannerIO::Grab(const jaco_mp_io::GrabCommandPtr& message)
{

	// std::lock_guard<std::mutex> lock(g_i_mutex);
	std::string name(message->file_name.data);

	if(message->grab == true)
	{
		boost::shared_ptr<std::vector<double>> joints = jsr->updateJoints();
		std::cout << "Grabbing object " << name << std::endl;
		jacoTrajectory->GrabObject(*joints,name);
	}else{
  		std::cout << "Releasing object " << name << std::endl;
		jacoTrajectory->ReleaseObject(name);
	}
}

void JACOMotionPlannerIO::LoadObject(const jaco_mp_io::LoadObjectPtr& message)
{

	// std::lock_guard<std::mutex> lock(g_i_mutex);
	std::string name(message->file_name.data);

	Eigen::Affine3d command;

	command.translation()[0] = message->pose.position.x;
	command.translation()[1] = message->pose.position.y;
	command.translation()[2] = message->pose.position.z;

	Eigen::Quaterniond quat(message->pose.orientation.w, message->pose.orientation.x,message->pose.orientation.y,message->pose.orientation.z);
	command.linear() = quat.toRotationMatrix();

  	std::cout << "Loading object " << name << std::endl;
	jacoTrajectory->Load(name);
	std::cout << "Transforming object " << name << std::endl;

	// geometry_msgs::Quaternion footQ= *fr->updateFoot();
	// OpenRAVE::Vector footOrientation(footQ.w,footQ.x,footQ.y,footQ.z); // expects x, y ,z, w

	// 	////POSSIBLE ERROR - if orientation looks wrong, it may be that the mapping is wrong
	// 	// eg geometry.x = openrave.w, geometry.y = openrave.x, geometry.z = openrave.y, geometry.w = openrave.z
	// jacoTrajectory->SetLFootFrame(OpenRAVE::Transform(footOrientation, OpenRAVE::Vector(0,0,0,0)));

	jacoTrajectory->TransformObject(name,command);

}

void JACOMotionPlannerIO::SetProblemParameters(const jaco_mp_io::SetProblemParametersPtr& message)
{
	// std::lock_guard<std::mutex> lock(g_i_mutex);
	jacoTrajectory->SetMode((jaco_traj::TrajoptMode)message->mode);
	jacoTrajectory->SetNumStep(message->num_step);
	jacoTrajectory->SetSmoothing(message->smoothing);
	save_pc = message->save_pc;
	load_saved_pc = message->load_saved_pc;
}
};