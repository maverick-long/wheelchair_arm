#include <jaco_mp_io/jaco_mp_io.h>

using namespace std;

namespace jaco{

JACOMotionPlannerIO::JACOMotionPlannerIO(std::string topicPrefix) : mNode(topicPrefix), jacoTrajectory(new jaco_traj::JACOTraj())
{
	ReceiveComputeTrajectoryTarget = mNode.subscribe("trajectory_target", 10, &JACOMotionPlannerIO::ComputeTrajectory, this);
	SendPlanResult = mNode.advertise<control_msgs::FollowJointTrajectoryGoal>("joint_trajectory", 1);
	jsr = boost::shared_ptr<JointStateReceiver>(new JointStateReceiver(mNode, "jaco_arm_driver/out/joint_state"));
}

JACOMotionPlannerIO::~JACOMotionPlannerIO(){

}

void JACOMotionPlannerIO::ComputeTrajectory(const jaco_mp_io::ArmMotionPlanningCommandPtr& command){ 
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

	JACOMotionPlannerIO::ComputeTrajectory(hand_target, command->load_pc, pos_gains, rot_gains, hand_offset);
}

void JACOMotionPlannerIO::ComputeTrajectory(Eigen::Affine3d hand_target, bool load_pc, Eigen::Vector3d pos_gains, Eigen::Vector3d rot_gains, Eigen::Vector3d hand_offset){
	boost::shared_ptr<std::vector<double>> start_state = jsr->updateJoints();
	jacoTrajectory->LoadGains(pos_gains,rot_gains,hand_offset);
	jacoTrajectory->SeeViewer(true);
	jacoTrajectory->IdleViewer(true);
	jacoTrajectory->SetNumStep(30);
	jacoTrajectory->SetSmoothing(true);

	/**test code for insert object**/
	Eigen::Affine3d affine(Eigen::Affine3d::Identity());
	std::string body = "box";
  	affine.translation() = Eigen::Vector3d(-0.35, 0.0, 0.3);
  	affine.linear() = Eigen::Quaterniond(1,0,0,0).toRotationMatrix();
  	jacoTrajectory->Load(body);
  	jacoTrajectory->TransformObject(body,affine);
	std::string debris = "debris";
  	affine.translation() = Eigen::Vector3d(0.0, -0.35, 0.2);
  	affine.linear() = Eigen::Quaterniond(1,0,0,0).toRotationMatrix();
  	jacoTrajectory->Load(debris);
  	jacoTrajectory->TransformObject(debris,affine);

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
};