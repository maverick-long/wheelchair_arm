

#include <arm_motionplanning/jaco_utils.hpp>
#include <arm_motionplanning/jaco_traj.hpp>
#include <arm_motionplanning/jaco_consts.hpp>

using namespace OpenRAVE;
using namespace std;

trajopt::TrajArray graspcontrol(trajopt::TrajArray traj, double degree){
	double current_degree = traj.row(traj.rows()-1)(6);
	double unit_degree = (degree-current_degree)/(traj.rows()-1);
	for(int i=0;i<traj.rows();i++){
		for(int j=0;j<traj.row(i).size();j++){
			if(j==6||j==7||j==8){
				traj.row(i)(j)=current_degree+unit_degree*i;
			}else{
				traj.row(i)(j)=traj.row(traj.rows()-1)(j);
			}
		}
	}
	return traj;
}

int main(){

  std::cout << "Starting Test" << std::endl;

  jaco_traj::JACOTraj door_test;
  std::string jaco = "jaco";
  std::vector<double>::iterator it;
  trajopt::TrajArray traj;

  // vector<double> start_default = {-1.6, -3, 1.2, 1.57, 0, 0};
  vector<double> start_default = {-1.6, -2.009, 0.8117, -0.878, 1.695, 3.190};

  Eigen::Affine3d affine(Eigen::Affine3d::Identity());

  door_test.SeeViewer(true);
  door_test.IdleViewer(true);
  door_test.SetNumStep(30);
  // door_test.SetSmoothing(true);

  std::string door = "door";
  affine.translation() = Eigen::Vector3d(0.0, -0.7, -0.5);
  affine.linear() = Eigen::Quaterniond(0.70711,0.70711,0,0).toRotationMatrix();
  door_test.Load(door);
  door_test.TransformObject(door,affine);

  KinBodyPtr doorPtr = door_test.GetKinBody("door");

  doorPtr->SetJointValues({0,3.14},1);

  //move to the door
  door_test.SetMode(jaco_traj::TrajoptMode::WheelChairDefault);
  door_test.LoadGains({1,1,1},{1,1,1},{0.0,0.0,-0.15});

  affine.translation() = Eigen::Vector3d(0.35, -0.6, 0.45);
  affine.linear() = (Eigen::Quaterniond(0.5,-0.5,0.5,0.5)).toRotationMatrix();
  traj = door_test.ComputeTrajectory(start_default, affine);
  door_test.ShowTraj(traj);

  //close hand
  traj = graspcontrol(traj,0.7);
  door_test.ShowTraj(traj);

  vector<double> last_state = getJointValuefromTraj(traj.row(traj.rows()-1));

  //rotate door handle
  start_default.clear();
  it=last_state.begin();
  start_default.assign(it,it+6);

  door_test.SetRobotPose(last_state);

  door_test.SetMode(jaco_traj::TrajoptMode::RotateDoorKnob);
  door_test.LoadGains({1,1,1},{1,1,1},{0,-0.05,-0.15});

  affine.translation() = Eigen::Vector3d(0.4, -0.6, 0.45);
  affine.linear() = (Eigen::Quaterniond(0.5,-0.5,0.5,0.5)*Eigen::Quaterniond(0.70711,0,0,-0.70711)).toRotationMatrix();
  traj = door_test.ComputeTrajectory(start_default, affine);
  door_test.ShowTraj(traj);

  last_state = getJointValuefromTraj(traj.row(traj.rows()-1));
  printcoll(last_state);

  // std::vector<double> last_state={-1.83624, -1.52625, 1.26508, 0.299543, 1.45154, 5.35624, 0.7, 0.7, 0.7, 0.355932, -0.27752, -4.00135e-12, 0.99912, 8.9998e-12, -2.07244e-09, -9.76996e-15};
  //pull door out
  start_default.clear();
  it=last_state.begin();
  start_default.assign(it,it+6);

  door_test.SetRobotPose(last_state);

  door_test.SetMode(jaco_traj::TrajoptMode::PullDoorOut);
  door_test.LoadGains({1,1,1},{1,0,1},{-0.857,-0.05+0.05,-0.25});

  affine.translation() = Eigen::Vector3d(-0.457, -0.55, 0.45);
  // affine.linear() = (Eigen::Quaterniond(0.5,-0.5,0.5,0.5)*Eigen::Quaterniond(0.70711,0,0,-0.70711)*Eigen::Quaterniond(0.76604,0,-0.64279,0)).toRotationMatrix();
  affine.linear() = (Eigen::Quaterniond(0.5,-0.5,0.5,0.5)*Eigen::Quaterniond(0.70711,0,0,-0.70711)).toRotationMatrix();
  traj = door_test.ComputeTrajectory(start_default,affine);
  door_test.ShowTraj(traj);

  //release the handle
  door_test.SetMode(jaco_traj::TrajoptMode::ReleaseHandle);
  traj = graspcontrol(traj,0.5);
  door_test.ShowTraj(traj);

  last_state = getJointValuefromTraj(traj.row(traj.rows()-1));
  printcoll(last_state);

  // std::vector<double> last_state={-1.60742, -1.21704, 0.505434, -0.78915, 1.10695, 6.17173, 0.5, 0.5, 0.5, -0.0348901, 0.473545, 1.4787e-09, 0.453545, -4.9542e-09, 1.62304e-09, -0.401529};
  //home arm
  start_default.clear();
  it=last_state.begin();
  start_default.assign(it,it+6);

  door_test.SetRobotPose(last_state);

  door_test.SetMode(jaco_traj::TrajoptMode::ReturntoHomePose);
  door_test.LoadGains({1,1,1},{1,1,1},{0,-0.05,-0.15});

  affine.translation() = Eigen::Vector3d(0.4, -0.6, 0.45);
  affine.linear() = (Eigen::Quaterniond(0.5,-0.5,0.5,0.5)*Eigen::Quaterniond(0.70711,0,0,-0.70711)).toRotationMatrix();
  traj = door_test.ComputeTrajectory(start_default, affine);
  door_test.ShowTraj(traj);

  last_state = getJointValuefromTraj(traj.row(traj.rows()-1));
  printcoll(last_state);

  // std::vector<double> last_state={-1.4, -2.5, 1.2, 1.57, -0, -0, 0.5, 0.5, 0.5, -0.034889, 0.473545, -0, 0.748737, -0, -0, -0.662868};
  //move through the door
  start_default.clear();
  it=last_state.begin();
  start_default.assign(it,it+6);

  door_test.SetRobotPose(last_state);

  door_test.SetMode(jaco_traj::TrajoptMode::MoveThroughDoor);
  door_test.LoadGains({1,1,1},{1,1,1},{0,0,0});

  affine.translation() = Eigen::Vector3d(0,-1.5,0);
  affine.linear() = Eigen::Quaterniond(0.70711,0,0,-0.70711).toRotationMatrix();
  traj = door_test.ComputeTrajectory(start_default,affine);
  door_test.ShowTraj(traj);

  last_state = getJointValuefromTraj(traj.row(traj.rows()-1));
  printcoll(last_state);

  while(1);

  return 0;
}