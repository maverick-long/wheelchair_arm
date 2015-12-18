

#include <arm_motionplanning/jaco_utils.hpp>
#include <arm_motionplanning/jaco_traj.hpp>
#include <arm_motionplanning/jaco_consts.hpp>

using namespace OpenRAVE;
using namespace std;


int main(){

  std::cout << "Starting Test" << std::endl;

  jaco_traj::JACOTraj door_test;

  // vector<double> start_default = {0,0,0,0,0,0};
  vector<double> start_default = {-1.794, -2.009, 0.8117, -0.878, 1.695, 3.190};

  Eigen::Affine3d affine(Eigen::Affine3d::Identity());

  door_test.SeeViewer(true);
  door_test.IdleViewer(true);
  door_test.SetNumStep(30);
  door_test.SetMode(jaco_traj::TrajoptMode::WheelChairDefault);
  // door_test.SetSmoothing(true);

  // std::string door = "door";
  // affine.translation() = Eigen::Vector3d(0.0, -0.7, -0.5);
  // affine.linear() = Eigen::Quaterniond(0.70711,0.70711,0,0).toRotationMatrix();
  // door_test.Load(door);
  // door_test.TransformObject(door,affine);

  // KinBodyPtr doorPtr = door_test.GetKinBody("door");

  // doorPtr->SetJointValues({0,3.14},1);

  door_test.LoadGains({1,1,1},{1,1,1},{0.0,0.0,-0.15});

  affine.translation() = Eigen::Vector3d(0.4, -1.6, 0.5);
  affine.linear() = (Eigen::Quaterniond(0.5,-0.5,0.5,0.5)).toRotationMatrix();
  door_test.ComputeTrajectory(start_default, affine);
  
  while(1);

  return 0;
}