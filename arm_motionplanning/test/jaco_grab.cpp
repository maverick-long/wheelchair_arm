

#include <arm_motionplanning/jaco_utils.hpp>
#include <arm_motionplanning/jaco_traj.hpp>
#include <arm_motionplanning/jaco_consts.hpp>

using namespace OpenRAVE;
using namespace std;


int main(){

  std::cout << "Starting Test" << std::endl;

  jaco_traj::JACOTraj grab_test;

  vector<double> start_default = {0,0,0,0,0,0};

  Eigen::Affine3d affine(Eigen::Affine3d::Identity());

  grab_test.SeeViewer(true);
  grab_test.IdleViewer(true);
  grab_test.SetNumStep(30);
  // grab_test.SetSmoothing(true);

  std::string debris = "debris";
  affine.translation() = Eigen::Vector3d(0.25, 0.3, 0.0);
  affine.linear() = Eigen::Quaterniond(1,0,0,0).toRotationMatrix();
  grab_test.Load(debris);
  grab_test.TransformObject(debris,affine);

  grab_test.LoadGains({1,1,1},{1,0,1},{0.0,0.0,-0.1});

  affine.translation() = Eigen::Vector3d(0.2, 0.5, 0.3);//Left: 0.55, 0.0, 1.15
  affine.linear() = (Eigen::Quaterniond(0.70711,-0.70711,0.0,0.0)).toRotationMatrix();
  grab_test.ComputeTrajectory(start_default, affine);
  while(1);

  return 0;
}