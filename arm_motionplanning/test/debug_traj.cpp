

#include <arm_motionplanning/jaco_utils.hpp>
#include <arm_motionplanning/jaco_traj.hpp>
#include <arm_motionplanning/jaco_consts.hpp>

using namespace OpenRAVE;
using namespace std;


int main(){

  std::cout << "Starting Debug" << std::endl;

  jaco_traj::JACOTraj debug_traj;

  debug_traj.SeeViewer(true);
  debug_traj.IdleViewer(true);
  debug_traj.SetNumStep(30);
  debug_traj.SetSmoothing(true);

  debug_traj.DebugTraj("/home/xianchao/error.txt");

  



  while(1);

  return 0;
}