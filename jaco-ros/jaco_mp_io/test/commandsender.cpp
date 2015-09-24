#include "ros/ros.h"
#include <jaco_mp_io/ArmMotionPlanningCommand.h>
#include <jaco_mp_io/GrabCommand.h>
#include <jaco_mp_io/LoadObject.h>
#include <jaco_mp_io/SetProblemParameters.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <vector>

void sendcommand(ros::Publisher command_pub, std::vector<double> target, std::vector<double> pos_gains, std::vector<double>rot_gains, std::vector<double>hand_offset){
    jaco_mp_io::ArmMotionPlanningCommand command;
    ros::Rate loop_rate(10);
    /**
    message structure:
    geometry_msgs/Pose hand_goal
    bool load_pc
    float32[3] pos_gains
    float32[3] rot_gains
    float32[3] hand_offset
    **/
    geometry_msgs::Pose pose;
    pose.position.x = target[0];//-0.1
    pose.position.y = target[1];//-0.5
    pose.position.z = target[2];//0.3
    pose.orientation.w = target[3];
    pose.orientation.x = target[4];
    pose.orientation.y = target[5];
    pose.orientation.z = target[6];
    command.hand_goal = pose;
    command.load_pc = true;
    command.pos_gains[0] = pos_gains[0];
    command.pos_gains[1] = pos_gains[1];
    command.pos_gains[2] = pos_gains[2];
    command.rot_gains[0] = rot_gains[0];
    command.rot_gains[1] = rot_gains[1];
    command.rot_gains[2] = rot_gains[2];
    command.hand_offset[0] = hand_offset[0];
    command.hand_offset[1] = hand_offset[1];
    command.hand_offset[2] = hand_offset[2];
    // geometry_msgs::Pose pose;
    // pose.position.x = -0.28;//-0.1
    // pose.position.y = -0.5;//-0.5
    // pose.position.z = -0.05;//0.3
    // pose.orientation.w = 0.70711;
    // pose.orientation.x = 0.70711;
    // pose.orientation.y = 0;
    // pose.orientation.z = 0;
    // command.hand_goal = pose;
    // command.load_pc = true;
    // command.pos_gains[0] = 1;
    // command.pos_gains[1] = 1;
    // command.pos_gains[2] = 1;
    // command.rot_gains[0] = 1;
    // command.rot_gains[1] = 0;
    // command.rot_gains[2] = 1;
    // command.hand_offset[0] = 0;
    // command.hand_offset[1] = 0;
    // command.hand_offset[2] = -0.17;

    while (command_pub.getNumSubscribers() == 0)
  {

    ros::spinOnce();
    loop_rate.sleep();

  }

    command_pub.publish(command);

    ros::spinOnce();

    ros::Duration(0.5).sleep();
}

void sendparameters(ros::Publisher problemparameters_pub, int mode, bool smoothing = false, bool save_pc = false, bool load_saved_pc = false, int num_step = 30){
/** message struct
int32 mode
int32 num_step
bool smoothing
bool save_pc
bool load_saved_pc
**/
  jaco_mp_io::SetProblemParameters problem_parameters;
  problem_parameters.mode = mode;
  problem_parameters.num_step = num_step;
  problem_parameters.smoothing = smoothing;
  problem_parameters.save_pc = save_pc;
  problem_parameters.load_saved_pc = load_saved_pc;
  problemparameters_pub.publish(problem_parameters);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "commandsender");

  ros::NodeHandle n("jaco_mp_io");

  ros::Publisher command_pub = n.advertise<jaco_mp_io::ArmMotionPlanningCommand>("trajectory_target",1,false);
  ros::Publisher loadobject_pub = n.advertise<jaco_mp_io::LoadObject>("load_object",1,false);
  ros::Publisher grabobject_pub = n.advertise<jaco_mp_io::GrabCommand>("grab",1,false);
  ros::Publisher problemparameters_pub = n.advertise<jaco_mp_io::SetProblemParameters>("problem_parameters",1,false);
  

  std::cout<< "JACO Arm Motion Planner (you can choose a motion you want):"<< std::endl;
  std::cerr<< "1 - grasp object; \t2 - move to human; \t3 - move back; \t4 - back to home pose"<< std::endl;

  int motionindex=100;

  jaco_mp_io::LoadObject loadobject_command;
  geometry_msgs::Pose object_pose;
  jaco_mp_io::GrabCommand grab_command;

  while(motionindex){
    std::cout<< "please enter your option:";
    std::cin >> motionindex;
    switch(motionindex){
      case 1:
      sendparameters(problemparameters_pub, 1);
      ros::Duration(0.5).sleep();
      sendcommand(command_pub,{-0.15,-0.47,-0.05,0.70711,0.70711,0,0},{1,1,1},{1,0,1},{0,0,-0.17});
      std::cin.get();
      break;

      case 2:      
      // loadobject_command.file_name.data = "cylinder";     
      // object_pose.position.x = -0.15;
      // object_pose.position.y = -0.47;
      // object_pose.position.z = -0.05;
      // object_pose.orientation.w = 0.70711;
      // object_pose.orientation.x = 0.70711;
      // object_pose.orientation.y = 0;
      // object_pose.orientation.z = 0;
      // loadobject_command.pose = object_pose;
      // loadobject_pub.publish(loadobject_command);
      // ros::Duration(0.5).sleep();
      
      // grab_command.file_name.data = "cylinder";
      // grab_command.grab = true;
      // grabobject_pub.publish(grab_command);
      // ros::Duration(0.5).sleep();
      sendparameters(problemparameters_pub, 2, true,true,false);
      ros::Duration(0.5).sleep();

      sendcommand(command_pub,{-0.48,-0.1,0.1,0.70711,0.70711,0,0},{1,1,1},{1,0,1},{0,0,-0.17});
      std::cin.get();
      break;

      case 3:   
      sendparameters(problemparameters_pub, 1, true,false,true);
      ros::Duration(0.5).sleep();   
      sendcommand(command_pub,{-0.26,-0.47,-0.05,0.70711,0.70711,0,0},{1,1,1},{1,0,1},{0,0,-0.17});
      std::cin.get();
      break;

      case 4:
      sendparameters(problemparameters_pub, 4);
      sendcommand(command_pub,{-0.48,-0.1,0.15,0.70711,0.70711,0,0},{1,1,1},{1,0,1},{0,0,-0.17});
      std::cin.get();
      break;

      default:
      std::cout<< "invalid choice"<<std::endl;
    }

  }


  return 0;
}