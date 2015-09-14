#include "ros/ros.h"
#include <jaco_mp_io/ArmMotionPlanningCommand.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "commandsender");

  ros::NodeHandle n("jaco_mp_io");

  ros::Publisher command_pub = n.advertise<jaco_mp_io::ArmMotionPlanningCommand>("trajectory_target", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    jaco_mp_io::ArmMotionPlanningCommand command;
    /**
    message structure:
    geometry_msgs/Pose hand_goal
    bool load_pc
    float32[3] pos_gains
    float32[3] rot_gains
    float32[3] hand_offset
    **/
    geometry_msgs::Pose pose;
    pose.position.x = -0.1;
    pose.position.y = -0.5;
    pose.position.z = 0.3;
    pose.orientation.w = 0.70711;
    pose.orientation.x = 0.70711;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    command.hand_goal = pose;
    command.load_pc = false;
    command.pos_gains[0] = 1;
    command.pos_gains[1] = 1;
    command.pos_gains[2] = 1;
    command.rot_gains[0] = 1;
    command.rot_gains[1] = 0;
    command.rot_gains[2] = 1;
    command.hand_offset[0] = 0;
    command.hand_offset[1] = 0;
    command.hand_offset[2] = -0.1;


    command_pub.publish(command);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}