#include <jaco_mp_io/JointStateReceiver.h>

namespace jaco{

JointStateReceiver::JointStateReceiver(ros::NodeHandle &nh, std::string subscribeTo):
currentJoints(new std::vector<double>),
latestJoints(new sensor_msgs::JointState)
{

	latestJoints->position = {-1.794,-2.009,0.8117,-0.878,1.695,3.190};

	nh_ = nh;

	subscriber = nh_.subscribe(subscribeTo, 1, &JointStateReceiver::jointMessageCallback, this);
	//while(subscriber.getNumPublishers() == 0);
}

boost::shared_ptr<std::vector<double>> JointStateReceiver::updateJoints(){
	ros::spinOnce();

	*currentJoints = latestJoints->position ;
	return currentJoints;

}

void JointStateReceiver::jointMessageCallback(sensor_msgs::JointStatePtr data){
	for(int i = 0; i <5 ; i++)
	{
		latestJoints->position[i] = data->position[i];
	}
}

}//namespace jaco