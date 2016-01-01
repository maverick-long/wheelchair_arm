#include <arm_motionplanning/jaco_utils.hpp>

using namespace OpenRAVE;

std::string convertMatrixtoString(Eigen::MatrixXf &traj){
  std::string s = "[";
  int num_col = traj.cols();
  int num_row = traj.rows();
  for(int i = 0;i<num_row;i++) {
    s.append("[");
    for(int j = 0;j<num_col;j++) {
      s.append(std::to_string(traj(i,j)));
      if(j!=num_col-1) s.append(",");
    }
    s.append("]");
    if(i!=num_row-1) s.append(",");
  }
  return s.append("]");
}

void Sleep(float s)
{
    int sec = int(s*1000000);
    usleep(sec);
}

std::vector<double> getJointValuefromTraj(trajopt::TrajArray row){
  std::vector<double> jointvalue(row.size());
	for(int i=0;i<row.size();i++){
		jointvalue[i]=row(i);
	}
	return jointvalue;
}

Transform getTransformfromTraj(trajopt::TrajArray row){
	std::cout<< row(row.size()-7)<<" "<<row(row.size()-6)<<" "<<row(row.size()-5)<<" "<<row(row.size()-4)<<" "<<row(row.size()-3)<<" "<<row(row.size()-2)<<" "<< row(row.size()-1)<<std::endl;
	Transform obj_transform = Transform(Vector(row(row.size()-3),row(row.size()-2),row(row.size()-1),row(row.size()-4)),Vector(row(row.size()-7),row(row.size()-6),row(row.size()-5)));
	return obj_transform;
}

std::vector<double> GetMainJoint(std::vector<double> traj,std::vector<int> activejoint){
	std::vector<double> jointstates;
	for(int i=0;i<activejoint.size();i++)
		jointstates.push_back(traj[activejoint[i]]);
	return jointstates;
}

std::vector<double> GetWholeJoint(RobotBasePtr robot, const std::vector<double>& jointvalue, const std::vector<int>& jointindex){
	int numJoint = robot->GetDOF();
	std::vector<double> whole_joint(numJoint,0.0);
	for(int i = 0; i<jointvalue.size();i++){
		whole_joint[jointindex[i]] = jointvalue[i];
	}

	//add finger state
	std::vector<int> fingerjoint = {robot->GetJoint("jaco_joint_finger_1")->GetDOFIndex(),robot->GetJoint("jaco_joint_finger_2")->GetDOFIndex(),robot->GetJoint("jaco_joint_finger_3")->GetDOFIndex()};
	std::vector<double> finger_state;
	robot->GetDOFValues(finger_state,fingerjoint);
	for(int i=0;i<finger_state.size();i++){
		whole_joint[fingerjoint[i]]=finger_state[i];
	}
	return whole_joint;
}

void vector_replace(std::vector<double>& raw_vector, int start, int end, double value){
  for(int i=start; i<end; i++){
    raw_vector[i] = value;
  }
}

std::vector<double> TransformtoVector(Transform T){
  std::vector<double> v = {T.trans.x,T.trans.y,T.trans.z,T.rot.x,T.rot.y,T.rot.z,T.rot.w};
  return v;
}


Eigen::MatrixXf build_traj(const std::vector<double>& pose1, const std::vector<double>& pose2, int num_step){
  int num_row = pose1.size();
  Eigen::MatrixXf traj(num_step,num_row);
  for(int i=0; i< num_row; i++){
	  if(num_step!=1){
		  traj.col(i) << Eigen::VectorXf::LinSpaced(num_step,pose1[i],pose2[i]);
	  }else{
		  traj.col(i) << pose1[i];
	  }
  }
  return traj;
}


std::vector<double> concatenate_vectors(const std::vector<double>& a, const std::vector<double>& b){
  std::vector<double> v = a;
  v.insert(v.end(),b.begin(),b.end());
  return v;

}


std::vector<int> vector_arange (int max){
  std::vector<int> v(max);
  for (int i=0; i<max;i++){
    v[i]=i;
  }
  return v;
}

std::string convertDoubleVectortoString(std::vector<double>& v){
	std::string s = "[";
	for(std::vector<double>::iterator it = v.begin(); it != v.end(); ++it) {
		if(it != v.begin())s.append(",");
		s.append(std::to_string(*it));
	}
	return s.append("]");
}

Eigen::VectorXd linspace(double a, double b, int n) {
	Eigen::VectorXd array(n);
    double step = (b-a) / (n-1);

    for(int i =0; i < n; i ++){
    	array[i] = a;
    	a += step;
    }
    return array;
}

int traj_is_safe(trajopt::TrajArray& traj, OpenRAVE::RobotBasePtr robot, int n){
/*    Returns the set of collisions.
    manip = Manipulator or list of indices
	*/
	int collision_times = 0;

	//Interpolate trajectory:
	trajopt::TrajArray trap_up = trajopt::interp2d(linspace(0,1,n),linspace(0,1,traj.rows()),traj);

	OpenRAVE::EnvironmentBasePtr env = robot->GetEnv();

	//Testing:
/*	std::cout<< "traj " << traj <<std::endl;
	std::cout<< "traj up! " << trap_up <<std::endl;
	std::cout<< "traj size " << trap_up.rows() << " by " << trap_up.cols() <<std::endl;*/

	//Check for collisions on interpolated trajectory
	std::vector<double> res(trap_up.cols());
	for(int n = 0 ; n < trap_up.rows() ; n++)
	{
		for(int i = 0; i < trap_up.cols(); i++)
		{
			res[i] = trap_up(n,i);
		}
		robot->SetActiveDOFValues(res);
		if (env->CheckCollision(robot)) collision_times +=1;
		CollisionReportPtr report(new CollisionReport());
		env->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
		int contactpoints = 0;
		if( robot->CheckSelfCollision(report) ) {
			if((report->plink1->GetName().compare("mtorso") && report->plink1->GetName().compare("pelvis")) &&
					(report->plink1->GetName().compare("pelvis") && report->plink1->GetName().compare("mtorso")) &&
					(report->plink1->GetName().compare("utorso") && report->plink1->GetName().compare("hokuyo_link")) &&
					(report->plink1->GetName().compare("hokuyo_link") && report->plink1->GetName().compare("utorso"))){
				contactpoints = (int)report->contacts.size();
				std::stringstream ss;
				ss << "body in self-collision "
						<< (!!report->plink1 ? report->plink1->GetName() : "") << ":"
						<< (!!report->plink2 ? report->plink2->GetName() : "") << " at "
						<< contactpoints << "contacts" << std::endl;
				for(int i = 0; i < contactpoints; ++i) {
					CollisionReport::CONTACT& c = report->contacts[i];
					ss << "contact" << i << ": pos=("
							<< c.pos.x << ", " << c.pos.y << ", " << c.pos.z << "), norm=("
							<< c.norm.x << ", " << c.norm.y << ", " << c.norm.z << ")" << std::endl;
				}
				RAVELOG_INFOA(ss.str());
				collision_times +=1;
			}
			else {
				RAVELOG_INFO("body not in collision\n");
			}
		}
	}
	return collision_times;
}
