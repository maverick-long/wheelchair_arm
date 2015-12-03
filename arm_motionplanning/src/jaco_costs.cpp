/**
 ********************************************************************************************************
 * @file    jaco_costs.cpp
 * @brief   TrajOpt Costs and Constrains for JACO Arm
 * @details Core code for handling the path planning costs and constrains
 ********************************************************************************************************
 */


/*** INCLUDE FILES ***/
#include <arm_motionplanning/jaco_traj.hpp>

using namespace jaco_traj;

void JACOTraj::SetTorqueCost(vector<double>& tau_cost,TrajoptMode mode){
	tau_cost.assign(robot->GetActiveDOF(),0.0);
	//set the Torque cost with the order:
	//		jaco_joint_1   jaco_joint_2   jaco_joint_3
	//		jaco_joint_4   jaco_joint_5   jaco_joint_6


	vector<double> majorjoints_cost;

	switch(mode){
	case TrajoptMode::GraspObject:
		majorjoints_cost = {
				0,	0,	0,	0,	0,	0
		};
		break;

	default:
		majorjoints_cost = {
				0,	0,	0,	0,	0,	0
		};

	}
	BuildJointCost(tau_cost,majorjoints_cost,mode);
}

void JACOTraj::SetVelCost(vector<double>& vel_cost,TrajoptMode mode){
	//set the velocity cost with the order:
	//		jaco_joint_1   jaco_joint_2   jaco_joint_3
	//		jaco_joint_4   jaco_joint_5   jaco_joint_6

	vel_cost.assign(robot->GetActiveDOF(),10.0);
	vector<double> majorjoints_cost;

	switch(mode){
	default:
		majorjoints_cost = {
			10,	10,	10,	10,	10,	0.1
		};

	}
	BuildJointCost(vel_cost,majorjoints_cost,mode);
}

void JACOTraj::BuildJointCost(vector<double>& vel_cost, vector<double>& majorjoints_cost,TrajoptMode mode){
	vector<int> majorjoints_index = Getactivejoint(mode);
	for(int i=0;i<majorjoints_index.size();i++){
		vel_cost[majorjoints_index[i]]=majorjoints_cost[i];
	}
}

void JACOTraj::SetPosCost(vector<double>& pos_cost,vector<double>& pos_vals,TrajoptMode mode){
	pos_cost.assign(robot->GetActiveDOF(),0.0);
	pos_vals.assign(robot->GetActiveDOF(),0.0);
	//set the joint pose cost with the order:
	//		jaco_joint_1   jaco_joint_2   jaco_joint_3
	//		jaco_joint_4   jaco_joint_5   jaco_joint_6
	vector<double> majorjoints_cost;
	vector<double> majorjoints_vals;
	switch(mode){

	default:
		majorjoints_cost = {
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0, 	0.0,	0.0,	0.0,	1.0,	0.0,	0.0,	0.0
		};
		majorjoints_vals = {
			0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	1.0,	1.0,	1.0,	1.0,	1.0,	1.0,	1.0
		};
	}

	BuildJointCost(pos_cost,majorjoints_cost,mode);

	BuildJointCost(pos_vals,majorjoints_vals,mode);
}

void JACOTraj::AddRequestHead(stringstream& request){
	request << "{\"basic_info\": { \
		  \"n_steps\" : "<< num_step <<", \
		  \"manip\" : \"active\", \
		  \"start_fixed\" : true \
		}, \
";
}

void JACOTraj::AddCostHead(stringstream& request, vector<double> vel_cost){
	request << "\"costs\" : [\
			{\
			  \"type\" : \"joint_vel\",\
			  \"params\": {\"coeffs\" : "<< convertDoubleVectortoString(vel_cost)<<"}\
";
}

void JACOTraj::AddContinueCollisionCost(stringstream& request, double coeffs, double dist_pen, int first_step, int last_step){
	request << "},\
			{\
			  \"type\" : \"collision\",\
			  \"name\" :\"cont_coll\",\
			  \"params\" : {\
			    \"continuous\" : true,\
			    \"coeffs\" : ["<< coeffs <<"],\
			    \"dist_pen\" : ["<< dist_pen <<"],\
			    \"first_step\" : "<<first_step<<",\
			    \"last_step\" : "<<last_step<<"\
			  }\
";
}

void JACOTraj::AddJointPositionCostorConstraint(stringstream& request, vector<double> pos_cost , vector<double> pos_vals, int time_step){
	request << "},\
			{\
			  \"type\" : \"joint_pos\",\
			  \"params\" : {\
			    \"coeffs\" : "<< convertDoubleVectortoString(pos_cost)<<",\
			    \"vals\" : "<< convertDoubleVectortoString(pos_vals)<<",\
			    \"timestep\" : "<< time_step <<"\
			  }\
";
}

void JACOTraj::AddTorqueCost(stringstream& request, vector<double> coeffs, int start_timestep, int end_timestep){
	for(int i=start_timestep;i<end_timestep+1;i++){
		request << "},\
				{\
				  \"type\" : \"static_torque\",\
				  \"params\" : {\
				    \"coeffs\" : "<< convertDoubleVectortoString(coeffs)<<",\
				    \"timestep\" : "<< i <<"\
				  }\
		";
	}
}

void JACOTraj::AddDiscontinueCollisionCost(stringstream& request, double coeffs, double dist_pen, int first_step, int last_step){
	request << "},\
			{\
			  \"type\" : \"collision\",\
			  \"name\" :\"disc_coll\",\
			  \"params\" : {\
			    \"continuous\" : false,\
			    \"coeffs\" : ["<< coeffs <<"],\
			    \"dist_pen\" : ["<< dist_pen <<"],\
			    \"first_step\" : "<<first_step<<",\
			    \"last_step\" : "<<last_step<<"\
			  }\
";
}

void JACOTraj::AddPoseCostorConstraint(stringstream& request, string linkname,vector<double> xyz, vector<double>wxyz, vector<double> pos_coeffs, vector<double> rot_coeffs, int start_timestep, int end_timestep, vector<double> offset){
	for(int i=start_timestep;i<end_timestep+1;i++){
	  request << "},\
	  {\
	  \"type\" : \"pose\",\
	  \"name\" : \""<<linkname<<"_pose\",\
	  \"params\" : {\"xyz\" : "<< convertDoubleVectortoString(xyz) <<",\
	              \"wxyz\" : "<<convertDoubleVectortoString(wxyz)<<",\
	              \"link\": \""<< linkname << "\" ,\
	              \"offset\": "<< convertDoubleVectortoString(offset) <<",\
	              \"pos_coeffs\": "<< convertDoubleVectortoString(pos_coeffs) <<",\
	              \"rot_coeffs\": "<< convertDoubleVectortoString(rot_coeffs) <<",\
	              \"timestep\" : "<< i <<"\
	              }\
	";
	}
}

void JACOTraj::AddPositionRegionConstraint(stringstream& request, string linkname,vector<double> plane1, vector<double> plane2, double dist_coeff, int time_step){
	request << "},\
			  {\
			  \"type\" : \"pose_limit\",\
			  \"name\" : \""<<linkname<<"_pose_l\",\
			  \"params\" : {\"plane1\" : "<< convertDoubleVectortoString(plane1) <<",\
			              \"plane2\" : "<<convertDoubleVectortoString(plane2)<<",\
			              \"link\": \""<< linkname << "\" ,\
			              \"dist_coeff\": "<< dist_coeff <<",\
			              \"timestep\" : "<< time_step <<"\
			              }\
			";
}

void JACOTraj::AddCostEnd(stringstream& request){
	request<<"}\
	],\
";
}

void JACOTraj::AddConstraintHead(stringstream& request){
	request << "\"constraints\" : [\
";
	for(int i=1;i<num_step;i++){
		request <<"{\
	  \"type\" : \"zmp_munip\",\
	  \"name\" : \"zmp_"<< i <<"\",\
	  \"params\" : {\"planted_links\": [\"l_foot\",\"r_foot\"],\
	                \"timestep\" : "<< i <<"\
	  }\
	";
		if(i!=num_step-1){
			request<<"},\
		";
		}
	}
}

void JACOTraj::AddConstraintHead(stringstream& request, string linkname,vector<double> xyz, vector<double>wxyz, vector<double> pos_coeffs, vector<double> rot_coeffs, int start_timestep, int end_timestep, vector<double> offset){
	request << "\"constraints\" : [\
";
	for(int i=start_timestep;i<end_timestep+1;i++){
	  request << "{\
	  \"type\" : \"pose\",\
	  \"name\" : \""<<linkname<<"_pose\",\
	  \"params\" : {\"xyz\" : "<< convertDoubleVectortoString(xyz) <<",\
	              \"wxyz\" : "<<convertDoubleVectortoString(wxyz)<<",\
	              \"link\": \""<< linkname << "\" ,\
	              \"offset\": "<< convertDoubleVectortoString(offset) <<",\
	              \"pos_coeffs\": "<< convertDoubleVectortoString(pos_coeffs) <<",\
	              \"rot_coeffs\": "<< convertDoubleVectortoString(rot_coeffs) <<",\
	              \"timestep\" : "<< i <<"\
	              }\
	";
	if(i!=end_timestep){
			request<<"},\
		";
		}
	}
}

void JACOTraj::AddJointPrime(stringstream& request, TrajoptMode mode, int start_timestep, int end_timestep){

	vector<double> jointprime;
	vector<double> jointcoeff;

	switch(mode){
		case TrajoptMode::ReturntoHomePose:
			jointcoeff = {1.0,1.0,1.0,1.0,1.0,1.0};
			jointprime = {-1.794, -2.009, 0.8117, -0.878, 1.695, 3.190};
			break;

		default:
			jointcoeff = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			jointprime = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	}

	vector<double> pos_vals  = GetWholeJoint(robot, jointprime, Getactivejoint(TrajoptMode::Default));
	vector<double> coeffs = GetWholeJoint(robot, jointcoeff, Getactivejoint(TrajoptMode::Default));

	vector<double> joint = concatenate_vectors(pos_vals,TransformtoVector(robot->GetTransform()));
	vector<double> joint_coeffs = concatenate_vectors(coeffs,{1,1,1,1,1,1,1});

	for(int i=start_timestep;i<end_timestep+1;i++){
		request << "},\
		{\
			\"type\" : \"joint\",\
			\"params\" : {\
				\"vals\" : "<< convertDoubleVectortoString(joint)<<",\
				\"coeffs\" : "<< convertDoubleVectortoString(joint_coeffs)<<",\
				\"timestep\" : "<< i <<"\
			}\
			";
	}
}

void JACOTraj::AddConstraintEnd(stringstream& request,Eigen::MatrixXf traj){
	request << "}\
		],\
		\"init_info\" : {\
		    \"type\" : \"given_traj\",\
		    \"data\" : " << convertMatrixtoString(traj) << "\
		}\
		}";
}
