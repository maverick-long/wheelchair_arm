/**
 ********************************************************************************************************
 * @file    jaco_traj.cpp
 * @brief   JACO arm motion planning class
 * @details Core code for generating manipulation trajectories for JACO arm
 ********************************************************************************************************
 */


/*** INCLUDE FILES ***/
#include <arm_motionplanning/jaco_traj.hpp>

using namespace jaco_traj;


JACOTraj::JACOTraj():robot_name("jaco"),see_viewer(false),idle_viewer(false),num_step(15),multi_init_guess(false),request_str(new std::stringstream()),
	collision_cost(100.0) ,dist_pen(0.05), current_mode(TrajoptMode::Default), pos_gains{1,1,1},rot_gains{1,1,1},hand_offset{0.1,0.0,0.0},load_waypoints(false),smooth_traj(false)
{
	Initialize();

	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

JACOTraj::~JACOTraj()
{
	env->Destroy();
}

void JACOTraj::Initialize()
{
	pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
	std::cout << "Initializing TrajOpt" << std::endl;
	//trajopt::RegisterHumanoidCostsAndCnts();
	RaveInitialize(true);

	env = OpenRAVE::RaveCreateEnvironment();
	assert(env);
	env->SetDebugLevel(Level_Debug);
	LoadRobot();
}

void JACOTraj::Defaults()
{
	this->robot_name = "jaco";
	this->see_viewer = false;
	this->idle_viewer = false;
	this->num_step = 15;
	this->multi_init_guess = false;
	this->request_str.reset(new std::stringstream());
	this->collision_cost = 100.0;
	this->dist_pen = 0.05;
	this->current_mode = TrajoptMode::Default;
	this->pos_gains = {1,1,1};
	this->rot_gains = {1,1,1};
	this->hand_offset = {0.1,0.0,0.0};
	this->load_waypoints = false;
}

void JACOTraj::LoadRobot()
{
	env->Load("jaco_arm_description/jaco_arm.dae");
	robot = env->GetRobot(robot_name);
	assert(robot);

	//Transform init_transform = Transform(lFootFrame);
	//robot->SetTransform(init_transform);
}

void JACOTraj::LoadGains(Eigen::Vector3d pos_gains, Eigen::Vector3d rot_gains, Eigen::Vector3d hand_offset)
{

	this->pos_gains = {pos_gains.x(),pos_gains.y(),pos_gains.z()};
	this->rot_gains = {rot_gains.x(),rot_gains.y(),rot_gains.z()};
	this->hand_offset = {hand_offset.x(),hand_offset.y(),hand_offset.z()};

}


vector<int> JACOTraj::Getactivejoint(TrajoptMode mode){
	vector<int> activejoint = {robot->GetJoint("jaco_joint_1")->GetDOFIndex(),robot->GetJoint("jaco_joint_2")->GetDOFIndex(),
							   robot->GetJoint("jaco_joint_3")->GetDOFIndex(),robot->GetJoint("jaco_joint_4")->GetDOFIndex(),
							   robot->GetJoint("jaco_joint_5")->GetDOFIndex(),robot->GetJoint("jaco_joint_6")->GetDOFIndex()};
	return activejoint;
}

trajopt::TrajArray JACOTraj::ComputeTrajectory(vector<double> start_state, Eigen::Affine3d hand)
{
	std::cout << "Computing trajectory" << std::endl;

	OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env->GetMutex());
	//Handle viewer:
	if(see_viewer) viewer = OSGViewer::GetOrCreate(env);

	vector<int> activejoint = Getactivejoint(current_mode);
	int current_active_dof = robot->GetActiveDOF();

	UpdateStateForRequest(start_state, hand, current_mode);

	if(see_viewer && idle_viewer)viewer->Idle();

	vector< Eigen::MatrixXf > initguess;
	if(load_waypoints){ //Load waypoints from file
		vector< vector<double> >way_points;
		LoadWaypoints(way_points,load_waypoints_file_path);
		initguess = GenerateInitGuess(multi_init_guess,start_state, target_state, activejoint,way_points);
	}else{	//OR:
		initguess = GenerateInitGuess(multi_init_guess,start_state, target_state, activejoint);
	}

	Eigen::Affine3d  global_trans= Eigen::Affine3d::Identity();
	/*Rotate the hand goal with the robot*/
	// Transform init_transform = Transform(lFootFrame);
	// global_trans.translate(Eigen::Vector3d(init_transform.trans.x, init_transform.trans.y ,init_transform.trans.z));
	// global_trans.linear() = Eigen::Quaterniond(init_transform.rot.x,init_transform.rot.y,init_transform.rot.z,init_transform.rot.w).toRotationMatrix();
	Eigen::Affine3d hand_goal;
	hand_goal = global_trans*hand;

	if(multi_init_guess && viewer && see_viewer)ClearViewer();

	int Num_threads = initguess.size();
	pthread_t threads[Num_threads];
	struct thread_data td[Num_threads];

	stringstream request;
	for(int i=0;i<Num_threads;i++){

		request.str(std::string()); // Clear the request
		td[i].thread_id = i;
		td[i].thread_env = env->CloneSelf(Clone_Bodies);
		td[i].thread_see_viewer = see_viewer;
		td[i].isdone = false;
		td[i].thread_robot_name = robot_name;
		request_traj = initguess[i];
		cout<< "Number of rows: " << request_traj.rows() << "Number of col: " << request_traj.cols() <<endl;

		ComposeRequest(request, current_mode, hand_goal);

		if(multi_init_guess){
			td[i].thread_request<<request.str();
			int rc = pthread_create(&threads[i],NULL,&JACOTraj::LaunchSolver,&td[i]);
			if (rc){
				cout << "Error:unable to create thread," << rc << endl;
				exit(-1);
			}
		}else{
			break;
		}
	}

	int target_tread = 0;

	trajopt::TrajArray traj;

	if(multi_init_guess)
	{
		struct thread_info ti;
		ti.pthread_data = td;
		ti.num_thread = Num_threads;
		pthread_t threadmanager;
		int rc = pthread_create(&threadmanager,NULL,&JACOTraj::ManageThread,&ti);
		if (rc){
				cout << "Error:unable to create thread," << rc << endl;
				exit(-1);
			}
		pthread_join(threadmanager,NULL);
		cout<<"trajopt done"<<endl;
		target_tread = ti.target_tread;

		traj = td[target_tread].traj;
	}
	else{
/*		for(int i=0;i<Num_threads;i++){
			pthread_join(threads[i],NULL);
		}*/

	// Parsing:
		Json::Value root;
		Json::Reader reader;
		bool parsedSuccess = reader.parse(request,
		                                 root,
		                                 false);

		if(not parsedSuccess)
		{
		 cerr<<"Failed to parse JSON"<<endl
		     <<reader.getFormatedErrorMessages()
		     <<endl;
		}

		if(see_viewer)
		{
			viewer->UpdateSceneData();
			viewer->Draw();
		}

		robot->SetActiveDOFs(vector_arange(current_active_dof),DOF_Transform);

		trajopt::TrajOptProbPtr prob = trajopt::ConstructProblem(root, env);

		//Add the configuration bias towards a particular pose. Not to be use right now:
/*		switch(current_mode)
		{
			case TrajoptMode::Door:
				std::cout << "!!!!!! Add configuration bias !!!!!!" << std::endl;
				AddConfigurationBias(prob,activejoint, num_step-1,num_step-1 );
				break;
			default:
				break;
		}
*/
		trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(prob, viewer);

/*		int not_safe = traj_is_safe(result->traj,robot);

		cout<< "Number of collisions: " << not_safe <<endl;

		if(not_safe != 0)
		{
			return traj;
		}*/

		traj = result->traj;
	}

	if(see_viewer && idle_viewer)viewer->Idle();

	PrintTraj(traj,activejoint);

	//SaveTraj(traj,activejoint,"trajectory.txt");

	if(smooth_traj){
		int issuccessful;
		issuccessful = Smoothing(traj,100,true);
		if(issuccessful ==0){
			cout<<"smoothing fails"<<endl;
			smooth_traj = 0;
		}
		if(see_viewer && idle_viewer)viewer->Idle();
	}

	if(see_viewer && !smooth_traj){
		ShowTraj(traj);
	}
	
	return traj;

}

void * JACOTraj::LaunchSolver(void *threadarg){
	struct thread_data *my_data;
	my_data = (struct thread_data *) threadarg;

	EnvironmentBasePtr thread_env = my_data->thread_env;
	RobotBasePtr thread_robot = thread_env->GetRobot(my_data->thread_robot_name);
	assert(thread_robot);

	//OSGViewerPtr thread_viewer = OSGViewer::GetOrCreate(thread_env);

	// Parsing:
	Json::Value root;
	Json::Reader reader;
	bool parsedSuccess = reader.parse(my_data->thread_request,
	                                 root,
	                                 false);

	if(not parsedSuccess)
	{
	 cerr<<"Failed to parse JSON"<<endl
	     <<reader.getFormatedErrorMessages()
	     <<endl;
	}

//	if(my_data->thread_see_viewer)
//	{
//		thread_viewer->UpdateSceneData();
//		thread_viewer->Draw();
//		Sleep(1);
//	}

//	robot->SetActiveDOFs(vector_arange(current_active_dof),DOF_Transform);
	trajopt::TrajOptProbPtr prob = trajopt::ConstructProblem(root, thread_env);

	//Add the configuration bias towards a particular pose. Not to be use right now:
	//AddConfigurationBias(prob,activejoint);

	trajopt::TrajOptResultPtr result = trajopt::OptimizeProblem(prob, my_data->thread_see_viewer);

	int not_safe = traj_is_safe(result->traj,thread_robot);

	cout<< "Number of collisions: " << not_safe <<endl;


	if(! not_safe)
	{
		trajopt::TrajArray traj = result->traj;
//		int traj_size = traj.rows();
//		for(int i = 0;i<traj_size;i++)
//		{
//			vector<double> jointstate = getJointValuefromTraj(traj.row(i));
//			thread_robot->SetActiveDOFValues(jointstate);
//			if(my_data->thread_see_viewer)thread_viewer->UpdateSceneData();
//			if(my_data->thread_see_viewer)thread_viewer->Draw();
//			if(my_data->thread_see_viewer)Sleep(0.2);
//		}
		my_data->isdone = true;
		my_data->traj =  traj;
	}
	thread_env->Destroy();
	pthread_exit(NULL);
}

void * JACOTraj::ManageThread(void *threadarg){
	struct thread_info *my_data;
	my_data = (struct thread_info *) threadarg;
	while(1){
		int flag = 0;
		for(int i=0;i<my_data->num_thread;i++){
			if(((my_data->pthread_data)+i)->isdone){
				cout<<"thread "<<i<<" done."<<endl;
				my_data->target_tread = i;
				flag = 1;
				break;
			}
		}
		if(flag==1)break;
		Sleep(1);
	}
	pthread_exit(NULL);
}

void JACOTraj::UpdateStateForRequest(vector<double>& start_state, Eigen::Affine3d hand, TrajoptMode mode)
{

	vector<int> activejoint = Getactivejoint(mode);

	robot = env->GetRobot("jaco");
	robot->SetDOFValues(start_state,1,activejoint);
	robot->GetDOFValues(start_state,activejoint);

	int current_active_dof = robot->GetActiveDOF();
	robot->SetActiveDOFs(vector_arange(current_active_dof),DOF_Transform);

	Eigen::Vector3d hand_pos = hand.translation();
	Eigen::Quaterniond hand_q = Eigen::Quaterniond(hand.linear());
	/*	calculate target state*/
	// target_state = manipInt.calculateEndState(start_state,hand,side, rot_gains);
	target_state = start_state;

	Eigen::Affine3d  global_trans= Eigen::Affine3d::Identity();
	/*  add transform for uneven ground */
	// Transform init_transform = Transform(lFootFrame);
	// global_trans.translate(Eigen::Vector3d(init_transform.trans.x, init_transform.trans.y ,init_transform.trans.z));
	// global_trans.linear() = Eigen::Quaterniond(init_transform.rot.x,init_transform.rot.y,init_transform.rot.z,init_transform.rot.w).toRotationMatrix();
	Eigen::Affine3d hand_goal;
	hand_goal = global_trans*hand;


	hand_pos = hand_goal.translation();
	hand_q = Eigen::Quaterniond(hand_goal.linear());


	xyz_target = {hand_pos.x(),hand_pos.y(),hand_pos.z()};//{0.82, 0.21, 1.23373}
	quat_target = {hand_q.w(), hand_q.x(), hand_q.y(), hand_q.z()};

	hand_str = "jaco_link_hand";

	SetVelCost(vel_cost,mode);
	SetPosCost(pos_cost,pos_vals,mode);
}


void JACOTraj::ComposeRequest(stringstream& request,TrajoptMode mode, Eigen::Affine3d hand)
{
	Vector temp = robot->GetLink(hand_str)->GetTransform()*Vector(hand_offset[0], hand_offset[1], hand_offset[2]);

	vector<double> hand_pose = TransformtoVector(robot->GetLink("jaco_link_hand")->GetTransform());

	vector<double> jointprime;
	vector<double> jointcoeff;

	Vector unitstep((xyz_target[0] - temp.x)/(num_step-1),(xyz_target[1] - temp.y)/(num_step-1),(xyz_target[2] - temp.z)/(num_step-1));


	OpenRAVE::Transform pose = robot->GetLink(hand_str)->GetTransform();

	vector<double> current_xyz_target = {temp.x,temp.y,temp.z};
	// eg geometry.x = openrave.w, geometry.y = openrave.x, geometry.z = openrave.y, geometry.w = openrave.z
	//	quat_target = {hand_q.w(), hand_q.x(), hand_q.y(), hand_q.z()};
	vector<double> current_quat_target = {pose.rot.x,pose.rot.y,pose.rot.z,pose.rot.w};

	switch(mode)
	{
		default:
			AddRequestHead(request);
			AddCostHead(request,vel_cost);
			AddContinueCollisionCost(request,collision_cost,dist_pen,0,num_step-1);
			AddDiscontinueCollisionCost(request,collision_cost,dist_pen,0,num_step-1);
			//AddPoseCostorConstraint(request,hand_str,{0.3,-0.089,0},{1,0,0,0},{0.01,0.01,0},{0,0,0},1,num_step-1,{0,0,0});
			//AddTorqueCost(request,tau_cost,1,num_step-1);
			//AddPoseCostorConstraint(request,hand_str,xyz_target,quat_target,{0.1,0.1,0.1},{0,0,0},1,num_step-1,hand_offset);
			//AddJointPositionCostorConstraint(request, pos_cost , pos_vals);
			AddCostEnd(request);
			AddConstraintHead(request,hand_str,xyz_target,quat_target,pos_gains,rot_gains,num_step-1,num_step-1,hand_offset);
			AddJointPrime(request, mode, 1, num_step-1);
			// AddPoseCostorConstraint(request,hand_str,xyz_target,quat_target,pos_gains,rot_gains,num_step-1,num_step-1,hand_offset);
			AddConstraintEnd(request,request_traj);
			break;
	}

}

vector< Eigen::MatrixXf > JACOTraj::GenerateInitGuess(bool multi_initguess,vector<double> start_state, vector<double> target_state, vector<int> activejoint){
	vector<double> affinetran = TransformtoVector(robot->GetTransform());

	vector<double> pose1 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	// vector<double> pose1 = {1.0 , -1.5, 0.5, -1.57, 0.0, 0.0};

	vector<double> pose2 = {-0.00100474 ,0.000482999 ,-0.00182004 ,-7.23329e-23 ,0.00212896 ,0.0557431};
	vector< vector<double> > waypoints = {concatenate_vectors(GetWholeJoint(robot, start_state, activejoint),affinetran)};
	vector< Eigen::MatrixXf > initguess;
	if (multi_initguess){
		waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, target_state, activejoint),affinetran));
		initguess.push_back(Buildtraj(waypoints));
		waypoints.clear();
		waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, start_state, activejoint),affinetran));
		waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, pose1, activejoint),affinetran));
		waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, pose2, activejoint),affinetran));
		waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, target_state, activejoint),affinetran));
		initguess.push_back(Buildtraj(waypoints));
		waypoints.clear();
		waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, start_state, activejoint),affinetran));
		waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, pose1, activejoint),affinetran));
		waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, target_state, activejoint),affinetran));
		initguess.push_back(Buildtraj(waypoints));
	}else{
		waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, pose1, activejoint),affinetran));
		// waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, target_state, activejoint),affinetran));
		initguess.push_back(Buildtraj(waypoints));
	}
	return initguess;
}

vector< Eigen::MatrixXf > JACOTraj::GenerateInitGuess(bool multi_initguess,vector<double> start_state, vector<double> target_state, vector<int> activejoint, vector< vector<double> >poses){
	vector<double> affinetran = TransformtoVector(robot->GetTransform());
	vector< vector<double> > waypoints = {concatenate_vectors(GetWholeJoint(robot, start_state, activejoint),affinetran)};
	vector< Eigen::MatrixXf > initguess;

	for(int i=1;i<poses.size();i++){
		waypoints.push_back(concatenate_vectors(GetWholeJoint(robot, poses[i], activejoint),affinetran));
	}
	initguess.push_back(Buildtraj(waypoints));

	return initguess;
}

Eigen::MatrixXf JACOTraj::Buildtraj(vector< vector<double> > waypoints){
	double waypoint_steps = num_step*1.0/(waypoints.size()-1);
	Eigen::MatrixXf traj(num_step,robot->GetActiveDOF());
	int traj_index = 0;
	int waypoints_index = 0;
	while(traj_index!=num_step){
//		cout<<waypoint_steps*(waypoints_index+1)<<endl;
		int temp = round(waypoint_steps*(waypoints_index+1));
//		cout<<temp<<endl;
		if(temp-traj_index>1){
//			printcoll(waypoints[waypoints_index]);
			traj.block(traj_index,0,temp-traj_index,robot->GetActiveDOF()) = build_traj(waypoints[waypoints_index],waypoints[waypoints_index+1],temp-traj_index);
		}else if(temp-traj_index==1){
//			printcoll(waypoints[waypoints_index]);
			traj.block(traj_index,0,temp-traj_index,robot->GetActiveDOF()) = build_traj(waypoints[waypoints_index],waypoints[waypoints_index],temp-traj_index);
		}
		traj_index = temp;
		waypoints_index++;
	}
	return traj;
}

int JACOTraj::Smoothing(trajopt::TrajArray traj,int sample_rate,bool launchviewer){
	TrajectoryBasePtr openravetraj = RaveCreateTrajectory(env,"");
	openravetraj->Init(robot->GetActiveConfigurationSpecification());
	int traj_size = traj.rows();
	for(int i = 0;i<traj_size;i++){
		openravetraj->Insert(i,getJointValuefromTraj(traj.row(i)));
	}
	int issuccessful;
	issuccessful = planningutils::SmoothActiveDOFTrajectory(openravetraj,robot);
	if(issuccessful == 1){
	std::ofstream outfile("traj.txt",ios::out);
	outfile.precision(16);
	openravetraj->serialize(outfile);
	outfile.close();
	cout<<"smoothing finished. waypoints: "<<openravetraj->GetNumWaypoints()<<endl;
	ConfigurationSpecification::Group joint_values_group = openravetraj->GetConfigurationSpecification().GetGroupFromName("joint_values jaco 0 1 2 3 4 5 6 7 8");
	ConfigurationSpecification joint_values_config(joint_values_group);
	double unit = openravetraj->GetDuration()/sample_rate;
	final_traj.resize(sample_rate);
	for(int i=0;i<sample_rate;i++){
		vector<double> data;
		openravetraj->Sample(data,unit*i,joint_values_config);
		final_traj[i].resize(6);
		for(int j=0;j<6;j++){
			final_traj[i][j] = data[j];
		}
		printcoll(final_traj[i]);
	}
	if(launchviewer){
	viewer = OSGViewer::GetOrCreate(env);
	for(int i=0;i<sample_rate;i++){
		vector<double> jointstate;
		openravetraj->Sample(jointstate,unit*i);
		robot->SetActiveDOFValues(jointstate);
		viewer->UpdateSceneData();
		viewer->Draw();
		Sleep(0.1);		
	}
	}
}
return issuccessful;
}

void JACOTraj::SeeViewer(bool see)
{
	this->see_viewer = see;
}

void JACOTraj::IdleViewer(bool idle)
{
	this->idle_viewer = idle;
}

void JACOTraj::ClearViewer(){
	vector<KinBodyPtr> bodies;
	env->GetBodies(bodies);
	for(int i=0;i<bodies.size();i++){
		viewer->RemoveKinBody(bodies[i]);
	}
}

void JACOTraj::SetNumStep(int num){
	this->num_step = num;
}

void JACOTraj::SetMode(TrajoptMode mode){
	std:cout << "Set Mode to " << (int)mode << std::endl;
	this->current_mode = mode;
}

void JACOTraj::SetMultiInit(bool multi_init){
	this->multi_init_guess = multi_init;
}

void JACOTraj::SetSmoothing(bool smooth_traj){
	this->smooth_traj = smooth_traj;
}

void JACOTraj::Load(std::string& object_name)
{
	std::stringstream ss;
	ss << "/data/" << object_name << ".xml";

	env->Load(ss.str());
}

void JACOTraj::TransformObject(std::string& object, Eigen::Affine3d trans)
{

	// Transform init_transform = Transform(lFootFrame);
	Eigen::Affine3d  global_trans= Eigen::Affine3d::Identity();
	// global_trans.translate(Eigen::Vector3d(init_transform.trans.x, init_transform.trans.y ,init_transform.trans.z));
	// global_trans.linear() = Eigen::Quaterniond(init_transform.rot.x,init_transform.rot.y,init_transform.rot.z,init_transform.rot.w).toRotationMatrix();
	Eigen::Affine3d obj_trans;
	obj_trans = global_trans*trans;

	KinBodyPtr obj = env->GetKinBody(object);
	Eigen::Vector3d pos = obj_trans.translation();
	Eigen::Quaterniond q = Eigen::Quaterniond(obj_trans.linear());

	Transform obj_transform = Transform(Vector(q.w(),q.x(),q.y(),q.z()),Vector(pos.x(),pos.y(),pos.z()));

	obj->SetTransform(obj_transform);
}

void JACOTraj::ResetEnvironment()
{
	env->Destroy();
	env = OpenRAVE::RaveCreateEnvironment();
	LoadRobot();
	Defaults();
}

void JACOTraj::GetFinalTraj(vector< vector<double> >& final_traj){
	final_traj = this->final_traj;
}