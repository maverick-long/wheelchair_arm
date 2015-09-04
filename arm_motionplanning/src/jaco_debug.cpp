/**
 ********************************************************************************************************
 * @file    jaco_debug.cpp
 * @brief   TrajOpt Debugging and Printing functions
 * @details Helper functions
 ********************************************************************************************************
 */

/*** INCLUDE FILES ***/
#include <arm_motionplanning/jaco_traj.hpp>

using namespace jaco_traj;

void JACOTraj::ShowTraj(trajopt::TrajArray traj){
	viewer = OSGViewer::GetOrCreate(env);
	int traj_size = traj.rows();
	for(int i = 0;i<traj_size;i++)
	{
		vector<double> jointstate = getJointValuefromTraj(traj.row(i));
		robot->SetActiveDOFValues(jointstate);
		viewer->UpdateSceneData();
		viewer->Draw();
		Sleep(0.2);
	}
	if(see_viewer && idle_viewer)viewer->Idle();
}

void JACOTraj::PreviewTraj(trajopt::TrajArray traj,vector<int> activejoint){
	viewer = OSGViewer::GetOrCreate(env);
  	vector<GraphHandlePtr> handles;

  for (int i=0; i < traj.rows() ; ++i) {
  	vector<double> jointstate = getJointValuefromTraj(traj.row(i));
    robot->SetDOFValues(jointstate, 1, activejoint);
    handles.push_back(viewer->PlotKinBody(robot));
    if( !((i == 0) || (i == traj.rows() -1)))SetTransparency(handles.back(), .3);
  }
  for(int i=1;i<=1000;i++){ // 30 seconds
  	viewer->UpdateSceneData();
  	viewer->Draw();
  	Sleep(0.03);
  }  	
}

void JACOTraj::PrintTraj(trajopt::TrajArray traj,vector<int> activejoint){
	int traj_size = traj.rows();
	for(int i = 0;i<traj_size;i++)
	{
		vector<double> jointstate = getJointValuefromTraj(traj.row(i));
		printcoll(GetMainJoint(jointstate,activejoint));

	}
}

void JACOTraj::SaveTraj(trajopt::TrajArray traj, string textdir, TrajoptMode traj_mode){
	vector<int> activejoint = Getactivejoint(traj_mode);

	std::ofstream out_file;
	out_file.open(textdir);
	int traj_size = traj.rows();
	for(int i=0;i<traj_size;i++){
		vector<double> jointstate = GetMainJoint(getJointValuefromTraj(traj.row(i)),activejoint);
		for(int j=0;j<jointstate.size();j++){
			out_file<<jointstate[j]<<" ";
		}
		out_file<<endl;
	}
	out_file.close();
	cout<<"save traj"<<endl;
}

void JACOTraj::SetWaypointsFromFile(string textd_dir){
	load_waypoints_file_path = textd_dir;
	load_waypoints = true;
}

bool JACOTraj::LoadWaypoints(trajopt::TrajArray& traj, string textdir){

	vector<int> activejoint = Getactivejoint(TrajoptMode::Default);
	vector< vector<double> > waypoints;
	string line;
	std::ifstream in_file(textdir);
	vector<double> temp;
	int index =0;
	if(in_file.is_open()){
		cout<<"Load waypoints from "<<textdir<<endl;

		while(getline(in_file,line)){
			std::size_t found = line.find(" ");
			while(found!=std::string::npos){
				temp.push_back(std::stod(line.substr(index,found-index)));
				index = found;
				found = line.find(" ",found+1,1);
			}
//			printcoll(temp);
			index = 0;
			waypoints.push_back(temp);
			temp.clear();
		}
		in_file.close();

		vector<double> affinetran = TransformtoVector(robot->GetTransform());
		vector< vector<double> > waypoints2;
		vector< Eigen::MatrixXf > initguess;

		for(int i=1;i<waypoints.size();i++){
			waypoints2.push_back(concatenate_vectors(GetWholeJoint(robot, waypoints[i], activejoint),affinetran));
		}

		traj.resize(waypoints2.size(),waypoints2[0].size());
		for(int i = 0 ; i < waypoints2.size() ; i ++){
			for(int m = 0 ; m < waypoints2[0].size() ; m++){
				traj(i,m) = waypoints2[i][m];
				cout << traj(i,m) << "," ;
			}
			cout << endl;
		}
		cout << "Shoved trajectory in there " << traj.rows() << " " << traj.cols() << endl;
		return true;
	}else{
		cout<<"Unable to open file"<<endl;
	}
	return false;
}

bool JACOTraj::LoadWaypoints(vector< vector<double> >& waypoints, string textdir){
	string line;
	std::ifstream in_file(textdir);
	vector<double> temp;
	int index =0;
	if(in_file.is_open()){
		cout<<"Load waypoints from "<<textdir<<endl;
		while(getline(in_file,line)){
			std::size_t found = line.find(" ");
			while(found!=std::string::npos){
				temp.push_back(std::stod(line.substr(index,found-index)));
				index = found;
				found = line.find(" ",found+1,1);
			}
//			printcoll(temp);
			index = 0;
			waypoints.push_back(temp);
			temp.clear();
		}
		in_file.close();
		return true;
	}else{
		cout<<"Unable to open file"<<endl;
	}
	return false;
}

// void JACOTraj::ShowCOM(vector<double> samplejointstate){
// 	viewer = OSGViewer::GetOrCreate(env);
// 	vector<int> activejoint = Getactivejoint(TrajoptMode::Default);
// 	robot->SetActiveDOFs(activejoint);
// 	robot->SetActiveDOFValues(samplejointstate);
// 	vector< KinBody::LinkPtr > links = robot->GetLinks();
// 	for(int i=0;i<links.size();i++){
// //		cout<<links[i]->GetName()<<" : "<<links[i]->GetGlobalCOM()<<endl;
// 		cout<<links[i]->GetName()<<" : "<<robot->GetLink("l_foot")->GetTransform().inverse()*links[i]->GetGlobalCOM()<<endl;
// 	}
// 	viewer->UpdateSceneData();
// 	viewer->Draw();
// 	if(see_viewer && idle_viewer)viewer->Idle();
// }

// void JACOTraj::ShowKinematics(vector< vector<double> > samplejointstate){
// 	viewer = OSGViewer::GetOrCreate(env);
// 	vector<int> activejoint = Getactivejoint(TrajoptMode::Default);
// 	robot->SetActiveDOFs(activejoint);
// 	for(int i = 0;i<samplejointstate.size();i++)
// 	{
// 		vector<double> jointstate = samplejointstate[i];
// 		robot->SetActiveDOFValues(jointstate);
// 		viewer->UpdateSceneData();
// 		viewer->Draw();
// 		if(see_viewer && idle_viewer)viewer->Idle();
// //		Vector pelvis_com = robot->GetLink("pelvis")->GetLocalCOM();
// //		Vector pelvis_com_off = robot->GetLink("pelvis")->GetCOMOffset();
// //		cout<<pelvis_com<<" offset: "<< pelvis_com_off<<endl;
// 		cout << "pelvis: ";
// 		vector<double> pelvis_pose = TransformtoVector(robot->GetLink("pelvis")->GetTransform());
// 		pelvis_pose[2] = pelvis_pose[2]-0.08;
// 		printcoll(pelvis_pose);
// 		cout<<"feet: ";
// 		vector<double> l_foot_pose = TransformtoVector(robot->GetLink("l_foot")->GetTransform());
// 		l_foot_pose[2] = l_foot_pose[2]-0.08;
// 		vector<double> r_foot_pose = TransformtoVector(robot->GetLink("r_foot")->GetTransform());
// 		r_foot_pose[2] = r_foot_pose[2]-0.08;
// 		l_foot_pose = concatenate_vectors(l_foot_pose,r_foot_pose);
// 		printcoll(l_foot_pose);
// 		Sleep(0.2);
// 	}
// }