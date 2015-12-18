/**
 ********************************************************************************************************
 * @file    jaco_traj.hpp
 * @brief   JACO arm motion planning class header
 * @details Core code for generating manipulation trajectories for the JACO arm
 ********************************************************************************************************
 */

#ifndef JACO_TRAJ_HPP
#define JACO_TRAJ_HPP

/*** INCLUDE FILES ***/

#pragma once

#include <arm_motionplanning/jaco_consts.hpp>
#include <arm_motionplanning/jaco_utils.hpp>

#include <json/json.h>
#include <iostream>
#include <openrave/openrave.h>
#include <openrave-0.9/openrave-core.h>
#include <openrave/planningutils.h>
#include <openrave/trajectory.h>
#include <trajopt/problem_description.hpp>
#include "trajopt/trajectory_costs.hpp"
#include "trajopt/kinematic_terms.hpp"
#include "trajopt/collision_checker.hpp"
#include <utils/eigen_conversions.hpp>
//#include <utils/interpolation.hpp>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <cstring>
#include <Eigen/Core>
#include <sstream>
#include <time.h>
#include <math.h> 
#include <pthread.h>
#include <mutex>
 
//Point Cloud stuff
#include <cloudproc/hacd_interface.hpp>
#include <osgviewer/osgviewer.hpp>
#include <cloudproc/cloudproc.hpp>
#include <cloudproc/mesh_simplification.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/bilateral.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/filters/voxel_grid.h>


namespace jaco_traj{
using namespace OpenRAVE;
using namespace std;

class JACOTraj
{

/* Mutex */ 
	std::mutex g_i_mutex;

public:
	/**
	 * @brief   JACOTraj class constructor, runs Initialize
	 * @param   None
	 * @return  None
	 */
	JACOTraj();

	/**
	 * @brief   JACOTraj class destructor
	 * @param   None
	 * @return  None
	 */
	~JACOTraj();

	/**
	 * @brief   Initialize environment, load robot model
	 * @param   None
	 * @return  None
	 */
	void Initialize();

	/**
	 * @brief   Load Gains
	 * @param   position, orientation and hand offset values
	 * @return  None
	 */
	void LoadGains(Eigen::Vector3d pos_gains = Eigen::Vector3d{1,1,1}, Eigen::Vector3d rot_gains = Eigen::Vector3d{1,1,1},
	 				Eigen::Vector3d hand_offset = Eigen::Vector3d{0,0.28,0});
	

	/**
	 * @brief   Compute Trajectory
	 * @param   start_state, hand target
	 * @return  None
	 */
	trajopt::TrajArray ComputeTrajectory(vector<double> start_state, Eigen::Affine3d hand_target);

	/**
	 * @brief ComposeRequest builds the JSON string from all external information
	 * @param request string, and and TrajoptMode
	 * @return None
	 */
	void ComposeRequest(stringstream& request,TrajoptMode mode, Eigen::Affine3d hand_target);
	
	/**
	 * @brief   Load Point Cloud into the environment, use latest point-cloud prepared by PrepPointCloud
	 * @param   None
	 * @return  None
	 */
	void LoadPointCloud(vector<double> start_state,TrajoptMode mode = TrajoptMode::Default);

	/**
	 * @brief   Prepare Point Cloud and stores into global variable
	 * @param   Point Cloud pointer, transformation to left_foot
	 * @return  None
	 */
	bool PrepPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, Eigen::Affine3d trans);
	
	/**
	 * @brief   Get The latest generate PolygonMesh from Point Cloud
	 * @param   None
	 * @return  Said PolygonMesh
	 */
	pcl::PolygonMesh GetPolygonMesh(); 

	/**
	 * @brief	Load polygon mesh into env
	 * @param	mesh pointer
	 * @return	true if loading is successful
	 */
	bool LoadPolygonMesh(pcl::PolygonMesh::Ptr simpleMesh);


	/**
	 * @brief   Load a file to the environment
	 * @param   File address
	 * @return  None
	 */
	void Load(std::string& object_name);

	/**
	 * @brief   Reset the Environment
	 * @param   File address
	 * @return  None
	 */
	void ResetEnvironment();

	/**
	 * @brief   Set variables back to their Default values
	 * @param   None
	 * @return  None
	 */
	void Defaults();

	/**
	 * @brief   Load robot(default file)
	 * @param   None
	 * @return  None
	 */
	void LoadRobot();

	/**
	 * @brief   Transform Environment Object
	 * @param   Object name, Transformation
	 * @return  None
	 */
	void TransformObject(std::string& object, Eigen::Affine3d trans);

	/**
	 * @brief   Load a file to the environment
	 * @param   File address
	 * @return  None
	 */
	void GrabObject(vector<double> start_state, std::string& object_name);

	/**
	 * @brief   Release object from the robot
	 * @param   File address
	 * @return  None
	 */
	void ReleaseObject(std::string& object_name);

	/**
	 * @brief   SeeViewer
	 * @param   see the viewer?
	 * @return  None
	 */
	void SeeViewer(bool see);

	/**
	 * @brief   SeeViewer
	 * @param   see the viewer?
	 * @return  None
	 */
	void IdleViewer(bool idle);

	/**
	 * @brief	Set Num of Step
	 * @param	num
	 * @return 	None
	 */
	void SetNumStep(int num);

	/**
	 * @brief	Set Trajopt Mode
	 * @param	TrajOpt Mode
	 * @return 	None
	 */
	void SetMode(TrajoptMode mode);

	/**
	 * @brief	Set Grasp Range Z_tol
	 * @param	TrajOpt Mode
	 * @return 	None
	 */
	void SetGraspRange(double d);

	/**
	 * @brief	Set Multi Initial guesses
	 * @param	num
	 * @return 	None
	 */
	void SetMultiInit(bool multi_init);

	/**
	 * @brief	Print the calculated trajectory
	 * @param	traj - generated trajectory including all joints state
	 * 			activejoint - joints value which need to send to controller
	 * @return	None
	 */
	void PrintTraj(trajopt::TrajArray traj,vector<int> activejoint);

	/**
	 * @brief	Save the calculated trajectory
	 * @param	traj - generated trajectory including all joints state
	 * 			activejoint - joints value which need to send to controller
	 * 			text_dir - directory of the text file
	 * @return	None
	 */
	void SaveTraj(trajopt::TrajArray traj, string text_dir, TrajoptMode traj_mode = TrajoptMode::Default);

	/**
	 * @brief	Load the waypoints from a text file
	 * @param	waypints - save waypoints into a vector
	 * 			text_dir - directory of the text file
	 * @return	None
	 */

	bool LoadWaypoints(vector< vector<double> >& waypoints,string text_dir);
	bool LoadWaypoints(trajopt::TrajArray & traj, string textdir);
	bool DebugTraj(string textdir);

	/**
	 * @brief	Set Waypoints from file
	 * @param	text_dir - file name
	 * @return	None
	 */
	void SetWaypointsFromFile(string textd_dir);


	/**
	 * @brief	Show the optimized motion
	 * @param	traj - generated trajectory
	 * @return	None
	 */
	void ShowTraj(trajopt::TrajArray traj);

	/**
	 * @brief	smooth the trajectory
	 * @param	sample_rate - the number of samples you want;
	 * 			launchviewer - want to see the new trajectory?
	 * 			traj - generated trajectory
	 * @return	None
	 */
	int Smoothing(trajopt::TrajArray traj,int sample_rate,bool launchviewer);

	/**
	 * @brief	Enable smoothing the trajectory
	 * @param	smooth the traj?
	 * @return	None
	 */
	void SetSmoothing(bool smooth_traj);

	/**
	 * @brief	Get current robot joint states in openrave
	 * @param	Current state
	 * @return	None
	 */
	void GetCurrentState(std::vector<double>& currentstate);

	/**
	 * @brief	UpdateStateForRequest Get current robot joint states in openrave
	 * @param	Current state data
	 * @return	None
	 */
	void UpdateStateForRequest(vector<double>& start_state, Eigen::Affine3d hand, TrajoptMode mode);

	/**
	 * @brief   Show all waypoints in a viwer
	 * @param   The generated trajectory
	 * @return  None
	 */
	void PreviewTraj(trajopt::TrajArray traj,vector<int> activejoint);

	/**
	 * @brief	Get Final Tragetory
	 * @param	The Storage Pointer
	 * @return 	None
	 */
	void GetFinalTraj(vector< vector<double> >& final_traj);

	KinBodyPtr GetKinBody(const std::string& kinbody_name);



private:

	std::shared_ptr<std::stringstream> request_str;
	jaco_traj::TrajoptMode current_mode;

	OSGViewerPtr viewer;
	EnvironmentBasePtr env;
	RobotBasePtr robot;
	bool see_viewer;
	bool idle_viewer;
	int num_step;
	string robot_name;

	bool multi_init_guess;
	bool smooth_traj;

	double collision_cost;
	double dist_pen;
	vector<double> target_state;
	Eigen::MatrixXf request_traj;

	//Loaded waypoints initial guess from file
	bool load_waypoints;
	std::string load_waypoints_file_path;

	//Target hand, there must be a better solution
	std::string hand_str;
	vector<double> xyz_target;
	vector<double> quat_target;
	vector<double> pos_gains;
	vector<double> rot_gains;
	vector<double> hand_offset;

	vector<double> tau_cost;
	vector<double> vel_cost;
	vector<double> pos_cost;
	vector<double> pos_vals;

	vector< vector<double> > final_traj; 

	/*Multithread data*/
	struct thread_data{
	  int thread_id;
	  EnvironmentBasePtr thread_env;
	  stringstream thread_request;
	  bool thread_see_viewer;
	  bool isdone;
	  trajopt::TrajArray traj;
	  std::string thread_robot_name;
	};

	struct thread_info{
		thread_data* pthread_data;
		int num_thread;
		int target_tread;
	};


	std::vector<pcl::PolygonMesh::Ptr> convexhulls;
	pcl::PolygonMesh polygonmesh;
	pcl::PolygonMesh polygonmesh_saved;


	/*Internal functions for build the problem */
	void AddConfigurationBias(trajopt::TrajOptProbPtr prob , vector<int> act_joints, int start_timestep, int end_timestep);

	void AddConfigurationBias(trajopt::TrajOptProbPtr prob , vector<int> act_joints, vector<double> config, int start_timestep, int end_timestep);

	vector<int> Getactivejoint(TrajoptMode mode);

	vector< Eigen::MatrixXf > GenerateInitGuess(bool multi_initguess,vector<double> start_state, vector<double> target_state, vector<int> activejoint);

	vector< Eigen::MatrixXf > GenerateInitGuess(bool multi_initguess,vector<double> start_state, vector<double> target_state, vector<int> activejoint, vector< vector<double> >poses);

	Eigen::MatrixXf Buildtraj(vector< vector<double> > waypoints);

	void GetLinkPosandQuat(string linkname, vector<double>& xyz, vector<double>& quat);

	void SetVelCost(vector<double>& vel_cost,TrajoptMode mode);

	void SetPosCost(vector<double>& pos_cost,vector<double>& pos_vals,TrajoptMode mode);

	void SetTorqueCost(vector<double>& tau_cost,TrajoptMode mode);

	void BuildJointCost(vector<double>& vel_cost, vector<double>& majorjoints_cost,TrajoptMode mode);

	void AddRequestHead(stringstream& request);

	void AddCostHead(stringstream& request,vector<double> vel_cost);

	void AddContinueCollisionCost(stringstream& request, double coeffs, double dist_pen,int first_step, int last_step);

	void AddDiscontinueCollisionCost(stringstream& request, double coeffs, double dist_pen,int first_step, int last_step);

	void AddTorqueCost(stringstream& request, vector<double> coeffs,int start_timestep, int end_timestep);

	void AddPoseCostorConstraint(stringstream& request, string linkname,vector<double> xyz, vector<double>wxyz, vector<double> pos_coeffs, vector<double> rot_coeffs, int start_timestep, int end_timestep,vector<double> offset);

	void AddPositionRegionConstraint(stringstream& request, string linkname,vector<double> plane1, vector<double> plane2, double dist_coeff, int time_step);

	void AddCostEnd(stringstream& request);

	void AddConstraintHead(stringstream& request);

	void AddConstraintHead(stringstream& request, string linkname,vector<double> xyz, vector<double>wxyz, vector<double> pos_coeffs, vector<double> rot_coeffs, int start_timestep, int end_timestep, vector<double> offset);

	void AddJointPrime(stringstream& request, TrajoptMode mode , int start_timestep, int end_timestep);

	void AddConstraintEnd(stringstream& request,Eigen::MatrixXf traj);

	void AddJointPrime(stringstream& request, vector<double> pos_vals, vector<double> coeffs, int time_step);

	void AddJointPositionCostorConstraint(stringstream& request, vector<double> pos_cost , vector<double> pos_vals, int time_step);

	void AddDDPoseCostorConstraint(stringstream& request, string linkname,vector<double> pos_coeffs, vector<double> rot_coeffs, int start_timestep, int end_timestep, vector<double> offset);

	void ClearViewer();

	static void * LaunchSolver(void *threadarg);

	static void * ManageThread(void *threadarg);
	
};

};





 #endif // JACO_TRAJ_HPP
