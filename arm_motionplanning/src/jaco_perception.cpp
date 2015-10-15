#include <arm_motionplanning/jaco_traj.hpp>

using namespace jaco_traj;

void JACOTraj::LoadPointCloud(vector<double> start_state,TrajoptMode mode){
	std::lock_guard<std::mutex> lock(g_i_mutex);
	vector<int> activejoint = Getactivejoint(mode);

	robot = env->GetRobot("jaco");

	robot->SetDOFValues(start_state,1,activejoint);

	for(int i = 0; i < convexhulls.size(); i++){
		OpenRAVE::EnvironmentMutex::scoped_lock lockenv(env->GetMutex());
		std::stringstream name;
		name <<"cd" << i;


		OpenRAVE::KinBodyPtr body = OpenRAVE::RaveCreateKinBody(env);
		OpenRAVE::TriMesh trimesh;

		pcl::PointCloud<pcl::PointXYZ> mycloud;

		pcl::fromPCLPointCloud2(convexhulls[i]->cloud, mycloud);

		for(pcl::PointXYZ origPoint : mycloud.points){
			OpenRAVE::Vector point;
			point.x = origPoint.x;
			point.y = origPoint.y;
			point.z = origPoint.z;
			point.w = 1;
			trimesh.vertices.push_back(point);
		}

		pcl::PolygonMesh::Ptr &curMesh = convexhulls[i];

		for(pcl::Vertices curVerts : curMesh->polygons){
			trimesh.indices.push_back(curVerts.vertices[0]);
			std::cout<<trimesh.indices.back()<<"  ";
			trimesh.indices.push_back(curVerts.vertices[1]);
			std::cout<<trimesh.indices.back()<<"  ";
			trimesh.indices.push_back(curVerts.vertices[2]);
			std::cout<<trimesh.indices.back()<<"   "<< trimesh.vertices.size()<<std::endl;
		}

		body->InitFromTrimesh(trimesh);

		body->SetName(name.str());

		env->Add(body);

		// body->SetTransform(lFootFrame);

		env->GetKinBody(name.str())->GetLinks()[0]->GetGeometries()[0]->SetAmbientColor(OpenRAVE::RaveVector<float>(1,1,1,1));

		//Removed self-collision, done beforehand
		trajopt::CollisionCheckerPtr checker = trajopt::CollisionChecker::GetOrCreate(*env);

		trajopt::vector<trajopt::Collision> collisions2;


		checker->BodyVsAll(*body,collisions2);

		for(int u = 0 ; u < collisions2.size(); u++)
		{

			//std::cout << collisions2[u].linkA->GetName() << " " << collisions2[u].linkB->GetName() << " " << collisions2[u].distance << " " << name.str() << std::endl;
			if((collisions2[u].linkA->GetName() != "base") || (collisions2[u].linkB->GetName() != "base"))
			{
				//std::cout << "REMOVING OBJECT " << std::endl;
				env->Remove(body);
				break;
			}

		}

	}


	if(see_viewer)
	{
		viewer = OSGViewer::GetOrCreate(env);
		viewer->UpdateSceneData();
		viewer->Draw();
		Sleep(0.2);
	}
	// if(see_viewer && idle_viewer)viewer->Idle();
}

bool JACOTraj::PrepPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, Eigen::Affine3d trans)
{
	assert(cloudPtr);

	pcl::transformPointCloud(*cloudPtr, *cloudPtr, trans);

	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::copyPointCloud<pcl::PointXYZRGB,pcl::PointXYZ>(*cloudPtr, *temp);

	////PASSTHROUGH FILTER

	bool print_progress = false;

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (temp);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (-0.5, 2);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*temp);

	pass.setInputCloud (temp);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (-1.0, 2.0);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*temp);

	pass.setInputCloud (temp);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-1.0, 1.5);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*temp);

	////// IS EMPTY?
	//If point-cloud is empty, just return the function
	if(temp->size() < 1){
		Sleep(0.5);
		return false;
	} 

	/////SMOOTH 

	pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>());

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (temp);
	sor.setLeafSize (0.02f, 0.02f, 0.02f);
	sor.filter (*temp2);



	if(print_progress) std::cout << "Smoothing :" << std::endl;

	// Create a KD-Tree
/*	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree3 (new pcl::search::KdTree<pcl::PointXYZ>);
	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal> mls_points;
	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals (true);
	// Set parameters
	mls.setInputCloud (temp);
	mls.setPolynomialFit (true);
	mls.setSearchMethod (tree3);
	mls.setSearchRadius (0.1);
	// Reconstruct
	mls.process (mls_points);
	pcl::PointCloud<pcl::PointXYZ>::Ptr smooth_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud (mls_points, *smooth_cloud);*/

	/////MESH FOR UNORGANIZED

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal> normals;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (temp2);
	n.setInputCloud (temp2);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*temp2, normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	//pcl::PolygonMesh triangles;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	//Very important in setting the size of blocks:
	gp3.setSearchRadius (0.2); //0.025

	// Set typical values for the parameters
	gp3.setMu (25);//2.5
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);
	////

	//pcl::PolygonMesh::Ptr bigMesh= cloudproc::meshOFM(temp, 3, 0.1);

	//Improve accuracy, but it takes longer. 0.02 = 1 sec, 0.2 = 2.5 sec ~ 
	pcl::PolygonMesh::Ptr simpleMesh =cloudproc::quadricSimplifyVTK(triangles, .03);
	//OR --- DON'T USE: Invalid Pointer bug
/*	pcl::PolygonMesh::Ptr simpleMesh(new pcl::PolygonMesh()); 
	pcl::MeshQuadricDecimationVTK mesh_decimator; 
	mesh_decimator.setInputMesh(triangles); 
	mesh_decimator.setTargetReductionFactor(1 - 0.1); 
	mesh_decimator.process(*simpleMesh); */

	g_i_mutex.lock();
	//Concativity : highly influence smoothness vs. polygon size.
	convexhulls = ConvexDecompHACD(*simpleMesh, 25);

	//Copy the simplified mesh into the global state polygonmesh
	polygonmesh = *simpleMesh;
	g_i_mutex.unlock();

	return true;
}