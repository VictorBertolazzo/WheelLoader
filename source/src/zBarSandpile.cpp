// First test of binding together granular matter and loader mechanism.
// Victor Bertolazzo

	// DEM-P with cohesion force at the moment.
	// mechanism, ChFunctionRecorder for data ....
	// Drive actuator with a CallbackSpring in which input your data.
	// Create a flag with which choose how to drive actuator(FORCE/DISPLACEMENT)
	// Choose actuator funciton in the main
	// bind together mech and sand
	// What happens when bucket hits the pile.
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <valarray>
#include <vector>
#include <numeric>
#include <functional>
#include <algorithm>
 // 03/11/2017 -->(10 days in euler37)
 // play with bins per axis 
 // replace ellipsoids with spheres
 // understand the major player(narrow,broad,solver)
 // try to relax time_step
#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChTimer.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChPointPointDrawing.h"

#include "chrono/utils/ChUtilsGenerators.h"


#define USE_PENALTY
using namespace chrono;


//////-----------------------------------------GLOBAL VARIABLES------------------------////////////////////////////////////
// Output directories
const std::string out_dir = "../DEMP_ZBARSANDPILE";
const std::string pov_dir = out_dir + "/POVRAY";

bool povray_output = false;
// Actuator test times : VALUES
double ta1 = 2.00; double ta2 = 11.00;double ta3 = 15.00;double ta4= 5.00;double ta5 =12.00;
// Actuator test times : TESTING IDEA
		// Tilt : constant until ta4, then negative displacement(piling) until ta5, at the end constant til ta3;
		// Lift : constant until ta1, then positive displacement(piling) til ta2, at the constant til ta3;
		// chassis : FWD motion til ta1, stop until ta2, then RWD til ta3;

int num_threads = 40;
	ChMaterialSurface::ContactMethod method = ChMaterialSurface::SMC;//NSC
	bool use_mat_properties = true;
	bool render = false;
	bool track_granule = false;
	bool track_flatten = false;
	double radius_g = 0.015;// 0.01 feasible dimension
							// 0.01 Working Desktop Version : switched to 5cm to reduce number of bodies
	double r = 1.01 * radius_g;
	int num_layers = 50;// Working Desktop Version : 24 ;

	double terrainHeight = .01;
	// BroadPhase Parameters
	int binsX = 30;
	int binsY = 30;
	int binsZ = 30;

	ChVector<> pilepoint(6.50,.0,.0);// Working Desktop Version x=5.50;
	ChVector<> terrain_center = pilepoint + ChVector<>(0.00, 0.0, terrainHeight);
	ChVector<> center = pilepoint + ChVector<>(0.00, 0, terrainHeight + r);

	// Create particles in layers until reaching the desired number of particles
	// Container dimensions
	double hdimX = 3.5; // 1.5 Working Desktop Version
	double hdimY = 3.5; // 1.5 Working Desktop Version 
	double hdimZ = 0.5;
	double hthick = 0.25;

	// Working Desktop Version
	//ChVector<> hdims(1.5 / 2 - r, 1.5 / 2 - r, 0);//W=.795, hdims object for the function gen.createObjectsBox accepts the	FULL dimensions in each direction:PAY ATTENTION
	ChVector<> hdims(hdimX / 2 - r, hdimY / 2 - r, 0);


	double Ra_d = 2.5*radius_g;//Distance from centers of particles.
	double Ra_r = 1.5*radius_g;//Default Size of particles.

	
	// Granular material properties
	int Id_g = 10000;
	double rho_g = 2500;
	double vol_g = (4.0 / 3) * CH_C_PI * radius_g * radius_g * radius_g;
	double mass_g = rho_g * vol_g;
	ChVector<> inertia_g = 0.4 * mass_g * radius_g * radius_g * ChVector<>(1, 1, 1);
	
	// Terrain contact properties---Default Ones are commented out.
	float friction_terrain = 0.7f;// (H,W) requires mi=.70;
	float restitution_terrain = 0.0f;
	float Y_terrain = 8e4f;
	float nu_terrain = 0.3f;
	float kn_terrain = 1.0e4f;// 1.0e7f;
	float gn_terrain = 1.0e2f;
	float kt_terrain = 2.86e3f;// 2.86e6f;
	float gt_terrain = 1.0e2f;
	float coh_pressure_terrain = 1e2f;// 0e3f;
	float coh_force_terrain = (float)(CH_C_PI * radius_g * radius_g) * coh_pressure_terrain;

	// Estimates for number of bins for broad-phase
	int factor = 2;
	// int binsX = (int)std::ceil(hdimX / radius_g) / factor;
	// int binsY = (int)std::ceil(hdimY / radius_g) / factor;
	// int binsZ = 1;

//////-----------------------------------------GLOBAL VARIABLES------------------------////////////////////////////////////

// ------------------CLASSES----------------------------------------------
class MyWheelLoader {
	public:
	// Data
	
	// Handles
	//std::shared_ptr<ChBody> lift;
	std::shared_ptr<ChBodyAuxRef> lift;
	std::shared_ptr<ChBody> rod;
	std::shared_ptr<ChBody> link;
	std::shared_ptr<ChBodyAuxRef> bucket;
	std::shared_ptr<ChBody> chassis;
	
	std::shared_ptr<ChLinkLockRevolute> rev_lift2rod;
	std::shared_ptr<ChLinkLockRevolute> rev_rod2link;
	std::shared_ptr<ChLinkLockRevolute> rev_lift2bucket;
	std::shared_ptr<ChLinkLockRevolute> rev_link2bucket;
	std::shared_ptr<ChLinkLockRevolute> rev_ch2lift;

	std::shared_ptr<ChLinkLinActuator> lin_ch2lift;
	std::shared_ptr<ChLinkLinActuator> lin_lift2rod;

	ChQuaternion<> z2y;
	ChQuaternion<> z2x;

	// Utility Functions
	enum BucketSide { LEFT, RIGHT };
struct Points {
	Points() {}
	Points(float x, float y, float z)
		: mx(x), my(y), mz(z) {}
	float mx; float my; float mz;
};
void ReadFile(const std::string& filename, std::vector<Points>& profile) {
	std::ifstream ifile(filename.c_str());
	std::string line;

	while (std::getline(ifile, line)) {
		std::istringstream iss(line);
		float xpos, ypos, zpos;
		iss >> xpos >> ypos >> zpos;
		if (iss.fail())
			break;
		profile.push_back(Points(xpos, ypos, zpos));
	}
	ifile.close();



}
void AddBucketHull(std::vector<Points> p_ext, std::vector<Points> p_int, std::shared_ptr<ChBody> bucket) {

	for (int iter = 0; iter < p_ext.size() - 1; iter++) {
		std::vector<ChVector<double>> cloud;
		double width = 1.0;// or halve an input

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my - width, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my - width, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter + 1].mx, p_ext[iter + 1].my - width, p_ext[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter].mx, p_ext[iter].my - width, p_ext[iter].mz));

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter + 1].mx, p_ext[iter + 1].my + width, p_ext[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter].mx, p_ext[iter].my + width, p_ext[iter].mz));



		bucket->GetCollisionModel()->AddConvexHull(cloud, ChVector<>(0, 0, 0), QUNIT);
		bucket->GetCollisionModel()->BuildModel();

		auto shape = std::make_shared<ChTriangleMeshShape>();
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(cloud, shape->GetMesh());
		//bucket->AddAsset(shape);

		//bucket->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
	}

}
void AddCapsHulls(std::vector<Points> p_int, BucketSide side, std::shared_ptr<ChBody> bucket) {


	for (int iter = 0; iter < p_int.size() - 1; iter++) {

		std::vector<ChVector<double>> cloud;
		double width;
		switch (side)
		{
		case LEFT:
			width = +1.;
			break;
		case RIGHT:
			width = -1.0;
			break;
		default:
			width = .0;
			break;
		}


		double th = .05;

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width - th, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width - th, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(.70, width - th, .25));

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width + th, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width + th, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(.70, width + th, .25));

		bucket->GetCollisionModel()->AddConvexHull(cloud, ChVector<>(0, 0, 0), QUNIT);
		bucket->GetCollisionModel()->BuildModel();

		auto shape = std::make_shared<ChTriangleMeshShape>();
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(cloud, shape->GetMesh());
		//bucket->AddAsset(shape);

		//bucket->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
	}
}

	// Constructor	

	MyWheelLoader(ChSystem& system){
	
	
			///////////////////////////////////////////////////////////////Constructor Utilities
	std::vector<Points> p_ext;
	std::vector<Points> p_int;
	const std::string out_dir = "../";
	const std::string& ext_profile = out_dir + "data/ext_profile.txt";
	const std::string& int_profile = out_dir + "data/int_profile.txt";
	ReadFile(ext_profile, p_ext);
	ReadFile(int_profile, p_int);
	// Create a material (will be used by both objects)
	auto materialDVI = std::make_shared<ChMaterialSurfaceNSC>();
	materialDVI->SetRestitution(0.1f);
	materialDVI->SetFriction(0.4f);
	// Create a material (will be used by both objects)
	auto materialDEM = std::make_shared<ChMaterialSurfaceSMC>();
	materialDEM->SetYoungModulus(1.0e7f);
	materialDEM->SetRestitution(0.1f);
	materialDEM->SetFriction(0.4f);
	materialDEM->SetAdhesion(0);  								// Magnitude of the adhesion in Constant adhesion model

	////////////////////////////////////////////////////////Numerical Data
	ChVector<> COG_chassis(0, 0, 1.575); // somewhere not defined
	ChVector<> COG_lift(2.0, 0., 1.05);
	ChVector<> COG_lever(3.6625, 0.0, 1.575);
	ChVector<> COG_rod(2.7, 0.0, 1.3125);
	ChVector<> COG_link(3.0, 0.0, 0.5);
	ChVector<> COG_bucket(4.0, .0, 0.525); // not easy definition

	ChVector<> POS_lift2rod(2.825, .0, 1.05);					//rev joint(LIFT ARM) abs frame
	ChVector<> POS_rod2lever(3.6625, 0., 1.575);				//rev joint(BUCKET LEVER) abs frame
	ChVector<> POS_lift2bucket(3.50, .0, .21);					//chassis piston(LIFT ARM) insertion abs frame
	ChVector<> POS_ch2lift(1.6, 0, 2.1);						//Rev chassis->lift
	ChVector<> POS_lift2lever(2.5, 0, 2.1);						//end position of actuator lift->lever
	ChVector<> PIS_ch2lift(1.6, 0, 1.05);						//Act chassis->lift
	ChVector<> PIS_lift2lever(2.0125, 0, 2.1);					//Act lift->lever

	ChVector<> POS_rod2link(2.6, 0, 0.4);						//Act lift->lever
	ChVector<> POS_link2bucket(3.69, .0, 0.71);					//chassis piston(BUCKET LEVER) insertion abs frame

	ChVector<> INS_ch2lift(1.8, 0, 1.1);						// Insertion of boom piston over lift body

	z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
	z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
	

			////////////////////////--------------Create rigid bodies-------------------------------////////////////////////////////////////////
			///////////////////////-----------------------------------------------------------------////////////////////////////////////////////
	// LIFT
	lift = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());//switched to ChBodyAuxRef
	//lift = std::shared_ptr<ChBody>(system.NewBody());//switched to ChBodyAuxRef
	system.Add(lift);
	lift->SetBodyFixed(false);
	lift->SetName("lift arm");
	lift->SetPos(POS_ch2lift);// switched to ChBodyAuxRef
	//lift->SetPos(COG_lift);// COG_lift changed
	ChVector<> u1 = (POS_lift2bucket - POS_ch2lift).GetNormalized();//switched to ChBodyAuxRef
	//ChVector<> u1 = (POS_lift2bucket - COG_lift).GetNormalized();//
	ChVector<> w1 = Vcross(u1, VECT_Y).GetNormalized();//overkill
	ChMatrix33<> rot1;//no control on orthogonality
	rot1.Set_A_axis(u1, VECT_Y, w1);
	lift->SetRot(rot1);
	//lift->SetFrame_COG_to_REF(ChFrame<>(lift->GetFrame_REF_to_abs().GetInverse() * COG_lift, QUNIT));//switched to ChBodyAuxRef
	lift->SetMass(993.5);
	lift->SetInertiaXX(ChVector<>(110.2, 1986.1, 1919.3));
	lift->SetInertiaXY(ChVector<>(0., 0., 339.6));

	// primitive visualization:
	auto lift_asset = std::make_shared<ChCylinderShape>();
	lift_asset->GetCylinderGeometry().rad = .025;
	lift_asset->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_ch2lift;
	lift_asset->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
	auto lift_asset1 = std::make_shared<ChCylinderShape>();
	lift_asset1->GetCylinderGeometry().rad = .025;
	lift_asset1->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
	lift_asset1->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2bucket;
	auto lift_asset2 = std::make_shared<ChCylinderShape>();
	lift_asset2->GetCylinderGeometry().rad = .025;
	lift_asset2->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_ch2lift;
	lift_asset2->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * INS_ch2lift;
	auto lift_asset3 = std::make_shared<ChCylinderShape>();
	lift_asset3->GetCylinderGeometry().rad = .025;
	lift_asset3->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * INS_ch2lift;
	lift_asset3->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2bucket;
	// lift->AddAsset(lift_asset);lift->AddAsset(lift_asset1);lift->AddAsset(lift_asset2);lift->AddAsset(lift_asset3);
	auto col_l = std::make_shared<ChColorAsset>();
	col_l->SetColor(ChColor(0.2f, 0.0f, 0.0f));
	//lift->AddAsset(col_l);
	geometry::ChTriangleMeshConnected lift_mesh;
	lift_mesh.LoadWavefrontMesh(out_dir + "data/boom_mod.obj", false, false);
	auto lift_mesh_shape = std::make_shared<ChTriangleMeshShape>();
	lift_mesh_shape->SetMesh(lift_mesh);
	lift_mesh_shape->SetName("boom");
	lift->AddAsset(lift_mesh_shape);

	// ROD
	rod = std::shared_ptr<ChBody>(system.NewBody());
	system.Add(rod);
	rod->SetName("rod arm");
	rod->SetIdentifier(3);
	//rod->SetPos(POS_lift2rod);//COG_rod
	rod->SetPos(COG_rod);//
	ChVector<> u3 = (POS_rod2link - POS_lift2rod).GetNormalized();
	ChVector<> w3 = Vcross(u3, VECT_Y).GetNormalized();//overkill
	ChMatrix33<> rot3;
	rot3.Set_A_axis(u3, VECT_Y, w3);
	rod->SetRot(rot3);
//	rod->SetFrame_COG_to_REF(ChFrame<>(rod->GetFrame_REF_to_abs().GetInverse() * COG_rod, QUNIT));//switched to ChBodyAuxRef
	rod->SetMass(381.5);
	rod->SetInertiaXX(ChVector<>(11.7, 33.4, 29.5));
	rod->SetInertiaXY(ChVector<>(0., 0., -12.1));
	// visualization properties:
	auto rod_asset = std::make_shared<ChCylinderShape>();
	rod_asset->GetCylinderGeometry().rad = .025;
	rod_asset->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2lever;
	rod_asset->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
	auto rod_asset1 = std::make_shared<ChCylinderShape>();
	rod_asset1->GetCylinderGeometry().rad = .025;
	rod_asset1->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
	rod_asset1->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_rod2link;
	rod->AddAsset(rod_asset);
	rod->AddAsset(rod_asset1);
	auto col_r = std::make_shared<ChColorAsset>();
	col_r->SetColor(ChColor(0.0f, 0.0f, 0.2f));
	rod->AddAsset(col_r);
	geometry::ChTriangleMeshConnected rocker_mesh;
	rocker_mesh.LoadWavefrontMesh(out_dir + "data/rockerarm_mod.obj", false, false);// file not present
	auto rocker_mesh_shape = std::make_shared<ChTriangleMeshShape>();
	rocker_mesh_shape->SetMesh(rocker_mesh);
	rocker_mesh_shape->SetName("boom");
	//rod->AddAsset(rocker_mesh_shape);//temporary

	//	// LINK
	link = std::shared_ptr<ChBody>(system.NewBody());
	system.Add(link);
	link->SetName("link arm");
	link->SetIdentifier(3);
	link->SetPos(COG_link);
	ChVector<> u4 = (POS_link2bucket - POS_rod2link).GetNormalized();
	ChVector<> w4 = Vcross(u4, VECT_Y).GetNormalized();//overkill
	ChMatrix33<> rot4;
	rot4.Set_A_axis(u4, VECT_Y, w4);
	link->SetRot(rot4);
	link->SetMass(277.2);
	link->SetInertiaXX(ChVector<>(3.2, 11.1, 13.6));
	link->SetInertiaXY(ChVector<>(0.0, 0.0, -.04));
	// visualization properties:
	auto link_asset = std::make_shared<ChCylinderShape>();
	link_asset->GetCylinderGeometry().rad = .025;
	link_asset->GetCylinderGeometry().p1 = link->GetFrame_COG_to_abs().GetInverse() * POS_rod2link;
	link_asset->GetCylinderGeometry().p2 = link->GetFrame_COG_to_abs().GetInverse() * POS_link2bucket;
	link->AddAsset(link_asset);
	auto col_l1 = std::make_shared<ChColorAsset>();
	col_l1->SetColor(ChColor(0.0f, 0.2f, 0.2f));
	link->AddAsset(col_l1);

	//	// BUCKET
	bucket = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());
	system.AddBody(bucket);
	bucket->SetName("benna");
	bucket->SetIdentifier(4);
	bucket->SetMass(200.0);//not confirmed data
	bucket->SetInertiaXX(ChVector<>(200, 500, 200));//not confirmed data
	bucket->SetPos(POS_lift2bucket);
	//bucket->SetFrame_COG_to_REF(ChFrame<> (bucket->GetFrame_REF_to_abs().GetInverse() * COG_bucket,QUNIT));
	bucket->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(.35, .0, .2), QUNIT));//tentative
	// Create contact geometry.
	bucket->SetCollide(true);
	bucket->GetCollisionModel()->ClearModel();
	AddBucketHull(p_ext, p_int, bucket);
	AddCapsHulls(p_int, BucketSide::LEFT, bucket);
	AddCapsHulls(p_int, BucketSide::RIGHT, bucket);
#ifdef USE_PENALTY//temporary workaround
	bucket->SetMaterialSurface(materialDEM);
#else
	bucket->SetMaterialSurface(materialDVI);
#endif
	bucket->GetCollisionModel()->BuildModel();
	geometry::ChTriangleMeshConnected bucket_mesh;
	bucket_mesh.LoadWavefrontMesh(out_dir + "data/bucket_mod.obj", false, false);
	auto bucket_mesh_shape = std::make_shared<ChTriangleMeshShape>();
	bucket_mesh_shape->SetMesh(bucket_mesh);
	bucket_mesh_shape->SetName("bucket");
	bucket->AddAsset(bucket_mesh_shape);

	// CHASSIS
	chassis = std::shared_ptr<ChBody>(system.NewBody());
	system.AddBody(chassis);
	chassis->SetBodyFixed(false);//temporary
	chassis->SetName("chassis");
	chassis->SetIdentifier(0);
	chassis->SetMass(2000.0);
	chassis->SetPos(COG_chassis);
	chassis->SetPos_dt(ChVector<>(.0, .0, .0));// u=2.25m/s
	chassis->SetInertiaXX(ChVector<>(500., 1000., 500.));
		// visualization properties
	auto chassis_asset = std::make_shared<ChSphereShape>();//asset
	chassis_asset->GetSphereGeometry().rad = .15;//asset
	chassis->AddAsset(chassis_asset);





	/////////////////////////////////////------------------Add joint constraints------------------////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Linear Actuator between the lift body and the so called rod(also known as rocker arm)
	lin_lift2rod = std::make_shared<ChLinkLinActuator>();
	ChVector<> u11 = (POS_lift2lever - PIS_lift2lever).GetNormalized();		//GetNormalized() yields a unit vector(versor)
	ChVector<> w11 = Vcross(u11, VECT_Y).GetNormalized();					//overkill
	ChMatrix33<> rot11;														//no control on orthogonality, IT'S UP TO THE USER'
	rot11.Set_A_axis(u11, VECT_Y, w11);
	lin_lift2rod->SetName("linear_lift2rod");
	lin_lift2rod->Initialize(rod, lift, false, ChCoordsys<>(POS_lift2lever, z2x >> rot11.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot11.Get_A_quaternion()));//m2 is the master
	lin_lift2rod->Set_lin_offset(Vlength(POS_lift2lever - PIS_lift2lever));
	// Asset for the linear actuator
	auto bp_asset = std::make_shared<ChPointPointSegment>();				//asset
	lin_lift2rod->AddAsset(bp_asset);
	// A test law for the actuator--> it'll be substitued by an accessor method->no more inplace law definiton in the future.
	// temporary piston law for lift2rod actuator -> it'll be set constant by default and changeable by accessor
	// Note : it is as displacement law
	auto legget1 = std::make_shared<ChFunction_Const>(); 
	auto legget2 = std::make_shared<ChFunction_Ramp>(); legget2->Set_ang(-.008);
	auto legget3 = std::make_shared<ChFunction_Const>();
	auto tilt_law = std::make_shared<ChFunction_Sequence>();tilt_law->InsertFunct(legget1, ta4, 1, true);tilt_law->InsertFunct(legget2, ta5-ta4, 1., true);tilt_law->InsertFunct(legget3, ta3-ta5, 1., true);
	lin_lift2rod->Set_dist_funct(tilt_law);
	system.Add(lin_lift2rod);
	
		// Revolute Joint between the lift body and the rod, located near the geometric baricenter(int(r dA)/int(dA)) of the rod LIFT-ROD. 
		// // It is the second of the three joints that interest the rod body(also known as rocker arm).
	rev_lift2rod = std::make_shared<ChLinkLockRevolute>();
	rev_lift2rod->SetName("revolute_lift2rod");
	rev_lift2rod->Initialize(rod, lift, ChCoordsys<>(POS_lift2rod, z2y >> rot3.Get_A_quaternion()));		// z-dir default rev axis is rotated on y-dir
	system.AddLink(rev_lift2rod);

		// Revolute Joint between the rod body and link, last of the three joints interesting the rod , rear joint of the two concerning the link.
	rev_rod2link = std::make_shared<ChLinkLockRevolute>();
	rev_rod2link->SetName("revolute_rod2link");
	ChMatrix33<> rotb44;
	rev_rod2link->Initialize(link, rod, ChCoordsys<>(POS_rod2link, z2y >> rotb44.Get_A_quaternion()));		// z-dir default rev axis is rotated on y-dir 
	system.AddLink(rev_rod2link);
		// LIFT-BUCKET revjoint
	rev_lift2bucket = std::make_shared<ChLinkLockRevolute>();
	rev_lift2bucket->SetName("revolute_lift2bucket");
	ChMatrix33<> rotb1;
	rev_lift2bucket->Initialize(bucket, lift, ChCoordsys<>(POS_lift2bucket, z2y >> rotb1.Get_A_quaternion()));	// z-dir default rev axis is rotated on y-dir
	system.AddLink(rev_lift2bucket);
		// LINK-BUCKET revjoint
	rev_link2bucket = std::make_shared<ChLinkLockRevolute>();
	rev_link2bucket->SetName("revolute_link2bucket");
	ChMatrix33<> rotb2;
	rev_link2bucket->Initialize(bucket, link, ChCoordsys<>(POS_link2bucket, z2y >> rotb2.Get_A_quaternion()));	// z-dir default rev axis is rotated on y-dir
	system.AddLink(rev_link2bucket);

		// CHASSIS-LIFT revjoint
	rev_ch2lift = std::make_shared<ChLinkLockRevolute>();
	rev_ch2lift->SetName("revolute_chassis2lift");
	rev_ch2lift->Initialize(lift, chassis,ChCoordsys<>(POS_ch2lift, z2y >> rot1.Get_A_quaternion()));	// z-dir default rev axis is rotated on y-dir
	system.AddLink(rev_ch2lift);
		// CHASSIS-LIFT linear actuator
	lin_ch2lift = std::make_shared<ChLinkLinActuator>();
	ChVector<> u22 = (INS_ch2lift - PIS_ch2lift).GetNormalized();					//GetNormalized()
	ChVector<> w22 = Vcross(u22, VECT_Y).GetNormalized();							//overkill
	ChMatrix33<> rot22;																//no control on orthogonality, IT'S UP TO THE USER
	rot22.Set_A_axis(u22, VECT_Y, w22);
	lin_ch2lift->SetName("linear_chassis2lift");
	lin_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(INS_ch2lift, z2x >> rot22.Get_A_quaternion()), ChCoordsys<>(PIS_ch2lift, z2x >> rot11.Get_A_quaternion()));//m2 is the master
	lin_ch2lift->Set_lin_offset(Vlength(INS_ch2lift - PIS_ch2lift));
	// temporary piston law for chassis2lift actuator -> it'll be set constant by default and changeable by accessor
	// Note : it is as displacement law
	auto leggel1 = std::make_shared<ChFunction_Const>();
	auto leggel2 = std::make_shared<ChFunction_Ramp>();leggel2->Set_ang(+.05);
	auto leggel3 = std::make_shared<ChFunction_Const>();
	auto lift_law = std::make_shared<ChFunction_Sequence>(); lift_law->InsertFunct(leggel1, ta1, 1, true); lift_law->InsertFunct(leggel2, ta2-ta1, 1., true); lift_law->InsertFunct(leggel3, ta3-ta2, 1., true);

	lin_ch2lift->Set_dist_funct(lift_law);
	system.Add(lin_ch2lift);
	}
	// Destructor
	~MyWheelLoader(){}
	// Getter
	void PrintValue(){
		GetLog() << "This is a PrintValue() class method: " << chassis->GetPos().z() << "\n";
	}
	// Setter

};

// ------------------CLASSES----------------------------------------------



// ------------------FUNCTIONS----------------------------------------------
enum BucketSide { LEFT, RIGHT };
struct Points {
	Points() {}
	Points(float x, float y, float z)
		: mx(x), my(y), mz(z) {}
	float mx; float my; float mz;
};
void ReadFile(const std::string& filename, std::vector<Points>& profile) {
	std::ifstream ifile(filename.c_str());
	std::string line;

	while (std::getline(ifile, line)) {
		std::istringstream iss(line);
		float xpos, ypos, zpos;
		iss >> xpos >> ypos >> zpos;
		if (iss.fail())
			break;
		profile.push_back(Points(xpos, ypos, zpos));
	}
	ifile.close();



}
void AddBucketHull(std::vector<Points> p_ext, std::vector<Points> p_int, std::shared_ptr<ChBody> bucket) {

	for (int iter = 0; iter < p_ext.size() - 1; iter++) {
		std::vector<ChVector<double>> cloud;
		double width = 1.0;// or halve an input

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my - width, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my - width, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter + 1].mx, p_ext[iter + 1].my - width, p_ext[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter].mx, p_ext[iter].my - width, p_ext[iter].mz));

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter + 1].mx, p_ext[iter + 1].my + width, p_ext[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter].mx, p_ext[iter].my + width, p_ext[iter].mz));



		bucket->GetCollisionModel()->AddConvexHull(cloud, ChVector<>(0, 0, 0), QUNIT);
		bucket->GetCollisionModel()->BuildModel();

		auto shape = std::make_shared<ChTriangleMeshShape>();
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(cloud, shape->GetMesh());
		//bucket->AddAsset(shape);

		//bucket->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
	}

}
void AddCapsHulls(std::vector<Points> p_int, BucketSide side, std::shared_ptr<ChBody> bucket) {


	for (int iter = 0; iter < p_int.size() - 1; iter++) {

		std::vector<ChVector<double>> cloud;
		double width;
		switch (side)
		{
		case LEFT:
			width = +1.;
			break;
		case RIGHT:
			width = -1.0;
			break;
		default:
			width = .0;
			break;
		}


		double th = .05;

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width - th, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width - th, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(.70, width - th, .25));

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width + th, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width + th, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(.70, width + th, .25));

		bucket->GetCollisionModel()->AddConvexHull(cloud, ChVector<>(0, 0, 0), QUNIT);
		bucket->GetCollisionModel()->BuildModel();

		auto shape = std::make_shared<ChTriangleMeshShape>();
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(cloud, shape->GetMesh());
		//bucket->AddAsset(shape);

		//bucket->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
	}
}
void CreateMechanism(ChSystem& system, std::shared_ptr<ChBody> ground){
	
	std::vector<Points> p_ext;
	std::vector<Points> p_int;
	const std::string out_dir = "../";
	const std::string& ext_profile = out_dir + "data/ext_profile.txt";
	const std::string& int_profile = out_dir + "data/int_profile.txt";
	ReadFile(ext_profile, p_ext);
	ReadFile(int_profile, p_int);
	// Create a material (will be used by both objects)
	auto materialDVI = std::make_shared<ChMaterialSurfaceNSC>();
	materialDVI->SetRestitution(0.1f);
	materialDVI->SetFriction(0.4f);
	// Create a material (will be used by both objects)
	auto materialDEM = std::make_shared<ChMaterialSurfaceSMC>();
	materialDEM->SetYoungModulus(1.0e7f);
	materialDEM->SetRestitution(0.1f);
	materialDEM->SetFriction(0.4f);
	materialDEM->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model



	ChVector<> COG_chassis(0, 0, 1.575); // somewhere
	ChVector<> COG_lift(2.0, 0., 1.05);
	ChVector<> COG_lever(3.6625, 0.0, 1.575);
	ChVector<> COG_rod(2.7, 0.0, 1.3125);
	ChVector<> COG_link(3.0, 0.0, 0.5);
	ChVector<> COG_bucket(4.0, .0, 0.525);

	ChVector<> POS_lift2rod(2.825, .0, 1.05);//rev joint(LIFT ARM) abs frame
	ChVector<> POS_rod2lever(3.6625, 0., 1.575);//rev joint(BUCKET LEVER) abs frame
	ChVector<> POS_lift2bucket(3.50, .0, .21);//chassis piston(LIFT ARM) insertion abs frame
	ChVector<> POS_ch2lift(1.6, 0, 2.1);//Rev chassis->lift
	ChVector<> POS_lift2lever(2.5, 0, 2.1);//end position of actuator lift->lever
	ChVector<> PIS_ch2lift(1.6, 0, 1.05);//Act chassis->lift
	ChVector<> PIS_lift2lever(2.0125, 0, 2.1);//Act lift->lever

	ChVector<> POS_rod2link(2.6, 0, 0.4);//Act lift->lever
	ChVector<> POS_link2bucket(3.69, .0, 0.71);//chassis piston(BUCKET LEVER) insertion abs frame

	ChVector<> INS_ch2lift(1.8, 0, 1.1);// Insertion of boom piston over lift body

	ChQuaternion<> z2y;
	ChQuaternion<> z2x;
	z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
	z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
	// 2 Create the rigid bodies

	// LIFT
	//auto lift = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());//switched to ChBodyAuxRef
	auto lift = std::shared_ptr<ChBody>(system.NewBody());//switched to ChBodyAuxRef
	system.Add(lift);
	lift->SetBodyFixed(false);
	lift->SetName("lift arm");
	//lift->SetPos(POS_ch2lift);// COG_lift changed
	lift->SetPos(COG_lift);// COG_lift changed
	// ChVector<> u1 = (POS_lift2bucket - POS_ch2lift).GetNormalized();//
	ChVector<> u1 = (POS_lift2bucket - COG_lift).GetNormalized();//
	ChVector<> w1 = Vcross(u1, VECT_Y).GetNormalized();//overkill
	ChMatrix33<> rot1;//no control on orthogonality
	rot1.Set_A_axis(u1, VECT_Y, w1);
	lift->SetRot(rot1);
	//lift->SetFrame_COG_to_REF(ChFrame<>(lift->GetFrame_REF_to_abs().GetInverse() * COG_lift, QUNIT));//switched to ChBodyAuxRef
	lift->SetMass(993.5);
	lift->SetInertiaXX(ChVector<>(110.2, 1986.1, 1919.3));
	lift->SetInertiaXY(ChVector<>(0., 0., 339.6));

	// visualization:
	auto lift_asset = std::make_shared<ChCylinderShape>();
	lift_asset->GetCylinderGeometry().rad = .025;
	lift_asset->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_ch2lift;
	lift_asset->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
	auto lift_asset1 = std::make_shared<ChCylinderShape>();
	lift_asset1->GetCylinderGeometry().rad = .025;
	lift_asset1->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
	lift_asset1->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2bucket;
	auto lift_asset2 = std::make_shared<ChCylinderShape>();
	lift_asset2->GetCylinderGeometry().rad = .025;
	lift_asset2->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_ch2lift;
	lift_asset2->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * INS_ch2lift;
	auto lift_asset3 = std::make_shared<ChCylinderShape>();
	lift_asset3->GetCylinderGeometry().rad = .025;
	lift_asset3->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * INS_ch2lift;
	lift_asset3->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2bucket;

					// lift->AddAsset(lift_asset);
					// lift->AddAsset(lift_asset1);
					// lift->AddAsset(lift_asset2);
					// lift->AddAsset(lift_asset3);
	auto col_l = std::make_shared<ChColorAsset>();
	// col_l->SetColor(ChColor(0.2f, 0.0f, 0.0f));
					lift->AddAsset(col_l);
	geometry::ChTriangleMeshConnected lift_mesh;
	lift_mesh.LoadWavefrontMesh(out_dir + "data/boom_mod.obj", false, false);
	auto lift_mesh_shape = std::make_shared<ChTriangleMeshShape>();
	lift_mesh_shape->SetMesh(lift_mesh);
	lift_mesh_shape->SetName("boom");
	lift->AddAsset(lift_mesh_shape);

	// ROD
	auto rod = std::shared_ptr<ChBody>(system.NewBody());
	system.Add(rod);
	rod->SetName("rod arm");
	rod->SetIdentifier(3);
	//rod->SetPos(POS_lift2rod);//COG_rod
	rod->SetPos(COG_rod);//
	ChVector<> u3 = (POS_rod2link - POS_lift2rod).GetNormalized();
	ChVector<> w3 = Vcross(u3, VECT_Y).GetNormalized();//overkill
	ChMatrix33<> rot3;
	rot3.Set_A_axis(u3, VECT_Y, w3);
	rod->SetRot(rot3);
//	rod->SetFrame_COG_to_REF(ChFrame<>(rod->GetFrame_REF_to_abs().GetInverse() * COG_rod, QUNIT));//switched to ChBodyAuxRef
	rod->SetMass(381.5);
	rod->SetInertiaXX(ChVector<>(11.7, 33.4, 29.5));
	rod->SetInertiaXY(ChVector<>(0., 0., -12.1));
	// No collision properties:
	// visualization properties:
	auto rod_asset = std::make_shared<ChCylinderShape>();
	rod_asset->GetCylinderGeometry().rad = .025;
	rod_asset->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2lever;
	rod_asset->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
	auto rod_asset1 = std::make_shared<ChCylinderShape>();
	rod_asset1->GetCylinderGeometry().rad = .025;
	rod_asset1->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
	rod_asset1->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_rod2link;
	rod->AddAsset(rod_asset);
	rod->AddAsset(rod_asset1);
	auto col_r = std::make_shared<ChColorAsset>();
	col_r->SetColor(ChColor(0.0f, 0.0f, 0.2f));
	rod->AddAsset(col_r);
	geometry::ChTriangleMeshConnected rocker_mesh;
	rocker_mesh.LoadWavefrontMesh(out_dir + "data/rockerarm_mod.obj", false, false);
	auto rocker_mesh_shape = std::make_shared<ChTriangleMeshShape>();
	rocker_mesh_shape->SetMesh(rocker_mesh);
	rocker_mesh_shape->SetName("boom");
	//rod->AddAsset(rocker_mesh_shape);

	// LINK
	auto link = std::shared_ptr<ChBody>(system.NewBody());
	system.Add(link);
	link->SetName("link arm");
	link->SetIdentifier(3);
	link->SetPos(COG_link);
	ChVector<> u4 = (POS_link2bucket - POS_rod2link).GetNormalized();
	ChVector<> w4 = Vcross(u4, VECT_Y).GetNormalized();//overkill
	ChMatrix33<> rot4;
	rot4.Set_A_axis(u4, VECT_Y, w4);
	link->SetRot(rot4);
	link->SetMass(277.2);
	link->SetInertiaXX(ChVector<>(3.2, 11.1, 13.6));
	link->SetInertiaXY(ChVector<>(0.0, 0.0, -.04));
	// No collision properties:
	// visualization properties:
	auto link_asset = std::make_shared<ChCylinderShape>();
	link_asset->GetCylinderGeometry().rad = .025;
	link_asset->GetCylinderGeometry().p1 = link->GetFrame_COG_to_abs().GetInverse() * POS_rod2link;
	link_asset->GetCylinderGeometry().p2 = link->GetFrame_COG_to_abs().GetInverse() * POS_link2bucket;
	link->AddAsset(link_asset);
	auto col_l1 = std::make_shared<ChColorAsset>();
	col_l1->SetColor(ChColor(0.0f, 0.2f, 0.2f));
	link->AddAsset(col_l1);
	// BUCKET
	auto bucket = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());
	system.AddBody(bucket);
	bucket->SetName("benna");
	bucket->SetIdentifier(4);
	bucket->SetMass(200.0);//not confirmed data
	bucket->SetInertiaXX(ChVector<>(200, 500, 200));//not confirmed data
	bucket->SetPos(POS_lift2bucket);
	//bucket->SetFrame_COG_to_REF(ChFrame<> (bucket->GetFrame_REF_to_abs().GetInverse() * COG_bucket,QUNIT));
	bucket->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(.35, .0, .2), QUNIT));//tentative
	// Create contact geometry.
	bucket->SetCollide(true);
	bucket->GetCollisionModel()->ClearModel();
	AddBucketHull(p_ext, p_int, bucket);
	AddCapsHulls(p_int, BucketSide::LEFT, bucket);
	AddCapsHulls(p_int, BucketSide::RIGHT, bucket);
#ifdef USE_PENALTY//temporary workaround
	bucket->SetMaterialSurface(materialDEM);
#else
	bucket->SetMaterialSurface(materialDVI);
#endif
	bucket->GetCollisionModel()->BuildModel();
	geometry::ChTriangleMeshConnected bucket_mesh;
	bucket_mesh.LoadWavefrontMesh(out_dir + "data/bucket_mod.obj", false, false);
	auto bucket_mesh_shape = std::make_shared<ChTriangleMeshShape>();
	bucket_mesh_shape->SetMesh(bucket_mesh);
	bucket_mesh_shape->SetName("bucket");
	bucket->AddAsset(bucket_mesh_shape);

	// CHASSIS
	auto chassis = std::shared_ptr<ChBody>(system.NewBody());
	system.AddBody(chassis);
	chassis->SetBodyFixed(false);
	chassis->SetName("chassis");
	chassis->SetIdentifier(0);
	chassis->SetMass(2000.0);
	chassis->SetPos(COG_chassis);
	chassis->SetInertiaXX(ChVector<>(500., 1000., 500.));
	 // visualization properties
	auto chassis_asset = std::make_shared<ChSphereShape>();				//asset
	chassis_asset->GetSphereGeometry().rad = .15;						//asset
	chassis->AddAsset(chassis_asset);


	// 3. Add joint constraints
	// LIFT-ROD spring(it simulates the actuator)
	auto springdamper_ground_ball = std::make_shared<ChLinkSpring>();
	springdamper_ground_ball->Initialize(rod, lift, false, POS_lift2lever, PIS_lift2lever, true, .00);
	springdamper_ground_ball->Set_SpringK(50.0);
	springdamper_ground_ball->Set_SpringR(0.0);
	auto lin_lift2rod = std::make_shared<ChLinkLinActuator>();
	ChVector<> u11 = (POS_lift2lever - PIS_lift2lever).GetNormalized();//Normalize,not GetNormalize
	ChVector<> w11 = Vcross(u11, VECT_Y).GetNormalized();//overkill
	ChMatrix33<> rot11;//no control on orthogonality
	rot11.Set_A_axis(u11, VECT_Y, w11);
	lin_lift2rod->SetName("linear_lift2rod");
	lin_lift2rod->Initialize(rod, lift, false, ChCoordsys<>(POS_lift2lever, z2x >> rot11.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot11.Get_A_quaternion()));//m2 is the master
	lin_lift2rod->Set_lin_offset(Vlength(POS_lift2lever - PIS_lift2lever));
	// Asset for the linear actuator
	auto bp_asset = std::make_shared<ChPointPointSegment>();				//asset
	lin_lift2rod->AddAsset(bp_asset);
	//
	auto legge1 = std::make_shared<ChFunction_Ramp>();
	legge1->Set_ang(.50);
	auto legge2 = std::make_shared<ChFunction_Const>();
	//legge2->Set_yconst(Vlength(POS_lift2lever - PIS_lift2lever) + 1.0);//Does it take the actual value or that one at the beginning?
	auto legge3 = std::make_shared<ChFunction_Ramp>();
	legge3->Set_ang(-.50);//Does it take the actual value or that one at the beginning?
	auto tilt_law = std::make_shared<ChFunction_Sequence>();
	tilt_law->InsertFunct(legge1, 1.0, 1, true);
	tilt_law->InsertFunct(legge2, 1.0, 1., true);
	tilt_law->InsertFunct(legge3, 1.0, 1., true);
	auto tilt_law_seq = std::make_shared<ChFunction_Repeat>();
	tilt_law_seq->Set_fa(tilt_law);
	tilt_law_seq->Set_window_length(3.0);
	tilt_law_seq->Set_window_start(0.0);
	tilt_law_seq->Set_window_phase(3.0);
	// test_law
	auto tilt_law_test = std::make_shared<ChFunction_Const>();
	tilt_law_test->Set_yconst(Vlength(VNULL));
	// end test_law
	lin_lift2rod->Set_dist_funct(tilt_law_test);

	//system->AddLink(springdamper_ground_ball);
	system.Add(lin_lift2rod);
	// LIFT-ROD inthe middle revjoint
	auto rev_lift2rod = std::make_shared<ChLinkLockRevolute>();
	rev_lift2rod->SetName("revolute_lift2rod");
	rev_lift2rod->Initialize(rod, lift, ChCoordsys<>(POS_lift2rod, z2y >> rot3.Get_A_quaternion()));
	system.AddLink(rev_lift2rod);
	// ROD-LINK revjoint
	auto rev_rod2link = std::make_shared<ChLinkLockRevolute>();
	rev_rod2link->SetName("revolute_rod2link");
	ChMatrix33<> rotb44;
	rev_rod2link->Initialize(link, rod, ChCoordsys<>(POS_rod2link, z2y >> rotb44.Get_A_quaternion()));
	system.AddLink(rev_rod2link);
	// LIFT-BUCKET revjoint
	auto rev_lift2bucket = std::make_shared<ChLinkLockRevolute>();
	rev_lift2bucket->SetName("revolute_lift2bucket");
	ChMatrix33<> rotb1;
	rev_lift2bucket->Initialize(bucket, lift, ChCoordsys<>(POS_lift2bucket, z2y >> rotb1.Get_A_quaternion()));
	system.AddLink(rev_lift2bucket);
	// LINK-BUCKET revjoint
	auto rev_link2bucket = std::make_shared<ChLinkLockRevolute>();
	rev_link2bucket->SetName("revolute_link2bucket");
	ChMatrix33<> rotb2;
	rev_link2bucket->Initialize(bucket, link, ChCoordsys<>(POS_link2bucket, z2y >> rotb2.Get_A_quaternion()));//Does it make sense?
	system.AddLink(rev_link2bucket);

	// CHASSIS-LIFT revjoint
	auto rev_ch2lift = std::make_shared<ChLinkLockRevolute>();
	rev_ch2lift->SetName("revolute_chassis2lift");
	rev_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(POS_ch2lift, z2y >> rot1.Get_A_quaternion()), ChCoordsys<>(POS_ch2lift, z2y >> rot1.Get_A_quaternion()));
	system.AddLink(rev_ch2lift);
	// CHASSIS-LIFT linear
	auto lin_ch2lift = std::make_shared<ChLinkLinActuator>();
	ChVector<> u22 = (INS_ch2lift - PIS_ch2lift).GetNormalized();//Normalize,not GetNormalize
	ChVector<> w22 = Vcross(u22, VECT_Y).GetNormalized();//overkill
	ChMatrix33<> rot22;//no control on orthogonality
	rot22.Set_A_axis(u22, VECT_Y, w22);
	lin_ch2lift->SetName("linear_chassis2lift");
	lin_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(INS_ch2lift, z2x >> rot22.Get_A_quaternion()), ChCoordsys<>(PIS_ch2lift, z2x >> rot11.Get_A_quaternion()));//m2 is the master
	lin_ch2lift->Set_lin_offset(Vlength(INS_ch2lift - PIS_ch2lift));
	auto lift_law = std::make_shared<ChFunction_Sequence>();
	lift_law->InsertFunct(legge1, 1.0, 1, true);
	lift_law->InsertFunct(legge2, 1.0, 1., true);
	lift_law->InsertFunct(legge3, 1.0, 1., true);
	auto lift_law_seq = std::make_shared<ChFunction_Repeat>();
	lift_law_seq->Set_fa(lift_law);
	lift_law_seq->Set_window_length(3.0);
	lift_law_seq->Set_window_start(0.0);
	lift_law_seq->Set_window_phase(3.0);
	auto lift_law_sine = std::make_shared<ChFunction_Sine>();
	lift_law_sine->Set_w(1.0472);
	lift_law_sine->Set_amp(.5*Vlength(INS_ch2lift - PIS_ch2lift));

	lin_ch2lift->AddAsset(std::make_shared<ChPointPointSegment>());

	// test_law
	auto lift_law_test = std::make_shared<ChFunction_Const>();
	lift_law_test->Set_yconst(Vlength(VNULL));
	// end test_law

	// Function ChFunctionRecorder
			// ChFunction_Recorder lift_law_fr;
	auto lift_law_fr = std::make_shared<ChFunction_Recorder>();
	
	std::vector<double> punti = {0.0,0.01,0.26,0.54,0.56,0.7,0.8,0.9,1.0};
	for (int i=0;i<punti.size();i++){
		lift_law_fr->AddPoint(punti[i],std::sin(punti[i]*CH_C_2PI));
	} 
	// end Function ChFunctionRecorder


	lin_ch2lift->Set_dist_funct(lift_law_fr);//Sine function test for lifting

	system.AddLink(lin_ch2lift);

	// CHASSIS-GROUND prismatic+linactuator
	auto prism_fix2ch = std::make_shared<ChLinkLockPrismatic>();
	prism_fix2ch->SetName("prismatic_ground2chassis");
	prism_fix2ch->Initialize(chassis, ground, ChCoordsys<>(COG_chassis, z2x));
	system.AddLink(prism_fix2ch);
	auto lin_fix2ch = std::make_shared<ChLinkLinActuator>();
	prism_fix2ch->SetName("linear_ground2chassis");
	lin_fix2ch->Initialize(chassis, ground, false, ChCoordsys<>(COG_chassis, z2x), ChCoordsys<>(COG_chassis, z2x));//m2 is the master
	lin_fix2ch->Set_lin_offset(Vlength(VNULL));
	system.AddLink(lin_fix2ch);
	auto chassis_law = std::make_shared<ChFunction_Ramp>();
	chassis_law->Set_ang(10);//it'll act as the chassis speed
	lin_fix2ch->Set_dist_funct(chassis_law);
	
}
// Create Particles function
utils::Generator CreateParticles(ChSystem* system, std::shared_ptr<ChMaterialSurface> material_terrain){
	// Aliquotes
	double quote_sp = 0.35;//1//.00
	double quote_bs = 0.10;//2
	double quote_el = 0.00;//3//.35
	double quote_cs = 0.00;//4
	double quote_bx = 0.40;//5
	double quote_rc = 0.00;//6

	double quote_sbx = 0.15;// Create a particle generator and a mixture entirely made out of bispheres
	utils::Generator gen(system);
	gen.setBodyIdentifier(Id_g);
	// SPHERES
	std::shared_ptr<utils::MixtureIngredient> m0 = gen.AddMixtureIngredient(utils::SPHERE, quote_sp);
	m0->setDefaultMaterial(material_terrain);
	m0->setDefaultDensity(rho_g);
	m0->setDefaultSize(radius_g);
	// BISPHERES
	std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::BISPHERE, quote_bs);
	m1->setDefaultMaterial(material_terrain);
	m1->setDefaultDensity(rho_g);
	m1->setDefaultSize(radius_g);
	// Add new types of shapes to the generator, giving the percentage of each one
	//ELLIPSOIDS
	std::shared_ptr<utils::MixtureIngredient> m2 = gen.AddMixtureIngredient(utils::ELLIPSOID, quote_el);
	m2->setDefaultMaterial(material_terrain);
	m2->setDefaultDensity(rho_g);
	m2->setDefaultSize(radius_g);// this need a vector  
	// Add new types of shapes to the generator, giving the percentage of each one
	//CAPSULES/
	std::shared_ptr<utils::MixtureIngredient> m3 = gen.AddMixtureIngredient(utils::CAPSULE, quote_cs);
	m3->setDefaultMaterial(material_terrain);
	m3->setDefaultDensity(rho_g);
	m3->setDefaultSize(radius_g);
	//BOXES/
	std::shared_ptr<utils::MixtureIngredient> m4 = gen.AddMixtureIngredient(utils::BOX, quote_bx);
	m4->setDefaultMaterial(material_terrain);
	m4->setDefaultDensity(rho_g);
	m4->setDefaultSize(radius_g);
	//ROUNDED-CYLINDERS/
	std::shared_ptr<utils::MixtureIngredient> m5 = gen.AddMixtureIngredient(utils::ROUNDEDCYLINDER, quote_rc);
	m5->setDefaultMaterial(material_terrain);
	m5->setDefaultDensity(rho_g);
	m5->setDefaultSize(radius_g);
	//BOXES/
	std::shared_ptr<utils::MixtureIngredient> m6 = gen.AddMixtureIngredient(utils::BOX, quote_sbx);
	m6->setDefaultMaterial(material_terrain);
	m6->setDefaultDensity(rho_g);
	m6->setDefaultSize(radius_g);


	

	for (int il = 0; il < num_layers; il++) {
		gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
		center.z() += 2 * r;
		// shrink uniformly the upper layer
		hdims.x() -= 2 * r;
		hdims.y() -= 2 * r;
		// move the center abscissa by a 1*r(DISABLED FOR THE MOMENT) 
		center.x() += r / 2 * pow(-1, il);

	}

return gen;
}
// Create Terrain function
std::shared_ptr<ChBody> CreateTerrain(ChSystem* system, std::shared_ptr<ChMaterialSurface> material_terrain){
	// Create container body
	auto container = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(container);
	container->SetIdentifier(-1);
	container->SetMass(1);
	container->SetPos(terrain_center);
	container->SetBodyFixed(true);
	container->SetCollide(true);
	container->SetMaterialSurface(material_terrain);

	container->GetCollisionModel()->ClearModel();
	// Bottom box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY , hthick), ChVector<>(0, 0, -2 * hthick),
		ChQuaternion<>(1, 0, 0, 0), true);
	// Adding a "roughness" to the terrain, consisting of sphere/capsule/ellipsoid grid
	//	double spacing = 3.5 * radius_g;
	int ccc = std::ceil(hdimX/Ra_d/2) + 15;

	for (int ix = -ccc; ix < ccc; ix++) {
		for (int iy = -ccc; iy < ccc; iy++) {
			ChVector<> pos(ix * Ra_d, iy * Ra_d, -Ra_r);
			utils::AddSphereGeometry(container.get(), Ra_r, pos);
		}
	}
	container->GetCollisionModel()->BuildModel();

	return container;
}

// --------------------------------------------------------------------------

int main(int argc, char** argv) {
	// Get number of threads from arguments (if specified)
	if (argc > 1) {
		num_threads = std::stoi(argv[1]);
	}

	std::cout << "Requested number of threads: " << num_threads << std::endl;

	std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;


	// --------------------------
	// Create output directories.
	// --------------------------

	if (povray_output) {
		if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
			std::cout << "Error creating directory " << out_dir << std::endl;
			return 1;
		}
		if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
			std::cout << "Error creating directory " << pov_dir << std::endl;
			return 1;
		}
	}

	// --------------------------
	// Create system and set method-specific solver settings
	// --------------------------

	chrono::ChSystemParallel* system;

	switch (method) {
	case ChMaterialSurface::SMC: {
		ChSystemParallelSMC* sys = new ChSystemParallelSMC;
		sys->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
		sys->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
		sys->GetSettings()->solver.use_material_properties = use_mat_properties;
		system = sys;

		break;
	}
	case ChMaterialSurface::NSC: {
		ChSystemParallelNSC* sys = new ChSystemParallelNSC;
		sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
		sys->GetSettings()->solver.max_iteration_normal = 0;
		sys->GetSettings()->solver.max_iteration_sliding = 200;
		sys->GetSettings()->solver.max_iteration_spinning = 0;
		sys->GetSettings()->solver.alpha = 0;
		sys->GetSettings()->solver.contact_recovery_speed = -1;
		sys->GetSettings()->collision.collision_envelope = 0.1 * radius_g;
		sys->ChangeSolverType(SolverType::APGD);
		system = sys;

		break;
	}
	}

	system->Set_G_acc(ChVector<>(0, 0, -9.81));
	system->GetSettings()->perform_thread_tuning = false;
	system->GetSettings()->solver.use_full_inertia_tensor = false;
	system->GetSettings()->solver.tolerance = 0.1;
	system->GetSettings()->solver.max_iteration_bilateral = 100;
	system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
	system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);

	// Set number of threads
	system->SetParallelThreadNumber(num_threads);
	CHOMPfunctions::SetNumThreads(num_threads);

	// Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master
	{ std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

	// ---------------------
	// Create terrain bodies
	// ---------------------

	// Create contact material for terrain
	std::shared_ptr<ChMaterialSurface> material_terrain;

	switch (method) {
	case ChMaterialSurface::SMC: {
		auto mat_ter = std::make_shared<ChMaterialSurfaceSMC>();
		mat_ter->SetFriction(friction_terrain);
		mat_ter->SetRestitution(restitution_terrain);
		mat_ter->SetYoungModulus(Y_terrain);
		mat_ter->SetPoissonRatio(nu_terrain);
		mat_ter->SetAdhesion(coh_force_terrain);
		mat_ter->SetKn(kn_terrain);
		mat_ter->SetGn(gn_terrain);
		mat_ter->SetKt(kt_terrain);
		mat_ter->SetGt(gt_terrain);

		material_terrain = mat_ter;

		break;
	}
	case ChMaterialSurface::NSC: {
		auto mat_ter = std::make_shared<ChMaterialSurfaceNSC>();
		mat_ter->SetFriction(friction_terrain);
		mat_ter->SetRestitution(restitution_terrain);
		mat_ter->SetCohesion(coh_force_terrain);

		material_terrain = mat_ter;

		break;
	}
	}
	// ----------------
	// Create the Terrain
	// ----------------

						std::shared_ptr<ChBody> container = CreateTerrain(system, material_terrain);
	
	// ----------------
	// Create the Mechanism
	// ----------------
	
	//CreateMechanism(*system, container);// Previous function.Obsolete.
	MyWheelLoader* mywl = new MyWheelLoader(*system);
	system->ShowHierarchy(GetLog());
					// Setting Lifting Function Example
		auto act = std::dynamic_pointer_cast<ChLinkLinActuator>(mywl->lin_ch2lift);
		auto fun = std::make_shared<ChFunction_Sine>();
		fun->Set_freq(2.0); fun->Set_amp(.05);
		//act->Set_dist_funct(fun);
		if (act->Get_dist_funct() == fun){ GetLog() << act->Get_dist_funct() << " = " << fun << "--> test passed." << "\n"; }
		else
		{
			GetLog() << "Distance Function Settings : Default\n";
		}
				// Setting Lifting Function Example
	// CHASSIS-GROUND prismatic+linactuator
	auto prism_fix2ch = std::make_shared<ChLinkLockPrismatic>();
	prism_fix2ch->SetName("prismatic_ground2chassis");
	prism_fix2ch->Initialize(mywl->chassis, container, ChCoordsys<>(mywl->chassis->GetPos(),mywl->z2x));
	system->AddLink(prism_fix2ch);

	auto lin_fix2ch = std::make_shared<ChLinkLinActuator>();
	prism_fix2ch->SetName("linear_ground2chassis");
	ChQuaternion<> prismrot;
	prismrot.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
	lin_fix2ch->Initialize(mywl->chassis, container, false, ChCoordsys<>(mywl->chassis->GetPos(), prismrot), ChCoordsys<>(ChVector<>(-10.0,0.,0.), prismrot));//m2 is the master
	lin_fix2ch->Set_lin_offset(Vlength(ChVector<>(-10.,0.,0.) - mywl->chassis->GetPos()));// A random far point is chosen s.t. in "reverse gear" motion linear displacement does not turn to negative(i.e. bad behaviour)
	system->AddLink(lin_fix2ch);

	// Note : it is as displacement law
	auto legge1 = std::make_shared<ChFunction_Ramp>();
	legge1->Set_ang(.50);
	auto legge2 = std::make_shared<ChFunction_Const>();
	auto legge3 = std::make_shared<ChFunction_Ramp>();
	legge3->Set_ang(-.50);//Does it take the actual value or that one at the beginning?
	auto chassis_law = std::make_shared<ChFunction_Sequence>();chassis_law->InsertFunct(legge1, ta1, 1, true);chassis_law->InsertFunct(legge2, ta2-ta1, 1., true);chassis_law->InsertFunct(legge3, ta3-ta2, 1., true);

	lin_fix2ch->Set_dist_funct(chassis_law);

	
	// ----------------
	// Create particles
	// ----------------
	
					utils::Generator gen = CreateParticles(system, material_terrain);
	
					// unsigned int num_particles = gen.getTotalNumBodies();
					// std::cout << "Generated particles:  " << num_particles << std::endl;


#ifdef CHRONO_OPENGL
	// -------------------------------
	// Create the visualization window
	// -------------------------------

	if (render) {
		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "Sandpile + zBar", system);
		gl_window.SetCamera(ChVector<>(5, 8, 0), ChVector<>(3.5, 0, 0), ChVector<>(0, 0, 1));
		gl_window.SetRenderMode(opengl::WIREFRAME);
	}
#endif

	// ---------------
	// Simulate system
	// ---------------

	double time_end = 15.0;
	double time_step = 1e-5;//1e-5 Desktop Working Version


	// Initialize simulation frame counter and simulation time
	int render_step_size = 50;
	int step_number = 0;
	int render_frame = 0;
	// Number of simulation steps between two 3D view render frames
	int render_steps = (int)std::ceil(render_step_size / time_step);

	char filename[100];


	utils::WriteMeshPovray("boom_mod.obj","boom.inc","\..");
	utils::WriteMeshPovray("bucket_mod.obj", "bucket.inc", "\..");


	while (system->GetChTime() < time_end) {
		if (povray_output){
			if (step_number % render_step_size == 0) {
				// Output render data
				sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
				utils::WriteShapesPovray(system, filename);
				std::cout << "Output frame:   " << render_frame << std::endl;
				std::cout << "Sim frame:      " << step_number << std::endl;
				std::cout << "Time:           " << time << std::endl;
				/*std::cout << "             throttle: " << driver.GetThrottle() << " steering: " << driver.GetSteering()
					<< std::endl;
				*/std::cout << std::endl;
				render_frame++;
			}
		}
		//std::cout << system->GetChTime() << std::endl;
		system->DoStepDynamics(time_step);

#ifdef CHRONO_OPENGL
		if (render) {
			opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
			if (gl_window.Active()) {
				gl_window.Render();
			}
			else {
				return 1;
			}
		}
#endif
		// Increment frame number
		step_number++;
	}

	return 0;
}