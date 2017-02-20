// First test of binding together granular matter and loader mechanism.
// Victor Bertolazzo
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

#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChTimer.h"
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
	auto materialDVI = std::make_shared<ChMaterialSurface>();
	materialDVI->SetRestitution(0.1f);
	materialDVI->SetFriction(0.4f);
	// Create a material (will be used by both objects)
	auto materialDEM = std::make_shared<ChMaterialSurfaceDEM>();
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
	auto lift = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());//switched to ChBodyAuxRef
	//		auto lift = std::make_shared<ChBody>(contact_method);
	system.Add(lift);
	lift->SetBodyFixed(false);
	lift->SetName("lift arm");
	lift->SetPos(POS_ch2lift);// COG_lift changed
	ChVector<> u1 = (POS_lift2bucket - POS_ch2lift).GetNormalized();//Normalize,not GetNormalize
	ChVector<> w1 = Vcross(u1, VECT_Y).GetNormalized();//overkill
	ChMatrix33<> rot1;//no control on orthogonality
	rot1.Set_A_axis(u1, VECT_Y, w1);
	lift->SetRot(rot1);
	lift->SetFrame_COG_to_REF(ChFrame<>(lift->GetFrame_REF_to_abs().GetInverse() * COG_lift, QUNIT));//switched to ChBodyAuxRef
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

	//				lift->AddAsset(lift_asset);
	//				lift->AddAsset(lift_asset1);
	//				lift->AddAsset(lift_asset2);
	//				lift->AddAsset(lift_asset3);
	auto col_l = std::make_shared<ChColorAsset>();
	col_l->SetColor(ChColor(0.2f, 0.0f, 0.0f));
	//				lift->AddAsset(col_l);
	geometry::ChTriangleMeshConnected lift_mesh;
	lift_mesh.LoadWavefrontMesh(out_dir + "data/boom_mod.obj", false, false);
	auto lift_mesh_shape = std::make_shared<ChTriangleMeshShape>();
	lift_mesh_shape->SetMesh(lift_mesh);
	lift_mesh_shape->SetName("boom");
	lift->AddAsset(lift_mesh_shape);

	// ROD
	auto rod = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());
	system.Add(rod);
	rod->SetName("rod arm");
	rod->SetIdentifier(3);
	rod->SetPos(POS_lift2rod);//COG_rod
	ChVector<> u3 = (POS_rod2link - POS_lift2rod).GetNormalized();
	ChVector<> w3 = Vcross(u3, VECT_Y).GetNormalized();//overkill
	ChMatrix33<> rot3;
	rot3.Set_A_axis(u3, VECT_Y, w3);
	rod->SetRot(rot3);
	rod->SetFrame_COG_to_REF(ChFrame<>(rod->GetFrame_REF_to_abs().GetInverse() * COG_rod, QUNIT));//switched to ChBodyAuxRef
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
	// collision properties:
	chassis->GetCollisionModel()->ClearModel();
	chassis->GetCollisionModel()->AddSphere(.05, VNULL);
	chassis->GetCollisionModel()->BuildModel();
	chassis->SetCollide(true);
	// visualization properties
	auto chassis_asset = std::make_shared<ChSphereShape>();//asset
	chassis_asset->GetSphereGeometry().rad = .05;//asset
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
	lin_lift2rod->SetName("linear_chassis2lift");
	lin_lift2rod->Initialize(rod, lift, false, ChCoordsys<>(POS_lift2lever, z2x >> rot11.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot11.Get_A_quaternion()));//m2 is the master
	lin_lift2rod->Set_lin_offset(Vlength(POS_lift2lever - PIS_lift2lever));
	// S� way
	auto bp_asset = std::make_shared<ChPointPointSegment>();//asset
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
	lin_ch2lift->Set_dist_funct(lift_law_sine);//Sine function test for lifting

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
// --------------------------------------------------------------------------

int main(int argc, char** argv) {
	int num_threads = 4;
	ChMaterialSurfaceBase::ContactMethod method = ChMaterialSurfaceBase::DEM;//DEM
	bool use_mat_properties = true;
	bool render = true;
	bool track_granule = false;
	bool track_flatten = false;
	double radius_g = 0.01;

	double Ra_d = 2.5*radius_g;//Distance from centers of particles.
	double Ra_r = 1.5*radius_g;//Default Size of particles.


	// Aliquotes
	double quote_sp = 0.00;//1
	double quote_bs = 0.10;//2
	double quote_el = 0.35;//3
	double quote_cs = 0.00;//4
	double quote_bx = 0.40;//5
	double quote_rc = 0.00;//6

	double quote_sbx = .15;


	// Get number of threads from arguments (if specified)
	if (argc > 1) {
		num_threads = std::stoi(argv[1]);
	}

	std::cout << "Requested number of threads: " << num_threads << std::endl;

	// ----------------
	// Model parameters
	// ----------------

	// Container dimensions
	double hdimX = 1.0;
	double hdimY = 1.0;
	double hdimZ = 0.5;
	double hthick = 0.25;

	// Granular material properties
	int Id_g = 10000;
	double rho_g = 2500;
	double vol_g = (4.0 / 3) * CH_C_PI * radius_g * radius_g * radius_g;
	double mass_g = rho_g * vol_g;
	ChVector<> inertia_g = 0.4 * mass_g * radius_g * radius_g * ChVector<>(1, 1, 1);
	int num_layers = 24;// 

	// Terrain contact properties---Default Ones are commented out.
	float friction_terrain = 0.7f;// (H,W) requires mi=.70;
	float restitution_terrain = 0.0f;
	float Y_terrain = 8e5f;
	float nu_terrain = 0.3f;
	float kn_terrain = 1.0e4f;// 1.0e7f;
	float gn_terrain = 1.0e3f;
	float kt_terrain = 2.86e3f;// 2.86e6f;
	float gt_terrain = 1.0e3f;
	float coh_pressure_terrain = 1e4f;// 0e3f;
	float coh_force_terrain = (float)(CH_C_PI * radius_g * radius_g) * coh_pressure_terrain;

	// Estimates for number of bins for broad-phase
	int factor = 2;
	int binsX = (int)std::ceil(hdimX / radius_g) / factor;
	int binsY = (int)std::ceil(hdimY / radius_g) / factor;
	int binsZ = 1;

	binsX = 20;
	binsY = 20;
	binsZ = 10;
	std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;

	// --------------------------
	// Create the parallel system
	// --------------------------

	// Create system and set method-specific solver settings
	chrono::ChSystemParallel* system;

	switch (method) {
	case ChMaterialSurfaceBase::DEM: {
		ChSystemParallelDEM* sys = new ChSystemParallelDEM;
		sys->GetSettings()->solver.contact_force_model = ChSystemDEM::Hertz;
		sys->GetSettings()->solver.tangential_displ_mode = ChSystemDEM::TangentialDisplacementModel::OneStep;
		sys->GetSettings()->solver.use_material_properties = use_mat_properties;
		system = sys;

		break;
	}
	case ChMaterialSurfaceBase::DVI: {
		ChSystemParallelDVI* sys = new ChSystemParallelDVI;
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
	std::shared_ptr<ChMaterialSurfaceBase> material_terrain;

	switch (method) {
	case ChMaterialSurfaceBase::DEM: {
		auto mat_ter = std::make_shared<ChMaterialSurfaceDEM>();
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
	case ChMaterialSurfaceBase::DVI: {
		auto mat_ter = std::make_shared<ChMaterialSurface>();
		mat_ter->SetFriction(friction_terrain);
		mat_ter->SetRestitution(restitution_terrain);
		mat_ter->SetCohesion(coh_force_terrain);

		material_terrain = mat_ter;

		break;
	}
	}

	// Create container body
	auto container = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(container);
	container->SetIdentifier(-1);
	container->SetMass(1);
	container->SetPos(ChVector<>(10.0, 0.0, 0.0));
	container->SetBodyFixed(true);
	container->SetCollide(true);
	container->SetMaterialSurface(material_terrain);

	container->GetCollisionModel()->ClearModel();
	// Bottom box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -2 * hthick),
		ChQuaternion<>(1, 0, 0, 0), true);
	// Adding a "roughness" to the terrain, consisting of sphere/capsule/ellipsoid grid
	//	double spacing = 3.5 * radius_g;

	for (int ix = -40; ix < 40; ix++) {
		for (int iy = -40; iy < 40; iy++) {
			ChVector<> pos(ix * Ra_d, iy * Ra_d, -Ra_r);
			utils::AddSphereGeometry(container.get(), Ra_r, pos);
		}
	}
	container->GetCollisionModel()->BuildModel();

	// ----------------
	// Create the Mechanism
	// ----------------
	CreateMechanism(*system, container);
	system->ShowHierarchy(GetLog());

	// ----------------
	// Create particles
	// ----------------

	// Create a particle generator and a mixture entirely made out of bispheres
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

	///////////////////////////////////////////-----THINGS TO DO-----//////////////////////////////
	// 3m X 20cm cylinder                  --> 15k particles
	// big box in the bottom               --> 
	// DVI no cohesion but                 --> spinning and rolling
	// see if you can get DEM-P without    --> 

	// mechanism, ChFunctionRecorder for data ....
	// bind together mech and sand
	///////////////////////////////////////////-----THINGS TO DO-----//////////////////////////////

	// Create particles in layers until reaching the desired number of particles
	double r = 1.01 * radius_g;
	ChVector<> hdims(hdimX / 2 - r, hdimY / 2 - r, 0);//W=.795, hdims object for the function gen.createObjectsBox accepts the	FULL dimensions in each direction:PAY ATTENTION
	ChVector<> center(10.0, 0, .5 * r);
	for (int il = 0; il < num_layers; il++) {
		gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
		center.z() += 2 * r;
		// shrink uniformly the upper layer
		hdims.x() -= 2 * r;
		hdims.y() -= 2 * r;
		// move the center abscissa by a 1*r(DISABLED FOR THE MOMENT) 
		center.x() += r / 2 * pow(-1, il);

	}

	unsigned int num_particles = gen.getTotalNumBodies();
	std::cout << "Generated particles:  " << num_particles << std::endl;


#ifdef CHRONO_OPENGL
	// -------------------------------
	// Create the visualization window
	// -------------------------------

	if (render) {
		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "Settling test", system);
		gl_window.SetCamera(ChVector<>(10, 11, 10), ChVector<>(3.5, 0, 0), ChVector<>(0, 0, 1));
		gl_window.SetRenderMode(opengl::WIREFRAME);
	}
#endif

	// ---------------
	// Simulate system
	// ---------------

	double time_end = 1.50;
	double time_step = 1e-4;//1e-4;e-





	while (system->GetChTime() < time_end) {



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
	}

	return 0;
}