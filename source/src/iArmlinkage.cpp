// Sequential iArmlinkage mechanism of a front wheel loader
// Victor Bertolazzo

// The mechanism consists of : lift arm, rod arm, link arm, bucket
// They are attached to a fictitious chassis via : revjoint btw chassis and lift arm, linear actuator btw chassis and lift arm
// The mechanism is essentially 2D even if modeled in 3D: Chrono solver takes care of redundant constraints.
// The piston movement is modeled as a linear actuator at the moment: it could be driven either via displacement-imposed(reverse dynamics happens) and force-imposed.
// This simple test takes care only of the displacement-imposed actuattion.
// Only the bucket body has collision shapes, visualization shapes of the other bodies are kept simplified for a better understanding of the mechanism.


#include <iostream>

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/collision/ChCCollisionUtils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono/utils/ChUtilsGenerators.h"

#include <stdio.h>
#include <vector>
#include <cmath>

#include <irrlicht.h>

#include "chrono_opengl/ChOpenGLWindow.h"


// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::collision;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// Define Flag in order to decide which contact method algorithm use:DVI or DEM
#define USE_PENALTY
// Enumerator used for creating the closing caps of the bucket
enum BucketSide { LEFT, RIGHT };
// Utilities 

// Structure define for containing information
struct Points {
	Points() {}
	Points(float x, float y, float z)
		: mx(x), my(y), mz(z) {}
	float mx; float my; float mz;
};
// It fills a vector of Points with information given by a file
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
// Bucket main body creation via ConvexHulls definition:it adds the collision shapes identified by points located into the two Points structures
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


		auto shape = std::make_shared<ChTriangleMeshShape>();
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(cloud, shape->GetMesh());
		//bucket->AddAsset(shape);

		//bucket->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
	}
}
// Bucket caps adding:
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
		
		auto shape = std::make_shared<ChTriangleMeshShape>();
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(cloud, shape->GetMesh());
		//bucket->AddAsset(shape);

		//bucket->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
	}
}

int main(int argc, char* argv[]) {
	// Utilities Management
	std::vector<Points> p_ext;
	std::vector<Points> p_int;
	const std::string out_dir = "../";
	const std::string& ext_profile = out_dir + "data/ext_profile.txt";
	const std::string& int_profile = out_dir + "data/int_profile.txt";
	ReadFile(ext_profile, p_ext);
	ReadFile(int_profile, p_int);

	// Create a material (will be used by both objects)
	auto material = std::make_shared<ChMaterialSurfaceNSC>();
	material->SetRestitution(0.1f);
	material->SetFriction(0.4f);
	// Create a material (will be used by both objects)
	auto materialDEM = std::make_shared<ChMaterialSurfaceSMC>();
	materialDEM->SetYoungModulus(1.0e7f);
	materialDEM->SetRestitution(0.1f);
	materialDEM->SetFriction(0.4f);
	materialDEM->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model


	// 0. Set the path to the Chrono data folder
	SetChronoDataPath(CHRONO_DATA_DIR);

	// 1. Create the system: it's creating with a boring method due previous bugs
#ifdef USE_PENALTY
	ChSystemSMC system;
	ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;
#else
	ChSystemNSC system;
	ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC;
#endif

	// Set the gravity acceleration
	system.Set_G_acc(ChVector<>(.0,.0,-9.81));



	//////////////////////////////////////////////////////--------------LIST OF THE BODIES-------------------////////////////////////////////////////////
	// GROUND
//	auto ground = std::shared_ptr<ChBody>(system.NewBody());
	auto ground = std::make_shared<ChBody>(contact_method);
	system.Add(ground);
	ground->SetBodyFixed(true);
	ground->SetIdentifier(-1);
	ground->SetName("ground");

    // measures are in [m]
		ChVector<> COG_chassis(0, 0, 1.575);							// somewhere
		ChVector<> COG_lift(2.0, 0., 1.05);
		ChVector<> COG_lever(3.6625, 0.0, 1.575);
		ChVector<> COG_rod(2.7, 0.0, 1.3125);
		ChVector<> COG_bucket(4.0,.0, 0.525);

		ChVector<> COG_conrod(1.0,0.,.7);
		ChVector<> COG_lshaped(3.0, .0, 1.1);

		ChVector<> POS_ch2conrod(1.6, .0, .95);							//Rev chassis->conrod
		ChVector<> PIS_rod2link(2.0125,0,2.1);							// rod piston(LINK ARM) insertion abs frame
		ChVector<> POS_conrod2rod(2.5,0.,.3);							//Rev conrod->rod
		ChVector<> POS_rod2link(2.9, 0, 1.5);							//Act rod->link end point
		

		ChVector<> POS_lift2rod(2.825,.0, 1.05);						//rev joint(LIFT ARM) abs frame
		ChVector<> POS_rod2lever(3.6625,0., 1.575);						//rev joint(BUCKET LEVER) abs frame
		ChVector<> POS_lift2bucket(3.50, .0,.21);						//chassis piston(LIFT ARM) insertion abs frame
		ChVector<> POS_ch2lift(1.6,0, 2.1);								//Rev chassis->lift
		ChVector<> POS_lift2lever(2.5, 0, 2.1);							//end position of actuator lift->lever
		ChVector<> PIS_ch2lift(1.6, 0, 1.05);							//Act chassis->lift
		ChVector<> PIS_lift2lever(2.0125, 0, 2.1);						//Act lift->lever

		ChVector<> POS_link2bucket(3.69, .0, 0.71);						//chassis piston(BUCKET LEVER) insertion abs frame

		ChVector<> INS_ch2lift(1.8,0,1.1);								// Insertion of boom piston over lift body


		ChVector<> COG_link = POS_rod2link / 2 + POS_link2bucket / 2;


		ChVector<> POS_link2lshaped = COG_link - ChVector<>(.000,.0,.00);
		ChVector<> POS_lift2lshaped = POS_lift2bucket / 2 + POS_lift2rod / 2 - ChVector<>(.01, 0., -.01);


		ChQuaternion<> z2y;
		ChQuaternion<> z2x;
		z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
		z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));


	// 2 Create the rigid bodies : THE MECHANISM BODIES WILL BE SETUP IN A SEPARATE FUNCTION

		// LIFT
//		auto lift = std::shared_ptr<ChBody>(system.NewBody());
		auto lift = std::make_shared<ChBody>(contact_method);
		system.Add(lift);
		lift->SetBodyFixed(false);
		lift->SetName("lift arm");
		lift->SetPos(COG_lift);
		ChVector<> u1 = (COG_lift - POS_ch2lift).GetNormalized();				//GetNormalized() yields a versor
		ChVector<> w1 = Vcross(u1, VECT_Y).GetNormalized();						//overkill
		ChMatrix33<> rot1;														//no control on orthogonality
		rot1.Set_A_axis(u1, VECT_Y, w1);
		lift->SetRot(rot1);
		lift->SetMass(993.5);
		lift->SetInertiaXX(ChVector<>(110.2, 1986.1, 1919.3));
		lift->SetInertiaXY(ChVector<>(0.,0.,339.6));

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

				lift->AddAsset(lift_asset);
				lift->AddAsset(lift_asset1);
				lift->AddAsset(lift_asset2);
				lift->AddAsset(lift_asset3);
				auto col_l = std::make_shared<ChColorAsset>();
				col_l->SetColor(ChColor(0.2f, 0.0f, 0.0f));
				lift->AddAsset(col_l);	
		
		// CONROD
				//		auto conrod = std::shared_ptr<ChBody>(system.NewBody());
				auto conrod = std::make_shared<ChBody>(contact_method);
				system.Add(conrod);
				conrod->SetBodyFixed(false);
				conrod->SetName("conrod");
				conrod->SetPos(COG_conrod);
				ChVector<> uc = (COG_conrod - POS_ch2conrod).GetNormalized();				//GetNormalized() yields a versor
				ChVector<> wc = Vcross(uc, VECT_Y).GetNormalized();						//overkill
				ChMatrix33<> rotc;														//no control on orthogonality
				rotc.Set_A_axis(uc, VECT_Y, wc);
				conrod->SetRot(rotc);
				conrod->SetMass(99.5);
				conrod->SetInertiaXX(ChVector<>(110.2, 1986.1, 1919.3));
				conrod->SetInertiaXY(ChVector<>(0., 0., 339.6));
					// visualization:
					auto conrod_asset = std::make_shared<ChCylinderShape>();
					conrod_asset->GetCylinderGeometry().rad = .025;
					conrod_asset->GetCylinderGeometry().p1 = conrod->GetFrame_COG_to_abs().GetInverse() * POS_ch2conrod;
					conrod_asset->GetCylinderGeometry().p2 = conrod->GetFrame_COG_to_abs().GetInverse() * POS_conrod2rod;

					conrod->AddAsset(conrod_asset);
					auto col_c = std::make_shared<ChColorAsset>();
					col_c->SetColor(ChColor(0.2f, 0.5f, 0.0f));
					conrod->AddAsset(col_c);

// ROD
			//		auto rod = std::shared_ptr<ChBody>(system.NewBody());
			auto rod = std::make_shared<ChBody>(contact_method);
			system.Add(rod);
			rod->SetName("rod arm");
			rod->SetIdentifier(3);
			rod->SetPos(COG_rod);
			ChVector<> u3 = (POS_rod2link - POS_lift2rod).GetNormalized();
			ChVector<> w3 = Vcross(u3, VECT_Y).GetNormalized();						//overkill
			ChMatrix33<> rot3;
			rot3.Set_A_axis(u3, VECT_Y, w3);
			rod->SetRot(rot3);
			rod->SetMass(381.5);
			rod->SetInertiaXX(ChVector<>(11.7, 33.4, 29.5));
			rod->SetInertiaXY(ChVector<>(0., 0., -12.1));
					// No collision properties:
					// visualization properties:
					auto rod_asset = std::make_shared<ChCylinderShape>();
					rod_asset->GetCylinderGeometry().rad = .025;
					rod_asset->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * PIS_rod2link;
					rod_asset->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
					auto rod_asset1 = std::make_shared<ChCylinderShape>();
					rod_asset1->GetCylinderGeometry().rad = .025;
					rod_asset1->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
					rod_asset1->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_conrod2rod;
					rod->AddAsset(rod_asset);
					rod->AddAsset(rod_asset1);
					auto col_r = std::make_shared<ChColorAsset>();
					col_r->SetColor(ChColor(0.0f, 0.0f, 0.2f));
					rod->AddAsset(col_r);
// LINK
				//		auto link = std::shared_ptr<ChBody>(system.NewBody());
				auto link = std::make_shared<ChBody>(contact_method);
				system.Add(link);
				link->SetName("link arm");
				link->SetIdentifier(3);
				link->SetPos(COG_link);
				ChVector<> u4 = (POS_link2bucket - POS_rod2link).GetNormalized();
				ChVector<> w4 = Vcross(u4, VECT_Y).GetNormalized();							//overkill
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
// LSHAPED
				//		auto lshaped = std::shared_ptr<ChBody>(system.NewBody());
				auto lshaped = std::make_shared<ChBody>(contact_method);
				system.Add(lshaped);
				link->SetName("lshaped body");
				lshaped->SetIdentifier(5);
				lshaped->SetPos(COG_lshaped);
				ChVector<> uls = (POS_lift2lshaped - POS_link2lshaped).GetNormalized();
				ChVector<> wls = Vcross(uls, VECT_Y).GetNormalized();							//overkill
				ChMatrix33<> rotls;
				rotls.Set_A_axis(uls, VECT_Y, wls);
				lshaped->SetRot(rotls);
				lshaped->SetMass(7.2);
				lshaped->SetInertiaXX(ChVector<>(3.2, 11.1, 13.6));
				lshaped->SetInertiaXY(ChVector<>(0.0, 0.0, -.04));
					// No collision properties:
					// visualization properties:
					auto lshaped_asset = std::make_shared<ChCylinderShape>();
					lshaped_asset->GetCylinderGeometry().rad = .025;
					lshaped_asset->GetCylinderGeometry().p1 = lshaped->GetFrame_COG_to_abs().GetInverse() * POS_link2lshaped;
					lshaped_asset->GetCylinderGeometry().p2 = lshaped->GetFrame_COG_to_abs().GetInverse() * POS_lift2lshaped;
					lshaped->AddAsset(lshaped_asset);
					auto lshaped_sphere = std::make_shared<ChSphereShape>();
					lshaped_sphere->GetSphereGeometry().rad = .050;
					lshaped_sphere->GetSphereGeometry().center = lshaped->GetFrame_COG_to_abs().GetInverse() * POS_link2lshaped;
//					lshaped->AddAsset(lshaped_sphere);
					auto col_ls = std::make_shared<ChColorAsset>();
					col_ls->SetColor(ChColor(0.8f, 0.0f, 0.2f));
					lshaped->AddAsset(col_ls);








				// CHASSIS
				//		auto chassis = std::shared_ptr<ChBody>(system.NewBody());
				auto chassis = std::make_shared<ChBody>(contact_method);
				system.AddBody(chassis);
				chassis->SetBodyFixed(true);
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
				auto chassis_asset = std::make_shared<ChSphereShape>();				//asset
				chassis_asset->GetSphereGeometry().rad = .05;						//asset
				chassis->AddAsset(chassis_asset);






				// BUCKET
				//		auto bucket = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());
				auto bucket = std::make_shared<ChBodyAuxRef>(contact_method);
				system.AddBody(bucket);
				bucket->SetName("benna");
				bucket->SetIdentifier(4);
				bucket->SetMass(200.0);														//not confirmed data
				bucket->SetInertiaXX(ChVector<>(20, 50, 20));								//not confirmed data
				bucket->SetPos(POS_lift2bucket);
				bucket->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(.35, .0, .2), QUNIT));		//tentative:NOT A KNOWN FEATURES 
				// Create contact geometry.
				bucket->SetCollide(true);
				bucket->GetCollisionModel()->ClearModel();
				AddBucketHull(p_ext, p_int, bucket);
				AddCapsHulls(p_int, BucketSide::LEFT, bucket);
				AddCapsHulls(p_int, BucketSide::RIGHT, bucket);
				bucket->GetCollisionModel()->BuildModel();
#ifdef USE_PENALTY
				bucket->SetMaterialSurface(materialDEM);
#else
				bucket->SetMaterialSurface(material_test);
#endif
				geometry::ChTriangleMeshConnected bucket_mesh;
				bucket_mesh.LoadWavefrontMesh(out_dir + "data/bucket_mod.obj", false, false);
				auto bucket_mesh_shape = std::make_shared<ChTriangleMeshShape>();
				bucket_mesh_shape->SetMesh(bucket_mesh);
				bucket->AddAsset(bucket_mesh_shape);




				// 3. Add joint constraints:REVOLUTE JOINTS+LINEAR ACTUATOR AT THE MOMENT
				// Please note the rotations due to the fact that:
				//													- rev joint rotates around its frame z-axis
				//													- lin actuator moves along its frame z-axis


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


				ChVector<> u11 = (POS_lift2lever - PIS_lift2lever).GetNormalized();			//GetNormalized()
				ChVector<> w11 = Vcross(u11, VECT_Y).GetNormalized();						//overkill
				ChMatrix33<> rot11;															//no control on orthogonality
				rot11.Set_A_axis(u11, VECT_Y, w11);
				// LIFT-LSHAPED revjoint 
				auto rev_lift2lshaped = std::make_shared<ChLinkLockRevolute>();
				rev_lift2lshaped->SetName("revolute_lift2lshaped");
				rev_lift2lshaped->Initialize(lshaped, lift, ChCoordsys<>(POS_lift2lshaped, z2y >> rotls.Get_A_quaternion()));
				system.AddLink(rev_lift2lshaped);
				// LINK-LSHAPED revjoint 
				auto rev_link2lshaped = std::make_shared<ChLinkLockRevolute>();
				rev_link2lshaped->SetName("revolute_linkd2lshaped");
				rev_link2lshaped->Initialize(lshaped, link, ChCoordsys<>(POS_link2lshaped, z2y >> rotls.Get_A_quaternion()));
				system.AddLink(rev_link2lshaped);

				// CONROD-ROD atthebottom revjoint 
				auto rev_conrod2rod = std::make_shared<ChLinkLockRevolute>();
				rev_conrod2rod->SetName("revolute_conrod2rod");
				rev_conrod2rod->Initialize(rod, conrod, ChCoordsys<>(POS_conrod2rod, z2y >> rotc.Get_A_quaternion()));
				system.AddLink(rev_conrod2rod);

				// LIFT-ROD inthe middle revjoint 
				auto rev_lift2rod = std::make_shared<ChLinkLockRevolute>();
				rev_lift2rod->SetName("revolute_lift2rod");
				rev_lift2rod->Initialize(rod, lift, ChCoordsys<>(POS_lift2rod, z2y >> rot3.Get_A_quaternion()));
				system.AddLink(rev_lift2rod);

				// CHASSIS-CONROD revjoint
				auto rev_ch2conrod = std::make_shared<ChLinkLockRevolute>();
				rev_ch2conrod->SetName("revolute_chassis2conrod");
				rev_ch2conrod->Initialize(conrod, chassis, false, ChCoordsys<>(POS_ch2conrod, z2y >> rotc.Get_A_quaternion()), ChCoordsys<>(POS_ch2conrod, z2y >> rotc.Get_A_quaternion()));
				system.AddLink(rev_ch2conrod);

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
				//	//	ASSET FOR LINEAR ACTUATOR
				lin_ch2lift->AddAsset(std::make_shared<ChPointPointSegment>());
				// test_law
				auto lift_law_test = std::make_shared<ChFunction_Sine>();
				lift_law_test->Set_amp(.1);
				lift_law_test->Set_freq(.5);

				// end test_law
				lin_ch2lift->Set_lin_offset(.3);
				lin_ch2lift->Set_dist_funct(lift_law_test);
				system.AddLink(lin_ch2lift);


				// ROD-LINK linear
				auto lin_rod2link = std::make_shared<ChLinkLinActuator>();
				ChVector<> url = (POS_rod2link - PIS_rod2link).GetNormalized();//Normalize,not GetNormalize
				ChVector<> wrl = Vcross(url, VECT_Y).GetNormalized();//overkill
				ChMatrix33<> rotrl;//no control on orthogonality
				rotrl.Set_A_axis(url, VECT_Y, wrl);
				lin_rod2link->SetName("linear_rod2link");
				lin_rod2link->Initialize(link, rod, false, ChCoordsys<>(POS_rod2link, z2x >> rotrl.Get_A_quaternion()), ChCoordsys<>(PIS_rod2link, z2x >> rotrl.Get_A_quaternion()));//m2 is the master
				lin_rod2link->Set_lin_offset(Vlength(POS_rod2link - PIS_rod2link));
				//	//	ASSET FOR LINEAR ACTUATOR
				lin_rod2link->AddAsset(std::make_shared<ChPointPointSegment>());
				// test_law
				auto tilt_law_test = std::make_shared<ChFunction_Sine>();
				tilt_law_test->Set_amp(.1);
				tilt_law_test->Set_freq(.5);

				// end test_law
				//lin_rod2link->Set_dist_funct(tilt_law_test);
				system.AddLink(lin_rod2link);

// 4. Write the system hierarchy to the console (default log output destination)
system.ShowHierarchy(GetLog());

	
// 5. Prepare visualization with Irrlicht
//    Note that Irrlicht uses left-handed frames with Y up.

	
	
	
	// Create the Irrlicht application and set-up the camera.
	ChIrrApp * application = new ChIrrApp(
		&system,                               // pointer to the mechanical system
		L"WL Second Example",                // title of the Irrlicht window
		core::dimension2d<u32>(800, 600),      // window dimension (width x height)
		false,                                 // use full screen?
		true);                                 // enable shadows?
	application->AddTypicalLogo();
	application->AddTypicalSky();
	application->AddTypicalLights();
	application->AddTypicalCamera(core::vector3df(+2,+4,-3),core::vector3df(2, 0, 0)); //'camera' location            // "look at" location
											   // Let the Irrlicht application convert the visualization assets.
	application->AssetBindAll();
	application->AssetUpdateAll();
    
	application->SetTimestep(0.01);
	application->SetTryRealtime(true);

	while (application->GetDevice()->run()) {

		// Irrlicht must prepare frame to draw
		application->BeginScene();
		// Irrlicht application draws all 3D objects and all GUI items
		application->DrawAll();
		// Draw an XZ grid at the global origin to add in visualization.
		ChIrrTools::drawGrid(
			application->GetVideoDriver(), 1, 1, 20, 20,
			ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
			video::SColor(255, 80, 100, 100), true);

		// Advance the simulation time step
		application->DoStep();


		// Irrlicht must finish drawing the frame
		application->EndScene();
	}




    return 0;
}
