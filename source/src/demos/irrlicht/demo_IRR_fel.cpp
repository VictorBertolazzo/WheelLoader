//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about
//
//     - based on demo_forklift.cpp(no more)	
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
//             http://www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono/assets/ChTexture.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono/utils/ChUtilsGenerators.h"

#include <stdio.h>
#include <vector>
#include <cmath>

#include <irrlicht.h>

// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
int main(int argc, char* argv[]) {

	// 0. Set the path to the Chrono data folder
	SetChronoDataPath(CHRONO_DATA_DIR);
	// 1. Create the system
    ChSystem system;
	system.Set_G_acc(ChVector<>(.0,.0,-9.81));
    /// .16 initial from chassis offset, .21 initial max height, .33 initial width 
	// measures are in [m]
		ChVector<> COG_chassis(0, 0, 1.575); // somewhere
		ChVector<> COG_lift(2.8375, 0., 1.05);
		ChVector<> COG_lever(3.6625, 0.0, 1.575);
		ChVector<> COG_rod(3.25, 0.0, 1.3125);
		ChVector<> COG_bucket(3.86875,.0, 0.525);

		ChVector<> POS_lift2rod(2.425,.0, 1.05);//rev joint(LIFT ARM) abs frame
		ChVector<> POS_rod2lever(3.6625,0., 1.575);//rev joint(BUCKET LEVER) abs frame
		ChVector<> POS_lift2bucket(4.075, .0,.21);//chassis piston(LIFT ARM) insertion abs frame
		ChVector<> POS_lever2bucket(3.86875,.0, 1.575);//chassis piston(BUCKET LEVER) insertion abs frame
		ChVector<> POS_ch2lift(1.6,0, 2.1);//Rev chassis->lift
		ChVector<> POS_lift2lever(3.25, 0, 2.1);//end position of actuator lift->lever
		ChVector<> PIS_ch2lift(1.6, 0, 1.05);//Act chassis->lift
		ChVector<> PIS_lift2lever(2.0125, 0, 2.1);//Act lift->lever

		ChQuaternion<> z2y;
		ChQuaternion<> z2x;
		z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
		z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
	// 2 Create the rigid bodies


		// GROUND
		auto ground = std::make_shared<ChBody>();
		system.Add(ground);
		ground->SetBodyFixed(true);
		ground->SetIdentifier(-1);
		ground->SetName("ground");
		//collision properties:
				ground->GetCollisionModel()->ClearModel();
				ground->GetCollisionModel()->AddBox(40., 40., 2.0, ChVector<>(.0,.0,-2.0), QUNIT);
				ground->GetCollisionModel()->BuildModel();
				ground->SetCollide(true);
		//visualization properties:
				auto ground_asset = std::make_shared<ChBoxShape>();
				ground_asset->GetBoxGeometry().Pos = ChVector<>(.0, .0, -2.0);
				ground_asset->GetBoxGeometry().Size = ChVector<>(40.,40.,2.0);
				ground->AddAsset(ground_asset);
		// CHASSIS
		auto chassis = std::make_shared<ChBody>();
		system.AddBody(chassis);
		chassis->SetName("chassis");
		chassis->SetIdentifier(0);
		chassis->SetMass(20.0);
		chassis->SetPos(COG_chassis);
		chassis->SetInertiaXX(ChVector<>(1., 1., 1.));
		// collision properties:
				chassis->GetCollisionModel()->ClearModel();
				chassis->GetCollisionModel()->AddSphere(.05, VNULL);
				chassis->GetCollisionModel()->BuildModel();
				chassis->SetCollide(true);
		// visualization properties
				auto chassis_asset = std::make_shared<ChSphereShape>();//asset
				chassis_asset->GetSphereGeometry().rad = .05;//asset
				chassis->AddAsset(chassis_asset);
		// LIFT
		auto lift = std::make_shared<ChBody>();
		system.Add(lift);
		lift->SetName("lift arm");
		lift->SetIdentifier(1);
		lift->SetPos(COG_lift);
		ChVector<> u1 = (COG_lift - POS_ch2lift).GetNormalized();//Normalize,not GetNormalize
		ChVector<> w1 = Vcross(u1, VECT_Y).GetNormalized();//overkill
		ChMatrix33<> rot1;//no control on orthogonality
		rot1.Set_A_axis(u1, VECT_Y, w1);
		lift->SetRot(rot1);
		lift->SetMass(7.0);
		lift->SetInertiaXX(ChVector<>(2., 2., 2.));
		// No collision properties:
		// visualization properties:
				auto lift_asset = std::make_shared<ChCylinderShape>();
				lift_asset->GetCylinderGeometry().rad = .03;
				lift_asset->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_ch2lift;
				lift_asset->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2bucket;
				auto lift_sphere = std::make_shared<ChSphereShape>();
				lift_sphere->GetSphereGeometry() = geometry::ChSphere(lift->GetFrame_COG_to_abs().GetInverse() * PIS_ch2lift, .05);

				lift->AddAsset(lift_asset);
				lift->AddAsset(lift_sphere);
				auto col_l = std::make_shared<ChColorAsset>();
				col_l->SetColor(ChColor(0.2f, 0.0f, 0.0f));
				lift->AddAsset(col_l);
		// LEVER
		auto lever = std::make_shared<ChBody>();
		system.Add(lever);
		lever->SetName("bucket lever");
		lever->SetIdentifier(2);
		lever->SetPos(COG_lever);
		ChVector<> u2 = (COG_lever - POS_lift2lever).GetNormalized();
		ChVector<> w2 = Vcross(u2, VECT_Y).GetNormalized();//overkill
		ChMatrix33<> rot2;
		rot2.Set_A_axis(u2, VECT_Y, w2);
		lever->SetRot(rot2);
		lever->SetMass(6.0);
		lever->SetInertiaXX(ChVector<>(1.5, 1.5, 1.5));
		// No collision properties:
		// visualization properties:
				auto lever_asset = std::make_shared<ChCylinderShape>();
				lever_asset->GetCylinderGeometry().rad = .03;
				lever_asset->GetCylinderGeometry().p1 = lever->GetFrame_COG_to_abs().GetInverse() * POS_lift2lever;
				lever_asset->GetCylinderGeometry().p2 = lever->GetFrame_COG_to_abs().GetInverse() * POS_lever2bucket;
				auto lever_sphere = std::make_shared<ChSphereShape>();
				lever_sphere->GetSphereGeometry() = geometry::ChSphere(lever->GetFrame_COG_to_abs().GetInverse() * PIS_lift2lever, .05);//fix this vis issue!

				lever->AddAsset(lever_asset);
				lever->AddAsset(lever_sphere);
				auto col_le = std::make_shared<ChColorAsset>();
				col_le->SetColor(ChColor(0.0f, 0.2f, 0.0f));
				lever->AddAsset(col_le);
		// ROD
		auto rod = std::make_shared<ChBody>();
		system.Add(rod);
		rod->SetName("rod arm");
		rod->SetIdentifier(3);
		rod->SetPos(COG_rod);
		ChVector<> u3 = (POS_rod2lever - POS_lift2rod).GetNormalized();
		ChVector<> w3 = Vcross(u3, VECT_Y).GetNormalized();//overkill
		ChMatrix33<> rot3;
		rot3.Set_A_axis(u3, VECT_Y, w3);
		rod->SetRot(rot3);
		rod->SetMass(3.5);
		rod->SetInertiaXX(ChVector<>(1., 1., 1.));
		// No collision properties:
		// visualization properties:
						auto rod_asset = std::make_shared<ChCylinderShape>();
						rod_asset->GetCylinderGeometry().rad = .025;
						rod_asset->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_rod2lever;
						rod_asset->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
						rod->AddAsset(rod_asset);
						auto col_r = std::make_shared<ChColorAsset>();
						col_r->SetColor(ChColor(0.0f, 0.0f, 0.2f));
						rod->AddAsset(col_r);
		// BUCKET
		auto bucket = std::make_shared<ChBody>();
		system.AddBody(bucket);
		bucket->SetName("benna");
		bucket->SetIdentifier(4);
		bucket->SetMass(5.0);
		bucket->SetPos(COG_bucket);
		bucket->SetInertiaXX(ChVector<>(.5, .5, .5));
		// collision properties(to do)###############################
						bucket->GetCollisionModel()->ClearModel();
						bucket->GetCollisionModel()->AddSphere(.015, VNULL);
						auto bucket_box = std::make_shared<ChBoxShape>();
						bucket->GetCollisionModel()->BuildModel();
						bucket->SetCollide(true);
		// visualization properties(to do)#############################
						auto bucket_asset = std::make_shared<ChSphereShape>();//asset
						bucket_asset->GetSphereGeometry().rad = .015;//asset
						bucket->AddAsset(bucket_asset);
						auto col_b = std::make_shared<ChColorAsset>();
						col_b->SetColor(ChColor(0.1f, 0.1f, 0.1f));
						bucket->AddAsset(col_b);

// 3. Add joint constraints
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
						chassis_law->Set_ang(.50);//it'll act as the chassis speed
						lin_fix2ch->Set_dist_funct(chassis_law);
			// CHASSIS-LIFT revjoint
						auto rev_ch2lift = std::make_shared<ChLinkLockRevolute>();
						rev_ch2lift->SetName("revolute_chassis2lift");
						rev_ch2lift->Initialize(lift, chassis, ChCoordsys<>(POS_ch2lift, z2y >> rot1.Get_A_quaternion() ));
						system.AddLink(rev_ch2lift);
			// CHASSIS-LIFT linear
						auto lin_ch2lift = std::make_shared<ChLinkLinActuator>();
						ChVector<> u11 = (COG_lift - PIS_ch2lift).GetNormalized();//Normalize,not GetNormalize
						ChVector<> w11 = Vcross(u11, VECT_Y).GetNormalized();//overkill
						ChMatrix33<> rot11;//no control on orthogonality
						rot11.Set_A_axis(u11, VECT_Y, w11);
						lin_ch2lift->SetName("linear_chassis2lift");
						lin_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(COG_lift, z2x >> rot11.Get_A_quaternion()) , ChCoordsys<>(PIS_ch2lift, z2x >> rot11.Get_A_quaternion() ));//m2 is the master
						lin_ch2lift->Set_lin_offset(Vlength(COG_lift - PIS_ch2lift));
						auto legge1 = std::make_shared<ChFunction_Ramp>();
						legge1->Set_ang(.00);
						auto legge2 = std::make_shared<ChFunction_Const>();
						legge2->Set_yconst(lin_ch2lift->Get_lin_offset());//Does it take the actual value or that one at the beginning?
						auto lift_law = std::make_shared<ChFunction_Sequence>();
						lift_law->InsertFunct(legge1, 0.3, 1, true);
						lift_law->InsertFunct(legge2,10.0,1.,true);
						lin_ch2lift->Set_dist_funct(lift_law);
						system.AddLink(lin_ch2lift);
			// LIFT-LEVER revjoint
						auto rev_lift2lever = std::make_shared<ChLinkLockRevolute>();
						rev_lift2lever->SetName("revolute_lift2lever");
						rev_lift2lever->Initialize(lever, lift, ChCoordsys<>(POS_lift2lever, z2y >>rot2.Get_A_quaternion() ));
						system.AddLink(rev_lift2lever);
			// LIFT-LEVER linear
						auto lin_lift2lever = std::make_shared<ChLinkLinActuator>();
						ChVector<> u22 = (POS_lift2lever - PIS_ch2lift).GetNormalized();//not Normalize, GetNormalize
						ChVector<> w22 = Vcross(u22, VECT_Y).GetNormalized();//overkill
						ChMatrix33<> rot22;//no control on orthogonality
						rot22.Set_A_axis(u22, VECT_Y, w22);
						lin_ch2lift->SetName("linear_lift2lever");
						lin_lift2lever->Initialize(lever, lift, false, ChCoordsys<>(POS_lift2lever, z2x >> rot22.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot22.Get_A_quaternion()));//m2 is the master
						lin_lift2lever->Set_lin_offset(Vlength(POS_lift2lever - PIS_lift2lever));
						auto legge1_r = std::make_shared<ChFunction_Ramp>();
						legge1_r->Set_ang(0.50);
						legge1_r->Set_y0(lin_lift2lever->Get_lin_offset());
						auto legge2_r = std::make_shared<ChFunction_Sine>();
						
						legge2_r->Set_amp(.050);
						legge2_r->Set_freq(0.250);
						auto lever_law = std::make_shared<ChFunction_Sequence>();
						lever_law->InsertFunct(legge1_r, 5.0, 1, true);
						lever_law->InsertFunct(legge2_r, 5.0, 1, true);
						lin_lift2lever->Set_dist_funct(lever_law);
						system.AddLink(lin_lift2lever);
			// LIFT-ROD revjoint
						auto rev_lift2rod = std::make_shared<ChLinkLockRevolute>();
						rev_lift2lever->SetName("revolute_lift2rod");
						rev_lift2rod->Initialize(rod, lift, ChCoordsys<>(POS_lift2rod, z2y >> rot3.Get_A_quaternion()));
						system.AddLink(rev_lift2rod);
			// ROD-LEVER revjoint
						auto rev_rod2lever = std::make_shared<ChLinkLockRevolute>();
						rev_rod2lever->SetName("revolute_rod2lever");
						rev_rod2lever->Initialize(lever, rod, ChCoordsys<>(POS_rod2lever, z2y >> rot3.Get_A_quaternion()));//Does it make sense?
						system.AddLink(rev_rod2lever);
			// LIFT-BUCKET revjoint
						auto rev_lift2bucket = std::make_shared<ChLinkLockRevolute>();
						rev_lift2bucket->SetName("revolute_lift2bucket");
//	not necessary now	ChVector<> a1 = (POS_lift2bucket - COG_lift).Normalize();
//	it'll depend on 	ChVector<> c1 = Vcross(a1, VECT_Y).Normalize();//overkill
//	angular sensors		ChMatrix33<> rot1;
//						rot1.Set_A_axis(a1, VECT_Y, c1);
						ChMatrix33<> rotb1;
						rev_lift2bucket->Initialize(bucket, lift, ChCoordsys<>(POS_lift2bucket, rotb1.Get_A_quaternion()));
						system.AddLink(rev_lift2bucket);
			// LEVER-BUCKET revjoint
						auto rev_lever2bucket = std::make_shared<ChLinkLockRevolute>();
						rev_lever2bucket->SetName("revolute_lever2bucket");
//						ChVector<> a2 = (POS_lever2bucket - COG_lever).Normalize();
//						ChVector<> c2 = Vcross(a2, VECT_Y).Normalize();//overkill
//						ChMatrix33<> rot2;
//						rot2.Set_A_axis(a2, VECT_Y, c2);
						ChMatrix33<> rotb2;
						rev_lever2bucket->Initialize(bucket, lever, ChCoordsys<>(POS_lever2bucket, rotb2.Get_A_quaternion()));//Does it make sense?
						system.AddLink(rev_lever2bucket);
			// BUCKET Coll and Vis Update
						for (int i = 0; i < 30; i++) {
							double angle = +CH_C_PI / 2 + CH_C_PI / 30 + i*0.1305516846;
							//ChVector<> pos_b(+.25*cos(angle), .0, +.25*sin(angle));// = () before gave me all sin(angle) vector as results;
							double R = 1.05 * exp(-.068*i);
							ChVector<> pos_b(R*cos(angle) + .3,.0,R*sin(angle));
							//GetLog() << angle << pos_b << cos(angle) << sin(angle) << "\n";//Debugging
							ChMatrix33<> rot_b;
							ChVector<> w_b = (pos_b - VNULL).GetNormalized();
							ChVector<> u_b = Vcross(VECT_Y, w_b).GetNormalized();//overkill
							rot_b.Set_A_axis(u_b, VECT_Y, w_b);
							bucket->GetCollisionModel()->AddBox(.05, .25, .05, pos_b, rot_b);

							auto bucket_box = std::make_shared<ChBoxShape>();
							bucket_box->GetBoxGeometry() = geometry::ChBox(pos_b, rot_b, ChVector<>(.05, .25, .05));
							bucket->AddAsset(bucket_box);
						}



				
				
				
				
				

// 4. Write the system hierarchy to the console (default log output destination)
system.ShowHierarchy(GetLog());


		
	
	// adding DEBRIS in front of the loader, 10.0 m away
int n_debris = 60;
for (int i = 0; i < n_debris;i++) {
	auto debris = std::make_shared<ChBodyEasyBox>(.2,.45,.3,10,true,true);
	debris->SetPos(ChVector<>(10.0 + ChRandom()*1.0 - ChRandom()*1.0, ChRandom()*.5 - ChRandom()*.5, .25*i));
	debris->GetMaterialSurface()->SetFriction(.2f);

	debris->GetMaterialSurface()->SetRestitution(.85f);

	debris->AddAsset(std::make_shared<ChTexture>(GetChronoDataFile("bluwhite.png")));

	system.Add(debris);
}
	
	
	
	// 5. Prepare visualization with Irrlicht
	//    Note that Irrlicht uses left-handed frames with Y up.

	// Create the Irrlicht application and set-up the camera.
	ChIrrApp * application = new ChIrrApp(
		&system,                               // pointer to the mechanical system
		L"WL First Example",                // title of the Irrlicht window
		core::dimension2d<u32>(800, 600),      // window dimension (width x height)
		false,                                 // use full screen?
		true);                                 // enable shadows?
	application->AddTypicalLogo();
	application->AddTypicalSky();
	application->AddTypicalLights();
	application->AddTypicalCamera(core::vector3df(2, 5, -3),core::vector3df(2, 0, 0)); //'camera' location            // "look at" location
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
