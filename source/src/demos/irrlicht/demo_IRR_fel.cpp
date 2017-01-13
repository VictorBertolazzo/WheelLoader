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
//     - based on demo_forklift.cpp	
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



    ChSystem system;
	system.Set_G_acc(ChVector<>(.0,.0,-9.81));
// the ChBody object
    // ..the chassis
	std::shared_ptr<ChBody> chassis;
    // ..the arms
    std::shared_ptr<ChBody> lift;
	std::shared_ptr<ChBody> lever;
	std::shared_ptr<ChBody> rod;
	std::shared_ptr<ChBody> bucket;

	std::shared_ptr<ChLinkLockRevolute> rev_lift2rod;
	std::shared_ptr<ChLinkLockRevolute> rev_rod2lever;
	std::shared_ptr<ChLinkLockRevolute> rev_lift2bucket;
	std::shared_ptr<ChLinkLockRevolute> rev_lever2bucket;

	// chassis 2 liftarm links
	std::shared_ptr<ChLinkLinActuator> lin_ch2lift;
    std::shared_ptr<ChLinkLockPrismatic> prism_ch2lift;
	// chassis 2 bucketlever links
	std::shared_ptr<ChLinkLinActuator> lin_ch2lever;
	std::shared_ptr<ChLinkLockPrismatic> prism_ch2lever;

		ChVector<> COG_chassis(0, 0, 1.0);
		ChVector<> COG_lift(1.6, 0., .8);
		ChVector<> COG_lever(1.0, 0.0, 1.4);
		ChVector<> COG_rod(1.3, 0.0, 1.1);
		ChVector<> COG_bucket( 2.1,.0, 1.2);

		ChVector<> POS_lift2rod(1.6,.0, 1.1);//rev joint(LIFT ARM) abs frame
		ChVector<> POS_rod2lever(1.,0.,1.4);//rev joint(BUCKET LEVER) abs frame
		ChVector<> POS_lift2bucket(1.8, .0,1.5);//chassis piston(LIFT ARM) insertion abs frame
		ChVector<> POS_lever2bucket(1.9,.0, .95);//chassis piston(BUCKET LEVER) insertion abs frame
		ChVector<> POS_ch2lift(.3,0,.9);//lift arm piston insertion rev frame
		ChVector<> POS_ch2lever(.4, 0, .8);//bucket lever piston insertion rev frame
		ChVector<> PIS_ch2lift(.3, 0, .75);//lift arm piston insertion act frame
		ChVector<> PIS_ch2lever(.4, 0, .65);//bucket lever piston insertion act frame

											// Define two quaternions representing:
											// - a rotation of -90 degrees around x (z2y)
											// - a rotation of +90 degrees around y (z2x)
		ChQuaternion<> z2y;
		ChQuaternion<> z2x;
		z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
		z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));

		auto fixed = std::make_shared<ChBody>();
		system.Add(fixed);
		fixed->SetBodyFixed(true);

// --- The chassis body ---
		{
				chassis = std::make_shared<ChBody>();
				system.AddBody(chassis);
				chassis->SetBodyFixed(false);
				chassis->SetName("chassis");
				chassis->SetMass(20.0);
				chassis->SetPos(COG_chassis);
				chassis->SetInertiaXX(ChVector<>(1., 1., 1.));
				// collision properties:
				chassis->GetCollisionModel()->ClearModel();
				chassis->GetCollisionModel()->AddSphere(.5, VNULL);
				chassis->GetCollisionModel()->BuildModel();
				chassis->SetCollide(true);
				// visualization properties
				auto chassis_asset = std::make_shared<ChSphereShape>();//asset
				chassis_asset->GetSphereGeometry().rad = .5;//asset
				chassis->AddAsset(chassis_asset);

				// chassis-fixed prismatic+linactuator
				auto prism_fix2ch = std::make_shared<ChLinkLockPrismatic>();
				prism_fix2ch->Initialize(chassis, fixed, ChCoordsys<>(COG_chassis, -z2x));
				system.Add(prism_fix2ch);
				auto lin_fix2ch = std::make_shared<ChLinkLinActuator>();
				system.Add(lin_fix2ch);
				lin_fix2ch->Initialize(chassis, fixed, false, ChCoordsys<>(COG_chassis,-z2x), ChCoordsys<>(COG_chassis, -z2x));//m2 is the master
				lin_fix2ch->Set_lin_offset(Vlength(VNULL));
				auto chassis_law = std::make_shared<ChFunction_Ramp>();
				chassis_law->Set_ang(1.);
				lin_fix2ch->Set_dist_funct(chassis_law);



		}
// Lift Arm + LA-CH link(revolute,prismatic,linactuator)
				{
					ChVector<> u = (COG_lift - COG_chassis).GetNormalized();
					ChVector<> w = Vcross(u, VECT_Y).GetNormalized();//overkill
					ChMatrix33<> rot;
					rot.Set_A_axis(u, VECT_Y,w);

					// ..the lift arm
					lift = std::make_shared<ChBody>();
					system.Add(lift);
					lift->SetPos(COG_lift);
					lift->SetRot(rot);
					lift->SetMass(.5);
					lift->SetInertiaXX(ChVector<>(.30, .30, .30));
					// No collision properties:
					// visualization properties:
					auto lift_asset = std::make_shared<ChCylinderShape>();
					lift_asset->GetCylinderGeometry().rad = .1;
					lift_asset->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_ch2lift;
					lift_asset->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2bucket;
					lift->AddAsset(lift_asset);
					auto col_l = std::make_shared<ChColorAsset>();
					col_l->SetColor(ChColor(0.2f, 0.0f, 0.0f));
					lift->AddAsset(col_l);

					// create revjoint btw chassis and lift
					auto rev_ch2lift = std::make_shared<ChLinkLockRevolute>();
					rev_ch2lift->Initialize(lift, chassis, ChCoordsys<>(POS_ch2lift, rot.Get_A_quaternion()));
					system.AddLink(rev_ch2lift);

					// .. create the linear actuator that pushes upward the liftarm
					lin_ch2lift = std::make_shared<ChLinkLinActuator>();
					lin_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(COG_lift, rot.Get_A_quaternion()), ChCoordsys<>(PIS_ch2lift, rot.Get_A_quaternion()));//m2 is the master
					lin_ch2lift->Set_lin_offset(Vlength( COG_lift- PIS_ch2lift));
					auto legge1 = std::make_shared<ChFunction_Ramp>();
					legge1->Set_ang(0.2);
					//auto legge2 = std::make_shared<ChFunction_Sequence>();
					//legge2->InsertFunct(legge1, 0.3, 1, true);
					lin_ch2lift->Set_dist_funct(legge1);
					system.AddLink(lin_ch2lift);
				}
// Bucket Lever + BL-CH link(revolute,prismatic,linactuator)
				{
					ChVector<> u = (COG_lever - COG_chassis).GetNormalized();
					ChVector<> w = Vcross(u, VECT_Y).GetNormalized();//overkill
					ChMatrix33<> rot;
					rot.Set_A_axis(u, VECT_Y, w);
					// ..the lift arm
					lever = std::make_shared<ChBody>();
					system.Add(lever);
					lever->SetPos(COG_lever);
					lever->SetRot(rot);
					lever->SetMass(.5);
					lever->SetInertiaXX(ChVector<>(.30, .30, .30));
					// No collision properties:

					// visualization properties:
					auto lever_asset = std::make_shared<ChCylinderShape>();
					lever_asset->GetCylinderGeometry().rad = .1;
					lever_asset->GetCylinderGeometry().p1 = lever->GetFrame_COG_to_abs().GetInverse() * POS_ch2lever;
					lever_asset->GetCylinderGeometry().p2 = lever->GetFrame_COG_to_abs().GetInverse() * POS_lever2bucket;
					lever->AddAsset(lever_asset);
					auto col_l = std::make_shared<ChColorAsset>();
					col_l->SetColor(ChColor(0.0f, 0.2f, 0.0f));
					lever->AddAsset(col_l);

					// create revjoint btw chassis and lever
					auto rev_ch2lever = std::make_shared<ChLinkLockRevolute>();
					rev_ch2lever->Initialize(lever, chassis, ChCoordsys<>(POS_ch2lever,rot.Get_A_quaternion()));
					system.AddLink(rev_ch2lever);
					
					// .. create the linear actuator that pushes upward the bucketlever
					lin_ch2lever = std::make_shared<ChLinkLinActuator>();
					lin_ch2lever->Initialize(lever, chassis, false, ChCoordsys<>(COG_lever, rot.Get_A_quaternion()), ChCoordsys<>(PIS_ch2lift, rot.Get_A_quaternion()));//m2 is the master
					lin_ch2lever->Set_lin_offset(Vlength(COG_lever-PIS_ch2lever));
					auto legge1 = std::make_shared<ChFunction_Sine>();
					legge1->Set_amp(0.1);
					legge1->Set_freq(0.8);
					//auto legge2 = std::make_shared<ChFunction_Sequence>();
					//legge2->InsertFunct(legge1, 0.3, 1, true);
					lin_ch2lever->Set_dist_funct(legge1);
					system.AddLink(lin_ch2lever);

				}
		
// Rod body + Connections
				{
					ChVector<> u = (POS_rod2lever - POS_lift2rod).GetNormalized();
					ChVector<> w = Vcross(u, VECT_Y).GetNormalized();//overkill
					ChMatrix33<> rot;
					rot.Set_A_axis(u, VECT_Y, w);
					// ..the lift arm
					rod = std::make_shared<ChBody>();
					system.Add(rod);
					rod->SetPos(COG_rod);
					rod->SetRot(rot);
					rod->SetMass(.35);
					rod->SetInertiaXX(ChVector<>(.1, .1, .1));
					// No collision properties:
					// visualization properties:
					auto rod_asset = std::make_shared<ChCylinderShape>();
					rod_asset->GetCylinderGeometry().rad = .05;
					rod_asset->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_rod2lever;
					rod_asset->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
					rod->AddAsset(rod_asset);
					auto col_r = std::make_shared<ChColorAsset>();
					col_r->SetColor(ChColor(0.0f, 0.0f, 0.2f));
					rod->AddAsset(col_r);

					// ..create lift arm--rod rev joint
					rev_lift2rod = std::make_shared<ChLinkLockRevolute>();
					rev_lift2rod->Initialize(rod,lift,ChCoordsys<>(POS_lift2rod,rot.Get_A_quaternion()));
					system.AddLink(rev_lift2rod);
					
					// ..create rod--bucket lever rev joint
					rev_lift2rod = std::make_shared<ChLinkLockRevolute>();
					rev_lift2rod->Initialize(lever, rod, ChCoordsys<>(POS_rod2lever, rot.Get_A_quaternion()));//Does it make sense?
					system.AddLink(rev_lift2rod);
				}
// Bucket Body + Connections
				{
					bucket = std::make_shared<ChBody>();
					system.AddBody(bucket);
					bucket->SetBodyFixed(false);
					bucket->SetMass(20.0);
					bucket->SetPos(COG_bucket);
					bucket->SetInertiaXX(ChVector<>(1., 1., 1.));
					// collision properties(to do)###############################
					bucket->GetCollisionModel()->ClearModel();
					bucket->GetCollisionModel()->AddSphere(.5, VNULL);
					bucket->GetCollisionModel()->BuildModel();
					bucket->SetCollide(true);
					// visualization properties(to do)#############################
					auto bucket_asset = std::make_shared<ChSphereShape>();//asset
					bucket_asset->GetSphereGeometry().rad = .15;//asset
					bucket->AddAsset(bucket_asset);
					auto col_b = std::make_shared<ChColorAsset>();
					col_b->SetColor(ChColor(0.1f, 0.1f, 0.1f));
					bucket->AddAsset(col_b);

					
					
					// ..create lift arm--bucket rev joint
					ChVector<> a1 = (POS_lift2bucket - COG_lift).GetNormalized();
					ChVector<> c1 = Vcross(a1, VECT_Y).GetNormalized();//overkill
					ChMatrix33<> rot1;
					rot1.Set_A_axis(a1, VECT_Y, c1);
					rev_lift2bucket= std::make_shared<ChLinkLockRevolute>();
					rev_lift2bucket->Initialize(bucket, lift, ChCoordsys<>(POS_lift2bucket, rot1.Get_A_quaternion()));
					system.AddLink(rev_lift2bucket);
					
					
					
					// ..create lift arm--bucket rev joint
					ChVector<> a2 = (POS_lever2bucket- COG_lever).GetNormalized();
					ChVector<> c2 = Vcross(a2, VECT_Y).GetNormalized();//overkill
					ChMatrix33<> rot2;
					rot2.Set_A_axis(a2, VECT_Y, c2);
					rev_lever2bucket = std::make_shared<ChLinkLockRevolute>();
					rev_lever2bucket->Initialize(bucket,lever, ChCoordsys<>(POS_lever2bucket, rot2.Get_A_quaternion()));//Does it make sense?
					system.AddLink(rev_lever2bucket);

				
				}
		
		
	// ..the world
    auto my_ground = std::make_shared<ChBodyEasyBox>(80, 80, 2, 1000, true, true);
    system.Add(my_ground);
    my_ground->SetBodyFixed(true);
    my_ground->SetPos(ChVector<>(0, -0, -1.5));
    my_ground->GetMaterialSurface()->SetSfriction(1.0);
    my_ground->GetMaterialSurface()->SetKfriction(1.0);
    auto mtexture = std::make_shared<ChTexture>(GetChronoDataFile("concrete.jpg"));
    my_ground->AddAsset(mtexture);

	
	
	
	
	
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

		// Advance the simulation time step
		application->DoStep();


		// Irrlicht must finish drawing the frame
		application->EndScene();
	}


    return 0;
}
