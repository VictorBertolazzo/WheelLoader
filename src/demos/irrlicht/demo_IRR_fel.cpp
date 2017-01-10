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
//     - modeling a complex mechanism:  a forklift
//     - loading .obj 3D meshes for 3d viewing
//	   - based on demo_forklift.cpp	
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

// First of all, define a class for the 'forklift' (that is, a set of
// bodies and links which are grouped within this class; so it is
// easier to manage data structures in this example).

class MySimpleForklift {
  public:
    // THE DATA

    double throttle;  // actual value 0...1 of gas throttle.
    double steer;     // actual value of steering
    double lift;      // actual value of fork lifting

    // The parts making the forklift, as 3d Irrlicht scene nodes, each containing
    // the ChBody object
    // .. truss:
    std::shared_ptr<ChBodyAuxRef> chassis;
    // .. right front wheel:
    std::shared_ptr<ChBody> wheelRF;
	std::shared_ptr<ChBody> spindleRF;
	std::shared_ptr<ChLinkEngine> link_revoluteRF;
	std::shared_ptr<ChLinkLockLock> link_spindleRF;
    // .. left front wheel:
    std::shared_ptr<ChBody> wheelLF;
	std::shared_ptr<ChBody> spindleLF;
    std::shared_ptr<ChLinkEngine> link_revoluteLF;
	std::shared_ptr<ChLinkLockLock> link_spindleLF;
	// .. right rear wheel:
	std::shared_ptr<ChBody> wheelRR;
	std::shared_ptr<ChBody> spindleRR;
	std::shared_ptr<ChLinkLockRevolute> link_revoluteRR;
	std::shared_ptr<ChLinkLockLock> link_spindleRR;
	// .. left rear wheel:
	std::shared_ptr<ChBody> wheelLR;
	std::shared_ptr<ChBody> spindleLR;
	std::shared_ptr<ChLinkLockRevolute> link_revoluteLR;
	std::shared_ptr<ChLinkLockLock> link_spindleLR;
	// .. back wheel:
    std::shared_ptr<ChBody> spindleB;
    std::shared_ptr<ChBody> wheelB;
    std::shared_ptr<ChLinkEngine> link_steer_engineB;
    std::shared_ptr<ChLinkEngine> link_engineB;
    // ..the arms
    std::shared_ptr<ChBodyAuxRef> arm;
	std::shared_ptr<ChBodyAuxRef> lever;
	
	std::shared_ptr<ChLinkLockRevolute> link_revoluteArm;
	std::shared_ptr<ChLinkLockRevolute> link_revoluteLever;

    // ..the fork
    std::shared_ptr<ChBody> fork;
	std::shared_ptr<ChLinkLinActuator> link_actuatorFork;
	std::shared_ptr<ChLinkLockPrismatic> link_prismaticFork;

	// chassis 2 liftarm links
	std::shared_ptr<ChLinkLinActuator> link_actuatorCH2LA;
    std::shared_ptr<ChLinkLockPrismatic> link_prismaticCH2LA;
	// chassis 2 bucketlever links
	std::shared_ptr<ChLinkLinActuator> link_actuatorCH2BL;
	std::shared_ptr<ChLinkLockPrismatic> link_prismaticCH2BL;

    video::ITexture* forkliftTiremap;

    // THE FUNCTIONS

    // Build and initialize the forklift, creating all bodies corresponding to
    // the various parts and adding them to the physical system - also creating
    // and adding constraints to the system.
	MySimpleForklift(ChIrrAppInterface* app, ChVector<> offset = ChVector<>(0, 0, 0)) {
		throttle = 0;  // initially, gas throttle is 0.
		steer = 0;
		lift = 0;

		ChVector<> COG_chassis(0, 0.4, 0.5);
		ChVector<> COG_wheelRF(-0.566, 0.282, 1.608);
		ChVector<> COG_wheelLF(0.566, 0.282, 1.608);
		ChVector<> COG_wheelRR(-0.566, 0.282, -1.608);
		ChVector<> COG_wheelLR(+0.566, 0.282, -1.608);
		ChVector<> COG_arm(0, 0.45, 1.9);
		ChVector<> COG_lever(0, 0.70, 2.1);

		ChVector<> COG_fork(0, 0.362, 2.100);
		ChVector<> COG_wheelB(0, 0.282, 0.003);
		ChVector<> POS_pivotarm(0, 0.150, 1.855);
		ChVector<> POS_prismatic(0, 0.150, 1.855);
		double RAD_back_wheel = 0.28;
		double RAD_front_wheel = 0.28;
		ChVector<> CH2LA(0, 0.35, 1.7);//rev joint(LIFT ARM) abs frame
		ChVector<> CH2BL(0, 0.60, 1.7);//rev joint(BUCKET LEVER) abs frame
		ChVector<> CHPIla(0, 0.30, 1.7);//chassis piston(LIFT ARM) insertion abs frame
		ChVector<> CHPIbl(0, 0.81, 1.7);//chassis piston(BUCKET LEVER) insertion abs frame
		ChVector<> LAPI(0,0,.6);//lift arm piston insertion rel frame
		ChVector<> BLPI(0, 0, .6);//bucket lever piston insertion rel frame



		forkliftTiremap = app->GetVideoDriver()->getTexture(GetChronoDataFile("tire_truck.png").c_str());


		// EXAMPLE 4
		GetLog() << " Example: create FEL like system: \n";
		{
			// --- The chassis body ---
			{
				chassis = std::make_shared<ChBodyAuxRef>();
				chassis->SetBodyFixed(false);
				chassis->SetName("chassis");
				chassis->SetMass(400.0);
				chassis->SetPos(COG_chassis);
				chassis->SetInertiaXX(ChVector<>(100, 100, 100));
				// collision properties:
				chassis->GetCollisionModel()->ClearModel();
				chassis->GetCollisionModel()->AddBox(1.227 / 2., 1.621 / 2., 1.864 / 2., ChVector<>(-0.003, 1.019, 0.192));
				chassis->GetCollisionModel()->AddBox(0.187 / 2., 0.773 / 2., 1.201 / 2., ChVector<>(0.486, 0.153, -0.047));
				chassis->GetCollisionModel()->AddBox(0.187 / 2., 0.773 / 2., 1.201 / 2., ChVector<>(-0.486, 0.153, -0.047));
				chassis->GetCollisionModel()->BuildModel();
				chassis->SetCollide(true);
				// visualization properties
				auto chassis_asset_assembly = std::make_shared<ChAssetLevel>();//asset
				chassis_asset_assembly->GetFrame().SetPos(-COG_chassis);//asset
				auto chassis_asset = std::make_shared<ChObjShapeFile>();//mesh
				chassis_asset->SetFilename(GetChronoDataFile("forklift_body.obj"));//mesh
				chassis_asset_assembly->AddAsset(chassis_asset);//mesh

				chassis->AddAsset(chassis_asset_assembly);

				app->GetSystem()->AddBody(chassis);
			}
		}

		{
			// Spindles
			{
				// ..the RF steering spindle (invisible, no mesh)
				spindleRF = std::make_shared<ChBody>();
				app->GetSystem()->Add(spindleRF);
				spindleRF->SetPos(COG_wheelRF);
				spindleRF->SetMass(10);
				spindleRF->SetInertiaXX(ChVector<>(1, 1, 1));

				// RF spindle chassis connection
				link_spindleRF = std::make_shared<ChLinkLockLock>();
				link_spindleRF->Initialize(spindleRF, chassis, ChCoordsys<>(COG_wheelRF, QUNIT));
				app->GetSystem()->AddLink(link_spindleRF);


				// ..the LF steering spindle (invisible, no mesh)
				spindleLF = std::make_shared<ChBody>();
				app->GetSystem()->Add(spindleLF);
				spindleLF->SetPos(COG_wheelLF);
				spindleLF->SetMass(10);
				spindleLF->SetInertiaXX(ChVector<>(1, 1, 1));

				// LF spindle chassis connection
				link_spindleLF = std::make_shared<ChLinkLockLock>();
				link_spindleLF->Initialize(spindleLF, chassis, ChCoordsys<>(COG_wheelLF, QUNIT));
				app->GetSystem()->AddLink(link_spindleLF);
				///////////////////////////////////////////
				///////////////////////////////////////////
				///////////////////////////////////////////
				
				
				// ..the RR steering spindle (invisible, no mesh)
				spindleRR = std::make_shared<ChBody>();
				app->GetSystem()->Add(spindleRR);
				spindleRR->SetPos(COG_wheelRR);
				spindleRR->SetMass(10);
				spindleRR->SetInertiaXX(ChVector<>(1, 1, 1));

				// RR spindle chassis connection
				link_spindleRR = std::make_shared<ChLinkLockLock>();
				link_spindleRR->Initialize(spindleRR, chassis, ChCoordsys<>(COG_wheelRR, QUNIT));
				app->GetSystem()->AddLink(link_spindleRR);

				
				// ..the LR steering spindle (invisible, no mesh)
				spindleLR = std::make_shared<ChBody>();
				app->GetSystem()->Add(spindleLR);
				spindleLR->SetPos(COG_wheelLR);
				spindleLR->SetMass(10);
				spindleLR->SetInertiaXX(ChVector<>(1, 1, 1));

				// LF spindle chassis connection
				link_spindleLR = std::make_shared<ChLinkLockLock>();
				link_spindleLR->Initialize(spindleLF, chassis, ChCoordsys<>(COG_wheelLR, QUNIT));
				app->GetSystem()->AddLink(link_spindleLR);
			}
		}
		
		{
			// Front wheels
			{
				// ..the right-front wheel

				wheelRF = std::make_shared<ChBody>();
				app->GetSystem()->Add(wheelRF);
				wheelRF->SetPos(COG_wheelRF);
				wheelRF->SetMass(20);
				wheelRF->SetInertiaXX(ChVector<>(2, 2, 2));
				// collision properties:
				ChMatrix33<> Arot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
				wheelRF->GetCollisionModel()->ClearModel();
				wheelRF->GetCollisionModel()->AddCylinder(RAD_front_wheel, RAD_front_wheel, 0.1, ChVector<>(0, 0, 0), Arot);
				wheelRF->GetCollisionModel()->BuildModel();
				wheelRF->SetCollide(true);
				// visualization properties:
				auto wheelRF_asset_assembly = std::make_shared<ChAssetLevel>();
				wheelRF_asset_assembly->GetFrame().SetPos(-COG_wheelRF);
				wheelRF->AddAsset(wheelRF_asset_assembly);
				auto wheelRF_mesh = std::make_shared<ChObjShapeFile>();
				wheelRF_mesh->SetFilename(GetChronoDataFile("wheel.obj"));
				wheelRF_asset_assembly->AddAsset(wheelRF_mesh);
				auto wheelRF_texture = std::make_shared<ChTexture>(GetChronoDataFile("tire_truck.png"));
				wheelRF_asset_assembly->AddAsset(wheelRF_texture);


				// .. create the revolute joint between the wheel and the spindle

				link_revoluteRF = std::make_shared<ChLinkEngine>();  // right, front, upper, 1
				link_revoluteRF->Initialize(wheelRF, spindleRF, ChCoordsys<>(COG_wheelRF, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
				link_revoluteRF->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
				link_revoluteRF->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
				app->GetSystem()->AddLink(link_revoluteRF);

				// ..the left-front wheel

				wheelLF = std::make_shared<ChBody>();
				app->GetSystem()->Add(wheelLF);
				wheelLF->SetPos(COG_wheelLF);
				wheelLF->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));  // reuse RF wheel shape, flipped
				wheelLF->SetMass(20);
				wheelLF->SetInertiaXX(ChVector<>(2, 2, 2));
				// collision properties:
				wheelLF->GetCollisionModel()->ClearModel();
				wheelLF->GetCollisionModel()->AddCylinder(RAD_front_wheel, RAD_front_wheel, 0.1, ChVector<>(0, 0, 0), Arot);
				wheelLF->GetCollisionModel()->BuildModel();
				wheelLF->SetCollide(true);
				// visualization properties:
				auto wheelLF_asset_assembly = std::make_shared<ChAssetLevel>();
				wheelLF_asset_assembly->GetFrame().SetPos(-COG_wheelRF);
				wheelLF->AddAsset(wheelLF_asset_assembly);
				auto wheelLF_mesh = std::make_shared<ChObjShapeFile>();
				wheelLF_mesh->SetFilename(GetChronoDataFile("wheel.obj"));
				wheelLF_asset_assembly->AddAsset(wheelLF_mesh);
				auto wheelLF_texture = std::make_shared<ChTexture>(GetChronoDataFile("tire_truck.png"));
				wheelLF_asset_assembly->AddAsset(wheelLF_texture);


				// .. create the revolute joint between the wheel and the spindle
				link_revoluteLF = std::make_shared<ChLinkEngine>();  // right, front, upper, 1
				link_revoluteLF->Initialize(wheelLF, spindleLF, ChCoordsys<>(COG_wheelLF, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
				link_revoluteLF->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
				link_revoluteLF->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);

				app->GetSystem()->AddLink(link_revoluteLF);
			}
		}
		
		{
			// Rear wheels
			{
				// ..the right-rear wheel

				wheelRR = std::make_shared<ChBody>();
				app->GetSystem()->Add(wheelRR);
				wheelRR->SetPos(COG_wheelRR);
				wheelRR->SetMass(20);
				wheelRR->SetInertiaXX(ChVector<>(2, 2, 2));
				// collision properties:
				ChMatrix33<> Arot(chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Z));
				wheelRR->GetCollisionModel()->ClearModel();
				wheelRR->GetCollisionModel()->AddCylinder(RAD_front_wheel, RAD_front_wheel, 0.1, ChVector<>(0, 0, 0), Arot);
				wheelRR->GetCollisionModel()->BuildModel();
				wheelRR->SetCollide(true);
				// visualization properties:
				auto wheelRR_asset_assembly = std::make_shared<ChAssetLevel>();
				wheelRR_asset_assembly->GetFrame().SetPos(-COG_wheelRR);
							wheelRR->AddAsset(wheelRR_asset_assembly);
				auto wheelRR_mesh = std::make_shared<ChObjShapeFile>();
				wheelRR_mesh->SetFilename(GetChronoDataFile("wheel.obj"));
				wheelRR_asset_assembly->AddAsset(wheelRR_mesh);
				auto wheelRR_texture = std::make_shared<ChTexture>(GetChronoDataFile("tire_truck.png"));
				wheelRR_asset_assembly->AddAsset(wheelRR_texture);

				GetLog() << "RR desired tyre pos : " << COG_wheelRR << "\n";

				// .. create the revolute joint between the wheel and the truss

				link_revoluteRR = std::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
				link_revoluteRR->Initialize(wheelRR, spindleRR, ChCoordsys<>(COG_wheelRR, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
				app->GetSystem()->AddLink(link_revoluteRR);

				// ..the left-rear wheel

				wheelLR = std::make_shared<ChBody>();
				app->GetSystem()->Add(wheelLR);
				wheelLR->SetPos(COG_wheelLR);
				wheelLR->SetRot(chrono::Q_from_AngAxis(CH_C_PI, VECT_Y));  // reuse RR wheel shape, flipped
				wheelLR->SetMass(20);
				wheelLR->SetInertiaXX(ChVector<>(2, 2, 2));
				// collision properties:
				wheelLR->GetCollisionModel()->ClearModel();
				wheelLR->GetCollisionModel()->AddCylinder(RAD_front_wheel, RAD_front_wheel, 0.1, ChVector<>(0, 0, 0), Arot);
				wheelLR->GetCollisionModel()->BuildModel();
				wheelLR->SetCollide(true);
				// visualization properties:
				auto wheelLR_asset_assembly = std::make_shared<ChAssetLevel>();
				wheelLR_asset_assembly->GetFrame().SetPos(-COG_wheelLR);
							wheelLR->AddAsset(wheelLR_asset_assembly);
				auto wheelLR_mesh = std::make_shared<ChObjShapeFile>();
				wheelLR_mesh->SetFilename(GetChronoDataFile("wheel.obj"));
				wheelLR_asset_assembly->AddAsset(wheelLR_mesh);
				auto wheelLR_texture = std::make_shared<ChTexture>(GetChronoDataFile("tire_truck.png"));
				wheelLR_asset_assembly->AddAsset(wheelLR_texture);


				// .. create the revolute joint between the wheel and the truss
				link_revoluteLR = std::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
				link_revoluteLR->Initialize(wheelLR, spindleLR, ChCoordsys<>(COG_wheelLR, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));
				app->GetSystem()->AddLink(link_revoluteLR);
			}
		}


			// Lift Arm + LA-CH link(revolute,prismatic,linactuator)
		{
			ChVector<> u = (COG_arm - CH2LA);//;
			ChVector<> v = Vcross(u, VECT_X).GetNormalized();//overkill
			u = Vcross(VECT_X, v);
			ChMatrix33<> rot;
			rot.Set_A_axis(VECT_X,  v, u);
			GetLog() << u << "\n";
			// ..the lift arm
			arm = std::make_shared<ChBodyAuxRef>();
			app->GetSystem()->Add(arm);
			arm->SetPos(COG_arm);
			arm->SetMass(100);
			arm->SetInertiaXX(ChVector<>(30, 30, 30));
			ChFrame<> liftarm_ref(COG_arm, rot);
			arm->SetFrame_COG_to_REF(ChFrame<>(VNULL, QUNIT));
			arm->SetFrame_REF_to_abs(liftarm_ref);
			LAPI = arm->GetFrame_REF_to_abs() *(LAPI);//I want to describe LAPI in abs coords-->!!!!
			GetLog() << LAPI << "\n";
			// visualization properties:
			auto arm_asset_assembly = std::make_shared<ChAssetLevel>();
			arm_asset_assembly->GetFrame().SetPos(-COG_arm);
			arm->AddAsset(arm_asset_assembly);
			auto arm_mesh = std::make_shared<ChObjShapeFile>();
			arm_mesh->SetFilename(GetChronoDataFile("wheel loader - boom-1.obj"));//
			arm_asset_assembly->AddAsset(arm_mesh);
		
			// .. create the revolute joint between the arm and chassis
			link_revoluteArm = std::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
			link_revoluteArm->Initialize(arm, chassis, ChCoordsys<>(CH2LA, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));//x is rev axis!
			app->GetSystem()->AddLink(link_revoluteArm);
			{
				// .. create the prismatic joint between the arm and chassis
				link_prismaticCH2LA = std::make_shared<ChLinkLockPrismatic>();
				ChVector<> u = (LAPI - CHPIla).GetNormalized();
				ChVector<> v = Vcross(u, VECT_X);//overkill
				ChMatrix33<> rot;
				rot.Set_A_axis(u, v, VECT_X);
				link_prismaticCH2LA->Initialize(arm, chassis, ChCoordsys<>(CHPIla, rot.Get_A_quaternion()));  // set prism as inclined as LAPI-CHPI (default would be aligned to z, horizontal)
				app->GetSystem()->AddLink(link_prismaticCH2LA);
				// .. create the linear actuator that pushes upward the liftarm
				link_actuatorCH2LA = std::make_shared<ChLinkLinActuator>();
				link_actuatorCH2LA->Initialize(arm, chassis, false,  ChCoordsys<>(LAPI, rot.Get_A_quaternion()),ChCoordsys<>(CHPIla, rot.Get_A_quaternion()));//m2 is the master
				app->GetSystem()->AddLink(link_actuatorCH2LA);
			}
		}
			// Bucket Lever + BL-CH link(revolute,prismatic,linactuator)
		{
			ChVector<> u = (COG_lever - CH2BL);//;
			ChVector<> v = Vcross(u, VECT_X).GetNormalized();//overkill
			u = Vcross(VECT_X, v);
			ChMatrix33<> rot;
			rot.Set_A_axis(VECT_X, v, u);			
			// ..the bucket lever
			lever = std::make_shared<ChBodyAuxRef>();
			app->GetSystem()->Add(lever);
			lever->SetPos(COG_lever);
			lever->SetMass(100);
			lever->SetInertiaXX(ChVector<>(30, 30, 30));
			ChFrame<> bucketlever_ref(COG_lever, rot);
			lever->SetFrame_COG_to_REF(ChFrame<>(VNULL, QUNIT));
			lever->SetFrame_REF_to_abs(bucketlever_ref);
			BLPI = lever->GetFrame_REF_to_abs() *(BLPI);//I want to describe BLPI in abs coords-->!!!!
//			GetLog() << BLPI << "\n";
			// visualization properties:
			auto arm_asset_assembly = std::make_shared<ChAssetLevel>();
			arm_asset_assembly->GetFrame().SetPos(-COG_lever);
			arm_asset_assembly->GetFrame().SetRot(rot);
			lever->AddAsset(arm_asset_assembly);
			auto right_mesh = std::make_shared<ChObjShapeFile>();
			right_mesh->SetFilename(GetChronoDataFile("wheel loader - connecting rod-1.obj"));//"connection-rod"!!!
			arm_asset_assembly->AddAsset(right_mesh);
			auto left_mesh = std::make_shared<ChObjShapeFile>();
			left_mesh->SetFilename(GetChronoDataFile("wheel loader - connecting rod-2.obj"));//"connection-rod"!!!
			arm_asset_assembly->AddAsset(left_mesh);


			// .. create the revolute joint between the arm and chassis
			link_revoluteLever = std::make_shared<ChLinkLockRevolute>();  // right, front, upper, 1
			link_revoluteLever->Initialize(lever, chassis, ChCoordsys<>(CH2BL, chrono::Q_from_AngAxis(CH_C_PI / 2, VECT_Y)));//x is rev axis!
			app->GetSystem()->AddLink(link_revoluteLever);
			{
				// .. create the prismatic joint between the bucketlever and chassis
				link_prismaticCH2BL = std::make_shared<ChLinkLockPrismatic>();
				ChVector<> u = (BLPI - CHPIbl).GetNormalized();
				ChVector<> v = Vcross(u, VECT_X);//overkill
				ChMatrix33<> rot;
				rot.Set_A_axis(u, v, VECT_X);
				link_prismaticCH2BL->Initialize(lever, chassis, ChCoordsys<>(CHPIbl, rot.Get_A_quaternion()));  // set prism as inclined as BLPI-CHPIbl (default would be aligned to z, horizontal)
				app->GetSystem()->AddLink(link_prismaticCH2BL);
				// .. create the linear actuator that pushes upward the bucketlever
				link_actuatorCH2BL = std::make_shared<ChLinkLinActuator>();
				link_actuatorCH2BL->Initialize(lever, chassis, false, ChCoordsys<>(BLPI, rot.Get_A_quaternion()),ChCoordsys<>(CHPIbl, rot.Get_A_quaternion()));//m2 is the master
				app->GetSystem()->AddLink(link_actuatorCH2BL);
			}
		}

		
		
		
		
		
		
		
		
		////////////////////////////////////////////////////////////////////
		// ..the fork

        fork = std::make_shared<ChBody>();
        app->GetSystem()->Add(fork);
        fork->SetPos(COG_fork);
        fork->SetMass(60);
        fork->SetInertiaXX(ChVector<>(15, 15, 15));
         // collision properties:
        fork->GetCollisionModel()->ClearModel();
        fork->GetCollisionModel()->AddBox(0.1 / 2., 0.032 / 2., 1.033 / 2.,ChVector<>(-0.352, -0.312, 0.613));
        fork->GetCollisionModel()->AddBox(0.1 / 2., 0.032 / 2., 1.033 / 2., ChVector<>(0.352, -0.312, 0.613));
        fork->GetCollisionModel()->AddBox(0.344 / 2., 1.134 / 2., 0.101 / 2., ChVector<>(-0.000, 0.321, -0.009));
        fork->GetCollisionModel()->BuildModel();
        fork->SetCollide(true);
         // visualization properties:
        auto fork_asset_assembly = std::make_shared<ChAssetLevel>();
        fork_asset_assembly->GetFrame().SetPos(-COG_fork);
        fork->AddAsset(fork_asset_assembly);
        auto fork_mesh = std::make_shared<ChObjShapeFile>();
        fork_mesh->SetFilename(GetChronoDataFile("forklift_forks.obj"));
        fork_asset_assembly->AddAsset(fork_mesh);


        // .. create the prismatic joint between the fork and arm
        link_prismaticFork = std::make_shared<ChLinkLockPrismatic>();
        link_prismaticFork->Initialize(
            fork, arm,
            ChCoordsys<>(
                POS_prismatic,
                chrono::Q_from_AngAxis(CH_C_PI / 2,
                                       VECT_X)));  // set prism as vertical (default would be aligned to z, horizontal
        app->GetSystem()->AddLink(link_prismaticFork);

        // .. create the linear actuator that pushes upward the fork
        link_actuatorFork = std::make_shared<ChLinkLinActuator>();
        link_actuatorFork->Initialize(fork, arm, false,
                                      ChCoordsys<>(POS_prismatic + ChVector<>(0, 0.01, 0), QUNIT),
                                      ChCoordsys<>(POS_prismatic, QUNIT));
        app->GetSystem()->AddLink(link_actuatorFork);

		/*
        // ..a pallet
		{
			auto pallet = std::make_shared<ChBodyEasyMesh>(
				GetChronoDataFile("pallet.obj"),  // mesh .OBJ file
				300,                              // density
				true,   // compute mass, inertia & COG from the mesh (must be a closed watertight mesh!)
				true,   // enable collision with mesh
				0.001,  // sphere swept inflate of mesh - improves robustness of collision detection
				true);  // enable visualization of mesh
			app->GetSystem()->Add(pallet);
			pallet->SetPos(ChVector<>(0, 0.4, 3));

			// apply also a texture to the pallet:
			auto pallet_texture = std::make_shared<ChTexture>();
			pallet_texture->SetTextureFilename(GetChronoDataFile("cubetexture.png"));
			pallet->AddAsset(pallet_texture);
		}
		*/
        //
        // Move the forklift to initial offset position
        //

        //				pallet->GetBody()->Move(offset);
        chassis->Move(offset);
        wheelRF->Move(offset);
        wheelLF->Move(offset);
		wheelRR->Move(offset);
		wheelLR->Move(offset);
		spindleLF->Move(offset);
		spindleRF->Move(offset);
        arm->Move(offset);
		lever->Move(offset);
        fork->Move(offset);
    }

    // Delete the car object, deleting also all bodies corresponding to
    // the various parts and removing them from the physical system.  Also
    // removes constraints from the system.
    ~MySimpleForklift() {
        ChSystem* mysystem = chassis->GetSystem();  // trick to get the system here

        mysystem->Remove(link_revoluteRF);
        mysystem->Remove(link_revoluteLF);
		mysystem->Remove(link_revoluteRR);
		mysystem->Remove(link_revoluteLR);
        mysystem->Remove(link_steer_engineB);
        mysystem->Remove(link_engineB);
//        mysystem->Remove(link_engineArm);
        mysystem->Remove(link_prismaticFork);
        mysystem->Remove(link_actuatorFork);
        mysystem->Remove(chassis);
        mysystem->Remove(wheelRF);
        mysystem->Remove(wheelLF);
		mysystem->Remove(wheelRR);
		mysystem->Remove(wheelLR);
		mysystem->Remove(spindleRF);
		mysystem->Remove(spindleLF);

        mysystem->Remove(wheelB);
        mysystem->Remove(spindleB);
        mysystem->Remove(arm);
        mysystem->Remove(fork);
    }
};

// Define a MyEventReceiver class which will be used to manage input
// from the GUI graphical user interface (the interface will
// be created with the basic -yet flexible- platform
// independent toolset of Irrlicht).

class MyEventReceiver : public IEventReceiver {
  public:
    MyEventReceiver(ChIrrAppInterface* myapp, MySimpleForklift* mlift) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        app = myapp;
        forklift = mlift;
    }

    bool OnEvent(const SEvent& event) {
        // check if user presses keys
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
                case irr::KEY_KEY_Q:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_steer_engineB->Get_rot_funct()))
                        mfun->Set_yconst(-0.6 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_W:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_steer_engineB->Get_rot_funct()))
                        mfun->Set_yconst(+0.3 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_A:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_revoluteLF->Get_spe_funct()))
                        mfun->Set_yconst(0.5 + mfun->Get_yconst());
					if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_revoluteRF->Get_spe_funct()))
						mfun->Set_yconst(0.5 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_Z:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_revoluteLF->Get_spe_funct()))
                        mfun->Set_yconst(-0.5 + mfun->Get_yconst());
					if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_revoluteRF->Get_spe_funct()))
						mfun->Set_yconst(-0.5 + mfun->Get_yconst());

                    return true;
                case irr::KEY_KEY_S:
//                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_actuatorFork->Get_dist_funct()))
//                        mfun->Set_yconst(0.05 + mfun->Get_yconst());
					if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_actuatorCH2LA->Get_dist_funct()))
						mfun->Set_yconst(0.05 + mfun->Get_yconst());
					return true;
                case irr::KEY_KEY_X:
                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_actuatorFork->Get_dist_funct()))
                        mfun->Set_yconst(-0.05 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_D:
//                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_engineArm->Get_rot_funct()))
//                        mfun->Set_yconst(0.005 + mfun->Get_yconst());
                    return true;
                case irr::KEY_KEY_C:
//                    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(forklift->link_engineArm->Get_rot_funct()))
//                        mfun->Set_yconst(-0.005 + mfun->Get_yconst());
                    return true;
            }
        }

        return false;
    }

  private:
    ChIrrAppInterface* app;
    MySimpleForklift* forklift;
};

//
// This is the program which is executed
//

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Drive a forklift", core::dimension2d<u32>(800, 600), false);

    // add text with info
    IGUIStaticText* textFPS = application.GetIGUIEnvironment()->addStaticText(
        L"Keys: steer=Q,W; throttle=A,Z; lift=S,X; bank=D,C", rect<s32>(150, 10, 430, 40), true);

    // Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(-6, 3, -6));

    // ..the world
    auto my_ground = std::make_shared<ChBodyEasyBox>(40, 2, 40, 1000, true, true);
    my_system.Add(my_ground);
    my_ground->SetBodyFixed(true);
    my_ground->SetPos(ChVector<>(0, -1, 0));
    my_ground->GetMaterialSurface()->SetSfriction(1.0);
    my_ground->GetMaterialSurface()->SetKfriction(1.0);
    auto mtexture = std::make_shared<ChTexture>(GetChronoDataFile("concrete.jpg"));
    my_ground->AddAsset(mtexture);



    // ..the fel (this class - see above - is a 'set' of bodies and links, automatically added at creation)
    MySimpleForklift* myforklift = new MySimpleForklift(&application);


    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

	
    //
    // USER INTERFACE
    //

    // Create some graphical-user-interface (GUI) items to show on the screen.
    // This requires an event receiver object -see above.
    // This is for GUI tweaking of system parameters..
		MyEventReceiver receiver(&application, myforklift);
    // note how to add a custom event receiver to the default interface:
	    application.SetUserEventReceiver(&receiver);

    //
    // SETTINGS
    //

    my_system.SetMaxItersSolverSpeed(20);  // the higher, the easier to keep the constraints 'mounted'.

    my_system.SetSolverType(ChSystem::SOLVER_SOR);

    //
    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
    //

    // This will help choosing an integration step which matches the
    // real-time step of the simulation..
    application.SetStepManage(true);
    application.SetTimestep(0.005);
	int counter = 0;
    while (application.GetDevice()->run()) {
        // Irrlicht must prepare frame to draw
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        // Irrlicht application draws all 3D objects and all GUI items
        application.DrawAll();

        // Advance the simulation time step
        application.DoStep();
		//RunTime informations
		{
			if (counter == 100) {
				GetLog() << "RF actual tyre pos : " << myforklift->wheelRF->GetPos() << "\n";
				GetLog() << "RR actual tyre pos : " << myforklift->wheelRR->GetPos() << "\n";
				counter = 0;		
			}

		}
		counter++;
        // Irrlicht must finish drawing the frame
        application.GetVideoDriver()->endScene();
    }

    if (myforklift)
        delete myforklift;

    return 0;
}
