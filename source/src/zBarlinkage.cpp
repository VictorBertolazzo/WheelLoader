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

#include <iostream>//Add 

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

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

struct Entry {
	Entry() {}
	Entry(float xpos,float zpos, float ang, float length)
		: m_xpos(xpos), m_zpos(zpos), m_ang(ang), m_length(length) {}
	float m_xpos;
	float m_zpos;
	float m_ang;
	float m_length;
};
struct Points {
	Points() {}
	Points(float x, float y, float z)
		: mx(x), my(y), mz(z) {}
	float mx; float my; float mz;
};

int main(int argc, char* argv[]) {
	const std::string out_dir = "../";
	const std::string& filename = out_dir + "data/bucket_coll.txt";
	std::ifstream ifile(filename.c_str());
	std::string line;
	std::vector<Entry> m_data;
		while (std::getline(ifile, line)) {
			std::istringstream iss(line);
			float xpos,zpos,ang,length;
			iss >> xpos >> zpos >> ang >> length ;
			if (iss.fail())
				break;
			m_data.push_back(Entry(xpos, zpos, ang, length));
			}
		ifile.close();
	std::vector<Points> p_ext;
	std::vector<Points> p_int;

	const std::string& filenameext = out_dir + "data/test_ext.txt";
	std::ifstream ifileext(filenameext.c_str());
	while (std::getline(ifileext, line)) {
		std::istringstream iss(line);
		float xpos, ypos, zpos;
		iss >> xpos >> ypos >> zpos;
		if (iss.fail())
			break;
		p_ext.push_back(Points(xpos, ypos, zpos));
	}
	ifileext.close();

	const std::string& filenameint = out_dir + "data/test_int.txt";
	std::ifstream ifileint(filenameint.c_str());
	while (std::getline(ifileint, line)) {
		std::istringstream iss(line);
		float xpos, ypos, zpos;
		iss >> xpos >> ypos >> zpos;
		if (iss.fail())
			break;
		p_int.push_back(Points(xpos, ypos, zpos));
	}
	ifileint.close();

	GetLog() << ChVector<>(p_int[0].mx,p_int[0].my,p_int[0].mz) << "\t" << ChVector<>(p_ext[0].mx, p_ext[0].my, p_ext[0].mz) << "\n";


	// 0. Set the path to the Chrono data folder
	SetChronoDataPath(CHRONO_DATA_DIR);
	// 1. Create the system
    ChSystem system;
	system.Set_G_acc(ChVector<>(.0,.0,.0));
	// GROUND
	auto ground = std::make_shared<ChBody>();
	system.Add(ground);
	ground->SetBodyFixed(true);
	ground->SetIdentifier(-1);
	ground->SetName("ground");

    /// .16 initial from chassis offset, .21 initial max height, .33 initial width 
	// measures are in [m]
		ChVector<> COG_chassis(0, 0, 1.575); // somewhere
		ChVector<> COG_lift(2.0, 0., 1.05);
		ChVector<> COG_lever(3.6625, 0.0, 1.575);
		ChVector<> COG_rod(2.7, 0.0, 1.3125);
		ChVector<> COG_link(3.0, 0.0, 0.5);
		ChVector<> COG_bucket(4.0,.0, 0.525);

		ChVector<> POS_lift2rod(2.825,.0, 1.05);//rev joint(LIFT ARM) abs frame
		ChVector<> POS_rod2lever(3.6625,0., 1.575);//rev joint(BUCKET LEVER) abs frame
		ChVector<> POS_lift2bucket(3.50, .0,.21);//chassis piston(LIFT ARM) insertion abs frame
		ChVector<> POS_ch2lift(1.6,0, 2.1);//Rev chassis->lift
		ChVector<> POS_lift2lever(2.5, 0, 2.1);//end position of actuator lift->lever
		ChVector<> PIS_ch2lift(1.6, 0, 1.05);//Act chassis->lift
		ChVector<> PIS_lift2lever(2.0125, 0, 2.1);//Act lift->lever

		ChVector<> POS_rod2link(2.6, 0, 0.4);//Act lift->lever
		ChVector<> POS_link2bucket(3.69, .0, 0.71);//chassis piston(BUCKET LEVER) insertion abs frame

		ChVector<> INS_ch2lift(1.8,0,1.1);// Insertion of boom piston over lift body

		ChQuaternion<> z2y;
		ChQuaternion<> z2x;
		z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
		z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
	// 2 Create the rigid bodies

		// LIFT
		auto lift = std::make_shared<ChBody>();
		system.Add(lift);
		lift->SetBodyFixed(false);
		lift->SetName("lift arm");
		lift->SetPos(COG_lift);
		ChVector<> u1 = (COG_lift - POS_ch2lift).GetNormalized();//Normalize,not GetNormalize
		ChVector<> w1 = Vcross(u1, VECT_Y).GetNormalized();//overkill
		ChMatrix33<> rot1;//no control on orthogonality
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
		// ROD
		auto rod = std::make_shared<ChBody>();
		system.Add(rod);
		rod->SetName("rod arm");
		rod->SetIdentifier(3);
		rod->SetPos(COG_rod);
		ChVector<> u3 = (POS_rod2link - POS_lift2rod).GetNormalized();
		ChVector<> w3 = Vcross(u3, VECT_Y).GetNormalized();//overkill
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
		// LINK
		auto link = std::make_shared<ChBody>();
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
		auto bucket = std::make_shared<ChBodyAuxRef>();
		system.AddBody(bucket);
		bucket->SetName("benna");
		bucket->SetIdentifier(4);
		bucket->SetMass(1.0);
		bucket->SetPos(POS_lift2bucket);
		bucket->SetRot(QUNIT);
		//bucket->SetFrame_COG_to_REF(ChFrame<> (bucket->GetFrame_REF_to_abs().GetInverse() * COG_bucket,QUNIT));
		bucket->SetInertiaXX(ChVector<>(.5, .5, .5));
		// collision properties(to do)###############################
						bucket->GetCollisionModel()->ClearModel();
						bucket->GetCollisionModel()->AddSphere(.015, VNULL);
						auto bucket_box = std::make_shared<ChBoxShape>();
						bucket->GetCollisionModel()->BuildModel();
						bucket->SetCollide(true);
		// visualization properties(to do)#############################
						auto bucket_asset = std::make_shared<ChSphereShape>();//asset
						bucket_asset->GetSphereGeometry().rad = .15;//asset
						bucket->AddAsset(bucket_asset);
						auto col_b = std::make_shared<ChColorAsset>();
						col_b->SetColor(ChColor(0.1f, 0.1f, 0.1f));
						bucket->AddAsset(col_b);
		// CHASSIS
		auto chassis = std::make_shared<ChBody>();
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

		// BUCKET Coll and Vis Update
						int nmax = m_data.size() ;
						for (int i = 0; i <  m_data.size(); i++) {
						double angle = +CH_C_PI / 2 + CH_C_PI / 30 + i*0.1305516846;
						ChVector<> pos_b(m_data[i].m_xpos,0.0,m_data[i].m_zpos);
						//GetLog() << angle << pos_b << cos(angle) << sin(angle) << "\n";//Debugging
						ChMatrix33<> rot_b;

						ChVector<> extr(m_data[i].m_length*cos(m_data[i].m_ang), 0.0, m_data[i].m_length*sin(m_data[i].m_ang));
						ChVector<> u_b = (extr - VNULL).GetNormalized();
						ChVector<> w_b = Vcross(VECT_Y, u_b).GetNormalized();//overkill
						//rot_b.Set_A_axis(u_b, VECT_Y, w_b);
						rot_b.Set_A_AngAxis(m_data[i].m_ang, -VECT_Y);
						bucket->GetCollisionModel()->AddBox(m_data[i].m_length, .25, .05, pos_b, rot_b);

						auto bucket_box = std::make_shared<ChBoxShape>();
						bucket_box->GetBoxGeometry() = geometry::ChBox(pos_b, rot_b, ChVector<>(m_data[i].m_length, .25, .05));
						bucket->AddAsset(bucket_box);
						}
						

						

// 3. Add joint constraints
			// LIFT-ROD  up revjoint-->IT's NOT A REV JOINT!! I keep it as a reminder.
						auto rev_lift2lever = std::make_shared<ChLinkLockRevolute>();
						rev_lift2lever->SetName("revolute_lift2lever");
						rev_lift2lever->Initialize(rod, lift, ChCoordsys<>(POS_lift2lever, z2y >>rot3.Get_A_quaternion() ));
						//system.AddLink(rev_lift2lever);
			// LIFT-ROD spring(it simulates the actuator)
						auto springdamper_ground_ball = std::make_shared<ChLinkSpring>();
						springdamper_ground_ball->Initialize(rod,lift,false,POS_lift2lever,PIS_lift2lever,true,.00);
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
// No way
						auto bp_asset = std::make_shared<ChPointPointSegment>();//asset
						lin_lift2rod->AddAsset(bp_asset);
//
						auto legge1 = std::make_shared<ChFunction_Ramp>();					
						legge1->Set_ang(1.0);
						auto legge2 = std::make_shared<ChFunction_Const>();
						//legge2->Set_yconst(Vlength(POS_lift2lever - PIS_lift2lever) + 1.0);//Does it take the actual value or that one at the beginning?
						auto legge3 = std::make_shared<ChFunction_Ramp>();
						legge3->Set_ang(-1.0);//Does it take the actual value or that one at the beginning?
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
						lin_lift2rod->Set_dist_funct(tilt_law_seq);

						//system.AddLink(springdamper_ground_ball);
						system.Add(lin_lift2rod);
			// LIFT-ROD inthe middle revjoint 
						auto rev_lift2rod = std::make_shared<ChLinkLockRevolute>();
						rev_lift2lever->SetName("revolute_lift2rod");
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
						rev_lift2bucket->Initialize(bucket, lift, ChCoordsys<>( POS_lift2bucket, z2y >> rotb1.Get_A_quaternion()));
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

						lin_ch2lift->AddAsset(std::make_shared<ChPointPointSegment>());

						// test_law
						auto lift_law_test = std::make_shared<ChFunction_Const>();
						lift_law_test->Set_yconst(Vlength(VNULL));
						// end test_law
						lin_ch2lift->Set_dist_funct(lift_law_seq);

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
						chassis_law->Set_ang(.10);//it'll act as the chassis speed
						lin_fix2ch->Set_dist_funct(chassis_law);

				
				
				
				
				

// 4. Write the system hierarchy to the console (default log output destination)
	//system.ShowHierarchy(GetLog());
	//GetLog() << rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2lever << "\n";
	//GetLog() << rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod<< "\n";
	//GetLog() << rod->GetFrame_COG_to_abs().GetInverse() *  POS_rod2link << "\n";
	//GetLog() << link->GetFrame_COG_to_abs().GetInverse() * POS_rod2link<< "\n";
	//GetLog() << link->GetFrame_COG_to_abs().GetInverse() * POS_link2bucket << "\n";


	
	// 5. Prepare visualization with Irrlicht
	//    Note that Irrlicht uses left-handed frames with Y up.

/*	
	
	
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

*/	

		//5. Prepare Visualization with OpenGL
		// Create OpenGL window and camera
		// -------------------------------
		

/*						
#ifdef CHRONO_OPENGL

		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "WL first example", &system);
		gl_window.SetCamera(ChVector<>(0, 0, 5), ChVector<>(0, 0, 0), ChVector<>(0, 1, 0));
		gl_window.SetRenderMode(opengl::SOLID);
		gl_window.Pause();
		
		double time_step = 0.01;
		while (true) {
			if (gl_window.Active()) {
				if (gl_window.Running()){
					gl_window.DoStepDynamics(time_step);
			}
			gl_window.Render();
		}
}
		

		
#else

#endif
*/

    return 0;
}
