// Author : Victor Bertolazzo
// Central Pivot Mechanism - as described in the cited patent
// https://www.google.com/patents/US4220348
// All the components present before the actuating cylinder in the driver to wheels chain are here neglected, and a fictitious displacement law has been applied to the former.
// Real
#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <valarray>
#include <vector>

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

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/collision/ChCCollisionUtils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/motion_functions/ChFunction_Integrate.h"
#include "chrono/motion_functions/ChFunction_Base.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/motion_functions/ChFunction_Sine.h"

#include <irrlicht.h>

#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"


using namespace chrono;
using namespace chrono::irrlicht;
using namespace postprocess;


using std::cout;
using std::endl;

bool render = true;
// --------------------------------------------------------------------------
int main(int argc, char** argv) {
	int num_threads = 4;
	// Get number of threads from arguments (if specified)
	if (argc > 1) {
		num_threads = std::stoi(argv[1]);
	}
	std::cout << "Requested number of threads: " << num_threads << std::endl;
	// Create the parallel system -- 
	ChSystemParallelDVI system;
	system.Set_G_acc(ChVector<>(.0, .0, -9.81));
	system.GetSettings()->perform_thread_tuning = false;
	system.GetSettings()->solver.use_full_inertia_tensor = false;
	system.GetSettings()->solver.tolerance = .1;// loosening the tolerance does not work(h=1e-4)
	system.GetSettings()->solver.max_iteration_bilateral = 0;
	system.GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
	system.GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
	// Set number of threads
	system.SetParallelThreadNumber(num_threads);
	CHOMPfunctions::SetNumThreads(num_threads);

	ChQuaternion<> z2y;
	ChQuaternion<> z2x;
	z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
	z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));

	// Mechanism Construction
	auto axle = std::make_shared<ChBody>();
	auto left_link = std::make_shared<ChBody>();
	auto right_link = std::make_shared<ChBody>();
	auto left_stub = std::make_shared<ChBody>();
	auto right_stub = std::make_shared<ChBody>();
	auto pivot = std::make_shared<ChBody>();
	
	auto left_rod = std::make_shared<ChBody>();
	auto right_rod = std::make_shared<ChBody>();


	auto rev_ll2pivot = std::make_shared<ChLinkLockRevolute>();
		auto rev_rl2pivot = std::make_shared<ChLinkLockRevolute>();

	auto rev_pivot2axle = std::make_shared<ChLinkEngine>();
	

	auto rev_ll2lr = std::make_shared<ChLinkLockSpherical>();// ball joint
	auto rev_ls2axle = std::make_shared<ChLinkLockRevolute>();
	auto rev_rl2rr = std::make_shared<ChLinkLockSpherical>();// ball joint
	auto rev_rs2axle = std::make_shared<ChLinkLockRevolute>();

	auto rev_lr2ls = std::make_shared<ChLinkLockRevolute>(); 
	auto rev_rr2rs = std::make_shared<ChLinkLockRevolute>(); 

	// POSITIONS & LOCATIONS
	double w = 2.0;
	ChVector<> COG_AXLE(0., 0., .4);
	ChVector<> COG_PIVOT(0.1, 0, .55);
	ChVector<> RIGHT_LINK(-.2, 0, .4);
	ChVector<> LEFT_LINK(-.2, 0, .3);
	ChVector<> RIGHT_STUB(0, -w / 2, .5);
	ChVector<> LEFT_STUB(0, +w / 2, .5);
	
	ChVector<> RIGHT_DIST(0, -w / 2 -.05, .45);
	ChVector<> LEFT_DIST(0, +w / 2 +.05, .45);

	ChVector<> RIGHT_BALL(-.2, -w / 2 + w/20, .35);
	ChVector<> LEFT_BALL(-.2, +w / 2 - w/20, .35);


	// Initialization
	axle->SetPos(COG_AXLE); system.Add(axle); axle->SetBodyFixed(true);
	auto a_asset = std::make_shared<ChBoxShape>(); a_asset->GetBoxGeometry().Size = ChVector<>(.05, w/2, .05); axle->AddAsset(a_asset);
	pivot->SetPos(COG_PIVOT); system.Add(pivot); pivot->SetBodyFixed(false);
	auto piv1_asset = std::make_shared<ChCylinderShape>(); piv1_asset->GetCylinderGeometry().rad = .015; piv1_asset->GetCylinderGeometry().p1 = pivot->GetFrame_COG_to_abs().GetInverse() * ChVector<>(.1, -w / 6 - w/8, 0.55); piv1_asset->GetCylinderGeometry().p2 = VNULL;
	pivot->AddAsset(piv1_asset);
	auto piv2_asset = std::make_shared<ChCylinderShape>(); piv2_asset->GetCylinderGeometry().rad = .015; piv2_asset->GetCylinderGeometry().p1 = pivot->GetFrame_COG_to_abs().GetInverse() * ChVector<>(-.2, 0, 0.55); piv2_asset->GetCylinderGeometry().p2 = VNULL;
	pivot->AddAsset(piv2_asset);
	rev_pivot2axle->Initialize(pivot, axle, ChCoordsys<>(pivot->GetPos(), QUNIT)); 
	rev_pivot2axle->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); rev_pivot2axle->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);system.AddLink(rev_pivot2axle);
	auto rot_fun = std::make_shared<ChFunction_Sine>(); rot_fun->Set_amp(.2); rot_fun->Set_freq(4.);
	rev_pivot2axle->Set_rot_funct(rot_fun);
	
	right_link->SetPos(RIGHT_LINK); system.Add(right_link);
	auto rl_asset = std::make_shared<ChCylinderShape>(); rl_asset->GetCylinderGeometry().rad = .03; rl_asset->GetCylinderGeometry().p1 = VNULL; rl_asset->GetCylinderGeometry().p2 = right_link->GetFrame_COG_to_abs().GetInverse() * RIGHT_BALL; right_link->AddAsset(rl_asset);
	left_link->SetPos(LEFT_LINK); system.Add(left_link);
	auto ll_asset = std::make_shared<ChCylinderShape>(); ll_asset->GetCylinderGeometry().rad = .03; ll_asset->GetCylinderGeometry().p1 = VNULL; ll_asset->GetCylinderGeometry().p2 = left_link->GetFrame_COG_to_abs().GetInverse() * LEFT_BALL; left_link->AddAsset(ll_asset);

	rev_ll2pivot->Initialize(left_link,pivot,ChCoordsys<>(LEFT_LINK,QUNIT)); system.AddLink(rev_ll2pivot);
	rev_rl2pivot->Initialize(right_link, pivot, ChCoordsys<>(RIGHT_LINK, QUNIT)); system.AddLink(rev_rl2pivot);
	
	// Killing two extra dofs.

	right_stub->SetPos(RIGHT_STUB); system.Add(right_stub); right_stub->SetRot(chrono::Q_from_AngAxis(CH_C_PI_2,VECT_Z));
	left_stub->SetPos(LEFT_STUB); system.Add(left_stub); left_stub->SetRot(chrono::Q_from_AngAxis(CH_C_PI_2, VECT_Z));
	ChVector<> patch(.07, 0, 0);
	ChVector<> offset(.4, 0, 0);
	auto rs_asset = std::make_shared<ChCylinderShape>(); rs_asset->GetCylinderGeometry().rad = .10; rs_asset->GetCylinderGeometry().p1 = -offset - patch; rs_asset->GetCylinderGeometry().p2 = -offset + patch; right_stub->AddAsset(rs_asset);
	auto ls_asset = std::make_shared<ChCylinderShape>(); ls_asset->GetCylinderGeometry().rad = .10; ls_asset->GetCylinderGeometry().p1 = offset - patch; ls_asset->GetCylinderGeometry().p2 = offset + patch; left_stub->AddAsset(ls_asset);
	auto rs_s_asset = std::make_shared<ChSphereShape>(); rs_s_asset->GetSphereGeometry().rad = .05; rs_s_asset->Pos = right_stub->GetFrame_COG_to_abs().GetInverse() * RIGHT_DIST;
	auto ls_s_asset = std::make_shared<ChSphereShape>(); ls_s_asset->GetSphereGeometry().rad = .05; ls_s_asset->Pos = left_stub->GetFrame_COG_to_abs().GetInverse() * LEFT_DIST;
	right_stub->AddAsset(rs_s_asset); left_stub->AddAsset(ls_s_asset);
	// Revolute Joints btw chassis and knuckles(z-axis)
	rev_ls2axle->Initialize(left_stub, axle, ChCoordsys<>(LEFT_STUB, QUNIT)); system.AddLink(rev_ls2axle);
	rev_rs2axle->Initialize(right_stub, axle, ChCoordsys<>(RIGHT_STUB, QUNIT)); system.AddLink(rev_rs2axle);
	// Distance Constraints btw steering links and knuckles
	auto dist_left = std::make_shared<ChLinkDistance>();
	dist_left->Initialize(left_stub, left_link, false, LEFT_BALL, LEFT_DIST);
	system.AddLink(dist_left);
	auto dist_right = std::make_shared<ChLinkDistance>();
	dist_right->Initialize(right_stub, right_link, false, RIGHT_BALL, RIGHT_DIST);
	system.AddLink(dist_right);

	//       // Directly to the stub
	//rev_lr2ls->Initialize(left_stub, left_link, ChCoordsys<>(LEFT_BALL, QUNIT)); system.AddLink(rev_lr2ls);
	//rev_rr2rs->Initialize(right_stub, right_link, ChCoordsys<>(RIGHT_BALL, QUNIT)); system.AddLink(rev_rr2rs);

	// No collision
	axle->SetCollide(false); right_link->SetCollide(false); left_link->SetCollide(false);
	pivot->SetCollide(false); right_stub->SetCollide(false); left_stub->SetCollide(false);

	// Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master
	{ std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

	// ---------------
	// Simulate system
	// ---------------

	double time_end = 1000.00;
	double time_step = 1e-4;

	//5. Prepare Visualization with OpenGL

	opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
	gl_window.Initialize(1280, 720, "Central Pivot Mechanism", &system);
	gl_window.SetCamera(ChVector<>(-3.5, 1., 1.5), ChVector<>(3.5, 0, 0), ChVector<>(0, 0, 1));
	gl_window.SetRenderMode(opengl::SOLID);


	// Simulate system

	while (system.GetChTime() < time_end) {

		system.DoStepDynamics(time_step);

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




	std::cout << std::endl;
	std::cout << "Central Pivot Mechanism: " << "    EOS  " << std::endl;
	std::cout << std::endl;


	return 0;
}