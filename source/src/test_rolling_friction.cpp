// Rolling Friction Assessment - One Single Sphere on Flat Terrain test.
// Data : dSphere = 10 mm; 
// Rolling Friction = ?;
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

using namespace chrono;
using namespace chrono::collision;
#include "chrono/core/ChLog.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/motion_functions/ChFunction_Sine.h"

#include "chrono_postprocess/ChGnuPlot.h"

using namespace postprocess;


// --------------------------------------------------------------------------


void TimingOutput(chrono::ChSystem* mSys) {
	double TIME = mSys->GetChTime();
	double STEP = mSys->GetTimerStep();
	double BROD = mSys->GetTimerCollisionBroad();
	double NARR = mSys->GetTimerCollisionNarrow();
	double SOLVER = mSys->GetTimerSolver();
	double UPDT = mSys->GetTimerUpdate();
	int REQ_ITS = 0;
	int BODS = mSys->GetNbodies();
	int CNTC = mSys->GetNcontacts();
	if (chrono::ChSystemParallel* parallel_sys = dynamic_cast<chrono::ChSystemParallel*>(mSys)) {
		REQ_ITS = std::static_pointer_cast<chrono::ChIterativeSolverParallel>(mSys->GetSolver())->GetTotalIterations();
	}

	printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f |\n", TIME, STEP, BROD, NARR,
		SOLVER, UPDT, BODS, CNTC, REQ_ITS);
}

double ComputeKineticEnergy(ChBody* body){
	
	double mass = body->GetMass();
	ChMatrix33<> I = body->GetInertia();
	ChVector <> xdot = body->GetPos_dt();
	ChVector <> omega = body->GetWvel_loc();
	
	double kin = mass* xdot.Dot(xdot) + omega.Dot(I.Matr_x_Vect(omega)) ;
	kin = kin / 2;	return kin;

}
// PovRay Output
bool povray_output = false;
const std::string out_dir = "../";
const std::string pov_dir = out_dir + "/POVRAY";

int out_fps = 60;

using std::cout;
using std::endl;
// --------------------------------------------------------------------------

int main(int argc, char** argv) {
	int num_threads = 4;
	ChMaterialSurfaceBase::ContactMethod method = ChMaterialSurfaceBase::DVI;//DEM
	bool use_mat_properties = true;
	bool render = true;
	bool track_granule = false;
	double radius_g = 0.005;

	double rollfr = 1.0 * radius_g;
	double Ra_d = 5.0*radius_g;//Distance from centers of particles.
	double Ra_r = 3.0*radius_g;//Default Size of particles.


	
	// --------------------------
	// Create output directories.
	// --------------------------

	if (povray_output) {
		if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
			cout << "Error creating directory " << out_dir << endl;
			return 1;
		}
		if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
			cout << "Error creating directory " << pov_dir << endl;
			return 1;
		}
	}

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

	// Terrain contact properties---Default Ones are commented out.
	float friction_terrain = 0.7f;// (H,W) requires mi=.70;
	float restitution_terrain = 0.0f;
	float Y_terrain = 1e6f;
	float nu_terrain = 0.3f;
	float kn_terrain = 1.0e7f;// 1.0e7f;
	float gn_terrain = 1.0e3f;
	float kt_terrain = 2.86e6f;// 2.86e6f;
	float gt_terrain = 1.0e3f;
	float coh_pressure_terrain = 0e4f;// 0e3f;
	float coh_force_terrain = (float)(CH_C_PI * radius_g * radius_g) * coh_pressure_terrain;

	// Estimates for number of bins for broad-phase
	int factor = 2;
	int binsX = (int)std::ceil(hdimX / radius_g) / factor;
	int binsY = (int)std::ceil(hdimY / radius_g) / factor;
	int binsZ = 1;

    binsX = 10;
    binsY = 10;
    binsZ = 20;
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
		sys->GetSettings()->solver.solver_mode = SolverMode::SPINNING;	
		sys->GetSettings()->solver.max_iteration_normal = 0;
		sys->GetSettings()->solver.max_iteration_sliding = 0;
		sys->GetSettings()->solver.max_iteration_spinning = 200;
		sys->GetSettings()->solver.alpha = 0;
		sys->GetSettings()->solver.contact_recovery_speed = 0.1;
		sys->GetSettings()->collision.collision_envelope = 0.05 * radius_g;//0.1
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

		mat_ter->SetSpinningFriction(rollfr);

		mat_ter->SetRollingFriction(rollfr);

		material_terrain = mat_ter;

		break;
	}
	}

	// Create container body
	auto container = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(container);
	container->SetIdentifier(-1);
	container->SetMass(1000.0);
	container->SetPos(ChVector<>(0., 0., -10 * radius_g));
	container->SetBodyFixed(true);
	container->SetCollide(true);
	container->SetMaterialSurface(material_terrain);
	container->GetCollisionModel()->ClearModel();
	
	// Bottom box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, 10*radius_g), ChVector<>(0, 0, 0*radius_g),
		ChQuaternion<>(1, 0, 0, 0), true);
	container->GetCollisionModel()->BuildModel();
	

	// Create a Sphere
	auto ball = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(ball);
	ball->SetIdentifier(+1);
	ball->SetDensity(rho_g);
	ball->SetBodyFixed(false);
	ball->SetCollide(true);
	ball->SetPos(ChVector<>(.0, .0, 2*radius_g + 1.0));
	ball->SetPos_dt(ChVector<>(.5, 0., 0.));
	//ball->SetWvel_par(ChVector<>(.0, 1.0, .0));
	ball->SetMaterialSurface(material_terrain);
	ball->GetCollisionModel()->ClearModel();
	// Bottom box
	utils::AddSphereGeometry(ball.get(),radius_g, ChVector<>(0, 0, 0),
		ChQuaternion<>(1, 0, 0, 0), true);
	ball->GetCollisionModel()->BuildModel();

	GetLog()<< "rolling : " <<ball->GetMaterialSurface()->GetRollingFriction()<<"\n";
	//		
	
	// Create the sampler
	utils::Generator gen(system);
	// SPHERES
	std::shared_ptr<utils::MixtureIngredient> m0 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
	m0->setDefaultMaterial(material_terrain);
	m0->setDefaultDensity(rho_g);
	m0->setDefaultSize(radius_g);
	gen.createObjectsCylinderZ(utils::POISSON_DISK, 2.4 * 1.01 *radius_g, ball->GetPos(), 0.030, 3*radius_g);


	// Create particle bodylist for Computing Averaging Kinetic,Potential Energy

	std::vector<std::shared_ptr<ChBody>> particlelist;
	auto original_bodylist = system->Get_bodylist();
	//for (int i = 0; i < original_bodylist->size(); i++) { auto mbody = std::shared_ptr<ChBody>(original_bodylist[original_bodylist.begin()+i]);
	//															particlelist.push_back(mbody);			}
	for (auto body = original_bodylist->begin(); body != original_bodylist->end(); ++body) {
		auto mbody = std::shared_ptr<ChBody>(*body);
		particlelist.push_back(mbody);
	}
	particlelist.erase(particlelist.begin()); // delete terrain body from the list
	//particlelist.erase(particlelist.begin()); // delete torus body from the list

	

	
#ifdef CHRONO_OPENGL
	// -------------------------------
	// Create the visualization window
	// -------------------------------

	if (render) {
		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "Settling test", system);
		gl_window.SetCamera(ChVector<>(-.06, -.11, +.1), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
		gl_window.SetRenderMode(opengl::WIREFRAME);
	}
#endif

	// ---------------
	// Simulate system
	// ---------------

	double time_end = 1.0025;
	double time_step = 1e-3;//1.5e-5;

	double cum_sim_time = 0;
	double cum_broad_time = 0;
	double cum_narrow_time = 0;
	double cum_solver_time = 0;
	double cum_update_time = 0;

	ChFunction_Recorder mfun;
	

	// Run simulation for specified time.
	int out_steps = std::ceil((1.0 / time_step) / out_fps);

	int sim_frame = 0;
	int out_frame = 0;
	int next_out_frame = 0;
	double avkinenergy = 0.;



	while (system->GetChTime() < time_end) {

		system->DoStepDynamics(time_step);
		sim_frame++;
		auto list = system->Get_bodylist();
		//		for (auto body = list->begin(); body != list->end(); ++body) {
		for (auto body = particlelist.begin(); body != particlelist.end(); ++body) {
			auto mbody = std::shared_ptr<ChBody>(*body);
			avkinenergy += ComputeKineticEnergy(mbody.get());
		}
		avkinenergy /= particlelist.size();
		mfun.AddPoint(system->GetChTime(), avkinenergy);

		cum_sim_time += system->GetTimerStep();
		cum_broad_time += system->GetTimerCollisionBroad();
		cum_narrow_time += system->GetTimerCollisionNarrow();
		cum_solver_time += system->GetTimerSolver();
		cum_update_time += system->GetTimerUpdate();

	
#ifdef CHRONO_OPENGL
		if (render) {
			opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
			if (gl_window.Active()) {
				gl_window.Render();
				//std::cout << ball->GetPos().z() <<std::endl;
			}
			else {
				return 1;
			}
		}
#endif
	}
	// Gnuplot
	ChGnuPlot mplot("__tmp_gnuplot_4.gpl");
	mplot.SetGrid();
	mplot.Plot(mfun, "Kinetic Energy of the system", " with lines lt -1 lc rgb'#00AAEE' ");

	std::cout << std::endl;
	std::cout << "Simulation time: " << cum_sim_time << std::endl;
	std::cout << "    Broadphase:  " << cum_broad_time << std::endl;
	std::cout << "    Narrowphase: " << cum_narrow_time << std::endl;
	std::cout << "    Solver:      " << cum_solver_time << std::endl;
	std::cout << "    Update:      " << cum_update_time << std::endl;
	std::cout << std::endl;

	
	return 0;
}