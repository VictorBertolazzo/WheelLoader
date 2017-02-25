// Granular Pile creation - Flat Terrain + Cascading Packing Conditions + only Spheres
// Article : http://dx.doi.org/10.1016/S0378-4371(99)00183-1
// Data : dSphere = 3-10 mm; dTube = 40-133 mm; tStep=4e-6 - 1.5e-5;
// Rolling Friction = 2e-4 m;
// tSim = ~20s;
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



// --------------------------------------------------------------------------

void TimingHeader() {
	printf("    TIME    |");
	printf("    STEP |");
	printf("   BROAD |");
	printf("  NARROW |");
	printf("  SOLVER |");
	printf("  UPDATE |");
	printf("# BODIES |");
	printf("# CONTACT|");
	printf(" # ITERS |");
	printf("\n\n");
}

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
	double kin = body->GetMass();
	ChMatrix33<> I = body->GetInertia();
	ChVector <> xdot = body->GetPos_dt();
	return kin;
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

	double Ra_d = 5.0*radius_g;//Distance from centers of particles.
	double Ra_r = 3.0*radius_g;//Default Size of particles.

	// Tweaking Parameters
	double time_hold = 5.0;

	// -------------------------
	// Aliquotes
	// -------------------------
	double quote_sp = 1.00;//
	
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

    binsX = 20;
    binsY = 20;
    binsZ = 30;
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

		mat_ter->SetSpinningFriction(2.0*radius_g);

		mat_ter->SetRollingFriction(2.0*radius_g);

		material_terrain = mat_ter;

		break;
	}
	}

	// Create container body
	auto container = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(container);
	container->SetIdentifier(-1);
	container->SetMass(1.0);
	container->SetBodyFixed(true);
	container->SetCollide(true);
	container->SetMaterialSurface(material_terrain);
	container->GetCollisionModel()->ClearModel();
	// Bottom box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, 2*radius_g), ChVector<>(0, 0, 0.0),
		ChQuaternion<>(1, 0, 0, 0), true);
	container->GetCollisionModel()->BuildModel();
	
	// Create TUBE body
	auto tube = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(tube);
	tube->SetIdentifier(-100);
	tube->SetPos_dt(ChVector<>(.0, .0, 0.0));// Initial Value
	tube->SetMass(1.0);
	tube->SetBodyFixed(false);// true + actuator yields two bodies explode
	tube->SetCollide(true);
	tube->SetMaterialSurface(material_terrain);
	tube->GetCollisionModel()->ClearModel();
	ChQuaternion<> qtube;
	qtube.Q_from_AngAxis(CH_C_PI/2 , ChVector<>(1, 0, 0));
	tube->SetPos(ChVector<>(0.0, .0, .0));
	tube->SetRot(qtube);
	for (int i = 0; i < 15; i++){
		utils::AddTorusGeometry(tube.get(), .133 / 2, .005, 5*20, 360, ChVector<>(0, 2 *i * 0.005, 0.), ChQuaternion<>(1.0, 0., 0., .0), true);
	}
	//utils::AddTorusGeometry(tube.get(), .133 / 2, .005, 20,360,ChVector<>(0,0,0),ChQuaternion<>(1.0,.0,.0,.0),true);
		tube->GetCollisionModel()->BuildModel();

		ChQuaternion<> z2z;
		//z2z.Q_from_AngAxis(0.0, ChVector<>(0, 1, 0));
	// Create a prismatic actuator btw CONTAINER and TUBE
		auto prismCT = std::make_shared<ChLinkLockPrismatic>();
		prismCT->Initialize(tube, container, ChCoordsys<>(ChVector<>(.0,.0,0.0), z2z));
		system->AddLink(prismCT);
		auto linCT = std::make_shared<ChLinkLinActuator>();
		linCT->Initialize(tube, container, ChCoordsys<>(ChVector<>(.0,.0,0.0), z2z));//m2 is the master
		linCT->Set_lin_offset(0.0);
		system->AddLink(linCT);
		auto legge1 = std::make_shared<ChFunction_Const>();
		legge1->Set_yconst(0.0);
		auto legge2 = std::make_shared<ChFunction_Ramp>();
		legge2->Set_ang(0.015);
		auto sequence = std::make_shared<ChFunction_Sequence>();
		sequence->InsertFunct(legge1,time_hold,1.0,true);
		sequence->InsertFunct(legge2, 300, 1.0, true);


		linCT->Set_dist_funct(sequence);

	//		
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


	// Create particles in layers until reaching the desired number of particles
	double r = 1.01 * radius_g;
	ChVector<> hdims(0.10 - r, 0.10 - r, 1.50);//W=.795, hdims object for the function gen.createObjectsBox accepts the	FULL dimensions in each direction:PAY ATTENTION
	ChVector<> center(0, 0, .330);//.800

	gen.createObjectsCylinderZ(utils::POISSON_DISK, 2.4 * r, center, .070/2, center.z()-.05);
	unsigned int num_particles = gen.getTotalNumBodies();
	std::cout << "Generated particles:  " << num_particles << std::endl;
		

	// Create particle bodylist for Computing Averaging Kinetic,Potential Energy

	std::vector<std::shared_ptr<ChBody>> particlelist;
	auto original_bodylist = system->Get_bodylist();
	//for (int i = 0; i < original_bodylist->size(); i++) { auto mbody = std::shared_ptr<ChBody>(original_bodylist[original_bodylist.begin()+i]);
	//															particlelist.push_back(mbody);			}
	for (auto body = original_bodylist->begin(); body != original_bodylist->end(); ++body) {
		auto mbody = std::shared_ptr<ChBody>(*body);
		particlelist.push_back(mbody);
	}
	//particlelist.erase(particlelist.begin()); // delete terrain body from the list
	//particlelist.erase(particlelist.begin()); // delete torus body from the list

	


	

	// If tracking a granule (roughly in the "middle of the pack"),
	// grab a pointer to the tracked body and open an output file.
	std::shared_ptr<ChBody> granule;  // tracked granule
	std::ofstream outf;             // output file stream

	if (track_granule) {
		int id = Id_g + num_particles -1;//Id_g + num_particles / 2;
		auto bodies = system->Get_bodylist();
		for (auto body = bodies->begin(); body != bodies->end(); ++body) {
			if ((*body)->GetIdentifier() == id) {
				granule = *body;
				break;
			}
		}

		outf.open("../settling_granule.dat", std::ios::out);
		outf.precision(7);
		outf << std::scientific;
	}


#ifdef CHRONO_OPENGL
	// -------------------------------
	// Create the visualization window
	// -------------------------------

	if (render) {
		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "Settling test", system);
		gl_window.SetCamera(ChVector<>(0, -1, 0.5), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
		gl_window.SetRenderMode(opengl::WIREFRAME);
	}
#endif

	// ---------------
	// Simulate system
	// ---------------

	double time_end = 300.00;
	double time_step = 1e-4;//1.5e-5;

	double cum_sim_time = 0;
	double cum_broad_time = 0;
	double cum_narrow_time = 0;
	double cum_solver_time = 0;
	double cum_update_time = 0;

	TimingHeader();


	// Run simulation for specified time.
	int out_steps = std::ceil((1.0 / time_step) / out_fps);

	int sim_frame = 0;
	int out_frame = 0;
	int next_out_frame = 0;


	while (system->GetChTime() < time_end) {

//		double time = system->GetChTime();
		system->DoStepDynamics(time_step);
		sim_frame++;

		//// Release the vehicle chassis at the end of the hold time.
		//if (tube->GetBodyFixed() && time > time_hold) {
		//	std::cout << std::endl << "Release tube t = " << time << std::endl;
		//	tube->SetBodyFixed(false);
		//	tube->SetCollide(false);
		//}

		cum_sim_time += system->GetTimerStep();
		cum_broad_time += system->GetTimerCollisionBroad();
		cum_narrow_time += system->GetTimerCollisionNarrow();
		cum_solver_time += system->GetTimerSolver();
		cum_update_time += system->GetTimerUpdate();

		if (track_granule) {
			assert(outf.is_open());
			assert(granule);
			const ChVector<>& pos = granule->GetPos();
			const ChVector<>& vel = granule->GetPos_dt();
			outf << system->GetChTime() << " ";  
			outf << system->GetNbodies() << " " << system->GetNcontacts() << " ";
			/*outf << pos.x() << " " << pos.y() << " " << pos.z() << " ";
			outf << vel.x() << " " << vel.y() << " " << vel.z();
			*/outf << std::endl << std::flush;
		}
#ifdef CHRONO_OPENGL
		if (render) {
			opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
			if (gl_window.Active()) {
				gl_window.Render();
				//std::cout << tube->GetPos_dt().z() <<std::endl;
			}
			else {
				return 1;
			}
		}
#endif
	}

	std::cout << std::endl;
	std::cout << "Simulation time: " << cum_sim_time << std::endl;
	std::cout << "    Broadphase:  " << cum_broad_time << std::endl;
	std::cout << "    Narrowphase: " << cum_narrow_time << std::endl;
	std::cout << "    Solver:      " << cum_solver_time << std::endl;
	std::cout << "    Update:      " << cum_update_time << std::endl;
	std::cout << std::endl;


	return 0;
}