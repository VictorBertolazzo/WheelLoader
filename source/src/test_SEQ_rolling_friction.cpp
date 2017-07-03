// Sequential test for rolling friction use case.
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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/motion_functions/ChFunction_Sine.h"

#include "chrono_postprocess/ChGnuPlot.h"

using namespace chrono;
using namespace chrono::collision;
using namespace postprocess;

#ifdef CHRONO_IRRLICHT
#include "chrono_irrlicht/ChIrrApp.h"
using namespace chrono::irrlicht;
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
#endif


// --------------------------------------------------------------------------
//#define USE_SINGLE_SPHERE

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
	ChVector <> omega = body->GetWvel_par();
	
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
	double time_step = 1e-3;
	double time_end = 1.00;

	uint max_iteration_normal = 0;
	uint max_iteration_sliding = 0;
	uint max_iteration_spinning = 100;
	uint max_iteration_bilateral = 100;



	int num_threads = 4;
	ChMaterialSurface::ContactMethod method = ChMaterialSurface::NSC;//SMC(rolling friction not available)
	bool use_mat_properties = true;
	bool render = true;
	bool track_granule = false;
	double radius_g = 0.01;
	double initial_angspeed = 10;
	double initial_linspeed = initial_angspeed * radius_g;
	double density = 2500;
	double mass = density * (4.0 / 3.0) * CH_C_PI * pow(radius_g, 3);
	double inertia = (2.0 / 5.0) * mass * pow(radius_g, 2);

	double rolling_friction = 0.075 * radius_g;//RAISING mi_r=.5*r, BALLS TREPASS FLOOR.WHY????Still with mi_r=.1*r) 
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
	float restitution_terrain = 0.5f;
	float Y_terrain = 1e6f;
	float nu_terrain = 0.3f;
	float kn_terrain = 1.0e7f;// 1.0e7f;
	float gn_terrain = 1.0e3f;
	float kt_terrain = 2.86e6f;// 2.86e6f;
	float gt_terrain = 1.0e3f;
	float coh_pressure_terrain = 0e4f;// 0e3f;
	float coh_force_terrain = (float)(CH_C_PI * radius_g * radius_g) * coh_pressure_terrain;

	
    int binsX = 10;
    int binsY = 10;
    int binsZ = 10;
	std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;

	// --------------------------
	// Create the serial system
	// --------------------------

	// Create system and set method-specific solver settings
	chrono::ChSystem* system;

	switch (method) {
	case ChMaterialSurface::SMC: {
		ChSystemSMC* sys = new ChSystemSMC;
		sys->SetContactForceModel(ChSystemSMC::Hertz);
		sys->SetTangentialDisplacementModel(ChSystemSMC::TangentialDisplacementModel::OneStep);
		sys->UseMaterialProperties(use_mat_properties);
		system = sys;

		break;
	}
	case ChMaterialSurface::NSC: {
		ChSystemNSC* sys = new ChSystemNSC;
		sys->SetMaxItersSolverSpeed(100);

		sys->SetMaxPenetrationRecoverySpeed(0.1);
		sys->SetSolverType(ChSolver::Type::APGD);
		system = sys;

		break;
	}
	}

	system->Set_G_acc(ChVector<>(0, 0, -9.81));
	system->SetTol(.1);
	system->SetMaxiter(max_iteration_bilateral);


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

		mat_ter->SetSpinningFriction(rolling_friction);
		mat_ter->SetRollingFriction(rolling_friction);

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
	// it's not the problem for using all iterations
	container->SetMaterialSurface(material_terrain);
	
	container->GetCollisionModel()->ClearModel();
	
	// Bottom box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, 10*radius_g), ChVector<>(0, 0, 0*radius_g),
		ChQuaternion<>(1, 0, 0, 0), true);
	container->GetCollisionModel()->BuildModel();
	
#ifdef USE_SINGLE_SPHERE
	// Create a Sphere
	auto ball = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(ball);			// FLAG
	ball->SetIdentifier(+1);
	ball->SetMass(mass);
	ball->SetInertiaXX(inertia*ChVector<>(1, 1, 1));
	ball->SetBodyFixed(false);
	ball->SetCollide(true);
	ball->SetPos(ChVector<>(.0, .0, 1*radius_g ));
	ball->SetPos_dt(ChVector<>(initial_linspeed, 0., 0.));
	ball->SetWvel_par(ChVector<>(.0, initial_angspeed, .0));
	//ball->SetMaterialSurface(material_terrain);
	ball->GetCollisionModel()->ClearModel();

	ball->GetMaterialSurfaceNSC()->SetFriction(friction_terrain);
	ball->GetMaterialSurfaceNSC()->SetRollingFriction(rolling_friction);
	ball->GetMaterialSurfaceNSC()->SetSpinningFriction(rolling_friction);
	
	utils::AddSphereGeometry(ball.get(),radius_g, ChVector<>(0, 0, 0),
		ChQuaternion<>(1, 0, 0, 0), true);
	ball->GetCollisionModel()->BuildModel();

	GetLog()<< "ball rolling : " <<ball->GetMaterialSurfaceNSC()->GetRollingFriction()<<"\n";
	//		
#else	
	// Create the sampler
	utils::Generator gen(system);
	// SPHERES
	std::shared_ptr<utils::MixtureIngredient> m0 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
	m0->setDefaultMaterial(material_terrain);
	m0->setDefaultDensity(density);
	m0->setDefaultSize(radius_g);
#ifdef USE_LAYERING
	ChVector<> center(.0, .0, 10 * radius_g);
	double r(1.01 * radius_g);
	ChVector<> hdims(10*r -r, 10*r -r, 0.0);
	int num_layers(2);
	for (int il = 0; il < num_layers; il++) {
			gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
			center.z() += 2 * r;
			// shrink uniformly the upper layer
			hdims.x() -= 2 * r;
			hdims.y() -= 2 * r;
			// move the center abscissa by a 1*r 
			center.x() += r * pow(-1, il);
		}
#else
	gen.createObjectsCylinderZ(utils::POISSON_DISK, 2.2 * 1.01 *radius_g, ChVector<>(.0, .0, 10 * radius_g), 0.030, 9 * radius_g);

#endif // USE_LAYERING

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
	int jiter = std::ceil(particlelist.size() / 2);
	double rgen = particlelist[jiter].get()->GetMaterialSurfaceNSC()->GetRollingFriction();
	GetLog() << "Generated particles rolling friction  : " << rgen << "\n";
	double rcon = container->GetMaterialSurfaceNSC()->GetRollingFriction();
	GetLog() << "Container rolling friction  : " << rcon << "\n";

#endif // USE_SINGLE_SPHERE
	GetLog() << "Created Particles : " << particlelist.size() << "\n";

	
#ifdef CHRONO_IRRLICHT
	// Create the Irrlicht visualization (open the Irrlicht device,bind a simple user interface, etc. etc.)
	ChIrrApp application(system, L"Contacts with rolling friction", core::dimension2d<u32>(800, 600), false,
		true);

	// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	ChIrrWizard::add_typical_Logo(application.GetDevice());
	ChIrrWizard::add_typical_Sky(application.GetDevice());
	ChIrrWizard::add_typical_Lights(application.GetDevice());
	application.AddTypicalCamera(core::vector3df(.5, .3, 0.), core::vector3df(0, 0, 0)); //'camera' location            // "look at" location
	
	//ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0., +1.,.5));

	// Use this function for adding a ChIrrNodeAsset to all already created items.
	// Otherwise use application.AssetBind(myitem); on a per-item basis.
	application.AssetBindAll();
	application.AssetUpdateAll();


#endif


	// ---------------
	// Simulate system
	// ---------------


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

	application.SetStepManage(true);
	application.SetTimestep(time_step);
	//application.SetTryRealtime(true);


	//while (system->GetChTime() < time_end) {
			while (application.GetDevice()->run()) {

				if (system->GetChTime() > time_end)
					break;
				// Irrlicht must prepare frame to draw
				application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

				application.DrawAll();

				application.DoStep();
			
		
				sim_frame++;

		
#ifdef USE_SINGLE_SPHERE
		avkinenergy = ComputeKineticEnergy(ball.get());
#else
		auto list = system->Get_bodylist();
				for (auto body = list->begin(); body != list->end(); ++body) {
		//for (auto body = particlelist.begin(); body != particlelist.end(); ++body) {
			auto mbody = std::shared_ptr<ChBody>(*body);
			avkinenergy += ComputeKineticEnergy(mbody.get());
		}
//		avkinenergy /= particlelist.size();
		avkinenergy /= list->size();
#endif
		mfun.AddPoint(system->GetChTime(), avkinenergy);
		avkinenergy = 0;// reset Ke

		cum_sim_time += system->GetTimerStep();
		cum_broad_time += system->GetTimerCollisionBroad();
		cum_narrow_time += system->GetTimerCollisionNarrow();
		cum_solver_time += system->GetTimerSolver();
		cum_update_time += system->GetTimerUpdate();

		application.GetVideoDriver()->endScene();


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