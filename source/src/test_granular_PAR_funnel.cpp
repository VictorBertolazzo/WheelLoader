// Granular Pile creation through funneling.
// Victor Bertolazzo

// mi=.75-->Repose angle ~ 36.87°
// [W,H]=[.795,.298]m
// particle size r=.01m
// #particles 15e3

// Funnel Dimensions= - Upper Base(open)=20r X 20r if spheres
//					  - Lower Base(open)=3.5r X 3.5r if spheres
//                    - Height = 10r

// Funnel Contact Shape creation : ConvexHulls

// Sphere Creation: Sample Generator that fits into Funnel.UpperBase dimensions, pouring into through gravity force;located over the Funnel.UpperBase
// Funnel.Velocity = 2.5r m/s
// Funnel.InitPos(LowerBase)=0.0;

// Note: consideration about funnel settle time, it should be leaved a certain amount of time for leaving the spheres settling into the funnel.
// Another Consideration-->The funnel should be too long for containing all the spheres, a runtime creation of the spheres must be thought.

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

#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono/collision/ChCCollisionUtils.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/motion_functions/ChFunction_Sine.h"

#include "chrono_postprocess/ChGnuPlot.h"




using namespace chrono;
using namespace chrono::collision;
using namespace postprocess;
// Computing Kinetic Energy of each particle
double ComputeKineticEnergy(ChBody* body){

	double mass = body->GetMass();
	ChMatrix33<> I = body->GetInertia();
	ChVector <> xdot = body->GetPos_dt();
	ChVector <> omega = body->GetWvel_loc();

	double kin = mass* xdot.Dot(xdot) + omega.Dot(I.Matr_x_Vect(omega));
	kin = kin / 2;	return kin;

}
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

// PovRay Output
bool povray_output = false;
const std::string out_dir = "../";
const std::string pov_dir = out_dir + "/POVRAY";
int out_fps = 60;

using std::cout;
using std::endl;
// --------------------------------------------------------------------------
// -------------------------
// Tuning Parameters
// -------------------------
double radius_g = 0.01;
double rollfr = 0.0 * radius_g;
// Tuning Parameters


// Funnel utility Function
	void AddWall(ChVector<> lower, ChVector<> upper, std::shared_ptr<ChBody> body,double r) {

		std::vector<ChVector<double>> cloud;
		double th = 0.5*r ;// or halve an input
		// if statement only on lower 
		if(lower.x() !=0. && lower.y() == 0.){
			double ss = std::abs(lower.x());

		cloud.push_back(lower + ChVector<>(-th,ss,0.));
		cloud.push_back(lower + ChVector<>(+th,ss,0.));
		cloud.push_back(lower + ChVector<>(-th,-ss,0.));
		cloud.push_back(lower + ChVector<>(+th,-ss,0.));
			double uu = std::abs(upper.x());
		cloud.push_back(upper + ChVector<>(-th,uu,0.));
		cloud.push_back(upper + ChVector<>(+th,uu,0.));
		cloud.push_back(upper + ChVector<>(-th,-uu,0.));
		cloud.push_back(upper + ChVector<>(+th,-uu,0.));
		
		}
		
		if(lower.x() == 0. && lower.y() != 0.){
			double ss = std::abs(lower.y());
		cloud.push_back(lower + ChVector<>(ss,-th,0.));
		cloud.push_back(lower + ChVector<>(ss,+th,0.));
		cloud.push_back(lower + ChVector<>(-ss,-th,0.));
		cloud.push_back(lower + ChVector<>(-ss,+th,0.));
			double uu = std::abs(upper.y());
		cloud.push_back(upper + ChVector<>(uu,-th,0.));
		cloud.push_back(upper + ChVector<>(uu,+th,0.));
		cloud.push_back(upper + ChVector<>(-uu,-th,0.));
		cloud.push_back(upper + ChVector<>(-uu,+th,0.));			
		}

		// Add a check on cloud.size()==8;


		body->GetCollisionModel()->AddConvexHull(cloud, ChVector<>(0, 0, 0), QUNIT);
		//body->GetCollisionModel()->BuildModel();

		auto shape = std::make_shared<ChTriangleMeshShape>();
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(cloud, shape->GetMesh());
		body->AddAsset(shape);

		body->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
	}
// Particle Runtime Generation	
	int SpawnParticles(utils::Generator* gen) {
		double dist = 2 * 0.99 * radius_g;

		////gen->createObjectsBox(utils::POISSON_DISK,
		////                     dist,
		////                     ChVector<>(9, 0, 3),
		////                     ChVector<>(0, 1, 0.5),
		////                     ChVector<>(-initVel, 0, 0));
		gen->createObjectsCylinderZ(utils::POISSON_DISK, dist, ChVector<>(0, 0, 0.25), 0.030f, 0, ChVector<>(0, 0, 0));
		cout << "  total bodies: " << gen->getTotalNumBodies() << endl;

		return gen->getTotalNumBodies();
	}

int main(int argc, char** argv) {
	int num_threads = 4;
	ChMaterialSurfaceBase::ContactMethod method = ChMaterialSurfaceBase::DVI;
	bool use_mat_properties = true;
	bool render = true;
	bool track_granule = false;

	

	// -------------------------
	// Bumby Terrain
	// Ra def := Ra_r/Ra_d;
	// Codec :  A=.5/1.5
	//			B=1.5/2.5
	//			C=2.5/3.5
	//			D=3.5/4.5
	// -------------------------
	double Ra_d = 5.0*radius_g;//Distance from centers of particles.
	double Ra_r = 3.0*radius_g;//Default Size of particles.
	// -------------------------
	// -------------------------
	// Aliquotes
	// -------------------------
	double quote_sp = 0.20;//1//.10 crash
	double quote_bs = 0.40;//2//.30 crash
	double quote_el = 0.40;//3//.30 crash
	double quote_cs = 0.00;//4//.30 crash
	double quote_bx = 0.00;//5//.55-->3.7r spacing
	double quote_rc = 0.00;//6

	double quote_sbx = 0.00;



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
	int num_layers = 22;

	// Terrain contact properties
	float friction_terrain = 0.9f;
	float restitution_terrain = 0.0f;
	float Y_terrain = 8e5f;
	float nu_terrain = 0.3f;
	float kn_terrain = 1.0e7f;
	float gn_terrain = 1.0e3f;
	float kt_terrain = 2.86e6f;
	float gt_terrain = 1.0e3f;
	float coh_pressure_terrain = 0e3f;
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
		sys->GetSettings()->collision.collision_envelope = 0.2 * radius_g;// 0.1*rg in origin
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
	container->SetMass(1);
	container->SetBodyFixed(true);
	container->SetCollide(true);
	container->SetMaterialSurface(material_terrain);

	container->GetCollisionModel()->ClearModel();
	// Bottom box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, 2 * radius_g), ChVector<>(0, 0, 0.0),
		ChQuaternion<>(1, 0, 0, 0), true);
	//// Front box
	//utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
	//	ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	//// Rear box
	//utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
	//	ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	//// Left box
	//utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
	//	ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	//// Right box
	//utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
	//	ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	container->GetCollisionModel()->BuildModel();
	//////// ----------------
	//////// Create Funnel
	//////// ----------------
	//////auto funnel = std::shared_ptr<ChBody>(system->NewBody());
	//////system->AddBody(funnel);
	//////funnel->SetIdentifier(-10);
	//////funnel->SetPos(ChVector<>(0.,0.,2*radius_g));
	//////funnel->SetMass(1);
	//////funnel->SetBodyFixed(false);
	//////funnel->SetMaterialSurface(material_terrain);
	//////switch (method) {
	//////	// Since it's not the contact btw funnel and particles what I'm interested in, I slow down mi-coefficient
	//////case ChMaterialSurfaceBase::DEM: {
	//////	funnel->GetMaterialSurfaceDEM()->SetFriction(.1f);
	//////	break;
	//////}
	//////case ChMaterialSurfaceBase::DVI: {
	//////	funnel->GetMaterialSurface()->SetFriction(.1f);
	//////	break;
	//////}
	//////}
	//////funnel->SetCollide(true);
	//////funnel->GetCollisionModel()->ClearModel();
	//////// OpenGL does not accept more than ONE visualization shape per body
	//////double half_low_size = 6 *radius_g ;
	//////double half_upp_size = 20 * radius_g ;
	//////double base2base_height = 100 * radius_g;
	//////AddWall(ChVector<>(half_low_size,.0,.0),ChVector<>(half_upp_size,.0,base2base_height),funnel,radius_g);
	//////AddWall(ChVector<>(0.,half_low_size,.0),ChVector<>(0.,half_upp_size,base2base_height),funnel,radius_g);
	//////AddWall(ChVector<>(-half_low_size,.0,.0),ChVector<>(-half_upp_size,.0,base2base_height),funnel,radius_g);
	//////AddWall(ChVector<>(0.,-half_low_size,.0),ChVector<>(0.,-half_upp_size,base2base_height),funnel,radius_g);
	//////funnel->GetCollisionModel()->BuildModel();
	//////
	////////----------------
	//////// Create the Funnel Movement: this should be a constant velocity trajectory in z direction
	//////// 
	//////		// Linear actuator btw the container and the funnel, it simulates the raising of the latter which must be always closely over the sand heap top.
	//////auto cont2fun = std::make_shared<ChLinkLockPrismatic>();
	//////cont2fun->Initialize(funnel, container, false, ChCoordsys<>(funnel->GetPos(), QUNIT), ChCoordsys<>(container->GetPos(), QUNIT));
	//////system->AddLink(cont2fun);
	//////auto container2funnel = std::make_shared<ChLinkLinActuator>();
	//////container2funnel->Initialize(funnel,container,false,ChCoordsys<>(funnel->GetPos(), QUNIT),ChCoordsys<>(container->GetPos(), QUNIT));
	//////auto funnel_law = std::make_shared<ChFunction_Ramp>();
	//////funnel_law->Set_ang(3.6*radius_g);// velocity of the funnel(10X test for simulation purposes).
	//////container2funnel->Set_lin_offset(Vlength(container->GetPos()-funnel->GetPos()));
	//////container2funnel->Set_dist_funct(funnel_law);
	//////system->AddLink(container2funnel);
	//////// test its goodness
	
	
	// Adding a "roughness" to the terrain, consisting of sphere/capsule/ellipsoid grid
	//	double spacing = 3.5 * radius_g;

	for (int ix = -40; ix < 40; ix++) {
		for (int iy = -40; iy < 40; iy++) {
			ChVector<> pos(ix * Ra_d, iy * Ra_d, 0.0);
			utils::AddSphereGeometry(container.get(), Ra_r, pos);
		}
	}
	container->GetCollisionModel()->BuildModel();

	
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
	m1->setDefaultSize(ChVector<>(.5*radius_g, radius_g / 2.0, 0));
	// Add new types of shapes to the generator, giving the percentage of each one
	//ELLIPSOIDS
	std::shared_ptr<utils::MixtureIngredient> m2 = gen.AddMixtureIngredient(utils::ELLIPSOID, quote_el);
	m2->setDefaultMaterial(material_terrain);
	m2->setDefaultDensity(rho_g);
	m2->setDefaultSize(ChVector<>(0.75*radius_g, 1.2*radius_g, radius_g / 2));// this need a vector  
	// Add new types of shapes to the generator, giving the percentage of each one
	//CAPSULES/
	std::shared_ptr<utils::MixtureIngredient> m3 = gen.AddMixtureIngredient(utils::CAPSULE, quote_cs);
	m3->setDefaultMaterial(material_terrain);
	m3->setDefaultDensity(rho_g);
	m3->setDefaultSize(ChVector<>(0.5*radius_g, 0.5*radius_g, radius_g / 2));
	//BOXES/
	std::shared_ptr<utils::MixtureIngredient> m4 = gen.AddMixtureIngredient(utils::BOX, quote_bx);
	m4->setDefaultMaterial(material_terrain);
	m4->setDefaultDensity(rho_g);
	m4->setDefaultSize(ChVector<>(radius_g, radius_g / 1.1, radius_g / 2.0));
	//ROUNDED-CYLINDERS/
	std::shared_ptr<utils::MixtureIngredient> m5 = gen.AddMixtureIngredient(utils::ROUNDEDCYLINDER, quote_rc);
	m5->setDefaultMaterial(material_terrain);
	m5->setDefaultDensity(rho_g);
	m5->setDefaultSize(ChVector<>(0.5*radius_g, 0.5*radius_g, radius_g / 2));
	//BOXES/
	std::shared_ptr<utils::MixtureIngredient> m6 = gen.AddMixtureIngredient(utils::BOX, quote_sbx);
	m6->setDefaultMaterial(material_terrain);
	m6->setDefaultDensity(rho_g);
	m6->setDefaultSize(radius_g);



	double r = 1.01 * radius_g;
	ChVector<> hdims(10* r - r, 10*r - r, 10*r);
	////ChVector<> center(0., 0., 0.0);//10r is the height of the funnel.
	ChVector<> center(0, 0, 0.125);
	gen.createObjectsCylinderZ(utils::POISSON_DISK, 2.3 * r, center, 0.030, .0);
	// gen.createObjectsBox(utils::POISSON_DISK, 2*r , funnel->GetPos() + ChVector<>(.0, .0, 3.0), hdims);
	//gen.createObjectsBox(utils::POISSON_DISK, 3 * r, funnel->GetPos() + ChVector<>(.0, .0, base2base_height+1.*r), hdims);

	std::shared_ptr<ChBody> granule;  // tracked granule
	std::ofstream outf;             // output file stream

	////if (track_granule) {
	////	int id = Id_g + num_particles -1;//Id_g + num_particles / 2;
	////	auto bodies = system->Get_bodylist();
	////	for (auto body = bodies->begin(); body != bodies->end(); ++body) {
	////		if ((*body)->GetIdentifier() == id) {
	////			granule = *body;
	////			break;
	////		}
	////	}

	////	outf.open("../settling_granule.dat", std::ios::out);
	////	outf.precision(7);
	////	outf << std::scientific;
	////}

#ifdef CHRONO_OPENGL
	// -------------------------------
	// Create the visualization window
	// -------------------------------

	if (render) {
		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "Settling test", system);
		gl_window.SetCamera(ChVector<>(0, -1, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
		gl_window.SetRenderMode(opengl::WIREFRAME);
	}
#endif

	// ---------------
	// Simulate system
	// ---------------

	double time_end = 1.00;
	double time_step = 1e-4;

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
	
	double funnel_count = 1.;
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

	double avkinenergy = 0.;
	ChFunction_Recorder mfun;

	while (system->GetChTime() < time_end) {
		system->DoStepDynamics(time_step);
		auto list = system->Get_bodylist();
//		for (auto body = list->begin(); body != list->end(); ++body) {
				for (auto body = particlelist.begin(); body != particlelist.end(); ++body) {
		auto mbody = std::shared_ptr<ChBody>(*body);
			avkinenergy += ComputeKineticEnergy(mbody.get());
					}
		avkinenergy /= particlelist.size();
		mfun.AddPoint(system->GetChTime(), avkinenergy);


		double test=system->GetChTime() / 0.1 - funnel_count;

		if (std::abs(test) < time_step) {
			SpawnParticles(&gen);
			funnel_count += 1.;
		}

		//TimingOutput(system);

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
			outf << pos.x() << " " << pos.y() << " " << pos.z() << " ";
			outf << vel.x() << " " << vel.y() << " " << vel.z();
			outf << std::endl << std::flush;
		}
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
	std::cout << "Simulation time: " << cum_sim_time << std::endl;
	std::cout << "    Broadphase:  " << cum_broad_time << std::endl;
	std::cout << "    Narrowphase: " << cum_narrow_time << std::endl;
	std::cout << "    Solver:      " << cum_solver_time << std::endl;
	std::cout << "    Update:      " << cum_update_time << std::endl;
	std::cout << std::endl;

	// Gnuplot
	ChGnuPlot mplot("__tmp_gnuplot_4.gpl");
	mplot.SetGrid();
	mplot.Plot(mfun, "Kinetic Energy of the system", " with lines lt -1 lc rgb'#00AAEE' ");

	return 0;
}