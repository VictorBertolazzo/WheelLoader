// Victor Bertolazzo
// Collection of all test granular codes.Switching from one to another can be done via case function.
// For reference original files are kept in the repo, activate them disabling the comment in CMakeLists.tex file
// ================================================================================================================

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

#include "chrono/collision/ChCCollisionUtils.h"

#include "chrono_parallel/collision/ChBroadphaseUtils.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#include "chrono_postprocess/ChGnuPlot.h"



#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

enum TestType {LAYER, FUNNEL, DROP, CASCADE};
TestType workcase = TestType::FUNNEL;
using namespace chrono;
using namespace postprocess;

// --------------------------------------------------------------------------


const std::string out_dir = "../";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string flatten = out_dir + "/funnel_DEMP_c10";
const std::string flatten_track = "funnel_DEMP_c10";

int out_fps = 60;

using std::cout;
using std::endl;

int num_threads = 40;
ChMaterialSurface::ContactMethod method = ChMaterialSurface::SMC;
// PovRay Output
bool povray_output = false;
// Material
bool use_mat_properties = true;
// Render 
bool render = false;
// Tracking Granule
bool track_granule = false;
// Roughness
bool roughness = false;
// Broad vs Narr
bool broad_narr = true;
// Tracking Flattening
bool track_flatten = true;
// --------------------------------------------------------------------------
double radius_g = 0.01;
// --------------------------------------------------------------------------
double r = 1.01 * radius_g;

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
int num_layers = 10;

// Terrain contact properties---Default Ones are commented out.
float friction_terrain = 0.7f;// 
float restitution_terrain = 0.0f;
float Y_terrain = 8e4f;
float nu_terrain = 0.3f;
float kn_terrain = 1.0e4f;// 1.0e7f;
float gn_terrain = 1.0e2f;
float kt_terrain = 2.86e3f;// 2.86e6f;
float gt_terrain = 1.0e2f;
float coh_pressure_terrain = 10.f;// 0e3f;
float coh_force_terrain = (float)(CH_C_PI * radius_g * radius_g) * coh_pressure_terrain;
float rolling_friction = 0.01 * radius_g;

//// Number of bins for broad-phase
int factor = 2;
int binsX = 10;
int binsY = 10;
int binsZ = 10;

// -------------------------
double Ra_d = 5.0*radius_g;//Distance from centers of particles.
double Ra_r = 3.0*radius_g;//Default Size of particles.



// ---------------------------FUNCTIONS--------------------------------
double ComputeKineticEnergy(ChBody* body){

	double mass = body->GetMass();
	ChMatrix33<> I = body->GetInertia();
	ChVector <> xdot = body->GetPos_dt();
	ChVector <> omega = body->GetWvel_par();

	double kin = mass* xdot.Dot(xdot) + omega.Dot(I.Matr_x_Vect(omega));
	kin = kin / 2;	return kin;

}
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
	printf("n\n");
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
// Funnel utility Function
void AddWall(ChVector<> lower, ChVector<> upper, std::shared_ptr<ChBody> body, double r) {

	std::vector<ChVector<double>> cloud;
	double th = 0.5*r;// or halve an input
	// if statement only on lower 
	if (lower.x() != 0. && lower.y() == 0.){
		double ss = std::abs(lower.x());

		cloud.push_back(lower + ChVector<>(-th, ss, 0.));
		cloud.push_back(lower + ChVector<>(+th, ss, 0.));
		cloud.push_back(lower + ChVector<>(-th, -ss, 0.));
		cloud.push_back(lower + ChVector<>(+th, -ss, 0.));
		double uu = std::abs(upper.x());
		cloud.push_back(upper + ChVector<>(-th, uu, 0.));
		cloud.push_back(upper + ChVector<>(+th, uu, 0.));
		cloud.push_back(upper + ChVector<>(-th, -uu, 0.));
		cloud.push_back(upper + ChVector<>(+th, -uu, 0.));

	}

	if (lower.x() == 0. && lower.y() != 0.){
		double ss = std::abs(lower.y());
		cloud.push_back(lower + ChVector<>(ss, -th, 0.));
		cloud.push_back(lower + ChVector<>(ss, +th, 0.));
		cloud.push_back(lower + ChVector<>(-ss, -th, 0.));
		cloud.push_back(lower + ChVector<>(-ss, +th, 0.));
		double uu = std::abs(upper.y());
		cloud.push_back(upper + ChVector<>(uu, -th, 0.));
		cloud.push_back(upper + ChVector<>(uu, +th, 0.));
		cloud.push_back(upper + ChVector<>(-uu, -th, 0.));
		cloud.push_back(upper + ChVector<>(-uu, +th, 0.));
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
// Particle Runtime Generation , case::DROP	
int SpawnParticles(utils::Generator* gen) {
	double dist = 2.3 * 1.01 * radius_g;

	////gen->createObjectsBox(utils::POISSON_DISK,
	////                     dist,
	////                     ChVector<>(9, 0, 3),
	////                     ChVector<>(0, 1, 0.5),
	////                     ChVector<>(-initVel, 0, 0));
	gen->createObjectsCylinderZ(utils::POISSON_DISK, dist, ChVector<>(0, 0, 0.15), 0.075f, 0, ChVector<>(0, 0, 0));
	std::cout << "  total bodies: " << gen->getTotalNumBodies() << std::endl;

	return gen->getTotalNumBodies();
}
// Funnel Generation , case::FUNNEL, 7.2r m/s speed.
void CreateFunnel(chrono::ChSystem* system, std::shared_ptr<ChBody> container, std::shared_ptr<ChMaterialSurface> material_terrain){
	auto funnel = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(funnel);
	funnel->SetIdentifier(-2);
	funnel->SetPos(ChVector<>(0., 0., 2 * radius_g));
	funnel->SetMass(1);
	funnel->SetBodyFixed(false);
	funnel->SetMaterialSurface(material_terrain);
	switch (method) {
		// Since it's not the contact btw funnel and particles what I'm interested in, I slow down mi-coefficient
	case ChMaterialSurface::SMC: {
										 funnel->GetMaterialSurfaceSMC()->SetFriction(.1f);
										 break;
	}
	case ChMaterialSurface::NSC: {
										 funnel->GetMaterialSurfaceNSC()->SetFriction(.1f);
										 break;
	}
	}
	funnel->SetCollide(true);
	funnel->GetCollisionModel()->ClearModel();
	// OpenGL does not accept more than ONE visualization shape per body
	double half_low_size = 6 * radius_g;
	double half_upp_size = 20 * radius_g;
	double base2base_height = 100 * radius_g;
	AddWall(ChVector<>(half_low_size, .0, .0), ChVector<>(half_upp_size, .0, base2base_height), funnel, radius_g);
	AddWall(ChVector<>(0., half_low_size, .0), ChVector<>(0., half_upp_size, base2base_height), funnel, radius_g);
	AddWall(ChVector<>(-half_low_size, .0, .0), ChVector<>(-half_upp_size, .0, base2base_height), funnel, radius_g);
	AddWall(ChVector<>(0., -half_low_size, .0), ChVector<>(0., -half_upp_size, base2base_height), funnel, radius_g);
	funnel->GetCollisionModel()->BuildModel();

	//----------------
	// Create the Funnel Movement: this should be a constant velocity trajectory in z direction
	// 
	// Linear actuator btw the container and the funnel, it simulates the raising of the latter which must be always closely over the sand heap top.
	auto cont2fun = std::make_shared<ChLinkLockPrismatic>();
	cont2fun->Initialize(funnel, container, false, ChCoordsys<>(funnel->GetPos(), QUNIT), ChCoordsys<>(container->GetPos(), QUNIT));
	system->AddLink(cont2fun);
	auto container2funnel = std::make_shared<ChLinkLinActuator>();
	container2funnel->Initialize(funnel, container, false, ChCoordsys<>(funnel->GetPos(), QUNIT), ChCoordsys<>(container->GetPos(), QUNIT));
	auto funnel_law = std::make_shared<ChFunction_Ramp>();
	funnel_law->Set_ang(7.2*radius_g);
	container2funnel->Set_lin_offset(Vlength(container->GetPos() - funnel->GetPos()));
	container2funnel->Set_dist_funct(funnel_law);
	system->AddLink(container2funnel);

}
// Hollowed Cylinder Generation , case::CASCADE
void CreateTube(chrono::ChSystem* system, std::shared_ptr<ChBody> container, std::shared_ptr<ChMaterialSurface> material_terrain, double time_hold){	// Create TUBE body
	auto tube = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(tube);
	tube->SetIdentifier(-2);
	tube->SetPos_dt(ChVector<>(.0, .0, 0.0));// Initial Value
	tube->SetMass(1.0);
	tube->SetBodyFixed(false);// true + actuator yields two bodies explode
	tube->SetCollide(true);
	tube->SetMaterialSurface(material_terrain);
	tube->GetCollisionModel()->ClearModel();
	ChQuaternion<> qtube;
	qtube.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(1, 0, 0));
	tube->SetPos(ChVector<>(0.0, .0, .0));
	tube->SetRot(qtube);
	// In original file measures fulfill to those proposed by the cited article, here the same are adapted to the TR requirements.
	for (int i = 0; i < 20; i++){
		utils::AddTorusGeometry(tube.get(), .150, .005, 5 * 20, 360, ChVector<>(0, 2 * i * 0.005, 0.), ChQuaternion<>(1.0, 0., 0., .0), true);
	}
	//utils::AddTorusGeometry(tube.get(), .133 / 2, .005, 20,360,ChVector<>(0,0,0),ChQuaternion<>(1.0,.0,.0,.0),true);
	tube->GetCollisionModel()->BuildModel();

	// Create a prismatic actuator btw CONTAINER and TUBE
	auto prismCT = std::make_shared<ChLinkLockPrismatic>();
	prismCT->Initialize(tube, container, ChCoordsys<>(ChVector<>(.0, .0, 0.0), QUNIT));
	system->AddLink(prismCT);
	auto linCT = std::make_shared<ChLinkLinActuator>();
	linCT->Initialize(tube, container, ChCoordsys<>(ChVector<>(.0, .0, 0.0), QUNIT));//m2 is the master
	linCT->Set_lin_offset(0.0);
	system->AddLink(linCT);
	auto legge1 = std::make_shared<ChFunction_Const>();
	legge1->Set_yconst(0.0);
	auto legge2 = std::make_shared<ChFunction_Ramp>();
	legge2->Set_ang(0.075);//.015 in origin
	auto sequence = std::make_shared<ChFunction_Sequence>();
	sequence->InsertFunct(legge1, time_hold, 1.0, true);
	sequence->InsertFunct(legge2, 300, 1.0, true);


	linCT->Set_dist_funct(sequence);
}
// ---------------------------FUNCTIONS--------------------------------
int main(int argc, char** argv) {
	uint max_iteration_normal = 0;
	uint max_iteration_sliding = 0;
	uint max_iteration_spinning = 200;
	uint max_iteration_bilateral = 0;
	// Create output directories.
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
		if (track_flatten) {
			if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
				cout << "Error creating directory " << out_dir << endl;
				return 1;
			}
			if (ChFileutils::MakeDirectory(flatten.c_str()) < 0) {
				cout << "Error creating directory " << flatten << endl;
				return 1;
			}
		}

	// Get number of threads from arguments (if specified)
	if (argc > 1) {
		num_threads = std::stoi(argv[1]);
	}
	std::cout << "Requested number of threads: " << num_threads << std::endl;
	// Model parameters
	std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;
	// Create the parallel system
	chrono::ChSystemParallel* system;
	// Create system and set method-specific solver settings
	switch (method) {
	case ChMaterialSurface::SMC: {
										 ChSystemParallelSMC* sys = new ChSystemParallelSMC;
										 sys->GetSettings()->solver.contact_force_model = ChSystemSMC::Hertz;
										 sys->GetSettings()->solver.tangential_displ_mode = ChSystemSMC::TangentialDisplacementModel::OneStep;
										 sys->GetSettings()->solver.use_material_properties = use_mat_properties;
										 system = sys;

										 break;
	}
	case ChMaterialSurface::NSC: {
										 ChSystemParallelNSC* sys = new ChSystemParallelNSC;
										 sys->GetSettings()->solver.solver_mode = SolverMode::SPINNING;
										 sys->GetSettings()->solver.max_iteration_normal = 0;
										 sys->GetSettings()->solver.max_iteration_sliding = 0;
										 sys->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
										 sys->GetSettings()->solver.alpha = 0;
										 sys->GetSettings()->solver.contact_recovery_speed = .1;
										 sys->GetSettings()->collision.collision_envelope = 0.05 * radius_g;
										 sys->ChangeSolverType(SolverType::APGD);
										 system = sys;

										 break;
	}
	}

	system->Set_G_acc(ChVector<>(0, 0, -9.81));
	system->GetSettings()->perform_thread_tuning = false;
	system->GetSettings()->solver.use_full_inertia_tensor = false;
	system->GetSettings()->solver.tolerance = 10.;//pumping it to tol=10, it achieves max_iter when new spawned particles
													// collide against the ones on the floor.And then keep oscillating.
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

	// -----------------------------------------------------------------------------------
	// ---------------------------------Create terrain bodies-----------------------------
	// -----------------------------------------------------------------------------------

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
										 mat_ter->SetCohesion(0.0);
										 mat_ter->SetSpinningFriction(rolling_friction);
										 mat_ter->SetRollingFriction(rolling_friction);

										 material_terrain = mat_ter;

										 break;
	}
	}
	// -----------------------------------------------------------------------------------
	// ---------------------------------Create terrain bodies-----------------------------
	// -----------------------------------------------------------------------------------

	// Create contact material for container, tube, funnel...
	std::shared_ptr<ChMaterialSurface> material_body;

	switch (method) {
	case ChMaterialSurface::SMC: {
										 auto mat_ter = std::make_shared<ChMaterialSurfaceSMC>();
										 mat_ter->SetFriction(friction_terrain);
										 mat_ter->SetRestitution(restitution_terrain);
										 mat_ter->SetYoungModulus(Y_terrain);
										 mat_ter->SetPoissonRatio(nu_terrain);
										 mat_ter->SetAdhesion(0.0);
										 mat_ter->SetKn(kn_terrain);
										 mat_ter->SetGn(gn_terrain);
										 mat_ter->SetKt(kt_terrain);
										 mat_ter->SetGt(gt_terrain);

										 material_body = mat_ter;

										 break;
	}
	case ChMaterialSurface::NSC: {
										 auto mat_ter = std::make_shared<ChMaterialSurfaceNSC>();
										 mat_ter->SetFriction(friction_terrain);
										 mat_ter->SetRestitution(restitution_terrain);
										 mat_ter->SetCohesion(0.0);
										 mat_ter->SetSpinningFriction(rolling_friction);
										 mat_ter->SetRollingFriction(rolling_friction);

										 material_body = mat_ter;

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
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick),
		ChQuaternion<>(1, 0, 0, 0), true);
	// Front box
	utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
		ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	// Rear box
	utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
		ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	// Left box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
		ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	// Right box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
		ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	container->GetCollisionModel()->BuildModel();

	if (roughness) {
	// Adding a "roughness" to the terrain, consisting of sphere/capsule/ellipsoid grid--ENABLING AT THE TOP OF THE FILE.

	for (int ix = -40; ix < 40; ix++) {
		for (int iy = -40; iy < 40; iy++) {
			ChVector<> pos(ix * Ra_d, iy * Ra_d, 0.0);
			utils::AddSphereGeometry(container.get(), Ra_r, pos);
		}
	}
	container->GetCollisionModel()->BuildModel();
	}

	// ----------------------------------------------------------------------------------------------------------------------- //
	// ------------------------------------------------Create particles------------------------------------------------------- //
	// ----------------------------------------------------------------------------------------------------------------------- //

	// // 
		// Create a particle generator and a mixture entirely made out of spheres
		utils::Generator gen(system);
		std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1);
		m1->setDefaultMaterial(material_terrain);
		m1->setDefaultDensity(rho_g);
		m1->setDefaultSize(radius_g);
		gen.setBodyIdentifier(Id_g);



		double time_end = 4.50;
		double time_step = 1e-4;
		double time_to_plot = 0.;

	switch (workcase) {
	case TestType::LAYER: {			
								  // Create particles in layers until reaching the desired number of particles
								  ChVector<> hdims(hdimX / 4.35 - r, hdimY / 4.35 - r, 0);
								  ChVector<> center(0, 0, 2 * r);

								  for (int il = 0; il < num_layers; il++) {
								  gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
								  center.z() += 2 * r;
								  // shrink uniformly the upper layer
								  hdims.x() -= 2 * r;
								  hdims.y() -= 2 * r;
								  // move the center abscissa by a 1*r 
								  center.x() += r * pow(-1, il);
								  if (method == ChMaterialSurface::NSC){ time_step = 1e-3; }


	}

		break;
								}
		case TestType::FUNNEL: {
								   ChVector<> hdims(10 * r - r, 10 * r - r, 10 * r);
								   ChVector<> center(0., 0., 52 * r + 25*r);//10r is the height of the funnel.

								   CreateFunnel(system, container, material_body);
								   gen.createObjectsCylinderZ(utils::POISSON_DISK, 2.4 * r, center, 10 * r, center.z() - .05 - 25*r);

								   time_to_plot = 1.75;
								   if (method == ChMaterialSurface::NSC){ time_step = 1e-3; }

								  break;
		}
		case TestType::DROP: {		// 1200 bodies instead of 1300, too high 
								 // Create particles in layers until reaching the desired number of particles
								 ChVector<> hdims(0.10 - r, 0.10 - r, 1.50);//
								 ChVector<> center(0, 0, .7500);//.800

								 //gen.createObjectsCylinderZ(utils::POISSON_DISK, 2.4 * r, center, 0.075, center.z() - .05);
								 unsigned int num_particles = gen.getTotalNumBodies();
								 std::cout << "Generated particles:  " << num_particles << std::endl;
								 time_end = 10.00;
								 time_to_plot = 6.5;
								 if (method == ChMaterialSurface::NSC){ time_step = 1e-3; }
								  break;
		}
		case TestType::CASCADE: {	double time_hold = 2.0;
									CreateTube(system, container, material_body, time_hold);

									// Create particles in layers until reaching the desired number of particles
									double r = 1.01 * radius_g;
									ChVector<> hdims(0.10 - r, 0.10 - r, 1.50);//W=.795, hdims object for the function gen.createObjectsBox accepts the	FULL dimensions in each direction:PAY ATTENTION
									ChVector<> center(0, 0, .500);//.800

									gen.createObjectsCylinderZ(utils::POISSON_DISK, 2.4 * r, center, .100 , center.z() - .05);
									unsigned int num_particles = gen.getTotalNumBodies();
									std::cout << "Generated particles:  " << num_particles << std::endl;
								  break;
		}

}





			// BRAODPHASE UTILS--still not used--future task
	vec3 bins = collision::function_Compute_Grid_Resolution(1000,real3(hdimX,hdimY,hdimZ),.1);

	unsigned int num_particles = gen.getTotalNumBodies();
	std::cout << "Generated particles:  " << num_particles << std::endl;

	// If tracking a granule (roughly in the "middle of the pack"),
	// grab a pointer to the tracked body and open an output file.
	std::shared_ptr<ChBody> granule;		// tracked granule
	std::ofstream outf;						// output file stream

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
			// Create a clone bodylist for extracting all bodies informations.
	std::vector<std::shared_ptr<ChBody>> particlelist;
	auto original_bodylist = system->Get_bodylist();
	for (auto body = original_bodylist->begin(); body != original_bodylist->end(); ++body) {
		auto mbody = std::shared_ptr<ChBody>(*body);
		particlelist.push_back(mbody);
	}
	//particlelist.erase(particlelist.begin());// 

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
	

	double cum_sim_time = 0;
	double cum_broad_time = 0;
	double cum_narrow_time = 0;
	double cum_solver_time = 0;
	double cum_update_time = 0;

	TimingHeader();


	int out_steps = std::ceil((1.0 / time_step) / out_fps);

	int sim_frame = 0;
	int out_frame = 0;
	int next_out_frame = 0;

	ChFunction_Recorder mfun;
	ChFunction_Recorder zfun;
	ChFunction_Recorder bnfun;

	double avkinenergy = 0.;
	
	

	while (system->GetChTime() < time_end) {

		system->DoStepDynamics(time_step);

		cum_sim_time += system->GetTimerStep();
		cum_broad_time += system->GetTimerCollisionBroad();
		cum_narrow_time += system->GetTimerCollisionNarrow();
		cum_solver_time += system->GetTimerSolver();
		cum_update_time += system->GetTimerUpdate();


		// ------------------------OPTIONAL IF STATEMENT FUNCTIONS
		// SPAWN PARTICLES-optional DROP case
		if (workcase == DROP && sim_frame % (int)(.1 / time_step) == 0 && system->GetChTime() < 6.00){
			SpawnParticles(&gen);
		}

				// ---------------------------PLOTTING-WRITING FUNCTIONS AFTER ASSESSMENT-------------------------------------------------- //
		if (system->GetChTime() > time_to_plot){
			// Perform Kinetic Energy and Maximum Height Detection
			auto list = system->Get_bodylist();
			std::vector<double> zs;
			for (auto body = list->begin(); body != list->end(); ++body) {
				//for (auto body = particlelist.begin(); body != particlelist.end(); ++body) {
				auto mbody = std::shared_ptr<ChBody>(*body);
				if (mbody->GetIdentifier() == -1 || mbody->GetIdentifier() == -2) {
				}
				else{
					avkinenergy += ComputeKineticEnergy(mbody.get());
					zs.push_back(mbody->GetPos().z());
				}


			}
			//		avkinenergy /= particlelist.size();
			avkinenergy /= list->size();
			mfun.AddPoint(system->GetChTime(), avkinenergy);
			auto biggest = std::max_element(std::begin(zs), std::end(zs));
			zfun.AddPoint(system->GetChTime(), *biggest);






			// ------------------------OPTIONAL IF STATEMENT FUNCTIONS

			// CALCULATE NARR/BROAD RATIO-if stat
			if (broad_narr){// temp loc
				int broad = system->data_manager->measures.collision.number_of_contacts_possible;
				int narr = system->data_manager->num_rigid_contacts;
					//std::cout << "Potential Contacts : " << broad << " , Actual Contacts : " << narr << std::endl;
				bnfun.AddPoint(system->GetChTime(), (float) narr / broad);
			}


			// TRACK ALL THE GRANULES- optional
			// Write a file each 60 time_steps(tunable) to 
			if (track_flatten) 	{
				//if (sim_frame == next_out_frame) {
				if (sim_frame % out_steps == 0) {
					std::ofstream outs;             // output file stream
					char filename[100];
					sprintf(filename, "../%s/data_%03d.dat", flatten_track.c_str(), out_frame + 1);
					outs.open(filename, std::ios::out);
					outs.precision(7);
					outs << std::scientific;
					for (auto body = particlelist.begin(); body != particlelist.end(); ++body) {
						double x = (*body)->GetPos().x(); double y = (*body)->GetPos().y(); double z = (*body)->GetPos().z();
						// no matter if one of the bodies is the terrain-Its erasing will be done offline
						outs << x << "\t" << y << "\t" << z << endl;
					}
					outs.close();
					out_frame++;
					next_out_frame += out_steps;
				}
			}


			// Track Single Granule-OPTIONAL
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




		}// All those actions takes place after "time_to_plot" simulation time.

		// VISUALIZATION-OPTIONAL
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
	
		sim_frame++;
		//TimingOutput(system);

	}


			// AFTER SIMULATION STUFFS

	// Gnuplot
	ChGnuPlot mplot("__tmp_gnuplot_4.gpl");
	mplot.SetGrid();
	mplot.Plot(mfun, "Kinetic Energy of the system", " with lines lt -1 lc rgb'#00AAEE' ");
	//
	ChGnuPlot zplot("__tmp_gnuplot_5.gpl");
	zplot.SetGrid();
	zplot.Plot(zfun, "Maximum particle height", " with lines lt -1 lc rgb'#00AAEE' ");
	//
	ChGnuPlot bnplot("__tmp_gnuplot_6.gpl");
	bnplot.SetGrid();
	bnplot.Plot(bnfun, "Narrow to Broad Phase collision contacts ratio.", " with lines lt -1 lc rgb'#00AAEE' ");
	//
	std::cout << std::endl;
	std::cout << "Simulation time: " << cum_sim_time << std::endl;
	std::cout << "    Broadphase:  " << cum_broad_time << std::endl;
	std::cout << "    Narrowphase: " << cum_narrow_time << std::endl;
	std::cout << "    Solver:      " << cum_solver_time << std::endl;
	std::cout << "    Update:      " << cum_update_time << std::endl;
	std::cout << std::endl;
	
	return 0;
}