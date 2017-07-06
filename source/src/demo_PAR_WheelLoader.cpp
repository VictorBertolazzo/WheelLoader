// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
//
// =============================================================================

#include <iostream>

// Chrono::Engine header files
#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Chrono::Parallel header files
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChSystemDescriptorParallel.h"
#include "chrono_parallel/collision/ChNarrowphaseRUtils.h"
#include "chrono_parallel/collision/ChBroadphaseUtils.h"

// Chrono::Parallel OpenGL header files
//#undef CHRONO_OPENGL

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

// Chrono utility header files
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Chrono vehicle header files
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"


// Chrono::Models header files
#include "chrono_models/vehicle/generic/Generic_SimplePowertrain.h"
#include "chrono_models/vehicle/generic/Generic_RigidTire.h"
#include "chrono_models/vehicle/generic/Generic_FialaTire.h"
// 
#include "subsystems/Articulated_Front.h"
#include "subsystems/Articulated_Rear.h"


using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;

using std::cout;
using std::endl;

// =============================================================================
// USER SETTINGS
// =============================================================================

// Comment the following line to use Chrono::Parallel
//#define USE_SEQ

// Comment the following line to use NSC contact
//#define USE_SMC

// -----------------------------------------------------------------------------
// Specification of the terrain
// -----------------------------------------------------------------------------

enum TerrainType { RIGID_TERRAIN, GRANULAR_TERRAIN };

// Type of terrain
TerrainType terrain_type = RIGID_TERRAIN;
// Type of tire model (RIGID or FIALA)
TireModelType tire_model = TireModelType::RIGID;

// Control visibility of containing bin walls
bool visible_walls = true;

// Dimensions
double hdimX = 4.5;
double hdimY = 1.75;
double hdimZ = 0.5;
double hthick = 0.25;
// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Parameters for granular material
int Id_g = 100;
double r_g = 0.02;
double rho_g = 2500;
double vol_g = (4.0 / 3) * CH_C_PI * r_g * r_g * r_g;
double mass_g = rho_g * vol_g;
ChVector<> inertia_g = 0.4 * mass_g * r_g * r_g * ChVector<>(1, 1, 1);

float mu_g = 0.8f;

unsigned int num_particles = 10e3; //// 40000;

// -----------------------------------------------------------------------------
// Specification of the vehicle model
// -----------------------------------------------------------------------------

// Initial vehicle position and orientation
ChVector<> initLoc(-hdimX + 4.5, 0, .35);//z=1
ChQuaternion<> initRot(1, 0, 0, 0);

// Simple powertrain model
std::string simplepowertrain_file("generic/powertrain/SimplePowertrain.json");

// -----------------------------------------------------------------------------
// Simulation parameters
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 1;

// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

// Total simulation duration.
double time_end = 7;

// Duration of the "hold time" (vehicle chassis fixed and no driver inputs).
// This can be used to allow the granular material to settle.
double time_hold = 0.2;

// Solver parameters
double time_step = 1e-3;  // 2e-4;

double tolerance = 0.01;

int max_iteration_bilateral = 1000;  // 1000;
int max_iteration_normal = 0;
int max_iteration_sliding = 200;  // 2000;
int max_iteration_spinning = 0;

float contact_recovery_speed = -1;

// Periodically monitor maximum bilateral constraint violation
bool monitor_bilaterals = false;
int bilateral_frame_interval = 100;

// Output directories
bool povray_output = false;

const std::string out_dir = "../WL";
const std::string pov_dir = out_dir + "/POVRAY";

int out_fps = 60;

// =============================================================================

double CreateParticles(ChSystem* system) {
	// Create a material
#ifdef USE_SMC
	auto mat_g = std::make_shared<ChMaterialSurfaceSMC>();
	mat_g->SetYoungModulus(1e8f);
	mat_g->SetFriction(mu_g);
	mat_g->SetRestitution(0.4f);
#else
	auto mat_g = std::make_shared<ChMaterialSurfaceNSC>();
	mat_g->SetFriction(mu_g);
#endif

	// Create a particle generator and a mixture entirely made out of spheres
	utils::Generator gen(system);
	std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1.0);
	m1->setDefaultMaterial(mat_g);
	m1->setDefaultDensity(rho_g);
	m1->setDefaultSize(r_g);

	// Set starting value for body identifiers
	gen.setBodyIdentifier(Id_g);

	// Create particles in layers until reaching the desired number of particles
	double r = 1.01 * r_g;
	ChVector<> hdims(hdimX - r, hdimY - r, 0);
	ChVector<> center(0, 0, 2 * r);

	while (gen.getTotalNumBodies() < num_particles) {
		gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
		center.z() += 2 * r;
	}

	std::cout << "Created " << gen.getTotalNumBodies() << " particles." << std::endl;

	return center.z();
}

// =============================================================================
// Utility function for displaying an ASCII progress bar for the quantity x
// which must be a value between 0 and n. The width 'w' represents the number
// of '=' characters corresponding to 100%.

void progressbar(unsigned int x, unsigned int n, unsigned int w = 50) {
	if ((x != n) && (x % (n / 100 + 1) != 0))
		return;

	float ratio = x / (float)n;
	unsigned int c = (unsigned int)(ratio * w);

	std::cout << std::setw(3) << (int)(ratio * 100) << "% [";
	for (unsigned int x = 0; x < c; x++)
		std::cout << "=";
	for (unsigned int x = c; x < w; x++)
		std::cout << " ";
	std::cout << "]\r" << std::flush;
}

// =============================================================================
int main(int argc, char* argv[]) {
	// -----------------
	// Initialize output
	// -----------------

	if (povray_output) {
		if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
			std::cout << "Error creating directory " << out_dir << std::endl;
			return 1;
		}

		if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
			std::cout << "Error creating directory " << pov_dir << std::endl;
			return 1;
		}
	}

	// --------------
	// Create system.
	// --------------

#ifdef USE_SEQ
	// ----  Sequential
#ifdef USE_SMC
	std::cout << "Create SMC system" << std::endl;
	ChSystemSMC* system = new ChSystemSMC();
#else
	std::cout << "Create NSC system" << std::endl;
	ChSystemNSC* system = new ChSystemNSC();
#endif

#else
	// ----  Parallel
#ifdef USE_SMC
	std::cout << "Create Parallel SMC system" << std::endl;
	ChSystemParallelSMC* system = new ChSystemParallelSMC();
#else
	std::cout << "Create Parallel NSC system" << std::endl;
	ChSystemParallelNSC* system = new ChSystemParallelNSC();
#endif

#endif

	system->Set_G_acc(ChVector<>(0, 0, -9.81));


	// ---------------------
	// Edit system settings.
	// ---------------------

#ifdef USE_SEQ

	////system->SetSolverType(ChSolver::Type::MINRES);
	system->SetMaxItersSolverSpeed(50);
	system->SetMaxItersSolverStab(50);
	////system->SetTol(0);
	////system->SetMaxPenetrationRecoverySpeed(1.5);
	////system->SetMinBounceSpeed(2.0);
	////system->SetSolverOverrelaxationParam(0.8);
	////system->SetSolverSharpnessParam(1.0);

#else

	// Set number of threads
	int max_threads = CHOMPfunctions::GetNumProcs();
	if (threads > max_threads)
		threads = max_threads;
	system->SetParallelThreadNumber(threads);
	CHOMPfunctions::SetNumThreads(threads);
	//omp_set_num_threads(threads);
	std::cout << "Using " << threads << " threads" << std::endl;

	system->GetSettings()->perform_thread_tuning = thread_tuning;

	// Set solver parameters
	system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
	system->GetSettings()->solver.use_full_inertia_tensor = false;
	system->GetSettings()->solver.tolerance = tolerance;

#ifndef USE_SMC
	system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
	system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
	system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
	system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
	system->GetSettings()->solver.alpha = 0;
	system->GetSettings()->solver.contact_recovery_speed = contact_recovery_speed;
	system->ChangeSolverType(SolverType::APGD);
	system->GetSettings()->collision.collision_envelope = 0.1 * r_g;
#else
	system->GetSettings()->solver.contact_force_model = ChSystemSMC::PlainCoulomb;
#endif
	// BRAODPHASE UTILS--still not used--future task
	vec3 bins = function_Compute_Grid_Resolution(12000, real3(hdimX, hdimY, hdimZ), 1.);
	int factor = 2;
	int binsX = (int)std::ceil(hdimX / r_g) / factor;
	int binsY = (int)std::ceil(hdimY / r_g) / factor;
	int binsZ = 1;

	system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
	//system->GetSettings()->collision.bins_per_axis = bins;



#endif


	// --------------------------
	// Construct the Wheel Loader vehicle
	// --------------------------

	// Create the front side
	Articulated_Front front_side(system); 
	front_side.Initialize(ChCoordsys<>(initLoc, initRot));
	front_side.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
	front_side.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
	front_side.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
	front_side.SetWheelVisualizationType(VisualizationType::NONE);
	
	// Create the rear side
	Articulated_Rear rear_side(std::static_pointer_cast<Articulated_Chassis>(front_side.GetChassis()));
	rear_side.Initialize();
	rear_side.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
	rear_side.SetWheelVisualizationType(VisualizationType::NONE);

	// Create the driver system
	// ...temporary workaround
	// ChDataDriver driver(front_side, vehicle::GetDataFile("M113/driver/Acceleration.txt"));
	ChDataDriver driver(front_side, "C:/Users/victo/Documents/chrono-build/bin/data/vehicle/M113/driver/Acceleration.txt");
	driver.Initialize();

	//front_side.GetSystem()->ShowHierarchy(GetLog());
	//system->ShowHierarchy(GetLog());

	// -------------------
	// Create the terrain.
	// -------------------

	// Contact material
#ifdef USE_SMC
	auto mat_g = std::make_shared<ChMaterialSurfaceSMC>();
	mat_g->SetYoungModulus(1e8f);
	mat_g->SetFriction(mu_g);
	mat_g->SetRestitution(0.4f);
#else
	auto mat_g = std::make_shared<ChMaterialSurfaceNSC>();
	mat_g->SetFriction(mu_g);
#endif

	////---------------------CREATE THE TERRAIN----------------------------
	//
	//RigidTerrain terrain(front_side.GetSystem());
	//terrain.SetContactFrictionCoefficient(0.9f);
	//terrain.SetContactRestitutionCoefficient(0.01f);
	//terrain.SetContactMaterialProperties(2e7f, 0.3f);
	//terrain.SetColor(ChColor(0.5f, 0.5f, 1));
	//terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
	//terrain.Initialize(terrainHeight, terrainLength, terrainWidth);
	//terrain.GetGroundBody()->GetCollisionModel()->ClearModel();

	//auto ground = std::shared_ptr<ChBody>(terrain.GetGroundBody());

    ///---------------------------OR CREATE A SIMPLE GROUND BODY------------------------

	// Ground body
	auto ground = std::shared_ptr<ChBody>(front_side.GetSystem()->NewBody());
	system->Add(ground);
	ground->SetIdentifier(-1); ground->SetName("Ground-Terrain");
	ground->SetBodyFixed(true);
	ground->SetCollide(true);

	ground->SetMaterialSurface(mat_g);

	ground->GetCollisionModel()->ClearModel();

	//--------------------------COMMON GEOMETRY DEFINITION-----------------------
	// Bottom box
	utils::AddBoxGeometry(ground.get(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick),
		ChQuaternion<>(1, 0, 0, 0), true);
	if (terrain_type == GRANULAR_TERRAIN) {
		// Front box
		utils::AddBoxGeometry(ground.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
			ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
		// Rear box
		utils::AddBoxGeometry(ground.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
			ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),
			visible_walls);
		// Left box
		utils::AddBoxGeometry(ground.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
			ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), visible_walls);
		// Right box
		utils::AddBoxGeometry(ground.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
			ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0),
			visible_walls);
	}

	//terrain.GetGroundBody()->GetCollisionModel()->BuildModel();// NO MORE NECESSARY
	ground->GetCollisionModel()->BuildModel();

	// Create the granular material.
	double vertical_offset = 0;

	if (terrain_type == GRANULAR_TERRAIN) {
		vertical_offset = CreateParticles(system);
	}

	// Create and initialize the powertrain system
	Generic_SimplePowertrain powertrain;
	powertrain.Initialize(front_side.GetChassisBody(), front_side.GetDriveshaft());

	// Create the front tires
	std::unique_ptr<ChTire> tire_FL;
	std::unique_ptr<ChTire> tire_FR;
	switch (tire_model) {
	case TireModelType::RIGID:
		tire_FL = std::unique_ptr<ChTire>(new Generic_RigidTire("FL"));
		tire_FR = std::unique_ptr<ChTire>(new Generic_RigidTire("FR"));
		break;
	case TireModelType::FIALA:
		tire_FL = std::unique_ptr<ChTire>(new Generic_FialaTire("FL"));
		tire_FR = std::unique_ptr<ChTire>(new Generic_FialaTire("FR"));
		break;
	default:
		std::cout << "Tire type not supported!" << std::endl;
		return 1;
	}

	tire_FL->Initialize(front_side.GetWheelBody(FRONT_LEFT), LEFT);
	tire_FR->Initialize(front_side.GetWheelBody(FRONT_RIGHT), RIGHT);
	tire_FL->SetVisualizationType(VisualizationType::PRIMITIVES);
	tire_FR->SetVisualizationType(VisualizationType::PRIMITIVES);

	// Create the rear tires
	std::unique_ptr<ChTire> tire_RL;
	std::unique_ptr<ChTire> tire_RR;
	switch (tire_model) {
	case TireModelType::RIGID:
		tire_RL = std::unique_ptr<ChTire>(new Generic_RigidTire("RL"));
		tire_RR = std::unique_ptr<ChTire>(new Generic_RigidTire("RR"));
		break;
	case TireModelType::FIALA:
		tire_RL = std::unique_ptr<ChTire>(new Generic_FialaTire("RL"));
		tire_RR = std::unique_ptr<ChTire>(new Generic_FialaTire("RR"));
		break;
	default:
		std::cout << "Tire type not supported!" << std::endl;
		return 1;
	}

	tire_RL->Initialize(rear_side.GetWheelBody(FRONT_LEFT), LEFT);
	tire_RR->Initialize(rear_side.GetWheelBody(FRONT_RIGHT), RIGHT);
	tire_RL->SetVisualizationType(VisualizationType::PRIMITIVES);
	tire_RR->SetVisualizationType(VisualizationType::PRIMITIVES);

	
	// ---------------
	// Simulation loop
	// ---------------

#ifdef CHRONO_OPENGL
	// Initialize OpenGL
	opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
	gl_window.Initialize(1280, 720, "Wheel Loader", system);
	gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
	gl_window.SetRenderMode(opengl::WIREFRAME);
#endif

	// Number of simulation steps between two 3D view render frames
	int out_steps = (int)std::ceil((1.0 / time_step) / out_fps);

	// Run simulation for specified time.
	double time = 0;
	int sim_frame = 0;
	int out_frame = 0;
	int next_out_frame = 0;
	double exec_time = 0;
	int num_contacts = 0;

	// Inter-module communication data
	TireForces tire_front_forces(2);
	TireForces tire_rear_forces(2);
	double driveshaft_speed;
	double powertrain_torque;
	double throttle_input;
	double steering_input;
	double braking_input;
	
	while (time < time_end) {
		// Collect output data from modules
		double throttle_input = driver.GetThrottle();
		double steering_input = driver.GetSteering();
		double braking_input = driver.GetBraking();
		powertrain_torque = powertrain.GetOutputTorque();

		tire_front_forces[0] = tire_FL->GetTireForce();
		tire_front_forces[1] = tire_FR->GetTireForce();
		tire_rear_forces[0] = tire_RL->GetTireForce();
		tire_rear_forces[1] = tire_RR->GetTireForce();

		driveshaft_speed = front_side.GetDriveshaftSpeed();

		WheelState wheel_FL = front_side.GetWheelState(FRONT_LEFT);
		WheelState wheel_FR = front_side.GetWheelState(FRONT_RIGHT);

		WheelState wheel_RL = rear_side.GetWheelState(FRONT_LEFT);
		WheelState wheel_RR = rear_side.GetWheelState(FRONT_RIGHT);

//
//		// Output
//		if (sim_frame == next_out_frame) {
//			cout << endl;
//			cout << "---- Frame:          " << out_frame + 1 << endl;
//			cout << "     Sim frame:      " << sim_frame << endl;
//			cout << "     Time:           " << time << endl;
//			cout << "     Avg. contacts:  " << num_contacts / out_steps << endl;
//			cout << "     Throttle input: " << throttle_input << endl;
//			cout << "     Braking input:  " << braking_input << endl;
//			cout << "     Steering input: " << steering_input << endl;
//			cout << "     Execution time: " << exec_time << endl;
//
//			if (povray_output) {
//				char filename[100];
//				sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame + 1);
//				utils::WriteShapesPovray(system, filename);
//			}
//
//			out_frame++;
//			next_out_frame += out_steps;
//			num_contacts = 0;
//		}

//		// Release the vehicle chassis at the end of the hold time.
//		if (vehicle.GetChassisBody()->GetBodyFixed() && time > time_hold) {
//			std::cout << std::endl << "Release vehicle t = " << time << std::endl;
//			vehicle.GetChassisBody()->SetBodyFixed(false);
//		}

		// Update modules (process inputs from other modules)
		driver.Synchronize(time);
        //------------COMMENT IF TERRAIN NOT PRESENT
		//tire_FL->Synchronize(time, wheel_FL, terrain) ;
		//tire_FR->Synchronize(time, wheel_FR, terrain);
		//tire_RL->Synchronize(time, wheel_RL, terrain);
		//tire_RR->Synchronize(time, wheel_RR, terrain);

		powertrain.Synchronize(time, throttle_input, driveshaft_speed);

		front_side.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_front_forces);
		rear_side.Synchronize(time, steering_input, braking_input, tire_rear_forces);

		// Advance simulation for one timestep for all modules
		driver.Advance(time_step);

		tire_FL->Advance(time_step);
		tire_FR->Advance(time_step);
		tire_RL->Advance(time_step);
		tire_RR->Advance(time_step);

		powertrain.Advance(time_step);

		front_side.Advance(time_step);

#ifdef CHRONO_OPENGL
		if (gl_window.Active())
			gl_window.Render();
		else
			break;
#endif

//		progressbar(out_steps + sim_frame - next_out_frame + 1, out_steps);
//
//		// Periodically display maximum constraint violation
//		if (monitor_bilaterals && sim_frame % bilateral_frame_interval == 0) {
//			vehicle.LogConstraintViolations();
//		}
//

		// Update counters.
		time += time_step;
		sim_frame++;
		exec_time += system->GetTimerStep();
		num_contacts += system->GetNcontacts();
		if (sim_frame == 85)// 172 if initLoc.z() == .5
			GetLog() << "Stop here and debug"<< "\n";
	}

//	// Final stats
//	std::cout << "==================================" << std::endl;
//	std::cout << "Simulation time:   " << exec_time << std::endl;
//	std::cout << "Number of threads: " << threads << std::endl;

	return 0;
}
