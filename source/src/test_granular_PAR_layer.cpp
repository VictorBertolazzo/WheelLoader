// Granular Pile creation through layering method.
// Victor Bertolazzo

// Scaled model of the pile: reduced from a H=3m,W/2=4m(pyramid shape)-->15e6 particles of .01m radius size(for excess rounded, since spheres do not fill the entire space).
// to have 15e3 particles of .01m-->Vol = .0628 m3 = W^2*H/3                     -->
//[I WANT A H/(W/2) = .75-->which leads to an approximate repose angle of 36.87°]-->
//																					combining-->W^2*.75W/2/3=Vol-->W=.795 and  H=.298
// Note: this are qualitative computations that does not take into account shapes,rest coeff and density.(Coulomb-Mohr based approximation of repose angle).

// TODO : Develop clever ways to check the effective repose angle:
//																	-
//																	-
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

// PovRay Output
bool povray_output = false;
const std::string out_dir = "../";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string flatten = out_dir + "/flatten_track_test006c1e4kn1e4kte3";
const std::string flatten_track = "flatten_track_test006c1e4kn1e4kte3";

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
	bool track_flatten = false;
	double radius_g = 0.01;

	// -------------------------
	// Bumby Terrain
	// Ra def := Ra_r/Ra_d;
	// Codec :  A=.5/1.5
	//			B=1.5/2.5
	//			C=2.5/3.5
	//			D=3.5/4.5
	// -------------------------
	double Ra_d = 2.5*radius_g;//Distance from centers of particles.
	double Ra_r = 1.5*radius_g;//Default Size of particles.


	// -------------------------
	// Aliquotes
	// -------------------------
	double quote_sp = 0.00;//1
	double quote_bs = 0.10;//2
	double quote_el = 0.35;//3
	double quote_cs = 0.00;//4
	double quote_bx = 0.40;//5
	double quote_rc = 0.00;//6

	double quote_sbx = .15;

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

	if (track_flatten) {
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
	// PAY ATTENTION TO THIS VALUE: IT STACKS THE SIMULATION IF YOU USE UNFEASIBLE ONE.
	// e.g., 1 single sphere layer is achieved with num_layers=10 and you set num_layers=12;
	// look at the for loop generation for more details.
	int num_layers = 24;// ceil(H/radius_g)=30

	// Terrain contact properties---Default Ones are commented out.
	float friction_terrain = 0.7f;// (H,W) requires mi=.70;
	float restitution_terrain = 0.0f;
	float Y_terrain = 8e5f;
	float nu_terrain = 0.3f;
	float kn_terrain = 1.0e4f;// 1.0e7f;
	float gn_terrain = 1.0e3f;
	float kt_terrain = 2.86e3f;// 2.86e6f;
	float gt_terrain = 1.0e3f;
	float coh_pressure_terrain = 1e4f;// 0e3f;
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
		sys->GetSettings()->collision.collision_envelope = 0.1 * radius_g;
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
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -2*hthick),
		ChQuaternion<>(1, 0, 0, 0), true);
	////// Front box
	////utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
	////	ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	////// Rear box
	////utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
	////	ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	////// Left box
	////utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
	////	ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	////// Right box
	////utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
	////	ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	//////container->GetCollisionModel()->BuildModel();
	////

	// Adding a "roughness" to the terrain, consisting of sphere/capsule/ellipsoid grid
//	double spacing = 3.5 * radius_g;

	for (int ix = -40; ix < 40; ix++) {
		for (int iy = -40; iy < 40; iy++) {
			ChVector<> pos(ix * Ra_d, iy * Ra_d, -Ra_r);
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
	m1->setDefaultSize(radius_g);
	// Add new types of shapes to the generator, giving the percentage of each one
		//ELLIPSOIDS
	std::shared_ptr<utils::MixtureIngredient> m2 = gen.AddMixtureIngredient(utils::ELLIPSOID, quote_el);
	m2->setDefaultMaterial(material_terrain);
	m2->setDefaultDensity(rho_g);
	m2->setDefaultSize(radius_g);// this need a vector  
	// Add new types of shapes to the generator, giving the percentage of each one
	//CAPSULES/
	std::shared_ptr<utils::MixtureIngredient> m3 = gen.AddMixtureIngredient(utils::CAPSULE, quote_cs);
	m3->setDefaultMaterial(material_terrain);
	m3->setDefaultDensity(rho_g);
	m3->setDefaultSize(radius_g);
	//BOXES/
	std::shared_ptr<utils::MixtureIngredient> m4 = gen.AddMixtureIngredient(utils::BOX, quote_bx);
	m4->setDefaultMaterial(material_terrain);
	m4->setDefaultDensity(rho_g);
	m4->setDefaultSize(radius_g);
	//ROUNDED-CYLINDERS/
	std::shared_ptr<utils::MixtureIngredient> m5 = gen.AddMixtureIngredient(utils::ROUNDEDCYLINDER, quote_rc);
	m5->setDefaultMaterial(material_terrain);
	m5->setDefaultDensity(rho_g);
	m5->setDefaultSize(radius_g);
	//BOXES/
	std::shared_ptr<utils::MixtureIngredient> m6 = gen.AddMixtureIngredient(utils::BOX, quote_sbx);
	m6->setDefaultMaterial(material_terrain);
	m6->setDefaultDensity(rho_g);
	m6->setDefaultSize(radius_g);
	
	///////////////////////////////////////////-----THINGS TO DO-----//////////////////////////////
	// 3m X 20cm cylinder                  --> 15k particles
	// big box in the bottom               --> 
	// DVI no cohesion but                 --> spinning and rolling
	// see if you can get DEM-P without    --> 

	// mechanism, ChFunctionRecorder for data ....
	// bind together mech and sand
	///////////////////////////////////////////-----THINGS TO DO-----//////////////////////////////


	// Create particles in layers until reaching the desired number of particles
	double r = 1.01 * radius_g;
	ChVector<> hdims(hdimX/2 - r, hdimY/2 - r, 0);//W=.795, hdims object for the function gen.createObjectsBox accepts the	FULL dimensions in each direction:PAY ATTENTION
	ChVector<> center(0, 0, .5 * r);

	for (int il = 0; il < num_layers; il++) {
		gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
		center.z() += 2 * r;
		// shrink uniformly the upper layer
		hdims.x() -= 2 * r;
		hdims.y() -= 2 * r;
		// move the center abscissa by a 1*r(DISABLED FOR THE MOMENT) 
		center.x() += r/2 * pow(-1, il);

	}

	unsigned int num_particles = gen.getTotalNumBodies();
	std::cout << "Generated particles:  " << num_particles << std::endl;

//			//-------------------------Checking Repose Angle----------------------------//--------------------->>>>>>>>IT THROWS AN EXCEPTION ON system->Getbodylist->ClearForceVariables

			std::vector<std::shared_ptr<ChBody>> particlelist;
			auto original_bodylist  = system->Get_bodylist();
			//for (int i = 0; i < original_bodylist->size(); i++) { auto mbody = std::shared_ptr<ChBody>(original_bodylist[original_bodylist.begin()+i]);
			//															particlelist.push_back(mbody);			}
			for (auto body = original_bodylist->begin(); body != original_bodylist->end(); ++body) {
							auto mbody = std::shared_ptr<ChBody>(*body);
							particlelist.push_back(mbody);
							}
			particlelist.erase(particlelist.begin());// BECAUSE I KNOW 1st element is the container-->Note: for huge number of particles, 1 single element affects poorly the statistics
			std::vector<double> rs;
			std::vector<double> zs;
			double mean_rs;
			double dev_rs;
			for (auto body = particlelist.begin(); body != particlelist.end(); ++body) {
				double r = sqrt(pow((*body)->GetPos().x(), 2) + pow((*body)->GetPos().y(), 2));
				rs.push_back(r);
				double z = (*body)->GetPos().z();
				zs.push_back(z);				
			}
			// Computation of sum,mean and sumofsquares,stdev
			double sum = std::accumulate(rs.begin(), rs.end(), 0.0);
			double m = sum / rs.size();

			double accum = 0.0;
			std::for_each(rs.begin(), rs.end(), [&](const double d) {
				accum += (d - m) * (d - m);
			});

			double stdev = sqrt(accum / (rs.size() - 1));
			// Outliers detecting WIP
			std::vector<size_t> outliers;
			auto lambda = [&m,&stdev](int& i, double m, double stdev) {return i > m + 2 * stdev; };
			auto it = std::find_if(rs.begin(), rs.end(), [&](int i) {return i > m + 2 * stdev; });
			while (it != rs.end()) {
				outliers.emplace_back(std::distance(rs.begin(), it));
				it = std::find_if(std::next(it), rs.end(), [](int i) {return i > 5; });
			}
			// Deleting outliers
			int jcounter = 0;
			for (int i = 0; i < outliers.size(); i++) {
				rs.erase(rs.begin()+outliers[i]-jcounter);
				zs.erase(zs.begin() + outliers[i] - jcounter);
				jcounter++;
			}
			// Calculate the repose angle.
			auto maxr = rs[0];
			auto maxz = zs[0];
						// check both sizes are equal
			for (int i = 1; i < rs.size(); i++) {
				if (rs[i] > maxr)
					maxr = rs[i];
				if (zs[i] > maxz)
					maxz = zs[i];
			}

			////-------------------------------------------------------------------------//
			//GetLog() << maxr << "\n";
			////-------------------------------------------------------------------------//
	

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
		gl_window.SetCamera(ChVector<>(0, -1, 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
		gl_window.SetRenderMode(opengl::WIREFRAME);
	}
#endif

	// ---------------
	// Simulate system
	// ---------------

	double time_end = 1.50;
	double time_step = 1e-2;//1e-4;e-

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


// Write a file each 60 time_steps(tunable) to 
		//if (track_flatten) 	
		if (sim_frame == next_out_frame) {
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
		//}

		system->DoStepDynamics(time_step);
		sim_frame++;

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


	return 0;
}