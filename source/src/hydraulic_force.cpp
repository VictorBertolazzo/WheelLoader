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


#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/motion_functions/ChFunction_Integrate.h"
#include "chrono/motion_functions/ChFunction_Base.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/motion_functions/ChFunction_Sine.h"


#include "chrono_postprocess/ChGnuPlot.h"

#include "chrono/physics/ChLinkLinActuator.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace postprocess;

// --------------------------------------------------------------------------
using std::cout;
using std::endl;
// -------------------TIME SERIES STRUCTURE----------------------------
struct TimeSeries {
	TimeSeries() {}
	TimeSeries(float t, float v)
		: mt(t), mv(v) {}
	float mt; float mv; 
};
// -------------------READ FILE FUNCTION----------------------------
void ReadFile(const std::string& filename, std::vector<TimeSeries>& profile) {
	std::ifstream ifile(filename.c_str());
	std::string line;

	while (std::getline(ifile, line)) {
		std::istringstream iss(line);
		float ttime, vvalue;
		iss >> ttime >> vvalue ;
		if (iss.fail())
			break;
		profile.push_back(TimeSeries(ttime, vvalue));
	}
	ifile.close();



}
// -------------------ENUMERATOR ----------------------------
enum FunctionSettingMode {INFILE , INPLACE};
FunctionSettingMode mode = INFILE;
bool render = true;

int main(int argc, char** argv) {
	int num_threads = 4;


	// Get number of threads from arguments (if specified)
	if (argc > 1) {
		num_threads = std::stoi(argv[1]);
	}

	std::cout << "Requested number of threads: " << num_threads << std::endl;

	std::vector<TimeSeries> input;
	const std::string& act_input = "../data/actuator_input.txt";
	ChFunction_Recorder pressure;
	if (mode == INFILE){
		ReadFile(act_input, input);
		//ChFunction_Recorder pressure;
		for (int i = 0; i < input.size(); i++){
			pressure.AddPoint(input[i].mt, input[i].mv);}
						}
	else if(mode == INPLACE){
		ChFunction_Sine pressure;
		pressure.Set_amp(1.0);
		pressure.Set_freq(1.0);
							}
	else { std::cout << "No valid input mode" << std::endl; }

	// Gnuplot--IT DOESN'T WORK WITH ChFunction_Sine or others.
	ChGnuPlot mplot("__tmp_gnuplot_4.gpl");
	mplot.SetGrid();
	mplot.Plot(pressure, "Pressure Function", " with lines lt -1 lc rgb'#00AAEE' ");

			//----------SYSTEM------------
	chrono::ChSystemParallel* system;

#ifdef USE_PENALTY
	ChSystemParallelDEM* sys = new ChSystemParallelDEM;
	sys->GetSettings()->solver.contact_force_model = ChSystemDEM::Hertz;
	sys->GetSettings()->solver.tangential_displ_mode = ChSystemDEM::TangentialDisplacementModel::OneStep;
	sys->GetSettings()->solver.use_material_properties = use_mat_properties;
	system = sys;
#else
	ChSystemParallelDVI* sys = new ChSystemParallelDVI;
	sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
	sys->GetSettings()->solver.max_iteration_normal = 0;
	sys->GetSettings()->solver.max_iteration_sliding = 200;
	sys->GetSettings()->solver.max_iteration_spinning = 0;
	sys->GetSettings()->solver.alpha = 0;
	sys->GetSettings()->solver.contact_recovery_speed = -1;
	sys->GetSettings()->collision.collision_envelope = 0.01;
	sys->ChangeSolverType(SolverType::APGD);
	system = sys;

#endif // USE_PENALTY

	system->Set_G_acc(ChVector<>(0, 0, -9.81));
	system->GetSettings()->perform_thread_tuning = false;
	system->GetSettings()->solver.use_full_inertia_tensor = false;
	system->GetSettings()->solver.tolerance = 0.1;
	system->GetSettings()->solver.max_iteration_bilateral = 100;
	system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
	system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
#ifdef CHRONO_OPENGL
	if (render) {
		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "Settling test", system);
		gl_window.SetCamera(ChVector<>(1., 1., +.1), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
		gl_window.SetRenderMode(opengl::WIREFRAME);
	}
#endif

	// Set number of threads
	system->SetParallelThreadNumber(20);
	CHOMPfunctions::SetNumThreads(20);
	// Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master 
	{ std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

	auto container = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(container);
	container->SetIdentifier(-1);
	container->SetMass(1000.0);
	container->SetPos(ChVector<>(0., 0., -10));
	container->SetBodyFixed(true);
	container->SetCollide(true);
	// it's not the problem for using all iterations
	//container->SetMaterialSurface(material_terrain);
	container->GetCollisionModel()->ClearModel();
	// Bottom box
	utils::AddBoxGeometry(container.get(), ChVector<>(10, 10, 9 ), ChVector<>(0, 0, 0 ),
		ChQuaternion<>(1, 0, 0, 0), true);
	container->GetCollisionModel()->BuildModel();




	auto bodyA = std::shared_ptr<ChBody>(system->NewBody()); bodyA->SetBodyFixed(true);
	system->Add(bodyA);
	bodyA->SetPos(ChVector<>(0,0,0));
	utils::AddBoxGeometry(bodyA.get(), ChVector<>(.2, .2, .2), ChVector<>(0, 0, 0 ),
		ChQuaternion<>(1, 0, 0, 0), true);
	bodyA->GetCollisionModel()->BuildModel();
	auto bodyB = std::shared_ptr<ChBody>(system->NewBody()); bodyB->SetBodyFixed(true);
	system->Add(bodyB);
	bodyB->SetPos(ChVector<>(1., 0, 0));
	utils::AddBoxGeometry(bodyB.get(), ChVector<>(.2, .2, .2), ChVector<>(0, 0, 0),
		ChQuaternion<>(1, 0, 0, 0), true);
	bodyB->GetCollisionModel()->BuildModel();

	auto linAB = std::make_shared<ChLinkLinActuator>(); system->AddLink(linAB);
	linAB->Initialize(bodyA,bodyB,ChCoordsys<>(ChVector<>(.5,0,0),chrono::Q_from_AngAxis(CH_C_PI_2,VECT_Y)));
	linAB->Set_lin_offset(10.);//safe
	
	// Setup the function
	auto pres_function = std::shared_ptr<ChFunction_Recorder>();
	if (mode == INFILE){
		ReadFile(act_input, input);
		//ChFunction_Recorder pressure;
		for (int i = 0; i < input.size(); i++){
			pres_function->AddPoint(input[i].mt, input[i].mv);
		}
	}

	linAB->Set_dist_funct(pres_function);


	while (system->GetChTime() < 50.) {

		system->DoStepDynamics(.01);

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

	return 0;
}