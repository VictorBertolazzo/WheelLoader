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


#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace postprocess;

// --------------------------------------------------------------------------
using std::cout;
using std::endl;
// --------------------------------------------------------------------------


//-- CLASSES------------
int main(int argc, char** argv) {
	int num_threads = 4;


	// Get number of threads from arguments (if specified)
	if (argc > 1) {
		num_threads = std::stoi(argv[1]);
	}

	std::cout << "Requested number of threads: " << num_threads << std::endl;

	
    int binsX = 20;
    int binsY = 20;
    int binsZ = 10;
	std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;

	// --------------------------
	// Create the parallel system
	// --------------------------

	// Create system and set method-specific solver settings
	chrono::ChSystemParallel* system;

	//---Function Creation--//
	auto pressure = std::make_shared<ChFunction_Recorder>();
	for (int i = 0; i < 10; i++){ pressure->AddPoint(i, 1.); }

	// Assuming pres_funct is of type ChFunction_Recorder
	double xmin = 0;
	double xmax = 0;
	pressure->Estimate_x_range(xmin, xmax);// double& x

	ChFunction_Recorder speed;
	double h = 1;//0
	double p = 0;
	//for (int i = 0; i < 101; i++){
	//	double h = i*(xmax - xmin) / 100;

		ChFunction_Integrate fun;
		fun.Set_order(2); fun.Set_x_start(xmin + p); fun.Set_x_end(xmin + h);
		fun.Set_num_samples(10); fun.Set_C_start(0.);
		fun.Set_fa(pressure);
		fun.ComputeIntegral();
		speed.AddPoint(xmin + h, fun.Get_y(xmin + p));//I'd want to access to array_x(row last, column 0) member
		p = h;
	//}

	// Gnuplot
	ChGnuPlot mplot("__tmp_gnuplot_4.gpl");
	mplot.SetGrid();
	mplot.Plot(speed, "Actuator Displacement Function", " with lines lt -1 lc rgb'#00AAEE' ");


	// Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master
	{ std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

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



	std::cout << std::endl;
	std::cout << "Simulation time: " << cum_sim_time << std::endl;
	std::cout << "    Broadphase:  " << cum_broad_time << std::endl;
	std::cout << "    Narrowphase: " << cum_narrow_time << std::endl;
	std::cout << "    Solver:      " << cum_solver_time << std::endl;
	std::cout << "    Update:      " << cum_update_time << std::endl;
	std::cout << std::endl;


	return 0;
}