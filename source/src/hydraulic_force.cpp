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

#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/motion_functions/ChFunction_Integrate.h"
#include "chrono/motion_functions/ChFunction_Base.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/motion_functions/ChFunction_Sine.h"


#include "chrono_postprocess/ChGnuPlot.h"


using namespace chrono;
using namespace postprocess;

// --------------------------------------------------------------------------
using std::cout;
using std::endl;
// --------------------------------------------------------------------------
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


	// Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master 
	{ std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

	
	return 0;
}