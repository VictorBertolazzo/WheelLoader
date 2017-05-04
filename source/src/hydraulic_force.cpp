// This example receives in input the actuator data and elaborates them feeding the latter.
// Moreover, it overloads the classical ChLinkLinActuator class in a way to accept nor only displacement but also
//			force inputs.
// It must be merged with test_HYDR_actuator.cpp
// Victor Bertolazzo
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
#include "chrono/assets/ChPointPointDrawing.h"


#include "chrono/physics/ChLinkLinActuator.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
using namespace chrono;
using namespace postprocess;
#include "chrono/physics/ChLinkMarkers.h"
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

// Class implemented to compute hydraulic force
class myHYDRforce {
	public:
	virtual ~myHYDRforce() {}
		

	virtual double operator()(double time,
							double pressureH,
							double areaH)
	{
		double force = pressureH * areaH; //pH*aH-pR*aR, formula for piston
		return force;
	}

};
// Class implemented to set up a hydraulic/data input force between two markers
class myHYDRactuator : public ChLinkMarkers{

public:
	myHYDRactuator() : m_force(0), m_force_fun(NULL), m_pressureH(NULL), m_pressureR(NULL), m_areaH(9), m_areaR(10) {};
	myHYDRactuator(const myHYDRactuator& other);
	virtual ~myHYDRactuator() {}


	void SetAreaH(double areaH){ m_areaH = areaH; }
	void SetAreaR(double areaR){ m_areaR = areaR; }
	double GetAreaH(){ return m_areaH; }
	double GetAreaR(){ return m_areaR; }

	void Set_HYDRforce(myHYDRforce* force) { m_force_fun = force; }
	void Set_PressureH(ChFunction* pressureH){ m_pressureH = pressureH; }
	void Set_PressureR(ChFunction* pressureR){ m_pressureR = pressureR; }


	void UpdateForces(double time)override{

		// Allow the base class to update itself (possibly adding its own forces)
		ChLinkMarkers::UpdateForces(time);

		// Get the pressure information
		double pressureH = m_pressureH->Get_y(time);
		// Invoke the provided functor to evaluate force
		m_force = m_force_fun ? (*m_force_fun)(time, pressureH, m_areaH) : 0;

		// Add to existing force.
		C_force += m_force * relM.pos.GetNormalized();

	}
protected:
	myHYDRforce* m_force_fun;  ///< functor for force calculation
	double m_areaH = 10.;
	double m_areaR = 9.;
	double m_force;  ///< functor for force calculation
	ChFunction* m_pressureH;
	ChFunction* m_pressureR;

};

int main(int argc, char** argv) {
	int num_threads = 4;


	// Get number of threads from arguments (if specified)
	if (argc > 1) {
		num_threads = std::stoi(argv[1]);
	}

	std::cout << "Requested number of threads: " << num_threads << std::endl;

	// Preliminary Settings, catch the input file or write a sine function

	std::vector<TimeSeries> input;
	const std::string& act_input = "../data/actuator_input.txt";
	ChFunction_Recorder pressure;
	if (mode == INFILE){
		ReadFile(act_input, input);
		for (int i = 0; i < input.size(); i++){
					// scalar gain to be observable in simulation
			pressure.AddPoint(input[i].mt, 10 *input[i].mv);}
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
	ChSystemParallelSMC* sys = new ChSystemParallelSMC;
	sys->GetSettings()->solver.contact_force_model = ChSystemDEM::Hertz;
	sys->GetSettings()->solver.tangential_displ_mode = ChSystemDEM::TangentialDisplacementModel::OneStep;
	sys->GetSettings()->solver.use_material_properties = use_mat_properties;
	system = sys;
#else
	ChSystemParallelNSC* sys = new ChSystemParallelNSC;
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

	// Rendering Settings
#ifdef CHRONO_OPENGL
	if (render) {
		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "Settling test", system);
		gl_window.SetCamera(ChVector<>(9., 0., 4.), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
		gl_window.SetRenderMode(opengl::WIREFRAME);
	}
#endif // CHRONO_OPENGL

	// Set number of threads
	system->SetParallelThreadNumber(20);
	CHOMPfunctions::SetNumThreads(20);
	// Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master 
	{ std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

	// Create the ground
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
	utils::AddBoxGeometry(container.get(), ChVector<>(10, 10, 10 ), ChVector<>(0, 0, 0 ),
		ChQuaternion<>(1, 0, 0, 0), true);
	container->GetCollisionModel()->BuildModel();



	// Create the to bodies: they'll be connected by a ChLinkSpringCB-like element
	              //or a ChlinkLinActuator(depending on USE_DISPLACEMENT flag)
	// An alternative would be overload ChLinkLinActuator with a UpdateForce method
	auto bodyA = std::shared_ptr<ChBody>(system->NewBody()); bodyA->SetBodyFixed(true);
	bodyA->SetCollide(true);
	system->Add(bodyA);
	bodyA->SetPos(ChVector<>(0,0,.25));
	utils::AddBoxGeometry(bodyA.get(), ChVector<>(.2, .2, .2), ChVector<>(0, 0, 0 ),
		ChQuaternion<>(1, 0, 0, 0), true);
	bodyA->GetCollisionModel()->BuildModel();
	auto bodyB = std::shared_ptr<ChBody>(system->NewBody()); bodyB->SetBodyFixed(false);
	bodyB->SetCollide(true);
	system->Add(bodyB);
	bodyB->SetPos(ChVector<>(5., 0, .25));
	utils::AddBoxGeometry(bodyB.get(), ChVector<>(.2, .2, .2), ChVector<>(0, 0, 0),
		ChQuaternion<>(1, 0, 0, 0), true);
	bodyB->GetCollisionModel()->BuildModel();

	auto prismatic = std::make_shared<ChLinkLockPrismatic>();
	prismatic->Initialize(bodyA, bodyB, ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngY(CH_C_PI / 4)));
	system->AddLink(prismatic);

	//Setup the function--Old
	
		auto pres_function = std::make_shared<ChFunction_Recorder>();//shared_ptr gives memory acces violation in AddPoint member
		if (mode == INFILE){
			ReadFile(act_input, input);

			std::cout << input[4].mv << std::endl;

			for (int i = 0; i < input.size(); i++){
				pres_function->AddPoint(input[i].mt, input[i].mv);
			}
		}
	

//#define USE_DISPLACEMENT
	// Choose between the two type of connections: check distances and orientations
#ifdef USE_DISPLACEMENT
	auto linAB = std::make_shared<ChLinkLinActuator>(); 
	linAB->Initialize(bodyA, bodyB, false, ChCoordsys<>(VNULL, Q_from_AngY(CH_C_PI / 4)), ChCoordsys<>(bodyB->GetPos(), Q_from_AngY(CH_C_PI / 4)));
	linAB->Set_lin_offset(5.);//safe
	system->AddLink(linAB);
	linAB->Set_dist_funct(pres_function);
	//	// Asset for the linear actuator--NOT WORKING WITH OPENGL
	//	auto bp_asset = std::make_shared<ChPointPointSegment>();				//asset
	//	linAB->AddAsset(bp_asset);
	//
#else
	myHYDRforce force;
	auto linAB = std::make_shared<myHYDRactuator>();
	linAB->Initialize(bodyA, bodyB, bodyB->GetCoord());
	linAB->Set_HYDRforce(&force);
	linAB->Set_PressureH(&pressure);
	system->AddLink(linAB);
	// Attach a visualization asset.
	linAB->AddAsset(std::make_shared<ChPointPointSpring>(0.05, 80, 15));	
#endif


	// Simulation

	while (system->GetChTime() < 5.) {

		system->DoStepDynamics(.001);
		std::cout << bodyB->GetPos().x() << std::endl;

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