// Vehicle Test
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
#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/utils/ChUtilsGenerators.h"



#include "chrono_models/vehicle/generic/Generic_Chassis.h"
#include "chrono_models/vehicle/generic/Generic_Wheel.h"
#include "chrono_models/vehicle/generic/Generic_Driveline2WD.h"//4wd
#include "chrono_models/vehicle/generic/Generic_BrakeSimple.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"

#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"


#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"


#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

//#define USE_PENALTY
using namespace chrono;
using namespace chrono::vehicle;

class CenterPivot : public ChSteering {
	public:

		CenterPivot(const std::string& name) : ChSteering(name) {}
		~CenterPivot() {}



		const double m_maxAngle = 40 * CH_C_DEG_TO_RAD; //40° rotation;
		const double CenterPivot::m_mass= 10; //total mass;

		void CenterPivot::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
			const ChVector<>& location,
			const ChQuaternion<>& rotation) override
		{
			// Express the steering reference frame in the absolute coordinate system.
			ChFrame<> steering_to_abs(location, rotation);
			steering_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
			// Create and initialize the steering link body
			ChVector<> link_local(0, 0, 0);
			ChVector<> link_abs = steering_to_abs.TransformPointLocalToParent(link_local);

			m_pivot = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
			m_pivot->SetNameString(m_name + "_pivot");
			m_pivot->SetPos(link_abs);
			m_pivot->SetRot(steering_to_abs.GetRot());
			chassis->GetSystem()->AddBody(m_pivot);

			m_pivoting = std::make_shared<ChLinkEngine>();
			m_pivoting->SetNameString(m_name + "_pivoting");
			m_pivoting->Initialize(chassis, m_pivot, ChCoordsys<>(link_abs, steering_to_abs.GetRot())); // it rotates about z axis of steering frame
			m_pivoting->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK); 
			m_pivoting->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION); 
			chassis->GetSystem()->AddLink(m_pivoting);

			// Create and initialize the RIGHT steering link body
			ChVector<> r_link_local(-.2, -1.0, 0);
			ChVector<> r_link_abs = steering_to_abs.TransformPointLocalToParent(r_link_local);
			// Create and initialize the LEFT steering link body
			ChVector<> l_link_local(-.2, 1.0, 0);
			ChVector<> l_link_abs = steering_to_abs.TransformPointLocalToParent(l_link_local);
			// Create and initialize the revolute joint btw links and pivot
			ChVector<> rev_local(-.2, 0., 0);
			ChVector<> rev_abs = steering_to_abs.TransformPointLocalToParent(rev_local);

			m_r_link = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
			m_r_link->SetNameString(m_name + "_r_link");
			m_r_link->SetPos(r_link_abs);
			m_r_link->SetRot(steering_to_abs.GetRot());
			chassis->GetSystem()->AddBody(m_r_link);

			m_l_link = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
			m_l_link->SetNameString(m_name + "_l_link");
			m_l_link->SetPos(l_link_abs);
			m_l_link->SetRot(steering_to_abs.GetRot());
			chassis->GetSystem()->AddBody(m_l_link);

			m_r_rev = std::make_shared<ChLinkLockRevolute>();
			m_r_rev->Initialize(m_pivot, m_r_link, ChCoordsys<>(rev_abs+ChVector<>(.0,.0,.1),steering_to_abs.GetRot()));
			chassis->GetSystem()->AddLink(m_r_rev);

			m_l_rev = std::make_shared<ChLinkLockRevolute>();
			m_l_rev->Initialize(m_pivot, m_l_link, ChCoordsys<>(rev_abs + ChVector<>(.0, .0, -.1), steering_to_abs.GetRot()));
			chassis->GetSystem()->AddLink(m_l_rev);


		}

		void CenterPivot::Synchronize(double time, double steering) override {
			double angle = steering * GetMaxAngle();
			if (auto fun = std::dynamic_pointer_cast<ChFunction_Const>(m_pivoting->Get_rot_funct()))
				fun->Set_yconst(angle);

		}
		double CenterPivot::GetMaxAngle() const { return m_maxAngle; }

		void CenterPivot::AddVisualizationAssets(chrono::vehicle::VisualizationType vis) override {
			if (vis == chrono::vehicle::VisualizationType::NONE)
				return;
			auto piv1_asset = std::make_shared<ChCylinderShape>(); 
			piv1_asset->GetCylinderGeometry().rad = .075; 
			piv1_asset->GetCylinderGeometry().p1 = ChVector<>(.0, -.2, 0.);
			piv1_asset->GetCylinderGeometry().p2 = VNULL;
			m_pivot->AddAsset(piv1_asset);
			auto piv2_asset = std::make_shared<ChCylinderShape>(); 
			piv2_asset->GetCylinderGeometry().rad = .075; 
			piv2_asset->GetCylinderGeometry().p1 =  ChVector<>(-.2, 0, 0.); 
			piv2_asset->GetCylinderGeometry().p2 = VNULL;
			m_pivot->AddAsset(piv2_asset);

			auto rl_asset = std::make_shared<ChCylinderShape>(); rl_asset->GetCylinderGeometry().rad = .05; 
			rl_asset->GetCylinderGeometry().p1 = ChVector<>(0, +.5, 0);
			rl_asset->GetCylinderGeometry().p2 = ChVector<>(0, -.5, 0);
			m_r_link->AddAsset(rl_asset);
			auto ll_asset = std::make_shared<ChCylinderShape>(); 
			ll_asset->GetCylinderGeometry().rad = .05; 
			ll_asset->GetCylinderGeometry().p1 = ChVector<>(0, +.5, 0);
			ll_asset->GetCylinderGeometry().p2 = ChVector<>(0 , -.5, 0); 
			m_l_link->AddAsset(ll_asset);
					
		}

		void CenterPivot::LogConstraintViolations() {
			// Engine joint
			{
				ChMatrix<>* C = m_pivoting->GetC();
				GetLog() << "Engine           ";
				GetLog() << "  " << C->GetElement(0, 0) << "  ";
				GetLog() << "  " << C->GetElement(1, 0) << "  ";
				GetLog() << "  " << C->GetElement(2, 0) << "  ";
				GetLog() << "  " << C->GetElement(3, 0) << "  ";
				GetLog() << "  " << C->GetElement(4, 0) << "\n";
			}

			// Revolute Joints
			{
			ChMatrix<>* C = m_l_rev->GetC();
			GetLog() << "Left Rev            ";
			GetLog() << "  " << C->GetElement(0, 0) << "  ";
			GetLog() << "  " << C->GetElement(1, 0) << "  ";
			GetLog() << "  " << C->GetElement(2, 0) << "  ";
			GetLog() << "  " << C->GetElement(3, 0) << "  ";
			GetLog() << "  " << C->GetElement(4, 0) << "\n";
		}
			{
				ChMatrix<>* C = m_r_rev->GetC();
				GetLog() << "Right Rev            ";
				GetLog() << "  " << C->GetElement(0, 0) << "  ";
				GetLog() << "  " << C->GetElement(1, 0) << "  ";
				GetLog() << "  " << C->GetElement(2, 0) << "  ";
				GetLog() << "  " << C->GetElement(3, 0) << "  ";
				GetLog() << "  " << C->GetElement(4, 0) << "\n";
			}
		}

		void CenterPivot::RemoveVisualizationAssets(){
			m_pivot->GetAssets().clear();
			m_l_link->GetAssets().clear();
			m_r_link->GetAssets().clear();

		}

		std::shared_ptr<ChBody> CenterPivot::GetSteeringLink() const { return std::shared_ptr<ChBody>(); }
		std::shared_ptr<ChBody> CenterPivot::GetSteeringLink(chrono::vehicle::VehicleSide side){
			if (side == chrono::vehicle::VehicleSide::LEFT){
				return m_l_link;
			}
			else
			{
				return m_r_link;
			}
		}

		double CenterPivot::GetMass() const { return m_mass; }

protected:
	std::shared_ptr<ChBody> m_pivot;
	std::shared_ptr<ChBody> m_l_link;
	std::shared_ptr<ChBody> m_r_link;
	std::shared_ptr<ChLinkEngine> m_pivoting;
	std::shared_ptr<ChLinkLockRevolute> m_l_rev;
	std::shared_ptr<ChLinkLockRevolute> m_r_rev;
	//static const double m_maxAngle;


};

class RigidAxle : public ChSuspension{
public:

	RigidAxle(const std::string& name) : ChSuspension(name) {}
	~RigidAxle() {}
/*
	bool IsSteerable(){}
	bool IsIndependent(){}
	void Initialize(){}
	void AddVisualizationAssets() override;
	void RemoveVisualizationAssets() override;
	double GetMass() {}
	double getSpindleRadius() {}
	double getSpindleWidth() {}*/


};
bool render = true;
int main(int argc, char* argv[]) {
	// --------------------------
	// Create system and set method-specific solver settings
	// --------------------------

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

	// Set number of threads
	system->SetParallelThreadNumber(20);
	CHOMPfunctions::SetNumThreads(20);

	// Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master
	{ std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

	//////////////////////////////////////////////////////////////////////////////////
	auto test = std::make_shared<CenterPivot>("Test");
	auto chass = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
	chass->SetBodyFixed(true);
	system->Add(chass);
	test->Initialize(chass, VNULL, QUNIT);
	std::shared_ptr<ChBody> body = test->GetSteeringLink(LEFT);
	std::shared_ptr<ChBody> corpo = test->GetSteeringLink(RIGHT);
	test->AddVisualizationAssets(VisualizationType::MESH);

#ifdef CHRONO_OPENGL
	if (render) {
		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "Sandpile + zBar", system);
		gl_window.SetCamera(ChVector<>(5, 8, 0), ChVector<>(3.5, 0, 0), ChVector<>(0, 0, 1));
		gl_window.SetRenderMode(opengl::WIREFRAME);
	}
#endif


	while (system->GetChTime() < 50.00) {
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

	return 0;
}