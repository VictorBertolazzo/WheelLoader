// Vehicle Test
// Victor Bertolazzo
	//  Discriminate parallel or serial simulation.
//    #define USE_PARALLEL
	#define USE_SEQUENTIAL
	// Use SMC/NSC contact method.
//#define USE_PENALTY
	// Additional features
//#define TEST_ENABLE 
//#define USE_SEP_VEH
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


#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"

#include "chrono_models/vehicle/generic/Generic_Chassis.h"
#include "chrono_models/vehicle/generic/Generic_Wheel.h"
#include "chrono_models/vehicle/generic/Generic_Driveline2WD.h"//4wd
#include "chrono_models/vehicle/generic/Generic_BrakeSimple.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Driveline4WD.h"

#include "chrono_models/vehicle/generic/Generic_SimplePowertrain.h"
#include "chrono_models/vehicle/generic/Generic_RigidTire.h"

#include "chrono_models/vehicle/generic/Generic_RackPinion.h"
#include "chrono_models/vehicle/generic/Generic_DoubleWishbone.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/ChSteering.h"

#include "chrono_vehicle/wheeled_vehicle/ChSuspension.h"


#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"

// If Irrlicht support is available...
#ifdef CHRONO_IRRLICHT
// ...include additional headers
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

// ...and specify whether the demo should actually use Irrlicht
#define USE_IRRLICHT
#endif


#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;
using namespace chrono::vehicle::hmmwv;


// Concrete class defining another steering mechanism(so called center pivot)
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
			ChVector<> link_local(0., .0, -0.02);
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
			ChVector<> r_link_local(-.10, -0.45, -0.02);
			r_link_abs = steering_to_abs.TransformPointLocalToParent(r_link_local);
			// Create and initialize the LEFT steering link body
			ChVector<> l_link_local(-.10, 0.45, -0.02);
			l_link_abs = steering_to_abs.TransformPointLocalToParent(l_link_local);
			// Create and initialize the revolute joint btw links and pivot
			ChVector<> rev_local(-.1, .0, -0.02);
			rev_abs = steering_to_abs.TransformPointLocalToParent(rev_local);

			m_r_link = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
			m_r_link->SetNameString(m_name + "_r_link");
			m_r_link->SetPos(rev_abs);
			m_r_link->SetRot(steering_to_abs.GetRot());
			chassis->GetSystem()->AddBody(m_r_link);

			m_l_link = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
			m_l_link->SetNameString(m_name + "_l_link");
			m_l_link->SetPos(rev_abs);
			m_l_link->SetRot(steering_to_abs.GetRot());
			chassis->GetSystem()->AddBody(m_l_link);

			m_r_rev = std::make_shared<ChLinkLockRevolute>();
			m_r_rev->Initialize(m_pivot, m_r_link, ChCoordsys<>(rev_abs+ChVector<>(.0,.0,.01),steering_to_abs.GetRot()));
			chassis->GetSystem()->AddLink(m_r_rev);

			m_l_rev = std::make_shared<ChLinkLockRevolute>();
			m_l_rev->Initialize(m_pivot, m_l_link, ChCoordsys<>(rev_abs + ChVector<>(.0, .0, -.01), steering_to_abs.GetRot()));
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
			piv1_asset->GetCylinderGeometry().rad = .015; 
			piv1_asset->GetCylinderGeometry().p1 = ChVector<>(.0, -.2, 0.);
			piv1_asset->GetCylinderGeometry().p2 = VNULL;
			m_pivot->AddAsset(piv1_asset);
			auto piv2_asset = std::make_shared<ChCylinderShape>(); 
			piv2_asset->GetCylinderGeometry().rad = .015; 
			piv2_asset->GetCylinderGeometry().p1 =  ChVector<>(-.2, 0, 0.); 
			piv2_asset->GetCylinderGeometry().p2 = VNULL;
			m_pivot->AddAsset(piv2_asset);

			auto rl_asset = std::make_shared<ChCylinderShape>(); rl_asset->GetCylinderGeometry().rad = .01; 
			rl_asset->GetCylinderGeometry().p1 = m_r_link->GetFrame_COG_to_abs().GetInverse() * (rev_abs + ChVector<>(.0, .0, .01));
			rl_asset->GetCylinderGeometry().p2 = m_r_link->GetFrame_COG_to_abs().GetInverse() * (r_link_abs);;
			m_r_link->AddAsset(rl_asset);
			auto ll_asset = std::make_shared<ChCylinderShape>(); 
			ll_asset->GetCylinderGeometry().rad = .01; 
			ll_asset->GetCylinderGeometry().p1 = m_l_link->GetFrame_COG_to_abs().GetInverse() * (rev_abs + ChVector<>(.0, .0, -.01));
			ll_asset->GetCylinderGeometry().p2 = m_l_link->GetFrame_COG_to_abs().GetInverse() * (l_link_abs);
			m_l_link->AddAsset(ll_asset);
			
			auto col = std::make_shared<ChColorAsset>(0.7f,0.0f,0.0f);
			/*m_r_link->AddAsset(col);
			m_l_link->AddAsset(col);
*/
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

		std::shared_ptr<ChBody> GetSteeringLink() const { return std::shared_ptr<ChBody>(); }
		std::shared_ptr<ChBody> GetSteeringLink(chrono::vehicle::VehicleSide side){
			if (side == chrono::vehicle::VehicleSide::LEFT){
				return m_l_link;
			}
			else
			{
				return m_r_link;
			}
		}
		std::shared_ptr<ChBody> GetSteeringLinks(){}
		double CenterPivot::GetMass() const { return m_mass; }
		ChVector<> rev_abs;
		ChVector<> r_link_abs;
		ChVector<> l_link_abs;

protected:
	std::shared_ptr<ChBody> m_pivot;
	std::shared_ptr<ChBody> m_l_link;
	std::shared_ptr<ChBody> m_r_link;
	std::shared_ptr<ChLinkEngine> m_pivoting;
	std::shared_ptr<ChLinkLockRevolute> m_l_rev;
	std::shared_ptr<ChLinkLockRevolute> m_r_rev;
	//static const double m_maxAngle;


};

// Concrete class defining the WL no-suspension mechanism
class RigidAxle : public ChSuspension{
public:

	RigidAxle(const std::string& name) : ChSuspension(name) {}
	~RigidAxle() {}

	bool IsSteerable() const final override { return false; }
	bool IsIndependent() const final override { return false; }

	void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
		const ChVector<>& location,
		std::shared_ptr<ChBody> tierod_body,
		double left_ang_vel,
		double right_ang_vel) {

		// Unit vectors for orientation matrices.
		ChVector<> u;ChVector<> v;ChVector<> w;ChMatrix33<> rot;

		// Express the suspension reference frame in the absolute coordinate system.
		ChFrame<> suspension_to_abs(location);
		suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

		// Transform all hardpoints to absolute frame.
		m_pointsL.resize(NUM_POINTS);
		m_pointsR.resize(NUM_POINTS);
		for (int i = 0; i < NUM_POINTS; i++) {
			ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
			m_pointsL[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
			rel_pos.y() = -rel_pos.y();
			m_pointsR[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
		}
		// Initialize left and right sides.
		InitializeSide(LEFT, chassis, tierod_body, m_pointsL, left_ang_vel);
		InitializeSide(RIGHT, chassis, tierod_body, m_pointsR, right_ang_vel);

	}

	void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
		const ChVector<>& location,
		std::shared_ptr<ChBody> tierod_body_left,
		std::shared_ptr<ChBody> tierod_body_right,
		double left_ang_vel,
		double right_ang_vel) {

		// Unit vectors for orientation matrices.
		ChVector<> u; ChVector<> v; ChVector<> w; ChMatrix33<> rot;

		// Express the suspension reference frame in the absolute coordinate system.
		ChFrame<> suspension_to_abs(location);
		suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

		// Transform all hardpoints to absolute frame.
		m_pointsL.resize(NUM_POINTS);
		m_pointsR.resize(NUM_POINTS);
		for (int i = 0; i < NUM_POINTS; i++) {
			ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
			m_pointsL[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
			rel_pos.y() = -rel_pos.y();
			m_pointsR[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
		}
		// Initialize left and right sides.
		InitializeSide(LEFT, chassis, tierod_body_left, m_pointsL, left_ang_vel);
		InitializeSide(RIGHT, chassis, tierod_body_right, m_pointsR, right_ang_vel);

	}

	void InitializeSide(VehicleSide side,
		std::shared_ptr<ChBodyAuxRef> chassis,
		std::shared_ptr<ChBody> tierod_body,
		const std::vector<ChVector<> >& points,
		double ang_vel) {
		std::string suffix = (side == LEFT) ? "_L" : "_R";
		// Unit vectors for orientation matrices.
		ChVector<> u;ChVector<> v;ChVector<> w;ChMatrix33<> rot;

		// Chassis orientation (expressed in absolute frame)
		// Recall that the suspension reference frame is aligned with the chassis.
		ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

		// Create and initialize knuckle body (same orientation as the chassis)
		m_knuckle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
		m_knuckle[side]->SetNameString(m_name + "_knuckle" + suffix);
		m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
		m_knuckle[side]->SetRot(chassisRot);
			m_knuckle[side]->SetMass(.01);	//m_knuckle[side]->SetMass(getKnuckleMass());
			m_knuckle[side]->SetInertiaXX(.01);//m_knuckle[side]->SetInertiaXX(getKnuckleInertia());
		chassis->GetSystem()->AddBody(m_knuckle[side]);

		// Create and initialize spindle body (same orientation as the chassis)
		m_spindle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
		m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
		m_spindle[side]->SetPos(points[SPINDLE]);
		m_spindle[side]->SetRot(chassisRot);
		m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
			m_spindle[side]->SetMass(.01);
			m_spindle[side]->SetInertiaXX(.01);
		chassis->GetSystem()->AddBody(m_spindle[side]);

		////// Create and initialize the tierod body.
		////m_tierod[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
		////m_tierod[side]->SetNameString(m_name + "_tierod" + suffix);
		////m_tierod[side]->SetPos((points[TIEROD_K]) / 2 + (points[TIEROD_S]) / 2);
		////m_tierod[side]->SetRot(chassis->GetFrame_REF_to_abs().GetRot());
		////	m_tierod[side]->SetMass(.01);
		////	m_tierod[side]->SetInertiaXX(.01);
		////chassis->GetSystem()->AddBody(m_tierod[side]);

					//---NEW--- Create and initialize the tierod distance constraint between chassis and upright.
		m_distTierod[side] = std::make_shared<ChLinkDistance>();
		m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
		m_distTierod[side]->Initialize(tierod_body, m_knuckle[side], false, points[TIEROD_C], points[TIEROD_U]);
		chassis->GetSystem()->AddLink(m_distTierod[side]);

		////// Create and initialize the joint between knuckle and tierod (one side has universal, the other has spherical)
		////		// TO DO.
		////m_sphericalTierod[side] = std::make_shared<ChLinkLockSpherical>();
		////m_sphericalTierod[side]->SetNameString(m_name + "_sphericalTierod" + suffix);
		////m_sphericalTierod[side]->Initialize(m_tierod[side], m_knuckle[side], ChCoordsys<>(points[TIEROD_K], QUNIT));
		////chassis->GetSystem()->AddLink(m_sphericalTierod[side]);

		// Create and initialize the revolute joint between --CHASSIS-- and knuckle.
			// Using CompositeInertia() function later on.
		// Determine the joint orientation matrix from the hardpoint locations by
		// constructing a rotation matrix with the z axis along the joint direction.
		w = points[KNUCKLE_L] - points[KNUCKLE_U];
		w.Normalize();
		u = Vcross(points[KNUCKLE_U] - points[SPINDLE], points[KNUCKLE_L] - points[SPINDLE]);
		u.Normalize();
		v = Vcross(w, u);
		rot.Set_A_axis(u, v, w);

		m_revoluteKingpin[side] = std::make_shared<ChLinkLockRevolute>();
		m_revoluteKingpin[side]->SetNameString(m_name + "_revoluteKingpin" + suffix);
		m_revoluteKingpin[side]->Initialize(
			chassis, m_knuckle[side], ChCoordsys<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.Get_A_quaternion()));
		chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

		////// Create and initialize the spherical joint between steering mechanism and tierod.
		////	// Even tierod body has two left/right part for center pivot.
		////m_sphericalSteering[side] = std::make_shared<ChLinkLockSpherical>();
		////m_sphericalSteering[side]->SetNameString(m_name + "_sphericalSteering" + suffix);
		////m_sphericalSteering[side]->Initialize(m_tierod[side], tierod_body, ChCoordsys<>(points[TIEROD_S], QUNIT));
		////chassis->GetSystem()->AddLink(m_sphericalSteering[side]);

		// Create and initialize the revolute joint between upright and spindle.
		ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
		m_revolute[side] = std::make_shared<ChLinkLockRevolute>();
		m_revolute[side]->SetNameString(m_name + "_revoluteSpindle" + suffix);
		m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side], rev_csys);
		chassis->GetSystem()->AddLink(m_revolute[side]);

		// Create and initialize the axle shaft and its connection to the spindle. Note that the
		// spindle rotates about the Y axis.
		m_axle[side] = std::make_shared<ChShaft>();
		m_axle[side]->SetNameString(m_name + "_axle" + suffix);
			//m_axle[side]->SetInertia(getAxleInertia());
		m_axle[side]->SetPos_dt(-ang_vel);
		chassis->GetSystem()->Add(m_axle[side]);

		m_axle_to_spindle[side] = std::make_shared<ChShaftsBody>();
		m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
		m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
		chassis->GetSystem()->Add(m_axle_to_spindle[side]);
	}

	void AddVisualizationAssets(VisualizationType vis) {
		ChSuspension::AddVisualizationAssets(vis);

		if (vis == VisualizationType::NONE)
			return;


		//AddVisualizationLink(m_tierod[LEFT], m_pointsL[TIEROD_S], m_pointsL[TIEROD_K], getTierodRadius(), ChColor(0.7f, 0.7f, 0.2f));
		//AddVisualizationLink(m_tierod[RIGHT], m_pointsR[TIEROD_S], m_pointsR[TIEROD_K], getTierodRadius(), ChColor(0.7f, 0.7f, 0.2f));
					//---NEW--- Add visualization for the tie-rods
		m_distTierod[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
		m_distTierod[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());
		m_distTierod[LEFT]->AddAsset(std::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
		m_distTierod[RIGHT]->AddAsset(std::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));


		AddVisualizationKnuckle(m_knuckle[LEFT], m_pointsL[KNUCKLE_U], m_pointsL[KNUCKLE_L], m_pointsL[TIEROD_K],
			getKnuckleRadius());
		AddVisualizationKnuckle(m_knuckle[RIGHT], m_pointsR[KNUCKLE_U], m_pointsR[KNUCKLE_L], m_pointsR[TIEROD_K],
			getKnuckleRadius());


	}

	void AddVisualizationLink(std::shared_ptr<ChBody> body,
		const ChVector<> pt_1,
		const ChVector<> pt_2,
		double radius,
		const ChColor& color) {
		// Express hardpoint locations in body frame.
		ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
		ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);

		auto cyl = std::make_shared<ChCylinderShape>();
		cyl->GetCylinderGeometry().p1 = p_1;
		cyl->GetCylinderGeometry().p2 = p_2;
		cyl->GetCylinderGeometry().rad = radius;
		body->AddAsset(cyl);

		auto col = std::make_shared<ChColorAsset>();
		col->SetColor(color);
		body->AddAsset(col);
	}

	void AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
		const ChVector<> pt_U,
		const ChVector<> pt_L,
		const ChVector<> pt_T,
		double radius) {
		static const double threshold2 = 1e-6;

		// Express hardpoint locations in body frame.
		ChVector<> p_U = knuckle->TransformPointParentToLocal(pt_U);
		ChVector<> p_L = knuckle->TransformPointParentToLocal(pt_L);
		ChVector<> p_T = knuckle->TransformPointParentToLocal(pt_T);

		if (p_L.Length2() > threshold2) {
			auto cyl_L = std::make_shared<ChCylinderShape>();
			cyl_L->GetCylinderGeometry().p1 = p_L;
			cyl_L->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
			cyl_L->GetCylinderGeometry().rad = radius;
			knuckle->AddAsset(cyl_L);
		}

		if (p_U.Length2() > threshold2) {
			auto cyl_U = std::make_shared<ChCylinderShape>();
			cyl_U->GetCylinderGeometry().p1 = p_U;
			cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
			cyl_U->GetCylinderGeometry().rad = radius;
			knuckle->AddAsset(cyl_U);
		}

		if (p_T.Length2() > threshold2) {
			auto cyl_T = std::make_shared<ChCylinderShape>();
			cyl_T->GetCylinderGeometry().p1 = p_T;
			cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
			cyl_T->GetCylinderGeometry().rad = radius;
			knuckle->AddAsset(cyl_T);
		}

		auto col = std::make_shared<ChColorAsset>();
		col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
		knuckle->AddAsset(col);
	}

/*
	void AddVisualizationAssets() override;
	void RemoveVisualizationAssets() override;
	*/
	double GetMass() const override{ return 1; }
	double getSpindleRadius() const override{ return 0.01; }
	double getSpindleWidth() const override{ return 0.02; }

	double getTierodRadius() const { return .025; }
	double getKnuckleRadius() const { return .030; }


protected:
	/// Identifiers for the various hardpoints.
	enum PointId {
		SHOCK_A,            ///< shock, axle
		SHOCK_C,            ///< shock, chassis
		KNUCKLE_L,          ///< lower knuckle point
		KNUCKLE_U,          ///< upper knuckle point
		LL_A,               ///< lower link, axle
		LL_C,               ///< lower link, chassis
		UL_A,               ///< upper link, axle
		UL_C,               ///< upper link, chassis
		SPRING_A,           ///< spring, axle
		SPRING_C,           ///< spring, chassis
		TIEROD_K,           ///< tierod, knuckle
		SPINDLE,            ///< spindle location
		KNUCKLE_CM,         ///< knuckle, center of mass
		LL_CM,              ///< lower link, center of mass
		UL_CM,              ///< upper link, center of mass
		BELLCRANK_TIEROD,   ///< bell crank to tierod
		BELLCRANK_AXLE,     ///< bell crank to axle
		BELLCRANK_DRAGLINK, ///< bell crank to draglink'
		DRAGLINK_C,         ///< draglink, chassis
		TIEROD_CM,
		TIEROD_S,
		TIEROD_C,  ///< tierod, chassis
		TIEROD_U,  ///< tierod, upright
		NUM_POINTS
	};
	const ChVector<> getLocation(PointId which) {
		switch (which) {
		case SHOCK_A:
			return ChVector<>(-0.065, 0.575, -0.025);
		case SHOCK_C:
			return ChVector<>(-0.080, 0.56, 0.3);
		case KNUCKLE_L:
			return ChVector<>(0.005, 0.7, -0.05);
		case KNUCKLE_U:
			return ChVector<>(-0.015, 0.675, 0.075);
		case LL_A:
			return ChVector<>(0.01, 0.6, -0.075);
		case LL_C:
			return ChVector<>(0.45, 0.35, -0.045);
		case UL_A:
			return ChVector<>(-0.055, 0.475, 0.15);
		case UL_C:
			return ChVector<>(0.355, 0.5, 0.15);
		case SPRING_A:
			return ChVector<>(-0.065, 0.575, -0.025);
		case SPRING_C:
			return ChVector<>(-0.080, 0.56, 0.3);
		case TIEROD_K:
			return ChVector<>(-0.075, 0.68, -0.065);
		case SPINDLE:
			return ChVector<>(0, 0.910, 0);
		case KNUCKLE_CM:
			return ChVector<>(0, 0.7, 0);
		case LL_CM:
			return ChVector<>(0.23, 0.475, -0.06);
		case UL_CM:
			return ChVector<>(0.15, 0.4875, 0.15);
		case BELLCRANK_TIEROD:
			return ChVector<>(-0.075, 0.325, -0.065);
		case BELLCRANK_AXLE:
			return ChVector<>(0, 0.325, -0.05);
		case BELLCRANK_DRAGLINK:
			return ChVector<>(0, 0.425, -0.05);
		case DRAGLINK_C:
			return ChVector<>(0.385, 0.45, -0.02);
		case TIEROD_CM:
			return ChVector<>(0, 0.425, -0.05);
		case TIEROD_S:
			return ChVector<>(-.175, 0.45, -0.02);
		case TIEROD_C:
			return 0.0254 * ChVector<>(-9.855, 17.655, 2.135);
		case TIEROD_U:
			return 0.0254 * ChVector<>(-6.922, 32.327, -0.643);

		default:
			return ChVector<>(0, 0, 0);
		}
	}
private:
	//const ChVector<> getLocation(PointId which) { return m_points[which]; }

	ChVector<> m_points[NUM_POINTS];

	// Hardpoint absolute locations
	std::vector<ChVector<>> m_pointsL;
	std::vector<ChVector<>> m_pointsR;

	std::shared_ptr<ChBody> m_tierod[2];        ///< handles to the tierod body
	std::shared_ptr<ChBody> m_knuckle[2];    ///< handles to the knuckle bodies (left/right)
	
	std::shared_ptr<ChLinkLockSpherical> m_sphericalTierod[2];
	std::shared_ptr<ChLinkLockRevolute> m_revoluteKingpin[2];
	std::shared_ptr<ChLinkLockSpherical> m_sphericalSteering[2];
	//std::shared_ptr<ChLinkLockRevolute> m_revolute[2];
	std::shared_ptr<ChLinkDistance> m_distTierod[2];

	//std::shared_ptr<ChShaft> m_axle[2];
	//std::shared_ptr<ChShaftsBody> m_axle_to_spindle[2];
	//std::shared_ptr<ChBody> m_spindle[2];    ///< handles to 

};

// Concrete class defining the WL steering mechanism--disconnected from ChSteering
class PivotWL {
	public:
		PivotWL(const std::string& name)  { }
		~PivotWL() {}
		
		const double PivotWL::m_maxAngle = 40 * CH_C_DEG_TO_RAD; //40° rotation;
		const double PivotWL::m_mass = 10; //total mass;
	const std::string& PivotWL::m_name = "Steering";// class name
// Steering function: it realizes a rotational engine link between chassis(front_body) and rear_body
		void PivotWL::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
			std::shared_ptr<ChBodyAuxRef> rear_body,
			const ChVector<>& location,
			const ChQuaternion<>& rotation)
		{		
			// Express the steering reference frame in the absolute coordinate system.
			ChFrame<> steering_to_abs(location, rotation);
			steering_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
			// Create and initialize the steering link body
			ChVector<> link_local(0., .0, -0.02);
			ChVector<> link_abs = steering_to_abs.TransformPointLocalToParent(link_local);

			m_pivoting = std::make_shared<ChLinkEngine>();
			m_pivoting->SetNameString(m_name + "_pivoting");
			// z-dir default rev_joint dir-NO rotation needed
			m_pivoting->Initialize(chassis, rear_body, ChCoordsys<>(link_abs, steering_to_abs.GetRot())); // it rotates about z axis of steering frame
			m_pivoting->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
			m_pivoting->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
			chassis->GetSystem()->AddLink(m_pivoting);

		}

		void PivotWL::Synchronize(double time, double steering)  {
			double angle = steering * GetMaxAngle();
			if (auto fun = std::dynamic_pointer_cast<ChFunction_Const>(m_pivoting->Get_rot_funct()))
				fun->Set_yconst(angle);

		}
		double PivotWL::GetMaxAngle() const { return m_maxAngle; }
		void PivotWL::AddVisualizationAssets(chrono::vehicle::VisualizationType vis)  {
			if (vis == chrono::vehicle::VisualizationType::NONE)
				return;
			// Add a z-oriented cylinder vis shape.
		}
		void PivotWL::LogConstraintViolations() {
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

		}
		void PivotWL::RemoveVisualizationAssets(){
			// Delete vis shape.
		}
		std::shared_ptr<ChBody> GetSteeringLink() const { return std::shared_ptr<ChBody>(); }
		double PivotWL::GetMass() const { return m_mass; }

protected:
	std::shared_ptr<ChLinkEngine> m_pivoting;
	std::string m_name;
};

//#define USE_RACKPIN		// Flag to test RigidAxle suspension with a classical steering mechanism
//#define USE_STEERLIST	// Flag to decide whether to use ChSteeringList or not()

// Concrete class for WheelLoader vehicle
class WheelLoader_Vehicle : public chrono::vehicle::ChWheeledVehicle {
public:
	WheelLoader_Vehicle(
		const bool fixed,
		chrono::vehicle::SuspensionType suspType,
		chrono::ChMaterialSurface::ContactMethod contactMethod = chrono::ChMaterialSurface::NSC)
		:ChWheeledVehicle(contactMethod), m_suspType(suspType){

		// Create the chassis subsystem
		m_chassis = std::make_shared<Generic_Chassis>("Chassis");

#ifdef USE_RACKPIN
		m_steerings.resize(1);
		m_steerings[0] = std::make_shared<Generic_RackPinion>("Steering");
#else
		// Create the rear body subsystem
		m_rear = std::shared_ptr<ChBodyAuxRef>(m_system->NewBodyAuxRef());
		m_rear->SetName("rear body");
		// Create the steering subsystem
		m_steerings.resize(0);//Unable to deal with ChSteeringList
		m_steer = std::make_shared<PivotWL>("Steering");

#endif
		
		// Create the suspension subsystems--RigidAxle
		m_suspensions.resize(2);
			m_suspensions[0] = std::make_shared<RigidAxle>("FrontSusp");
			m_suspensions[1] = std::make_shared<RigidAxle>("RearSusp");
		

		// Create the wheels
		m_wheels.resize(4);
		m_wheels[0] = std::make_shared<Generic_Wheel>("Wheel_FL");
		m_wheels[1] = std::make_shared<Generic_Wheel>("Wheel_FR");
		m_wheels[2] = std::make_shared<Generic_Wheel>("Wheel_RL");
		m_wheels[3] = std::make_shared<Generic_Wheel>("Wheel_RR");
		// Create the driveline
		m_driveline = std::make_shared<HMMWV_Driveline4WD>("driveline");
		// Create the brakes
		m_brakes.resize(4);
		m_brakes[0] = std::make_shared<Generic_BrakeSimple>("Brake_FL");
		m_brakes[1] = std::make_shared<Generic_BrakeSimple>("Brake_FR");
		m_brakes[2] = std::make_shared<Generic_BrakeSimple>("Brake_RL");
		m_brakes[3] = std::make_shared<Generic_BrakeSimple>("Brake_RR");
	}

	~WheelLoader_Vehicle() {}

	virtual int GetNumberAxles() const override { return 2; }

	double GetSpringForce(const chrono::vehicle::WheelID& wheel_id) const;
	double GetSpringLength(const chrono::vehicle::WheelID& wheel_id) const;
	double GetSpringDeformation(const chrono::vehicle::WheelID& wheel_id) const;

	double GetShockForce(const chrono::vehicle::WheelID& wheel_id) const;
	double GetShockLength(const chrono::vehicle::WheelID& wheel_id) const;
	double GetShockVelocity(const chrono::vehicle::WheelID& wheel_id) const;

	void Initialize(const chrono::ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override
	{
		m_chassis->Initialize(m_system, chassisPos, chassisFwdVel);

#ifdef USE_RACKPIN
		ChVector<> offset(1.60, 0, -0.07);

		m_steerings[0]->Initialize(m_chassis->GetBody(), offset, ChQuaternion<>(1, 0, 0, 0));
		m_suspensions[0]->Initialize(m_chassis->GetBody(), ChVector<>(1.6914, 0, 0), m_steerings[0]->GetSteeringLink());
		m_suspensions[1]->Initialize(m_chassis->GetBody(), ChVector<>(-1.6914, 0, 0), m_chassis->GetBody());

#else
		//check the transform
		m_rear->SetFrame_REF_to_abs(ChFrame<>(chassisPos) * ChFrame<>(ChVector<>(-1.5,0.,0.),QUNIT));// Random pos for the rear body
		m_rear->SetMass(m_chassis->GetMass()); m_rear->SetInertia(m_chassis->GetInertia());
		m_system->Add(m_rear);
		m_steer->Initialize(m_chassis->GetBody(), m_rear, ChVector<>(-.4, 0, -.0), ChQuaternion<>(1, 0, 0, 0));
		m_suspensions[0]->Initialize(m_chassis->GetBody(), ChVector<>(1.6914, 0, 0), m_chassis->GetBody());
		m_suspensions[1]->Initialize(m_rear, ChVector<>(-1.6914, 0, 0), m_rear);
		

#endif //USE_RACKPIN

		

		m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
		m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
		m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
		m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

		std::vector<int> driven_susp(2);//(4WD)
		driven_susp[0] = 0; driven_susp[1] = 1;
		m_driveline->Initialize(m_chassis->GetBody(), m_suspensions, driven_susp);

		m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
		m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
		m_brakes[2]->Initialize(m_suspensions[1]->GetRevolute(LEFT));
		m_brakes[3]->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
	}

	void Synchronize(double time,
		double steering,
		double braking,
		double powertrain_torque,
		const TireForces& tire_forces) override{
		
		ChWheeledVehicle::Synchronize(time, steering, braking, powertrain_torque, tire_forces);
#ifdef USE_RACKPIN

#else
		m_steer->Synchronize(time, steering);
#endif //USE_RACKPIN

	}


	// Log debugging information
	void LogHardpointLocations();  /// suspension hardpoints at design
	void DebugLog(int what);       /// shock forces and lengths, constraints, etc.

	// Synchronize method override (first call Synchronize base class method, then add whatever you want)


private:
	chrono::vehicle::SuspensionType m_suspType;
	std::shared_ptr<ChBodyAuxRef> m_rear;
	//enum SuspensionType { RIGID_AXLE };
#ifdef USE_RACKPIN
#else
	std::shared_ptr<PivotWL> m_steer;

#endif //USE_RACKPIN


};


// =============================================================================
bool render = true;
// =============================================================================

// Initial vehicle position
ChVector<> initLoc(0, 0, 1.0);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Simulation step size
double step_size = 0.001;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 1;  // once a second

#ifdef USE_IRRLICHT
// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);
#else
double tend = 20.0;

const std::string out_dir = "../GENERIC_VEHICLE";
const std::string pov_dir = out_dir + "/POVRAY";
#endif


int main(int argc, char* argv[]) {
	
#ifdef USE_PARALLEL
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
#endif

#ifdef TEST_ENABLE
// First type of steering: useless.
		//auto test = std::make_shared<CenterPivot>("Test");
		//test->Initialize(chass, ChVector<>(1.60, 0, -.0), QUNIT);
		//std::shared_ptr<ChBody> body = test->GetSteeringLink(LEFT);
		//std::shared_ptr<ChBody> corpo = test->GetSteeringLink(RIGHT);
		//test->AddVisualizationAssets(VisualizationType::MESH);


		//auto test1 = std::make_shared<RigidAxle>("Test_1");
		//test1->Initialize(chass, ChVector<>(1.6914,0.,0.), test->GetSteeringLink(LEFT), test->GetSteeringLink(RIGHT),0, 0);
		//test1->AddVisualizationAssets(VisualizationType::MESH);

		//auto test2 = std::make_shared<RigidAxle>("Test_2");
		//test2->Initialize(chass, ChVector<>(1.6914, 0, 0), chass, 0, 0);
		//test2->AddVisualizationAssets(VisualizationType::MESH);

////////////////////-------------LAST TESTS---------------///////

		// Pivot Steering: .
		auto test00 = std::make_shared<PivotWL>("Test_00");
		test00->Initialize(chass, rear, ChVector<>(1.60, 0, -.0), QUNIT);
		test00->AddVisualizationAssets(VisualizationType::MESH);
		// Rigid Axle
		auto test01 = std::make_shared<RigidAxle>("Test_01");
		test01->Initialize(chass, ChVector<>(1.6914, 0, 0), chass, 0, 0);
		test01->AddVisualizationAssets(VisualizationType::MESH);
		auto test02 = std::make_shared<RigidAxle>("Test_02");
		test02->Initialize(rear, ChVector<>(1.6914, 0, 0), rear, 0, 0);
		test02->AddVisualizationAssets(VisualizationType::MESH);
#endif
		
#ifdef USE_SEP_VEH
	ChSuspensionList m_suspensions;
	//std::vector<std::shared_ptr<RigidAxle>> m_suspensions;
	std::shared_ptr<PivotWL> m_steerings;
	ChWheelList m_wheels;
	ChBrakeList m_brakes;
	std::shared_ptr<HMMWV_Driveline4WD> m_driveline;

	auto m_chassis = std::make_shared<Generic_Chassis>("Chassis");
	//m_steerings.resize(1);
	m_steerings = std::make_shared<PivotWL>("Steering");
	m_suspensions.resize(2);
	m_suspensions[0] = std::make_shared<RigidAxle>("FrontSusp");
	m_suspensions[1] = std::make_shared<RigidAxle>("RearSusp");
	m_wheels.resize(4);
	m_wheels[0] = std::make_shared<Generic_Wheel>("Wheel_FL");
	m_wheels[1] = std::make_shared<Generic_Wheel>("Wheel_FR");
	m_wheels[2] = std::make_shared<Generic_Wheel>("Wheel_RL");
	m_wheels[3] = std::make_shared<Generic_Wheel>("Wheel_RR");
	m_brakes.resize(4);
	m_brakes[0] = std::make_shared<Generic_BrakeSimple>("Brake_FL");
	m_brakes[1] = std::make_shared<Generic_BrakeSimple>("Brake_FR");
	m_brakes[2] = std::make_shared<Generic_BrakeSimple>("Brake_RL");
	m_brakes[3] = std::make_shared<Generic_BrakeSimple>("Brake_RR");
	m_driveline = std::make_shared<HMMWV_Driveline4WD>("driveline");

	m_chassis->Initialize(system, ChCoordsys<>(VNULL,QUNIT), 0.);
	m_steerings->Initialize(m_chassis->GetBody(), rear, ChVector<>(1.60, 0, -.0), ChQuaternion<>(1, 0, 0, 0));
	m_suspensions[0]->Initialize(m_chassis->GetBody(), ChVector<>(1.6914, 0, 0), m_chassis->GetBody(), 0, 0);
	m_suspensions[1]->Initialize(rear, ChVector<>(-1.6865, 0, 0), rear,0,0);
	m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
	m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
	m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
	m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));
	m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
	m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
	m_brakes[2]->Initialize(m_suspensions[1]->GetRevolute(LEFT));
	m_brakes[3]->Initialize(m_suspensions[1]->GetRevolute(RIGHT));

	std::vector<int> driven_susp(2);//(4WD)
	driven_susp[0] = 0; driven_susp[1] = 1;
	m_driveline->Initialize(m_chassis->GetBody(), m_suspensions, driven_susp);
#endif



	WheelLoader_Vehicle vehicle(true, SuspensionType::MULTI_LINK);// to fix
	vehicle.Initialize(ChCoordsys<>(initLoc + ChVector<>(0, 0, .0), initRot));
	vehicle.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
	vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
	vehicle.SetSteeringVisualizationType(VisualizationType::MESH);
	vehicle.SetWheelVisualizationType(VisualizationType::PRIMITIVES);

	// Create the terrain
	RigidTerrain terrain(vehicle.GetSystem());
	terrain.SetContactFrictionCoefficient(0.9f);
	terrain.SetContactRestitutionCoefficient(0.01f);
	terrain.SetContactMaterialProperties(2e7f, 0.3f);
	terrain.SetColor(ChColor(0.5f, 0.5f, 1));
	terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
	terrain.Initialize(terrainHeight, terrainLength, terrainWidth);


////////////-----------------------------------------//////////////////////

	// Create and initialize the powertrain system
	Generic_SimplePowertrain powertrain;

	powertrain.Initialize(vehicle.GetChassisBody(), vehicle.GetDriveshaft());

	// Create the tires
	Generic_RigidTire tire_front_left("FL");
	Generic_RigidTire tire_front_right("FR");
	Generic_RigidTire tire_rear_left("RL");
	Generic_RigidTire tire_rear_right("RR");

	tire_front_left.Initialize(vehicle.GetWheelBody(FRONT_LEFT), LEFT);
	tire_front_right.Initialize(vehicle.GetWheelBody(FRONT_RIGHT), RIGHT);
	tire_rear_left.Initialize(vehicle.GetWheelBody(REAR_LEFT), LEFT);
	tire_rear_right.Initialize(vehicle.GetWheelBody(REAR_RIGHT), RIGHT);

	tire_front_left.SetVisualizationType(VisualizationType::PRIMITIVES);
	tire_front_right.SetVisualizationType(VisualizationType::PRIMITIVES);
	tire_rear_left.SetVisualizationType(VisualizationType::PRIMITIVES);
	tire_rear_right.SetVisualizationType(VisualizationType::PRIMITIVES);




	vehicle.GetSystem()->ShowHierarchy(GetLog());








#ifdef USE_IRRLICHT
	ChWheeledVehicleIrrApp app(&vehicle, &powertrain, L"Articulated Vehicle Demo");

	app.SetSkyBox();
	app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
	app.SetChaseCamera(trackPoint, 6.0, 0.5);

	app.SetTimestep(step_size);

	app.AssetBindAll();
	app.AssetUpdateAll();

	ChIrrGuiDriver driver(app);

	// Set the time response for steering and throttle keyboard inputs.
	// NOTE: this is not exact, since we do not render quite at the specified FPS.
	double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
	double throttle_time = 1.0;  // time to go from 0 to +1
	double braking_time = 0.3;   // time to go from 0 to +1
	driver.SetSteeringDelta(render_step_size / steering_time);
	driver.SetThrottleDelta(render_step_size / throttle_time);
	driver.SetBrakingDelta(render_step_size / braking_time);

#else
	Generic_FuncDriver driver(vehicle);
#endif

	driver.Initialize();


	// Inter-module communication data
	TireForces tire_forces(4);
	TireForces tr_tire_forces(4);
	WheelState wheel_states[4];
	double driveshaft_speed;
	double powertrain_torque;
	double throttle_input;
	double steering_input;
	double braking_input;

	// Number of simulation steps between two 3D view render frames
	int render_steps = (int)std::ceil(render_step_size / step_size);

	// Number of simulation steps between two output frames
	int output_steps = (int)std::ceil(output_step_size / step_size);

	// Initialize simulation frame counter and simulation time
	int step_number = 0;
	double time = 0;

#ifdef USE_IRRLICHT

	ChRealtimeStepTimer realtime_timer;

	while (app.GetDevice()->run()) {
		// Render scene
		if (step_number % render_steps == 0) {
			app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
			app.DrawAll();
			app.EndScene();
		}

#ifdef DEBUG_LOG
		if (step_number % output_steps == 0) {
			GetLog() << "\n\n============ System Information ============\n";
			GetLog() << "Time = " << time << "\n\n";
			vehicle.DebugLog(DBG_SPRINGS | DBG_SHOCKS | DBG_CONSTRAINTS);
		}
#endif

		// Collect output data from modules (for inter-module communication)
		throttle_input = driver.GetThrottle();
		steering_input = driver.GetSteering();
		braking_input = driver.GetBraking();

		powertrain_torque = powertrain.GetOutputTorque();

		tire_forces[FRONT_LEFT.id()] = tire_front_left.GetTireForce();
		tire_forces[FRONT_RIGHT.id()] = tire_front_right.GetTireForce();
		tire_forces[REAR_LEFT.id()] = tire_rear_left.GetTireForce();
		tire_forces[REAR_RIGHT.id()] = tire_rear_right.GetTireForce();


		driveshaft_speed = vehicle.GetDriveshaftSpeed();

		wheel_states[FRONT_LEFT.id()] = vehicle.GetWheelState(FRONT_LEFT);
		wheel_states[FRONT_RIGHT.id()] = vehicle.GetWheelState(FRONT_RIGHT);
		wheel_states[REAR_LEFT.id()] = vehicle.GetWheelState(REAR_LEFT);
		wheel_states[REAR_RIGHT.id()] = vehicle.GetWheelState(REAR_RIGHT);

		// Update modules (process inputs from other modules)
		time = vehicle.GetSystem()->GetChTime();

		driver.Synchronize(time);

		terrain.Synchronize(time);

		tire_front_left.Synchronize(time, wheel_states[FRONT_LEFT.id()], terrain);
		tire_front_right.Synchronize(time, wheel_states[FRONT_RIGHT.id()], terrain);
		tire_rear_left.Synchronize(time, wheel_states[REAR_LEFT.id()], terrain);
		tire_rear_right.Synchronize(time, wheel_states[REAR_RIGHT.id()], terrain);

		powertrain.Synchronize(time, throttle_input, driveshaft_speed);

		vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
		// steering input synchronize??? 

		app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

		// Advance simulation for one timestep for all modules
		double step = realtime_timer.SuggestSimulationStep(step_size);

		driver.Advance(step);

		terrain.Advance(step);

		tire_front_right.Advance(step);
		tire_front_left.Advance(step);
		tire_rear_right.Advance(step);
		tire_rear_left.Advance(step);

		powertrain.Advance(step);

		vehicle.Advance(step);

		app.Advance(step);

		// Increment frame number
		step_number++;
	}
#endif


#ifdef USE_PARALLEL

#ifdef CHRONO_OPENGL
	if (render) {
		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "demo_vehicle_WheelLoader", vehicle.GetSystem());
		gl_window.SetCamera(ChVector<>(100, 100, 100), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1));
		gl_window.SetRenderMode(opengl::SOLID);
	}
#endif


	while (vehicle.GetSystem()->GetChTime() < 500000.00) {
		vehicle.GetSystem()->DoStepDynamics(.01);

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
#endif // USE_PARALLEL

	return 0;
}