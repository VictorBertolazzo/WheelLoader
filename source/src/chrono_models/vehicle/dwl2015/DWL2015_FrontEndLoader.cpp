// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Victor Bertolazzo
// =============================================================================
//
// DWL2015 front end loader model.
//
// =============================================================================
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"


#include "chrono_models/vehicle/dwl2015/DWL2015_FrontEndLoader.h"
namespace chrono {
namespace vehicle {
	namespace dwl2015 {
			// Static variables
		const std::string DWL2015_FrontEndLoader::m_meshNameBucket = "bucket_loader";
		const std::string DWL2015_FrontEndLoader::m_meshFileBucket = "dwl2015/frontendloader/bucket.obj";

			
			const double DWL2015_FrontEndLoader::m_bucketLeverMass = 3.6814844109;
			const double DWL2015_FrontEndLoader::m_liftArmMass = 1.605;
			const double DWL2015_FrontEndLoader::m_lshapedMass = 3.6814844109;
			const double DWL2015_FrontEndLoader::m_loaderMass = 3.6814844109;
			const double DWL2015_FrontEndLoader::m_idleMass = 3.6814844109;

			const double DWL2015_FrontEndLoader::m_bucketLeverRadius = 0.075;
			const double DWL2015_FrontEndLoader::m_liftArmRadius = 0.050;
			const double DWL2015_FrontEndLoader::m_lshapedRadius = 0.03;
			const double DWL2015_FrontEndLoader::m_loaderRadius = 0.5;
			const double DWL2015_FrontEndLoader::m_idleRadius = 0.03;

			const double DWL2015_FrontEndLoader::m_maxAngle = 50.0 * (CH_C_PI / 180);

			const ChVector<> DWL2015_FrontEndLoader::m_bucketLeverInertia(0.252, 0.00233, 0.254);
			const ChVector<> DWL2015_FrontEndLoader::m_liftArmInertia(0.00638, 0.00756, 0.00150);
			const ChVector<> DWL2015_FrontEndLoader::m_lshapedInertia(0.4, 0.00233, 0.254);
			const ChVector<> DWL2015_FrontEndLoader::m_loaderInertia(0.6, 0.00233, 0.254);
			const ChVector<> DWL2015_FrontEndLoader::m_idleInertia(0.8, 0.00233, 0.254);


		// -----------------------------------------------------------------------------
		DWL2015_FrontEndLoader::DWL2015_FrontEndLoader(const std::string& name) : ChFrontEndLoader(name) {}

		void DWL2015_FrontEndLoader::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& location, const ChQuaternion<>& rotation) { ChFrontEndLoader::Initialize(chassis, location, rotation); }
		void DWL2015_FrontEndLoader::AddVisualizationAssets(VisualizationType vis) { ChFrontEndLoader::AddVisualizationAssets(vis);

		m_loader->GetAssets().clear();
		geometry::ChTriangleMeshConnected trimesh;
		trimesh.LoadWavefrontMesh(GetChronoDataFile(m_meshFileBucket),false,false);
		auto bari = trimesh.Baricenter();
		
		GetLog() << bari << "\n";
		trimesh.Transform(ChVector<>(0.,0.0,0.0),ChMatrix33<>(1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0)*(1.0/10.0) );
		auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
		trimesh_shape->SetMesh(trimesh);
		trimesh_shape->SetName(m_meshNameBucket);
		trimesh_shape->Pos = m_b;
		m_loader->AddAsset(trimesh_shape);	
	
		}
		void DWL2015_FrontEndLoader::RemoveVisualizationAssets(){}

		void DWL2015_FrontEndLoader::Synchronize(double time, double steering, double valve_inputs[4]){}
		double DWL2015_FrontEndLoader::GetMass() const{return 0.0;}
		void DWL2015_FrontEndLoader::LogConstraintViolations(){}
		
//		double GetPneuForce(ChLinkPneumaticActuator& pneum){}
		
		const ChVector<> DWL2015_FrontEndLoader::getLocation(PointId which) {
			switch (which) {
			case BUCKETLEVER:
				return ChVector<>(0.129, 0, 0.3);
			case LIFTARM:
				return ChVector<>(2.0, 0, 0.25);
			case LSHAPED:
				return ChVector<>(2.45, 0, 0.35);
			case LOADER:
				return ChVector<>(3.0, 0, 0.40);
			case IDLE:
				return ChVector<>(1.45, 0, 0.3);
			case VERTREV:
				return ChVector<>(1.3, 0, 2);
			case HORZREV_BL_LS:
				return ChVector<>(1.8, 0, 0.25);
			case HORZREV_LA_LS:
				return ChVector<>(1.9, 0, 0.45);
			case HORZREV_BL_B:
				return ChVector<>(1.8, 0, 0.5);
			case HORZREV_LA_B:
				return ChVector<>(2.0, 0, 0.3);
			case PNEUMA1_ID:
				return ChVector<>(1.5, 0, 1.5);
			case PNEUMA1_BL:
				return ChVector<>(2.4, 0, 1.3);
			case PNEUMA2_ID:
				return ChVector<>(1.5, 0, 0.25);
			case PNEUMA2_LA:
				return ChVector<>(2.0, 0, 0.48);
			default:
				return ChVector<>(0, 0, 0);
			}
		}
		const ChVector<> DWL2015_FrontEndLoader::getDirection(DirectionId which) {
			switch (which) {
			case VERTREV_AXIS:
				return ChVector<>(0, 0, 1);
			case HORZREV_AXIS:
				return ChVector<>(0, 1, 0);
			default:
				return ChVector<>(0, 1, 0);
			}
		}

} // END DWL2015
} // END VEHICLE
} // END CHRONO
