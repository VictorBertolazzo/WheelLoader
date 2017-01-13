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
#ifndef DWL2015_FRONTENDLOADER_H
#define DWL2015_FRONTENDLOADER_H

#include "chrono_vehicle/wheeled_vehicle/frontendloader/ChFrontEndLoader.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {
	class CH_MODELS_API DWL2015_FrontEndLoader : public ChFrontEndLoader {
	public:
		DWL2015_FrontEndLoader(const std::string& name) ;
		~DWL2015_FrontEndLoader() {}
		//--------To avoid c2259
		void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
			const ChVector<>& location,
			const ChQuaternion<>& rotation
		);
		// Visualization assets.
		void AddVisualizationAssets(VisualizationType vis);
		// Remove visualization assets.
		void RemoveVisualizationAssets();

		void Synchronize(double time,     ///< [in] current time
			double steering,  ///< [in] current steering input [-1,+1]
			double valve_inputs[4]
		)override;

		double GetMass() const ;
		void LogConstraintViolations() ;
		//------------
		//Get pneumatic force.
//		double GetPneuForce(ChLinkPneumaticActuator& pneum);
		//----------------------

		double getBucketLeverMass() const override { return m_bucketLeverMass; }
		double getLiftArmMass() const override { return m_liftArmMass; }
		double getLShapedMass() const override { return m_lshapedMass; }
		double getLoaderMass() const override { return m_loaderMass; }
		double getIdleMass() const override { return m_idleMass; }

		double getBucketLeverRadius() const override { return m_bucketLeverRadius; }
		double getLiftArmRadius() const override { return m_liftArmRadius; }
		double getLShapedRadius() const override { return m_lshapedRadius; }
		double getLoaderRadius() const override { return m_loaderRadius; }
		double getIdleRadius() const override { return m_idleRadius; }

		const chrono::ChVector<>& getBucketLeverInertia() const override { return m_bucketLeverInertia; }
		const chrono::ChVector<>& getLiftArmInertia() const override { return m_liftArmInertia; }
		const chrono::ChVector<>& getLShapedInertia() const override { return m_lshapedInertia; }
		const chrono::ChVector<>& getLoaderInertia() const override { return m_loaderInertia; }
		const chrono::ChVector<>& getIdleInertia() const override { return m_idleInertia; }

		double getMaxAngle() const override { return m_maxAngle; }

		const chrono::ChVector<> getLocation(PointId which) override;
		const chrono::ChVector<> getDirection(DirectionId which) override;

	private:
		static const double m_bucketLeverMass;
		static const double m_liftArmMass;
		static const double m_lshapedMass;
		static const double m_loaderMass;
		static const double m_idleMass;

		static const double m_bucketLeverRadius;
		static const double m_liftArmRadius;
		static const double m_lshapedRadius;
		static const double m_loaderRadius;
		static const double m_idleRadius;

		static const double m_maxAngle;

		static const chrono::ChVector<> m_bucketLeverInertia;
		static const chrono::ChVector<> m_liftArmInertia;
		static const chrono::ChVector<> m_lshapedInertia;
		static const chrono::ChVector<> m_loaderInertia;
		static const chrono::ChVector<> m_idleInertia;

		static const std::string m_meshNameBucket;
		static const std::string m_meshFileBucket;

	};

}
}
}

#endif 