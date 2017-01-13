// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// =============================================================================
//
// Base class for a front end loader subsystem.
//
// =============================================================================

#ifndef CH_FRONTENDLOADER_H
#define CH_FRONTENDLOADER_H

#include <string>

#include "chrono_parallel/collision/ChCollisionModelParallel.h" //Add 5/12

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyAuxRef.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
	namespace vehicle {

// Base class for a classic front end loader subsystem.
// All the implementations are not definitive.
// It's an abstract class, but for the moment it is directly derived from ChPart
// Afterwards I'll take into account the possibility to further divide it
///-----------------------------------------------------------------------------------
/// The front end loader subsystem is modeled with respect to a right-handed frame with
/// with X pointing towards the front, Y to the left, and Z up (ISO standard).
///
/// When attached to a chassis, both an offset and a rotation (as a quaternion)
/// are provided.

class CH_VEHICLE_API ChFrontEndLoader : public ChPart {
	public:
		ChFrontEndLoader(const std::string& name);
		virtual ~ChFrontEndLoader() {}

		// Initialize the front end loader subsystem.
		virtual void Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
								const ChVector<>& location,
								const ChQuaternion<>& rotation
								) = 0;
		// Visualization assets.
		virtual void AddVisualizationAssets(VisualizationType vis) override;
		// Remove visualization assets.
		virtual void RemoveVisualizationAssets() override;

		// Update the state of FEL subsystem.
		/// Update the state of this idle steering at the current time.
        // In this case steering refers to revolute joint btw idle body and the chassis
		// This function is called during the vehicle update.
		virtual void Synchronize(double time,     ///< [in] current time
			double steering,
			double valve_inputs[4]///< [in] current steering input [-1,+1]
		);// //ChPart does not own a Synchronize function, s.t. override is not necessary;//


		
		// Get the total mass of the FEL subsystem.
		virtual double GetMass() const = 0;

		// Log current constraint violations.
		virtual void LogConstraintViolations() = 0;

		//Set valves command.
		virtual void ValveCommand(std::shared_ptr<ChLinkPneumaticActuator> pneum, double a, double b);
		//Get valves command
		virtual double *GetValveCommand(std::shared_ptr<ChLinkPneumaticActuator> pneum);
		//Get pneumatic force.
		double GetPneuForce(std::shared_ptr<ChLinkPneumaticActuator> pneum);
		

protected:

	/// Identifiers for the various hardpoints.
	enum PointId {
		BUCKETLEVER,/// bucket lever COM location
		LIFTARM,/// lift arm COM location
		LSHAPED,/// L shaped body COM location
		LOADER,/// bucket COM location
		IDLE,/// idle COM location

		VERTREV,//  vertical rev joint btw idle and chassis ID-CH location

		HORZREV_BL_LS,/// BL-LS rev joint location
		HORZREV_LA_LS,/// LA-LS rev joint location
		HORZREV_BL_B,/// BL-B rev joint location
		HORZREV_LA_B,/// LA-B rev joint location

		PNEUMA1_ID,/// idle pneum1 connection point location --joint idle-BL
		PNEUMA1_BL,/// bucket lever pneum1 connection point location --joint idle-BL

		PNEUMA2_ID,/// idle pneum2 connection point location --joint idle-LA
		PNEUMA2_LA,/// lift arm pneum2 connection point location --joint idle-LA
		NUM_POINTS
	};
	/// Identifiers for the various direction unit vectors
	enum DirectionId {
		VERTREV_AXIS,/// revolute joint
		HORZREV_AXIS,
		NUM_DIRS
	};

	/// Return the location of the specified hardpoint.
	/// The returned location must be expressed in the suspension reference frame.
	virtual const ChVector<> getLocation(PointId which) = 0;
	/// Return the unit vector for the specified direction.
	/// The returned vector must be expressed in the suspension reference frame.
	virtual const ChVector<> getDirection(DirectionId which) = 0;


	/// Return the mass of the steering link body.
	virtual double getBucketLeverMass() const = 0;
	/// Return the mass of the Pitman arm body.
	virtual double getLiftArmMass() const = 0;
	/// Return the mass of the steering link body.
	virtual double getLShapedMass() const = 0;
	/// Return the mass of the Pitman arm body.
	virtual double getLoaderMass() const = 0;
	/// Return the mass of the Pitman arm body.
	virtual double getIdleMass() const = 0;

	/// Return the moments of inertia of the steering link body.
	virtual const ChVector<>& getBucketLeverInertia() const = 0;
	/// Return the moments of inertia of the Pitman arm body.
	virtual const ChVector<>& getLiftArmInertia() const = 0;
	/// Return the moments of inertia of the steering link body.
	virtual const ChVector<>& getLShapedInertia() const = 0;
	/// Return the moments of inertia of the Pitman arm body.
	virtual const ChVector<>& getLoaderInertia() const = 0;
	/// Return the moments of inertia of the Pitman arm body.
	virtual const ChVector<>& getIdleInertia() const = 0;

	/// Return the radius of the steering link body (visualization only).
	virtual double getBucketLeverRadius() const = 0;
	/// Return the radius of the Pitman arm body (visualization only).
	virtual double getLiftArmRadius() const = 0;
	/// Return the radius of the steering link body (visualization only).
	virtual double getLShapedRadius() const = 0;
	/// Return the radius of the Pitman arm body (visualization only).
	virtual double getLoaderRadius() const = 0;
	/// Return the radius of the steering link body (visualization only).
	virtual double getIdleRadius() const = 0;

	/// Return the maximum rotation angle of the revolute joint btw idle and chassis.
	virtual double getMaxAngle() const = 0;





	std::shared_ptr<collision::ChCollisionModelParallel> m_parcollisionmodel;

	std::shared_ptr<ChBody> m_bucketlever;  // handle to the bucket lever body
	std::shared_ptr<ChBody> m_liftarm;  // handle to the lift arm body
	std::shared_ptr<ChBody> m_lshaped;  // handle to the L shaped body
	std::shared_ptr<ChBody> m_loader;  // handle to the loader (bucket) body
	std::shared_ptr<ChBody> m_idle;  // handle to the idle body

	std::shared_ptr<ChLinkLockRevolute> m_revBL_LS; // handle to the bucket lever-lshaped rev joint
	std::shared_ptr<ChLinkLockRevolute> m_revBL_B; // handle to the bucket lever-loader rev joint
	std::shared_ptr<ChLinkLockRevolute> m_revLA_LS; // handle to the lift arm-lshaped rev joint
	std::shared_ptr<ChLinkLockRevolute> m_revLA_B; // handle to the lift arm-loader rev joint
	public:
	std::shared_ptr<ChLinkPneumaticActuator> m_pneumaBL_ID; // handle to the bucket lever-idle piston joint
	std::shared_ptr<ChLinkPneumaticActuator> m_pneumaLA_ID; // handle to the lift arm-idle piston joint
	protected:
	std::shared_ptr<ChLinkEngine> m_revID_CH; // handle to the idle-chassis rev joint


	// Points for visualization
	ChVector<> m_bl1;
	ChVector<> m_bl2;
	ChVector<> m_la1;
	ChVector<> m_la2;
	ChVector<> m_ls1;
	ChVector<> m_ls2;
	ChVector<> m_i;
	ChVector<> m_b;


		};
	} // end namespace vehicle
}     // end namespace chrono
#endif