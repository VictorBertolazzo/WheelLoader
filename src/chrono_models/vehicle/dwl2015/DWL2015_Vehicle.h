// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for the DWL2015 vehicle models
//
// =============================================================================

#ifndef DWL2015_VEHICLE_H
#define DWL2015_VEHICLE_H

#include <vector>
#include "chrono/core/ChCoordsys.h"
#include "chrono/physics/ChMaterialSurfaceBase.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_parallel/physics/ChSystemParallel.h"//Add 5/12, maybe a file with this one called


#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
// Header for handling the front end loader, since I did not embedded it in the general ChWheeledVehicle class!!!!
#include "chrono_models/vehicle/dwl2015/DWL2015_FrontEndLoader.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

class CH_MODELS_API DWL2015_Vehicle : public ChWheeledVehicle {
  public:
    virtual ~DWL2015_Vehicle() {}

    virtual int GetNumberAxles() const override { return 2; }

    void SetInitWheelAngVel(const std::vector<double>& omega) {
        assert(omega.size() == 4);
        m_omega = omega;
    }

	ChSystemParallel* GetSystemParallel() { return m_system_parallel; }//Add 5/12
  protected:
    DWL2015_Vehicle(ChMaterialSurfaceBase::ContactMethod contactMethod, DrivelineType driveType)
        : ChWheeledVehicle(contactMethod), m_driveType(driveType), m_omega({ 0, 0, 0, 0 }) {}

    DWL2015_Vehicle(ChSystem* system, DrivelineType driveType) : ChWheeledVehicle(system), m_driveType(driveType), m_omega({ 0, 0, 0, 0 }) {}

    DrivelineType m_driveType;
    std::vector<double> m_omega;

	ChSystemParallel* m_system_parallel;// Add 5/12

	

	public:
//	/// Get the specified f.e.l. subsystem.
	std::shared_ptr<ChFrontEndLoader> GetFrontEndLoader() const { return m_loader; }
	/// Set the valve command to the respective pneumatic actuator
	void SetValveCommand(int what, double a, double b) const {

		if (what & M_PENUMA_BL_ID) {
			return std::static_pointer_cast<DWL2015_FrontEndLoader>(m_loader)->ValveCommand(m_loader->m_pneumaBL_ID, a, b);
		}
		if (what & M_PNEUMA_LA_ID) {
			return std::static_pointer_cast<DWL2015_FrontEndLoader>(m_loader)->ValveCommand(m_loader->m_pneumaLA_ID, a, b);
		}
	};
	double *GetValveCommand(int what) const {

			if (what & M_PENUMA_BL_ID) {
				return std::static_pointer_cast<DWL2015_FrontEndLoader>(m_loader)->GetValveCommand(m_loader->m_pneumaBL_ID);
			}
			if (what & M_PNEUMA_LA_ID) {
				return std::static_pointer_cast<DWL2015_FrontEndLoader>(m_loader)->GetValveCommand(m_loader->m_pneumaLA_ID);
			}
	};
	/// Get Pneumatic Force from the respective pneumatic actuator
	double DWL2015_Vehicle::GetPneumaticForce(int what) const {

		if (what & M_PENUMA_BL_ID){
			return std::static_pointer_cast<DWL2015_FrontEndLoader>(m_loader)->GetPneuForce(m_loader->m_pneumaBL_ID);
		}
		if (what & M_PNEUMA_LA_ID) {
			return std::static_pointer_cast<DWL2015_FrontEndLoader>(m_loader)->GetPneuForce(m_loader->m_pneumaLA_ID);
		}
	}
	enum PneumaticInfo {
		M_PENUMA_BL_ID,
		M_PNEUMA_LA_ID
	};
	// Overriding
	void DWL2015_Vehicle::Synchronize(double time, double steering, double braking, double powertrain_torque, const TireForces & tire_forces)override {}
	// Overload of the ChWheeledVehicle function/member, adding the valve inputs
	void DWL2015_Vehicle::Synchronize(double time, double steering, double braking, double powertrain_torque, const TireForces & tire_forces, double * valve_inputs)
	{
		// Apply powertrain torque to the driveline's input shaft.
		m_driveline->Synchronize(powertrain_torque);

		// Let the steering subsystems process the steering input.
		for (unsigned int i = 0; i < m_steerings.size(); i++) {
			m_steerings[i]->Synchronize(time, steering);
		}

		// Apply tire forces to spindle bodies and apply braking.
		for (unsigned int i = 0; i < m_suspensions.size(); i++) {
			m_suspensions[i]->Synchronize(LEFT, tire_forces[2 * i]);
			m_suspensions[i]->Synchronize(RIGHT, tire_forces[2 * i + 1]);

			m_brakes[2 * i]->Synchronize(braking);
			m_brakes[2 * i + 1]->Synchronize(braking);
		}

		// Let the pneumatic actuators process the valve inputs
		m_loader->Synchronize(time, steering, valve_inputs); // to be changed,this represents idle rotation(that may be locked in the beginning)

	}
	/// Set visualization type for the FEL subsystem.
	/// This function should be called only after vehicle initialization.
	void DWL2015_Vehicle::SetFrontEndLoaderVisualizationType(VisualizationType vis) {
		m_loader->SetVisualizationType(vis);
	}


	protected:
	std::shared_ptr<DWL2015_FrontEndLoader> m_loader;  ///< handle to the front end loader subsystem
};

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono

#endif
