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
// Wrapper classes for modeling an entire DWL2015 vehicle assembly
// (including the vehicle itself, the powertrain, and the tires).
//
// =============================================================================

#ifndef DWL2015_H
#define DWL2015_H

#include <array>
#include <string>

#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"

#include "chrono_parallel/physics/ChSystemParallel.h"//Add 5/12, maybe a file with this one called

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_VehicleFull.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_VehicleReduced.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_Powertrain.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_SimplePowertrain.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_RigidTire.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_LugreTire.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_FialaTire.h"

#ifdef CHRONO_FEA
#include "chrono_models/vehicle/dwl2015/DWL2015_ANCFTire.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_ReissnerTire.h"
#endif

namespace chrono {
namespace vehicle {
namespace dwl2015 {

class CH_MODELS_API DWL2015 {
		public:
			virtual ~DWL2015();

			void SetContactMethod(ChMaterialSurfaceBase::ContactMethod val) { m_contactMethod = val; }

			void SetChassisFixed(bool val) { m_fixed = val; }

			void SetDriveType(DrivelineType val) { m_driveType = val; }
			void SetPowertrainType(PowertrainModelType val) { m_powertrainType = val; }
			void SetTireType(TireModelType val) { m_tireType = val; }

			void SetInitPosition(const ChCoordsys<>& pos) { m_initPos = pos; }
			void SetInitFwdVel(double fwdVel) { m_initFwdVel = fwdVel; }
			void SetInitWheelAngVel(const std::vector<double>& omega) { m_initOmega = omega; }

			void SetTireStepSize(double step_size) { m_tire_step_size = step_size; }
			void SetPacejkaParamfile(const std::string& filename) { m_pacejkaParamFile = filename; }

			ChSystem* GetSystem() const { return m_vehicle->GetSystem(); }
			ChSystemParallel* GetSystemParallel() const { return m_vehicle->GetSystemParallel(); }// Add 5/12
			ChWheeledVehicle& GetVehicle() const { return *m_vehicle; }
			std::shared_ptr<ChChassis> GetChassis() const { return m_vehicle->GetChassis(); }
			std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_vehicle->GetChassisBody(); }
			ChPowertrain& GetPowertrain() const { return *m_powertrain; }
			ChTire* GetTire(WheelID which) const { return m_tires[which.id()]; }
			std::shared_ptr<ChFrontEndLoader> GetFrontEndLoader() const { return m_vehicle->GetFrontEndLoader(); }
//			double GetPneumaticForce(std::shared_ptr<ChLinkPneumaticActuator> pneum) const { return m_vehicle->GetPneumaticForce(pneum); }

			void Initialize();

			void SetChassisVisualizationType(VisualizationType vis) { m_vehicle->SetChassisVisualizationType(vis); }
			void SetSuspensionVisualizationType(VisualizationType vis) { m_vehicle->SetSuspensionVisualizationType(vis); }
			void SetSteeringVisualizationType(VisualizationType vis) { m_vehicle->SetSteeringVisualizationType(vis); }
			void SetFrontEndLoaderVisualizationType(VisualizationType vis) {m_vehicle->SetFrontEndLoaderVisualizationType(vis);} 
			void SetWheelVisualizationType(VisualizationType vis) { m_vehicle->SetWheelVisualizationType(vis); }
			void SetTireVisualizationType(VisualizationType vis);

			void Synchronize(double time,
				double steering_input,
				double braking_input,
				double throttle_input,
				const ChTerrain& terrain);
			void Synchronize(double time,
				double steering_input,
				double braking_input,
				double throttle_input,
				const ChTerrain& terrain,
				double valve_inputs[4]);

			void Advance(double step);

		protected:
			// Protected constructors -- this class cannot be instantiated by itself.
			DWL2015();
			DWL2015(ChSystem* system);
			DWL2015(ChSystemParallel* system);//Add 5/12

			virtual DWL2015_Vehicle* CreateVehicle() = 0;

			ChMaterialSurfaceBase::ContactMethod m_contactMethod;
			bool m_fixed;

			DrivelineType m_driveType;
			PowertrainModelType m_powertrainType;
			TireModelType m_tireType;

			double m_tire_step_size;
			std::string m_pacejkaParamFile;

			ChCoordsys<> m_initPos;
			double m_initFwdVel;
			std::vector<double> m_initOmega;

			ChSystem* m_system;
			ChSystemParallel* m_system_parallel;//Add 5/12
			DWL2015_Vehicle* m_vehicle;
			ChPowertrain* m_powertrain;
			std::array<ChTire*, 4> m_tires;
		};

class CH_MODELS_API DWL2015_Full : public DWL2015 {
		public:
			DWL2015_Full() {}
			DWL2015_Full(ChSystem* system) : DWL2015(system) {}
			DWL2015_Full(ChSystemParallel* system) : DWL2015(system) {}// Add 5/12


			void LogHardpointLocations() { ((DWL2015_VehicleFull*)m_vehicle)->LogHardpointLocations(); }
			void DebugLog(int what) { ((DWL2015_VehicleFull*)m_vehicle)->DebugLog(what); }

			double GetPneumaticForce(int what) const { return ((DWL2015_VehicleFull*)m_vehicle)->GetPneumaticForce(what); }
			void SetValveCommand(int what, double a, double b) const { return ((DWL2015_VehicleFull*)m_vehicle)->SetValveCommand(what, a, b); }
			double *GetValveCommand(int what) const { return ((DWL2015_VehicleFull*)m_vehicle)->GetValveCommand(what); }




		private:
			virtual DWL2015_Vehicle* CreateVehicle() override {
				return m_system ? new DWL2015_VehicleFull(m_system, m_fixed, m_driveType)
					: new DWL2015_VehicleFull(m_fixed, m_driveType, m_contactMethod);
			}
		};

class CH_MODELS_API DWL2015_Reduced : public DWL2015 {
		public:
			DWL2015_Reduced() {}
			DWL2015_Reduced(ChSystem* system) : DWL2015(system) {}

		private:
			virtual DWL2015_Vehicle* CreateVehicle() override {
				return m_system ? new DWL2015_VehicleReduced(m_system, m_fixed, m_driveType)
					: new DWL2015_VehicleReduced(m_fixed, m_driveType, m_contactMethod);
			}
		};

}  // end namespace dwl2015
}      // end namespace vehicle
}      // end namespace chrono 
#endif
