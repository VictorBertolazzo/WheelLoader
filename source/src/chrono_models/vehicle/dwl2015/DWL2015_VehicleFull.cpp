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
// Authors: Radu Serban, Justin Madsen, Daniel Melanz
// =============================================================================
//
// DWL2015 full vehicle model...
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/dwl2015/DWL2015_VehicleFull.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DWL2015_VehicleFull::DWL2015_VehicleFull(const bool fixed,
                                     DrivelineType driveType,
                                     ChMaterialSurfaceBase::ContactMethod contactMethod)
    : DWL2015_Vehicle(contactMethod, driveType) {
    Create(fixed);
}

DWL2015_VehicleFull::DWL2015_VehicleFull(ChSystem* system, const bool fixed, DrivelineType driveType)
    : DWL2015_Vehicle(system, driveType) {
    Create(fixed);
}


void DWL2015_VehicleFull::Create(bool fixed) {
    // -------------------------------------------
    // Create the chassis subsystem
    // -------------------------------------------
    m_chassis = std::make_shared<DWL2015_Chassis>("Chassis", fixed);

    // -------------------------------------------
    // Create the suspension subsystems
    // -------------------------------------------
    m_suspensions.resize(2);
    m_suspensions[0] = std::make_shared<DWL2015_DoubleWishboneFront>("FrontSusp");
    m_suspensions[1] = std::make_shared<DWL2015_DoubleWishboneRear>("RearSusp");

    // -----------------------------
    // Create the steering subsystem
    // -----------------------------
    m_steerings.resize(1);
    m_steerings[0] = std::make_shared<DWL2015_PitmanArm>("Steering");

    // -----------------
    // Create the wheels
    // -----------------
    m_wheels.resize(4);
    m_wheels[0] = std::make_shared<DWL2015_WheelLeft>("Wheel_FL");
    m_wheels[1] = std::make_shared<DWL2015_WheelRight>("Wheel_FR");
    m_wheels[2] = std::make_shared<DWL2015_WheelLeft>("Wheel_RL");
    m_wheels[3] = std::make_shared<DWL2015_WheelRight>("Wheel_RR");

    // --------------------
    // Create the driveline
    // --------------------
    switch (m_driveType) {
        case DrivelineType::FWD:
        case DrivelineType::RWD:
            m_driveline = std::make_shared<DWL2015_Driveline2WD>("Driveline");
            break;
        case DrivelineType::AWD:
            m_driveline = std::make_shared<DWL2015_Driveline4WD>("Driveline");
            break;
        case DrivelineType::SIMPLE:
            m_driveline = std::make_shared<DWL2015_SimpleDriveline>("Driveline");
            break;
    }

    // -----------------
    // Create the brakes
    // -----------------
    m_brakes.resize(4);
    m_brakes[0] = std::make_shared<DWL2015_BrakeSimple>("Brake_FL");
    m_brakes[1] = std::make_shared<DWL2015_BrakeSimple>("Brake_FR");
    m_brakes[2] = std::make_shared<DWL2015_BrakeSimple>("Brake_RL");
    m_brakes[3] = std::make_shared<DWL2015_BrakeSimple>("Brake_RR");

	// -----------------------------
	// Create the front end loader subsystem
	// -----------------------------
	m_loader = std::make_shared<DWL2015_FrontEndLoader>("Loader");

}

DWL2015_VehicleFull::~DWL2015_VehicleFull() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void DWL2015_VehicleFull::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel);

    // Initialize the steering subsystem (specify the steering subsystem's frame
    // relative to the chassis reference frame).
    ChVector<> offset = ChVector<>(1.24498, 0, 0.101322);
    ChQuaternion<> rotation = Q_from_AngAxis(18.5 * CH_C_PI / 180, ChVector<>(0, 1, 0));
    m_steerings[0]->Initialize(m_chassis->GetBody(), offset, rotation);

    // Initialize the suspension subsystems (specify the suspension subsystems'
    // frames relative to the chassis reference frame).
    m_suspensions[0]->Initialize(m_chassis->GetBody(), ChVector<>(1.688965, 0, 0), m_steerings[0]->GetSteeringLink(),
                                 m_omega[0], m_omega[1]);
    m_suspensions[1]->Initialize(m_chassis->GetBody(), ChVector<>(-1.688965, 0, 0), m_chassis->GetBody(), m_omega[2],
                                 m_omega[3]);

    // Initialize wheels
    m_wheels[0]->Initialize(m_suspensions[0]->GetSpindle(LEFT));
    m_wheels[1]->Initialize(m_suspensions[0]->GetSpindle(RIGHT));
    m_wheels[2]->Initialize(m_suspensions[1]->GetSpindle(LEFT));
    m_wheels[3]->Initialize(m_suspensions[1]->GetSpindle(RIGHT));

    // Initialize the driveline subsystem
    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());

    switch (m_driveType) {
        case DrivelineType::FWD:
            driven_susp_indexes[0] = 0;
            break;
        case DrivelineType::RWD:
            driven_susp_indexes[0] = 1;
            break;
        case DrivelineType::AWD:
        case DrivelineType::SIMPLE:
            driven_susp_indexes[0] = 0;
            driven_susp_indexes[1] = 1;
            break;
    }

    m_driveline->Initialize(m_chassis->GetBody(), m_suspensions, driven_susp_indexes);

    // Initialize the four brakes
    m_brakes[0]->Initialize(m_suspensions[0]->GetRevolute(LEFT));
    m_brakes[1]->Initialize(m_suspensions[0]->GetRevolute(RIGHT));
    m_brakes[2]->Initialize(m_suspensions[1]->GetRevolute(LEFT));
    m_brakes[3]->Initialize(m_suspensions[1]->GetRevolute(RIGHT));
	
	// Initialize the frontendloader subsystem (specify the frontendloader subsystem's frame
	// relative to the chassis reference frame).
	ChVector<> off = ChVector<>(0.0, 0, 0.0);
	ChQuaternion<> rot = Q_from_AngAxis(0 * CH_C_PI / 180, ChVector<>(0, 1, 0));
	m_loader->Initialize(m_chassis->GetBody(), off, rot);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double DWL2015_VehicleFull::GetSpringForce(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetSpringForce(wheel_id.side());
}

double DWL2015_VehicleFull::GetSpringLength(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetSpringLength(wheel_id.side());
}

double DWL2015_VehicleFull::GetSpringDeformation(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetSpringDeformation(wheel_id.side());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double DWL2015_VehicleFull::GetShockForce(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetShockForce(wheel_id.side());
}

double DWL2015_VehicleFull::GetShockLength(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetShockLength(wheel_id.side());
}

double DWL2015_VehicleFull::GetShockVelocity(const WheelID& wheel_id) const {
    return std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[wheel_id.axle()])->GetShockVelocity(wheel_id.side());
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void DWL2015_VehicleFull::LogHardpointLocations() {
    GetLog().SetNumFormat("%7.3f");

    GetLog() << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[0])->LogHardpointLocations(ChVector<>(-37.78, 0, 30.77), true);

    GetLog() << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChDoubleWishbone>(m_suspensions[1])->LogHardpointLocations(ChVector<>(-170.77, 0, 30.77), true);

    GetLog() << "\n\n";

    GetLog().SetNumFormat("%g");
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------
void DWL2015_VehicleFull::DebugLog(int what) {
    GetLog().SetNumFormat("%10.2f");

    if (what & OUT_SPRINGS) {
        GetLog() << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [m]       " << GetSpringLength(FRONT_LEFT) << "  " << GetSpringLength(FRONT_RIGHT) << "  "
                 << GetSpringLength(REAR_LEFT) << "  " << GetSpringLength(REAR_RIGHT) << "\n";
        GetLog() << "Deformation [m]  " << GetSpringDeformation(FRONT_LEFT) << "  " << GetSpringDeformation(FRONT_RIGHT)
                 << "  " << GetSpringDeformation(REAR_LEFT) << "  " << GetSpringDeformation(REAR_RIGHT) << "\n";
        GetLog() << "Force [N]         " << GetSpringForce(FRONT_LEFT) << "  " << GetSpringForce(FRONT_RIGHT) << "  "
                 << GetSpringForce(REAR_LEFT) << "  " << GetSpringForce(REAR_RIGHT) << "\n";
    }

    if (what & OUT_SHOCKS) {
        GetLog() << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
        GetLog() << "Length [m]       " << GetShockLength(FRONT_LEFT) << "  " << GetShockLength(FRONT_RIGHT) << "  "
                 << GetShockLength(REAR_LEFT) << "  " << GetShockLength(REAR_RIGHT) << "\n";
        GetLog() << "Velocity [m/s]   " << GetShockVelocity(FRONT_LEFT) << "  " << GetShockVelocity(FRONT_RIGHT) << "  "
                 << GetShockVelocity(REAR_LEFT) << "  " << GetShockVelocity(REAR_RIGHT) << "\n";
        GetLog() << "Force [N]         " << GetShockForce(FRONT_LEFT) << "  " << GetShockForce(FRONT_RIGHT) << "  "
                 << GetShockForce(REAR_LEFT) << "  " << GetShockForce(REAR_RIGHT) << "\n";
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }

    GetLog().SetNumFormat("%g");
}


}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono
