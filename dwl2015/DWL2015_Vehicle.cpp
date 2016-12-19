#include "DWL2015_Vehicle.h"

void chrono::vehicle::dwl2015::DWL2015_Vehicle::Synchronize(double time, 
															double steering, 
															double braking, 
															double powertrain_torque, 
															const TireForces & tire_forces, 
															double pneumatic1, 
															double pneumatic2)
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
    
	// Apply valve command to the pistons:
}
