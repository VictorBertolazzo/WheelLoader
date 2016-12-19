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
// DWL2015 rack-pinion steering model.
//
// =============================================================================

#include "chrono_models/vehicle/dwl2015/DWL2015_RackPinion.h"

namespace chrono{
namespace vehicle{
namespace dwl2015 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double DWL2015_RackPinion::m_steeringLinkMass = 9.072;
const ChVector<> DWL2015_RackPinion::m_steeringLinkInertia(1, 1, 1);
const double DWL2015_RackPinion::m_steeringLinkCOM = 0;
const double DWL2015_RackPinion::m_steeringLinkLength = 0.896;
const double DWL2015_RackPinion::m_steeringLinkRadius = 0.03;

const double DWL2015_RackPinion::m_pinionRadius = 0.1;

const double DWL2015_RackPinion::m_maxAngle = 50.0 * (CH_C_PI / 180);

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DWL2015_RackPinion::DWL2015_RackPinion(const std::string& name) : ChRackPinion(name) {}

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono
