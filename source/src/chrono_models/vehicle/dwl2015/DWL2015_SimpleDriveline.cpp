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
// DWL2015 simple driveline model.
//
// =============================================================================

#include "chrono_models/vehicle/dwl2015/DWL2015_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double DWL2015_SimpleDriveline::m_front_torque_frac = 0.5;
const double DWL2015_SimpleDriveline::m_front_diff_bias = 2.0;
const double DWL2015_SimpleDriveline::m_rear_diff_bias = 2.0;

// -----------------------------------------------------------------------------
// Constructor of DWL2015_SimpleDriveline.
// -----------------------------------------------------------------------------
DWL2015_SimpleDriveline::DWL2015_SimpleDriveline(const std::string& name) : ChSimpleDriveline(name) {}

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono
