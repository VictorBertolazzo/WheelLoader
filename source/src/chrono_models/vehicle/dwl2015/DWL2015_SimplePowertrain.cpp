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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Simple powertrain model for the DWL2015 vehicle.
// - trivial speed-torque curve
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/dwl2015/DWL2015_SimplePowertrain.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double DWL2015_SimplePowertrain::m_max_torque = 2400 / 8.851;
const double DWL2015_SimplePowertrain::m_max_speed = 2000;
const double DWL2015_SimplePowertrain::m_fwd_gear_ratio = 0.3;
const double DWL2015_SimplePowertrain::m_rev_gear_ratio = -0.3;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DWL2015_SimplePowertrain::DWL2015_SimplePowertrain() : ChSimplePowertrain() {}

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono
