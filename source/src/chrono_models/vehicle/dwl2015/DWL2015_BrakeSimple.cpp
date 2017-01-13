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
// Authors: Alessandro Tasora
// =============================================================================
//
// DWL2015 simple brake model.
//
// =============================================================================

#include "chrono_models/vehicle/dwl2015/DWL2015_BrakeSimple.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

const double DWL2015_BrakeSimple::m_maxtorque = 4000;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DWL2015_BrakeSimple::DWL2015_BrakeSimple(const std::string& name) : ChBrakeSimple(name) {}

}  // end namespace hmmwv
}  // end namespace vehicle
}  // end namespace chrono
