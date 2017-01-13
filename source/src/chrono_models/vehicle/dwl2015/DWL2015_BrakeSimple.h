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
// DWL2015 simple brake models (front and rear).
//
// =============================================================================

#ifndef DWL2015_BRAKESIMPLE_H
#define DWL2015_BRAKESIMPLE_H

#include "chrono_vehicle/wheeled_vehicle/brake/ChBrakeSimple.h"

#include "chrono_models/ChApiModels.h"


namespace chrono {
namespace vehicle {
namespace dwl2015 {

class CH_MODELS_API DWL2015_BrakeSimple : public ChBrakeSimple {
	public:
		DWL2015_BrakeSimple(const std::string& name);
		virtual ~DWL2015_BrakeSimple() {}

		virtual double GetMaxBrakingTorque() override { return m_maxtorque; }

	private:
		static const double m_maxtorque;
};

}  // end namespace chrono
}  // end namespace vehicle
}  // end namespace chrono

#endif
