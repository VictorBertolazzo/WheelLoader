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
// DWL2015 9-body vehicle model...
//
// =============================================================================

#ifndef DWL2015_VEHICLE_REDUCED_H
#define DWL2015_VEHICLE_REDUCED_H

#include "chrono_models/ChApiModels.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_Vehicle.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_Chassis.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_BrakeSimple.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_DoubleWishboneReduced.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_Driveline2WD.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_Driveline4WD.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_SimpleDriveline.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_RackPinion.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_Wheel.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

class CH_MODELS_API DWL2015_VehicleReduced : public DWL2015_Vehicle {
  public:
    DWL2015_VehicleReduced(const bool fixed = false,
                         DrivelineType driveType = DrivelineType::AWD,
                         ChMaterialSurfaceBase::ContactMethod contactMethod = ChMaterialSurfaceBase::DVI);

    DWL2015_VehicleReduced(ChSystem* system, const bool fixed = false, DrivelineType driveType = DrivelineType::AWD);

    ~DWL2015_VehicleReduced();

    virtual void Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel = 0) override;

  private:
    void Create(bool fixed);
};

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono

#endif
