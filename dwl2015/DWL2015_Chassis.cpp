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
// DWL2015 chassis subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/dwl2015/DWL2015_Chassis.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const double DWL2015_Chassis::m_mass = 2086.52;
const ChVector<> DWL2015_Chassis::m_inertia(1078.52, 2955.66, 3570.20);
const ChVector<> DWL2015_Chassis::m_COM_loc(0.056, 0, 0.523);
const ChCoordsys<> DWL2015_Chassis::m_driverCsys(ChVector<>(0.87, -0.27, 1.05), ChQuaternion<>(1, 0, 0, 0));

const std::string DWL2015_Chassis::m_meshName = "hmmwv_chassis_POV_geom"; 
const std::string DWL2015_Chassis::m_meshFile = "dwl2015/hmmwv_chassis.obj";// I keep hmmwv mesh and obj files for the moment

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DWL2015_Chassis::DWL2015_Chassis(const std::string& name, bool fixed) : ChChassis(name, fixed) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void DWL2015_Chassis::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetName(m_meshName);
        m_body->AddAsset(trimesh_shape);
    } else {
        ChChassis::AddVisualizationAssets(vis);
    }
}

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono
