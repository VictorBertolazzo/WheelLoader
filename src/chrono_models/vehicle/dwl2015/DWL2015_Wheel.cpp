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
// DWL2015 wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/dwl2015/DWL2015_Wheel.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

	const double DWL2015_Wheel::m_mass = 45.4;
	const ChVector<> DWL2015_Wheel::m_inertia(0.113, 0.113, 0.113);

	const double DWL2015_Wheel::m_radius = 0.268;
	const double DWL2015_Wheel::m_width = 0.22;

const std::string DWL2015_WheelLeft::m_meshName = "wheel_L_POV_geom";
const std::string DWL2015_WheelLeft::m_meshFile = vehicle::GetDataFile("hmmwv/wheel_L.obj");

const std::string DWL2015_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string DWL2015_WheelRight::m_meshFile = vehicle::GetDataFile("hmmwv/wheel_R.obj");

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DWL2015_Wheel::DWL2015_Wheel(const std::string& name) : ChWheel(name) {}

DWL2015_WheelLeft::DWL2015_WheelLeft(const std::string& name) : DWL2015_Wheel(name) {}

DWL2015_WheelRight::DWL2015_WheelRight(const std::string& name) : DWL2015_Wheel(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void DWL2015_Wheel::AddVisualizationAssets(VisualizationType vis) {
	if (vis == VisualizationType::MESH) {
		geometry::ChTriangleMeshConnected trimesh;
		trimesh.LoadWavefrontMesh(GetMeshFile(), false, false);
		m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
		m_trimesh_shape->SetMesh(trimesh);
		m_trimesh_shape->SetName(GetMeshName());
		m_spindle->AddAsset(m_trimesh_shape);
	}
	else {
		ChWheel::AddVisualizationAssets(vis);
	}
}

void DWL2015_Wheel::RemoveVisualizationAssets() {
	ChWheel::RemoveVisualizationAssets();

	// Make sure we only remove the assets added by DWL2015_Wheel::AddVisualizationAssets.
	// This is important for the ChWheel object because a tire may add its own assets
	// to the same body (the spindle).
	auto it = std::find(m_spindle->GetAssets().begin(), m_spindle->GetAssets().end(), m_trimesh_shape);
	if (it != m_spindle->GetAssets().end())
		m_spindle->GetAssets().erase(it);
}

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono
