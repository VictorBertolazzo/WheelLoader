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
// DWL2015 wheel subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double DWL2015_RigidTire::m_radius = 18.5 * in2m;
const double DWL2015_RigidTire::m_width = 10 * in2m;

const std::string DWL2015_RigidTire::m_meshName = "hmmwv_tire_POV_geom";
const std::string DWL2015_RigidTire::m_meshFile = "hmmwv/hmmwv_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DWL2015_RigidTire::DWL2015_RigidTire(const std::string& name, bool use_mesh) : ChRigidTire(name) {
	SetContactFrictionCoefficient(0.9f);
	SetContactRestitutionCoefficient(0.1f);
	SetContactMaterialProperties(2e7f, 0.3f);
	SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);

	if (use_mesh) {
		SetMeshFilename(GetDataFile("hmmwv/hmmwv_tire.obj"), 0.005);
	}
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void DWL2015_RigidTire::AddVisualizationAssets(VisualizationType vis) {
	if (vis == VisualizationType::MESH) {
		geometry::ChTriangleMeshConnected trimesh;
		trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
		m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
		m_trimesh_shape->SetMesh(trimesh);
		m_trimesh_shape->SetName(m_meshName);
		m_wheel->AddAsset(m_trimesh_shape);
	}
	else {
		ChRigidTire::AddVisualizationAssets(vis);
	}
}

void DWL2015_RigidTire::RemoveVisualizationAssets() {
	ChRigidTire::RemoveVisualizationAssets();

	// Make sure we only remove the assets added by DWL2015_RigidTire::AddVisualizationAssets.
	// This is important for the ChTire object because a wheel may add its own assets
	// to the same body (the spindle/wheel).
	auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
	if (it != m_wheel->GetAssets().end())
		m_wheel->GetAssets().erase(it);
}

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono

