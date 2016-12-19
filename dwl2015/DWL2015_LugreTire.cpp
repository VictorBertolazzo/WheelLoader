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
// DWL2015 LuGre subsystem
//
// =============================================================================

#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_models/vehicle/dwl2015/DWL2015_LugreTire.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double DWL2015_LugreTire::m_radius = 18.15 * in2m;
const double DWL2015_LugreTire::m_discLocs[] = {-5 * in2m, 0 * in2m, 5 * in2m};

const double DWL2015_LugreTire::m_normalStiffness = 2e6;
const double DWL2015_LugreTire::m_normalDamping = 1e3;

const std::string DWL2015_LugreTire::m_meshName = "hmmwv_tire_POV_geom";  // for the moment I don't own DWL obj files
const std::string DWL2015_LugreTire::m_meshFile = "hmmwv/hmmwv_tire.obj"; // ""


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DWL2015_LugreTire::DWL2015_LugreTire(const std::string& name) : ChLugreTire(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void DWL2015_LugreTire::SetLugreParams() {
    // Longitudinal direction
    m_sigma0[0] = 181;
    m_sigma1[0] = 1;
    m_sigma2[0] = 0.02;

    m_Fc[0] = 0.6;
    m_Fs[0] = 1.0;

    m_vs[0] = 3.5;

    // Lateral direction
    m_sigma0[1] = 60;
    m_sigma1[1] = 0.1;
    m_sigma2[1] = 0.002;

    m_Fc[1] = 0.6;
    m_Fs[1] = 1.0;

    m_vs[1] = 3.5;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void DWL2015_LugreTire::AddVisualizationAssets(VisualizationType vis) {
	if (vis == VisualizationType::MESH) {
		geometry::ChTriangleMeshConnected trimesh;
		trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
		m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
		m_trimesh_shape->SetMesh(trimesh);
		m_trimesh_shape->SetName(m_meshName);
		m_wheel->AddAsset(m_trimesh_shape);
	}
	else {
		ChLugreTire::AddVisualizationAssets(vis);
	}
}

void DWL2015_LugreTire::RemoveVisualizationAssets() {
	ChLugreTire::RemoveVisualizationAssets();

	// Make sure we only remove the assets added by DWL2015_LugreTire::AddVisualizationAssets.
	// This is important for the ChTire object because a wheel may add its own assets
	// to the same body (the spindle/wheel).
	auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
	if (it != m_wheel->GetAssets().end())
		m_wheel->GetAssets().erase(it);
}

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono
