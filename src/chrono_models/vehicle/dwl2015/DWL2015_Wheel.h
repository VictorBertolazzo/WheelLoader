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

#ifndef DWL2015_WHEEL_H
#define DWL2015_WHEEL_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheel.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

class CH_MODELS_API DWL2015_Wheel : public ChWheel {
  public:
    DWL2015_Wheel(const std::string& name);
    ~DWL2015_Wheel() {}

	virtual double GetMass() const override { return m_mass; }
	virtual ChVector<> GetInertia() const override { return m_inertia; }
	virtual double GetRadius() const { return m_radius; }
	virtual double GetWidth() const { return m_width; }

	virtual void AddVisualizationAssets(VisualizationType vis) override;
	virtual void RemoveVisualizationAssets() override;

protected:
	virtual std::string GetMeshName() const = 0;
	virtual std::string GetMeshFile() const = 0;

	std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;

	static const double m_radius;
	static const double m_width;
	static const double m_mass;
	static const ChVector<> m_inertia;
};

class DWL2015_WheelLeft : public DWL2015_Wheel {
  public:
    DWL2015_WheelLeft(const std::string& name);
    ~DWL2015_WheelLeft() {}

	virtual std::string GetMeshName() const override { return m_meshName; }
	virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

private:
	static const std::string m_meshName;
	static const std::string m_meshFile;
};

class DWL2015_WheelRight : public DWL2015_Wheel {
  public:
    DWL2015_WheelRight(const std::string& name);
    ~DWL2015_WheelRight() {}

	virtual std::string GetMeshName() const override { return m_meshName; }
	virtual std::string GetMeshFile() const override { return GetDataFile(m_meshFile); }

private:
	static const std::string m_meshName;
	static const std::string m_meshFile;
};

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono

#endif
