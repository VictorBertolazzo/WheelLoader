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
// DWL2015 rigid tire subsystem
//
// =============================================================================

#ifndef DWL2015_RIGID_TIRE_H
#define DWL2015_RIGID_TIRE_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChRigidTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

class CH_MODELS_API DWL2015_RigidTire : public ChRigidTire {
  public:
    DWL2015_RigidTire(const std::string& name, bool use_mesh = false);
    ~DWL2015_RigidTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual double GetWidth() const override { return m_width; }

	virtual void AddVisualizationAssets(VisualizationType vis) override;
	virtual void RemoveVisualizationAssets() override final;

private:
	static const double m_radius;
	static const double m_width;

	static const std::string m_meshName;
	static const std::string m_meshFile;
	std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono


#endif
