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
// DWL2015 LuGre tire subsystem
//
// =============================================================================

#ifndef DWL2015_LUGRE_TIRE_H
#define DWL2015_LUGRE_TIRE_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChLugreTire.h"

#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace dwl2015 {

class CH_MODELS_API DWL2015_LugreTire : public ChLugreTire {
  public:
    DWL2015_LugreTire(const std::string& name);
    ~DWL2015_LugreTire() {}

    virtual double GetRadius() const override { return m_radius; }
    virtual int GetNumDiscs() const override { return m_numDiscs; }
    virtual const double* GetDiscLocations() const override { return m_discLocs; }

    virtual double GetNormalStiffness() const override { return m_normalStiffness; }
    virtual double GetNormalDamping() const override { return m_normalDamping; }

    virtual void SetLugreParams() override;

	virtual void AddVisualizationAssets(VisualizationType vis) override;
	virtual void RemoveVisualizationAssets() override final;

private:
	static const double m_radius;
	static const int m_numDiscs = 3;
	static const double m_discLocs[m_numDiscs];

	static const double m_normalStiffness;
	static const double m_normalDamping;

	static const std::string m_meshName;
	static const std::string m_meshFile;
	std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

}  // end namespace dwl2015
}  // end namespace vehicle
}  // end namespace chrono

#endif
