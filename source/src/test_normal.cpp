#include <iostream>//Add 

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono/utils/ChUtilsGenerators.h"

#include <stdio.h>
#include <vector>
#include <cmath>

#include <irrlicht.h>

#include "chrono_opengl/ChOpenGLWindow.h"


// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;


int main(int argc, char* argv[]) {
	// 0. Set the path to the Chrono data folder
	SetChronoDataPath(CHRONO_DATA_DIR);
	// 1. Create the system
    ChSystem system;
	system.Set_G_acc(ChVector<>(0.0,0.0, +9.81));
		ChVector<> COG_bucket(4.0,.0, 0.525);
		ChVector<> POS_lift2bucket(3.50, .0,.21);//chassis piston(LIFT ARM) insertion abs frame
		ChVector<> POS_Ball(3.50, 0.0, .21-3.0);
		ChQuaternion<> z2y;
		ChQuaternion<> z2x;
		z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
		z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
		// BUCKET
		auto bucket = std::make_shared<ChBodyAuxRef>();
		system.AddBody(bucket);
		bucket->SetBodyFixed(true);
		bucket->SetName("benna");
		//bucket->SetMaterialSurface(ChMaterialSurface::DEM);
		bucket->SetIdentifier(4);
		bucket->SetMass(1.0);
		bucket->SetPos(POS_lift2bucket);
		bucket->SetRot(QUNIT);
		//bucket->SetFrame_COG_to_REF(ChFrame<> (bucket->GetFrame_REF_to_abs().GetInverse() * COG_bucket,QUNIT));
		bucket->SetInertiaXX(ChVector<>(.5, .5, .5));
		// BUCKET Coll and Vis Update
						// trimesh Usage
						geometry::ChTriangleMeshConnected m_trimesh;
						int n_verts = 8;
						int n_faces = 12;
						// Resize mesh arrays.
						m_trimesh.getCoordsVertices().resize(n_verts);
						m_trimesh.getCoordsNormals().resize(n_verts);
						m_trimesh.getCoordsUV().resize(n_verts);
						m_trimesh.getCoordsColors().resize(n_verts);

						m_trimesh.getIndicesVertexes().resize(n_faces);
						m_trimesh.getIndicesNormals().resize(n_faces);
						// Load mesh vertices.
						std::cout << "Load vertices...maybe" << std::endl;
						float hy = 1.2;
						float hx = .7;
						float hz = .7;

						m_trimesh.getCoordsVertices()[0] = ChVector<>(hx,-hy,hz);
						m_trimesh.getCoordsVertices()[1] = ChVector<>(hx, -hy,-hz);
						m_trimesh.getCoordsVertices()[2] = ChVector<>(-hx, -hy, hz);
						m_trimesh.getCoordsVertices()[3] = ChVector<>(-hx, -hy, -hz);

						m_trimesh.getCoordsVertices()[4] = ChVector<>(hx,hy,hz);
						m_trimesh.getCoordsVertices()[5] = ChVector<>(hx, hy, -hz);
						m_trimesh.getCoordsVertices()[6] = ChVector<>(-hx, hy, hz);
						m_trimesh.getCoordsVertices()[7] = ChVector<>(-hx, hy, -hz);

						// Load mesh faces
						// Specify the face vertices counter-clockwise.
						// Set the normal indices same as the vertex indices.
						std::cout << "Load faces..." << std::endl;
						auto ones = ChVector<int>(1, 1, 1);
						m_trimesh.getIndicesVertexes()[0] = ChVector<int>(1, 4, 2) - ones;
						m_trimesh.getIndicesVertexes()[1] = ChVector<int>(1, 3, 4) - ones;
						m_trimesh.getIndicesVertexes()[2] = ChVector<int>(5, 6, 8) - ones;
						m_trimesh.getIndicesVertexes()[3] = ChVector<int>(5, 8, 7) - ones;
						m_trimesh.getIndicesVertexes()[4] = ChVector<int>(1, 2, 6) - ones;
						m_trimesh.getIndicesVertexes()[5] = ChVector<int>(1, 6, 5) - ones;
						m_trimesh.getIndicesVertexes()[6] = ChVector<int>(3, 7, 8) - ones;
						m_trimesh.getIndicesVertexes()[7] = ChVector<int>(3, 8, 4) - ones;
						m_trimesh.getIndicesVertexes()[8] = ChVector<int>(1, 5, 7) - ones;
						m_trimesh.getIndicesVertexes()[9] = ChVector<int>(1, 7, 3) - ones;
						m_trimesh.getIndicesVertexes()[10] = ChVector<int>(2, 4, 8) - ones;
						m_trimesh.getIndicesVertexes()[11] = ChVector<int>(2, 8, 6) - ones;

						// Let use the same
						for (int j = 0; j < n_faces; j++) {
							m_trimesh.getIndicesNormals()[j] = m_trimesh.getIndicesVertexes()[j];
						}
						// Readibility aliases
						std::vector<ChVector<> >& vertices = m_trimesh.getCoordsVertices();
						std::vector<ChVector<> >& normals = m_trimesh.getCoordsNormals();
						std::vector<ChVector<int> >& idx_vertices = m_trimesh.getIndicesVertexes();
						std::vector<ChVector<int> >& idx_normals = m_trimesh.getIndicesNormals();

						for (unsigned int it = 0; it < n_faces; ++it) {
							// Calculate the triangle normal as a normalized cross product.
							ChVector<> nrm = Vcross(vertices[idx_vertices[it].y] - vertices[idx_vertices[it].x], vertices[idx_vertices[it].z] - vertices[idx_vertices[it].x]);
							nrm.Normalize();
							// Increment the normals of all incident vertices by the face normal
							normals[idx_normals[it].x] += nrm;
							normals[idx_normals[it].y] += nrm;
							normals[idx_normals[it].z] += nrm;
							// Increment the count of all incident vertices by 1
							//accumulators[idx_normals[it].x] += 1;
							//accumulators[idx_normals[it].y] += 1;
							//accumulators[idx_normals[it].z] += 1;
						}
						auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
						trimesh_shape->SetMesh(m_trimesh);
						trimesh_shape->SetName("triangular_mesh");
						bucket->AddAsset(trimesh_shape);


						// Create contact geometry.
						bucket->SetCollide(true);
						bucket->GetCollisionModel()->ClearModel();
						//bucket->GetCollisionModel()->AddBox(hx, hy, hz, VNULL);
						bucket->GetCollisionModel()->AddTriangleMesh(m_trimesh, true, false, ChVector<>(0, 0, 0));
						bucket->GetCollisionModel()->BuildModel();
		

						auto ball = std::make_shared<ChBodyEasySphere>(.63,1.,true,true,ChMaterialSurface::DVI);
						ball->SetPos(POS_Ball);
						ball->SetCollide(true);
						system.Add(ball);

	

		// Create the Irrlicht application and set-up the camera.
		ChIrrApp * application = new ChIrrApp(
			&system,                               // pointer to the mechanical system
			L"WL First Example",                // title of the Irrlicht window
			core::dimension2d<u32>(800, 600),      // window dimension (width x height)
			false,                                 // use full screen?
			true);                                 // enable shadows?
		application->AddTypicalLogo();
		application->AddTypicalSky();
		application->AddTypicalLights();
		application->AddTypicalCamera(core::vector3df(2, 5, -3), core::vector3df(POS_lift2bucket.x, POS_lift2bucket.y, POS_lift2bucket.z)); //'camera' location            // "look at" location
																							// Let the Irrlicht application convert the visualization assets.
		application->AssetBindAll();
		application->AssetUpdateAll();

		application->SetTimestep(0.01);
		application->SetTryRealtime(true);

		while (application->GetDevice()->run()) {

			// Irrlicht must prepare frame to draw
			application->BeginScene();
			// Irrlicht application draws all 3D objects and all GUI items
			application->DrawAll();
			// Draw an XZ grid at the global origin to add in visualization.
			ChIrrTools::drawGrid(
				application->GetVideoDriver(), 1, 1, 20, 20,
				ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
				video::SColor(255, 80, 100, 100), true);

			// Advance the simulation time step
			application->DoStep();


			// Irrlicht must finish drawing the frame
			application->EndScene();
		}



		return 0;
}