#include <iostream>//Add 
#include <stdio.h>
#include <vector>
#include <cmath>
#include <irrlicht.h>


#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/collision/ChCCollisionUtils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_opengl/ChOpenGLWindow.h"

#include "chrono/physics/ChContactContainerNSC.h"
#include "chrono/solver/ChSolverSOR.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChContactContainerSMC.h"
#include "chrono/solver/ChSolverSMC.h"



// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;

//#define USE_PENALTY

enum BucketSide { LEFT, RIGHT};


struct Points {
	Points() {}
	Points(float x, float y, float z)
		: mx(x), my(y), mz(z) {}
	float mx; float my; float mz;
};
void ReadFile(const std::string& filename, std::vector<Points>& profile) {
	std::ifstream ifile(filename.c_str());
	std::string line;

	while (std::getline(ifile, line)) {
		std::istringstream iss(line);
		float xpos, ypos, zpos;
		iss >> xpos >> ypos >> zpos;
		if (iss.fail())
			break;
		profile.push_back(Points(xpos, ypos, zpos));
	}
	ifile.close();



}
void AddBucketMesh(std::vector<Points> p_ext, std::vector<Points> p_int, std::shared_ptr<ChBody> bucket) {
	// it's missing check for profile equal sizes
	
	for (int iter = 0; iter < p_ext.size() - 1; iter++) {
		// trimesh Usage
		geometry::ChTriangleMeshConnected m_trimesh;
		// readability Aliases
		std::vector<ChVector<> >& vertices = m_trimesh.getCoordsVertices();
		std::vector<ChVector<> >& normals = m_trimesh.getCoordsNormals();
		std::vector<ChVector<int> >& idx_vertices = m_trimesh.getIndicesVertexes();
		std::vector<ChVector<int> >& idx_normals = m_trimesh.getIndicesNormals();
		// problem size
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
		std::cout << "Load vertices.." << std::endl;
		float hy = .5;
		m_trimesh.getCoordsVertices()[0] = ChVector<>(p_int[iter].mx, -hy / 2, p_int[iter].mz);
		m_trimesh.getCoordsVertices()[3] = ChVector<>(p_int[iter + 1].mx, -hy / 2, p_int[iter + 1].mz);
		m_trimesh.getCoordsVertices()[2] = ChVector<>(p_ext[iter + 1].mx, -hy / 2, p_ext[iter + 1].mz);
		m_trimesh.getCoordsVertices()[1] = ChVector<>(p_ext[iter].mx, -hy / 2, p_ext[iter].mz);
		
		m_trimesh.getCoordsVertices()[4] = ChVector<>(p_int[iter].mx, +hy / 2, p_int[iter].mz);
		m_trimesh.getCoordsVertices()[7] = ChVector<>(p_int[iter + 1].mx, +hy / 2, p_int[iter + 1].mz);
		m_trimesh.getCoordsVertices()[6] = ChVector<>(p_ext[iter + 1].mx, +hy / 2, p_ext[iter + 1].mz);
		m_trimesh.getCoordsVertices()[5] = ChVector<>(p_ext[iter].mx, +hy / 2, p_ext[iter].mz);

		// Load mesh faces
		// Specify the face vertices counter-clockwise.
		// Set the normal indices same as the vertex indices.
		std::cout << "Load faces..." << std::endl;
		idx_vertices[0] = ChVector<int>(0, 1, 3);
		idx_vertices[1] = ChVector<int>(1, 2, 3);
		idx_vertices[2] = ChVector<int>(4, 7, 5);
		idx_vertices[3] = ChVector<int>(7, 6, 5);

		idx_vertices[4] = ChVector<int>(0, 3, 4);
		idx_vertices[5] = ChVector<int>(3, 7, 4);
		idx_vertices[6] = ChVector<int>(1, 5, 2);
		idx_vertices[7] = ChVector<int>(2, 5, 6);

		idx_vertices[8] = ChVector<int>(3, 6, 7);
		idx_vertices[9] = ChVector<int>(3, 2, 6);
		idx_vertices[10] = ChVector<int>(0, 4, 5);
		idx_vertices[11] = ChVector<int>(0, 5, 1);

		auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
		trimesh_shape->SetMesh(m_trimesh);
		trimesh_shape->SetName("triangular_mesh");
		bucket->AddAsset(trimesh_shape);


		bucket->GetCollisionModel()->AddTriangleMesh(m_trimesh, true, false, ChVector<>(0, 0, 0));
	}

	}
void AddBucketHull(std::vector<Points> p_ext, std::vector<Points> p_int, std::shared_ptr<ChBody> bucket) {
	
	for (int iter = 0; iter < p_ext.size() - 1; iter++) {
		std::vector<ChVector<double>> cloud;
		double width = .5;// or halve an input

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my - width, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my - width, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter + 1].mx, p_ext[iter + 1].my - width, p_ext[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter].mx, p_ext[iter].my - width, p_ext[iter].mz));

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter + 1].mx, p_ext[iter + 1].my + width, p_ext[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter].mx, p_ext[iter].my + width, p_ext[iter].mz));



		bucket->GetCollisionModel()->AddConvexHull(cloud, ChVector<>(0, 0, 0), QUNIT);
		bucket->GetCollisionModel()->BuildModel();

		auto shape = std::make_shared<ChTriangleMeshShape>();
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(cloud, shape->GetMesh());
		bucket->AddAsset(shape);

		bucket->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
	}

}
void AddCapsHulls(std::vector<Points> p_int, BucketSide side, std::shared_ptr<ChBody> bucket) {


	for (int iter = 0; iter < p_int.size() - 1; iter++) {

	std::vector<ChVector<double>> cloud;
	double width;
	switch (side)
	{
	case LEFT: 
		width = +.5;
		break;
	case RIGHT:
		width = -.5;
		break;
	default:   
		width = .0;
		break;
	}


	double th = .05;

	cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width - th, p_int[iter].mz));
	cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter+1].my + width - th, p_int[iter + 1].mz));
	cloud.push_back(ChVector<>(.70, width - th, .25));

	cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width + th, p_int[iter].mz));
	cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width + th, p_int[iter + 1].mz));
	cloud.push_back(ChVector<>(.70, width + th, .25));
	
	bucket->GetCollisionModel()->AddConvexHull(cloud, ChVector<>(0, 0, 0), QUNIT);
	bucket->GetCollisionModel()->BuildModel();

	auto shape = std::make_shared<ChTriangleMeshShape>();
	collision::ChConvexHullLibraryWrapper lh;
	lh.ComputeHull(cloud, shape->GetMesh());
	bucket->AddAsset(shape);

	bucket->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
}
}
void AddMeshWall(std::shared_ptr<ChBody> body, const ChVector<>& dim, const ChVector<>& loc) {
	geometry::ChTriangleMeshConnected trimesh;

	std::vector<ChVector<> >& vertices = trimesh.getCoordsVertices();
	std::vector<ChVector<> >& normals = trimesh.getCoordsNormals();
	std::vector<ChVector<int> >& idx_vertices = trimesh.getIndicesVertexes();
	std::vector<ChVector<int> >& idx_normals = trimesh.getIndicesNormals();

	int num_vert = 8;
	int num_faces = 8;

	vertices.resize(num_vert);
	normals.resize(num_vert);
	trimesh.getCoordsUV().resize(num_vert);
	trimesh.getCoordsColors().resize(num_vert);

	idx_vertices.resize(num_faces);
	idx_normals.resize(num_faces);

	vertices[0] = ChVector<>(-dim.x(), -dim.y(), -dim.z());
	vertices[1] = ChVector<>(-dim.x(), +dim.y(), -dim.z());
	vertices[2] = ChVector<>(+dim.x(), +dim.y(), -dim.z());
	vertices[3] = ChVector<>(+dim.x(), -dim.y(), -dim.z());

	vertices[4] = ChVector<>(-dim.x(), -dim.y(), +dim.z());
	vertices[5] = ChVector<>(-dim.x(), +dim.y(), +dim.z());
	vertices[6] = ChVector<>(+dim.x(), +dim.y(), +dim.z());
	vertices[7] = ChVector<>(+dim.x(), -dim.y(), +dim.z());

	idx_vertices[0] = ChVector<int>(0, 1, 3);
	idx_vertices[1] = ChVector<int>(1, 2, 3);
	idx_vertices[2] = ChVector<int>(4, 7, 5);
	idx_vertices[3] = ChVector<int>(7, 6, 5);

	idx_vertices[4] = ChVector<int>(0, 3, 4);
	idx_vertices[5] = ChVector<int>(3, 7, 4);
	idx_vertices[6] = ChVector<int>(1, 5, 2);
	idx_vertices[7] = ChVector<int>(2, 5, 6);

	//idx_vertices[8] = ChVector<int>(3, 6, 7);
	//idx_vertices[9] = ChVector<int>(3, 2, 6);
	//idx_vertices[10] = ChVector<int>(0, 4, 5);
	//idx_vertices[11] = ChVector<int>(0, 5, 1);

	body->GetCollisionModel()->AddTriangleMesh(trimesh, true, true, loc);

	auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
	trimesh_shape->SetMesh(trimesh);
	trimesh_shape->SetName("my_mesh");
	body->AddAsset(trimesh_shape);
}

int main(int argc, char* argv[]) {
	std::vector<Points> p_ext;
	std::vector<Points> p_int;
	std::vector<Points> hull;

	const std::string out_dir = "../";//Directory di Lavoro settata in $(OutDir)!!!
	const std::string& ext_profile = out_dir + "data/ext_profile.txt";
	const std::string& int_profile = out_dir + "data/int_profile.txt";
	const std::string& points_hull = out_dir + "data/points_hull.txt";


	ReadFile(ext_profile, p_ext);
	ReadFile(int_profile, p_int);
	ReadFile(points_hull, hull);

	// 0. Set the path to the Chrono data folder
	SetChronoDataPath(CHRONO_DATA_DIR);
	
	
	 // Create a material (will be used by both objects)
	auto material = std::make_shared<ChMaterialSurfaceNSC>();
	material->SetRestitution(0.1f);
	material->SetFriction(0.4f);
	// Create a material (will be used by both objects)
	auto materialDEM = std::make_shared<ChMaterialSurfaceSMC>();
	materialDEM->SetYoungModulus(1.0e7f);
	materialDEM->SetRestitution(0.1f);
	materialDEM->SetFriction(0.4f);
	materialDEM->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

	////////////////////////////////////////////////////////////////////////////////////////////////
	
	// parameters for tests
#ifdef USE_PENALTY
	ChSystemSMC system;
#else
	ChSystemNSC system;
#endif

	system.Set_G_acc(ChVector<>(0., +10.0, 0.));
	ChVector<> pos_ball(.5, -1.0, .1);
	auto material_test = material;
	double time_step = .001;
	double radius = .15;

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
	application->AddTypicalCamera(core::vector3df(0, 3, -6)); //'camera' location            // "look at" location
																																		// Let the Irrlicht application convert the visualization assets.

	// This means that contactforces will be shown in Irrlicht application
	application->SetSymbolscale(1e-4);
	application->SetContactsDrawMode(ChIrrTools::eCh_ContactsDrawMode::CONTACT_FORCES);

	// Create the ball
		auto ball = std::make_shared<ChBody>( );//.15 before
		ball->SetMass(1000);
		ball->SetPos(pos_ball);
		ball->SetBodyFixed(false);
#ifdef USE_PENALTY
		ball->SetMaterialSurface(materialDEM);
#else
		ball->SetMaterialSurface(material_test);
#endif
		ball->SetCollide(true);

		ball->GetCollisionModel()->ClearModel();
		ball->GetCollisionModel()->AddSphere(radius);
		ball->GetCollisionModel()->BuildModel();

		ball->SetInertiaXX(0.4 * 1000 * radius * radius * ChVector<>(1, 1, 1));

		auto sphere = std::make_shared<ChSphereShape>();
		sphere->GetSphereGeometry().rad = radius;
		ball->AddAsset(sphere);

		auto mtexture = std::make_shared<ChTexture>();
		mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
		ball->AddAsset(mtexture);

		system.AddBody(ball);

		// BUCKET
		auto bucket = std::make_shared<ChBodyAuxRef>();//ChBodyAuxRef
		bucket->SetBodyFixed(true);
		bucket->SetName("benna");
		bucket->SetIdentifier(4);
		bucket->SetMass(1.0);
		bucket->SetInertiaXX(ChVector<>(400, 400, 400));
		bucket->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(.0, .0, .0),QUNIT));//remember it
		// Create contact geometry.
		bucket->SetCollide(true);
		bucket->GetCollisionModel()->ClearModel();
		//bucket->GetCollisionModel()->AddBox(hx, hy, hz, VNULL);// it works
		//AddMeshWall(bucket, dim, VNULL);
		//AddBucketMesh(p_ext,p_int,bucket);
		AddBucketHull(p_ext,p_int,bucket);
		AddCapsHulls(p_int, BucketSide::LEFT, bucket);
		bucket->SetMaterialSurface(material_test);
		bucket->GetCollisionModel()->BuildModel();
		//bucket->SetMaterialSurface(materialDEM);
		system.AddBody(bucket);


		application->AssetBindAll();
		application->AssetUpdateAll();
		application->SetTimestep(time_step);

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
