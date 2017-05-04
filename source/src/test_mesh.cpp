
#include <iostream>//Add 

#include "chrono/physics/ChSystemNSC.h"

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

struct Entry {
	Entry() {}
	Entry(float xpos,float zpos, float ang, float length)
		: m_xpos(xpos), m_zpos(zpos), m_ang(ang), m_length(length) {}
	float m_xpos;
	float m_zpos;
	float m_ang;
	float m_length;
};
struct Points {
	Points() {}
	Points(float x, float y, float z)
		: mx(x), my(y), mz(z) {}
	float mx; float my; float mz;
};

class Vertex {
public:
	Vertex() {};
	Vertex(int id, float xpos, float ypos, float zpos) :m_id( id ), m_xpos(xpos), m_ypos(ypos), m_zpos(zpos){}

	int m_id; float m_xpos,m_ypos,m_zpos;
};
class Face {
	Face() {};
	Face(int f, int s, int t)
		: first(f), second(s), third(t) {}
	int first; int second; int third ;
};

class Wavefront {
public:
	Wavefront() {};
	Wavefront(Vertex* vertices, Face* faces) {};

	Vertex* vertices;
	Face* faces;

	Vertex GetVertex(int i) { return *(vertices + i); }
};

int main(int argc, char* argv[]) {
	
		const std::string out_dir = "../";//Directory di Lavoro settata in $(OutDir)!!!
		const std::string& filename = out_dir + "data/bucket_coll.txt";
		std::ifstream ifile(filename.c_str());
		std::string line;
		std::vector<Entry> m_data;
		while (std::getline(ifile, line)) {
			std::istringstream iss(line);
			float xpos, zpos, ang, length;
			iss >> xpos >> zpos >> ang >> length;
			if (iss.fail())
				break;
			m_data.push_back(Entry(xpos, zpos, ang, length));
		}
		ifile.close();
		std::vector<Points> p_ext;
		std::vector<Points> p_int;

		std::string lineext;
		const std::string& filenameext = out_dir + "data/ext_profile.txt";
		std::ifstream ifileext(filenameext.c_str());
		while (std::getline(ifileext, lineext)) {
			std::istringstream iss(lineext);
			float xpos, ypos, zpos;
			iss >> xpos >> ypos >> zpos;
			if (iss.fail())
				break;
			p_ext.push_back(Points(xpos, ypos, zpos));
		}
		ifileext.close();

		std::string lineint;
		const std::string& filenameint = out_dir + "data/int_profile.txt";
		std::ifstream ifileint(filenameint.c_str());
		while (std::getline(ifileint, lineint)) {
			std::istringstream iss(lineint);
			float xpos, ypos, zpos;
			iss >> xpos >> ypos >> zpos;
			if (iss.fail())
				break;
			p_int.push_back(Points(xpos, ypos, zpos));
		}
		ifileint.close();
	
	chrono::GetLog() << ChVector<>(p_int[0].mx,p_int[0].my,p_int[0].mz) << "\t" << ChVector<>(p_ext[0].mx, p_ext[0].my, p_ext[0].mz) << "\n";

	


	// 0. Set the path to the Chrono data folder
	SetChronoDataPath(CHRONO_DATA_DIR);
	// 1. Create the system
    ChSystemNSC system;
	system.Set_G_acc(ChVector<>(.0,.0,.0));
    /// .16 initial from chassis offset, .21 initial max height, .33 initial width 
	// measures are in [m]
		ChVector<> COG_chassis(0, 0, 1.575); // somewhere
		ChVector<> COG_lift(2.0, 0., 1.05);
		ChVector<> COG_lever(3.6625, 0.0, 1.575);
		ChVector<> COG_rod(2.7, 0.0, 1.3125);
		ChVector<> COG_link(3.0, 0.0, 0.5);
		ChVector<> COG_bucket(4.0,.0, 0.525);

		ChVector<> POS_lift2rod(2.825,.0, 1.05);//rev joint(LIFT ARM) abs frame
		ChVector<> POS_rod2lever(3.6625,0., 1.575);//rev joint(BUCKET LEVER) abs frame
		ChVector<> POS_lift2bucket(3.50, .0,.21);//chassis piston(LIFT ARM) insertion abs frame
		ChVector<> POS_ch2lift(1.6,0, 2.1);//Rev chassis->lift
		ChVector<> POS_lift2lever(2.5, 0, 2.1);//end position of actuator lift->lever
		ChVector<> PIS_ch2lift(1.6, 0, 1.05);//Act chassis->lift
		ChVector<> PIS_lift2lever(2.0125, 0, 2.1);//Act lift->lever

		ChVector<> POS_rod2link(2.6, 0, 0.4);//Act lift->lever
		ChVector<> POS_link2bucket(3.69, .0, 0.71);//chassis piston(BUCKET LEVER) insertion abs frame

		ChVector<> INS_ch2lift(1.8,0,1.1);// Insertion of boom piston over lift body

		ChQuaternion<> z2y;
		ChQuaternion<> z2x;
		z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
		z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
		// BUCKET
		auto bucket = std::make_shared<ChBodyAuxRef>();
		system.AddBody(bucket);
		bucket->SetName("benna");
		bucket->SetIdentifier(4);
		bucket->SetMass(1.0);
		bucket->SetPos(POS_lift2bucket);
		bucket->SetRot(QUNIT);
		//bucket->SetFrame_COG_to_REF(ChFrame<> (bucket->GetFrame_REF_to_abs().GetInverse() * COG_bucket,QUNIT));
		bucket->SetInertiaXX(ChVector<>(.5, .5, .5));
		// collision properties(to do)###############################
						bucket->GetCollisionModel()->ClearModel();
						bucket->GetCollisionModel()->AddSphere(.015, VNULL);
						auto bucket_box = std::make_shared<ChBoxShape>();
						bucket->GetCollisionModel()->BuildModel();
						bucket->SetCollide(true);
		// visualization properties(to do)#############################
						auto bucket_asset = std::make_shared<ChSphereShape>();//asset
						bucket_asset->GetSphereGeometry().rad = .15;//asset
						bucket->AddAsset(bucket_asset);
						auto col_b = std::make_shared<ChColorAsset>();
						col_b->SetColor(ChColor(0.1f, 0.1f, 0.1f));
						bucket->AddAsset(col_b);
		// BUCKET Coll and Vis Update
						int nmax = m_data.size() ;
						/*
						for (int i = 0; i <  m_data.size(); i++) {
						double angle = +CH_C_PI / 2 + CH_C_PI / 30 + i*0.1305516846;
						ChVector<> pos_b(m_data[i].m_xpos,0.0,m_data[i].m_zpos);
						//GetLog() << angle << pos_b << cos(angle) << sin(angle) << "\n";//Debugging
						ChMatrix33<> rot_b;

						ChVector<> extr(m_data[i].m_length*cos(m_data[i].m_ang), 0.0, m_data[i].m_length*sin(m_data[i].m_ang));
						ChVector<> u_b = (extr - VNULL).GetNormalized();
						ChVector<> w_b = Vcross(VECT_Y, u_b).GetNormalized();//overkill
						//rot_b.Set_A_axis(u_b, VECT_Y, w_b);
						rot_b.Set_A_AngAxis(m_data[i].m_ang, -VECT_Y);
						bucket->GetCollisionModel()->AddBox(m_data[i].m_length, .25, .05, pos_b, rot_b);

						auto bucket_box = std::make_shared<ChBoxShape>();
						bucket_box->GetBoxGeometry() = geometry::ChBox(pos_b, rot_b, ChVector<>(m_data[i].m_length, .25, .05));
						bucket->AddAsset(bucket_box);
						}
						*/
						
		for (int iter=0; iter<p_ext.size()-1;iter++){
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
						float hy = .5;

						m_trimesh.getCoordsVertices()[0] = ChVector<>(p_int[iter].mx, -hy / 2, p_int[iter].mz);
						m_trimesh.getCoordsVertices()[1] = ChVector<>(p_int[iter+1].mx, -hy / 2, p_int[iter + 1].mz);
						m_trimesh.getCoordsVertices()[2] = ChVector<>(p_ext[iter].mx, -hy / 2, p_ext[iter].mz);
						m_trimesh.getCoordsVertices()[3] = ChVector<>(p_ext[iter + 1].mx, -hy / 2, p_ext[iter+1].mz);

						m_trimesh.getCoordsVertices()[4] = ChVector<>(p_int[iter].mx, +hy / 2, p_int[iter].mz);
						m_trimesh.getCoordsVertices()[5] = ChVector<>(p_int[iter+1].mx, +hy / 2, p_int[iter +1].mz);
						m_trimesh.getCoordsVertices()[6] = ChVector<>(p_ext[iter].mx, +hy / 2, p_ext[iter].mz);
						m_trimesh.getCoordsVertices()[7] = ChVector<>(p_ext[iter+1].mx, +hy / 2, p_ext[iter+1].mz);

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
							ChVector<> nrm = Vcross(vertices[idx_vertices[it].y()] - vertices[idx_vertices[it].x()], vertices[idx_vertices[it].z()] - vertices[idx_vertices[it].x()]);
							nrm.Normalize();
							// Increment the normals of all incident vertices by the face normal
							normals[idx_normals[it].x()] += nrm;
							normals[idx_normals[it].y()] += nrm;
							normals[idx_normals[it].z()] += nrm;
							// Increment the count of all incident vertices by 1
							//accumulators[idx_normals[it].x()] += 1;
							//accumulators[idx_normals[it].y()] += 1;
							//accumulators[idx_normals[it].z()] += 1;
						}
						auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
						trimesh_shape->SetMesh(m_trimesh);
						trimesh_shape->SetName("triangular_mesh");
						bucket->AddAsset(trimesh_shape);


						// Create contact geometry.
						bucket->GetCollisionModel()->ClearModel();
						bucket->GetCollisionModel()->AddTriangleMesh(m_trimesh, true, false, ChVector<>(0, 0, 0));
						bucket->GetCollisionModel()->BuildModel();
		}
		// Right Bucket Side-->Test Normals
		for (int iter = 0; iter < p_int.size() - 1; iter++) {
			// trimesh Usage
			geometry::ChTriangleMeshConnected m_trimesh;
			//cuneo like meshes
			int n_verts = 6;
			int n_faces = 8;
			// Resize mesh arrays.
			m_trimesh.getCoordsVertices().resize(n_verts);
			m_trimesh.getCoordsNormals().resize(n_verts);
			m_trimesh.getCoordsUV().resize(n_verts);
			m_trimesh.getCoordsColors().resize(n_verts);

			m_trimesh.getIndicesVertexes().resize(n_faces);
			m_trimesh.getIndicesNormals().resize(n_faces);
			// Load mesh vertices.
			float hy = .5; float ty = .05;
			m_trimesh.getCoordsVertices()[0] = ChVector<>(p_int[iter].mx, -hy / 2 -ty/2, p_int[iter].mz);
			m_trimesh.getCoordsVertices()[1] = ChVector<>(p_int[iter + 1].mx, -hy / 2 -ty/2, p_int[iter + 1].mz);
			m_trimesh.getCoordsVertices()[2] = ChVector<>(.70, -hy / 2 -ty/2, .25); // coord of the mid point,outer face of the tab

			m_trimesh.getCoordsVertices()[3] = ChVector<>(p_int[iter].mx, -hy / 2 + ty / 2, p_int[iter].mz);
			m_trimesh.getCoordsVertices()[4] = ChVector<>(p_int[iter + 1].mx, -hy / 2 + ty / 2, p_int[iter + 1].mz);
			m_trimesh.getCoordsVertices()[5] = ChVector<>(.70, -hy / 2 + ty / 2, .25); // coord of the mid point,inner face of the tab

																					   // Load mesh faces
																					   // Specify the face vertices counter-clockwise.
																					   // Set the normal indices same as the vertex indices.
			std::cout << "Load faces...normals have to be tested" << std::endl;
			auto ones = ChVector<int>(1, 1, 1);
			m_trimesh.getIndicesVertexes()[0] = ChVector<int>(1,2,3) - ones;//outer side
			m_trimesh.getIndicesVertexes()[1] = ChVector<int>(4,5,6) - ones;//inner side
			m_trimesh.getIndicesVertexes()[2] = ChVector<int>(1,6,4) - ones;//sand side t1
			m_trimesh.getIndicesVertexes()[3] = ChVector<int>(1,3,6) - ones;//sand side t2
			m_trimesh.getIndicesVertexes()[4] = ChVector<int>(1, 4, 5) - ones;//upper side t1
			m_trimesh.getIndicesVertexes()[5] = ChVector<int>(1, 5, 2) - ones;//upper side t2
			m_trimesh.getIndicesVertexes()[6] = ChVector<int>(2,5,6) - ones;//joint side t1
			m_trimesh.getIndicesVertexes()[7] = ChVector<int>(2,6,3) - ones;//joint side t2

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
				ChVector<> nrm = Vcross(vertices[idx_vertices[it].y()] - vertices[idx_vertices[it].x()], vertices[idx_vertices[it].z()] - vertices[idx_vertices[it].x()]);
				nrm.Normalize();
				// Increment the normals of all incident vertices by the face normal
				normals[idx_normals[it].x()] += nrm;
				normals[idx_normals[it].y()] += nrm;
				normals[idx_normals[it].z()] += nrm;
				// Increment the count of all incident vertices by 1
				//accumulators[idx_normals[it].x()] += 1;
				//accumulators[idx_normals[it].y()] += 1;
				//accumulators[idx_normals[it].z()] += 1;
			}
			auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
			trimesh_shape->SetMesh(m_trimesh);
			trimesh_shape->SetName("triangular_mesh");
			bucket->AddAsset(trimesh_shape);


			// Create contact geometry.
			bucket->GetCollisionModel()->ClearModel();
			bucket->GetCollisionModel()->AddTriangleMesh(m_trimesh, true, false, ChVector<>(0, 0, 0));
			bucket->GetCollisionModel()->BuildModel();


		}
		// Left Bucket Side-->Test Normals
		for (int iter = 0; iter < p_int.size() - 1; iter++) {
			// trimesh Usage
			geometry::ChTriangleMeshConnected m_trimesh;
			//cuneo like meshes
			int n_verts = 6;
			int n_faces = 8;
			// Resize mesh arrays.
			m_trimesh.getCoordsVertices().resize(n_verts);
			m_trimesh.getCoordsNormals().resize(n_verts);
			m_trimesh.getCoordsUV().resize(n_verts);

			m_trimesh.getCoordsColors().resize(n_verts);

			m_trimesh.getIndicesVertexes().resize(n_faces);
			m_trimesh.getIndicesNormals().resize(n_faces);
			// Load mesh vertices.
			float hy = .5; float ty = .05;
			m_trimesh.getCoordsVertices()[0] = ChVector<>(p_int[iter].mx, +hy / 2 - ty / 2, p_int[iter].mz);
			m_trimesh.getCoordsVertices()[1] = ChVector<>(p_int[iter + 1].mx, +hy / 2 - ty / 2, p_int[iter + 1].mz);
			m_trimesh.getCoordsVertices()[2] = ChVector<>(.70, +hy / 2 - ty / 2, .25); // coord of the mid point,outer face of the tab

			m_trimesh.getCoordsVertices()[3] = ChVector<>(p_int[iter].mx, +hy / 2 + ty / 2, p_int[iter].mz);
			m_trimesh.getCoordsVertices()[4] = ChVector<>(p_int[iter + 1].mx, +hy / 2 + ty / 2, p_int[iter + 1].mz);
			m_trimesh.getCoordsVertices()[5] = ChVector<>(.70, +hy / 2 + ty / 2, .25); // coord of the mid point,inner face of the tab

																					   // Load mesh faces
																					   // Specify the face vertices counter-clockwise.
																					   // Set the normal indices same as the vertex indices.
			std::cout << "Load faces...normals have to be tested" << std::endl;
			auto ones = ChVector<int>(1, 1, 1);
			m_trimesh.getIndicesVertexes()[0] = ChVector<int>(1, 2, 3) - ones;//outer side
			m_trimesh.getIndicesVertexes()[1] = ChVector<int>(4, 5, 6) - ones;//inner side
			m_trimesh.getIndicesVertexes()[2] = ChVector<int>(1, 6, 4) - ones;//sand side t1
			m_trimesh.getIndicesVertexes()[3] = ChVector<int>(1, 3, 6) - ones;//sand side t2
			m_trimesh.getIndicesVertexes()[4] = ChVector<int>(1, 4, 5) - ones;//upper side t1
			m_trimesh.getIndicesVertexes()[5] = ChVector<int>(1, 5, 2) - ones;//upper side t2
			m_trimesh.getIndicesVertexes()[6] = ChVector<int>(2, 5, 6) - ones;//joint side t1
			m_trimesh.getIndicesVertexes()[7] = ChVector<int>(2, 6, 3) - ones;//joint side t2

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
				ChVector<> nrm = Vcross(vertices[idx_vertices[it].y()] - vertices[idx_vertices[it].x()], vertices[idx_vertices[it].z()] - vertices[idx_vertices[it].x()]);
				nrm.Normalize();
				// Increment the normals of all incident vertices by the face normal
				normals[idx_normals[it].x()] += nrm;
				normals[idx_normals[it].y()] += nrm;
				normals[idx_normals[it].z()] += nrm;
				// Increment the count of all incident vertices by 1
				//accumulators[idx_normals[it].x()] += 1;
				//accumulators[idx_normals[it].y()] += 1;
				//accumulators[idx_normals[it].z()] += 1;
			}
			auto trimesh_shape = std::make_shared<ChTriangleMeshShape>();
			trimesh_shape->SetMesh(m_trimesh);
			trimesh_shape->SetName("triangular_mesh");
			bucket->AddAsset(trimesh_shape);


			// Create contact geometry.
			bucket->GetCollisionModel()->ClearModel();
			bucket->GetCollisionModel()->AddTriangleMesh(m_trimesh, true, false, ChVector<>(0, 0, 0));
			bucket->GetCollisionModel()->BuildModel();


		}

	
	
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
	application->AddTypicalCamera(core::vector3df(2, 5, -3),core::vector3df(2, 0, 0)); //'camera' location            // "look at" location
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
