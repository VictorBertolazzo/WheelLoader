// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// =============================================================================
//
// Base class for a Front End Loader subsystem.
//
// =============================================================================

#include <vector>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChAssetLevel.h"
#include "chrono/assets/ChObjShapeFile.h"
#include "chrono/assets/ChTexture.h"

#include "chrono_vehicle/wheeled_vehicle/frontendloader/ChFrontEndLoader.h"

namespace chrono {
	namespace vehicle {

		ChFrontEndLoader::ChFrontEndLoader(const std::string& name) : ChPart(name) {}

		void ChFrontEndLoader::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
										  const ChVector<>& location,
										  const ChQuaternion<>& rotation){
			// Express the FEL reference frame in the absolute coordinate system.
			ChFrame<> frontendloader_to_abs(location, rotation);
			frontendloader_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
			// Transform all points and directions to absolute frame.
			std::vector<ChVector<> > points(NUM_POINTS);
			std::vector<ChVector<> > dirs(NUM_DIRS);

			for (int i = 0; i < NUM_POINTS; i++) {
				ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
				points[i] = frontendloader_to_abs.TransformPointLocalToParent(rel_pos);
			}

			for (int i = 0; i < NUM_DIRS; i++) {
				ChVector<> rel_dir = getDirection(static_cast<DirectionId>(i));
				dirs[i] = frontendloader_to_abs.TransformDirectionLocalToParent(rel_dir);
			}

			// Unit vectors for orientation matrices.
			ChVector<> u;
			ChVector<> v;
			ChVector<> w;
			ChMatrix33<> rot;

			// Create and Initialize the bucket lever body
			m_bucketlever = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
			m_bucketlever->SetNameString(m_name + "_bucketlever");
			m_bucketlever->SetPos(points[BUCKETLEVER]);
			m_bucketlever->SetRot(frontendloader_to_abs.GetRot());
			m_bucketlever->SetMass(getBucketLeverMass());
			m_bucketlever->SetInertiaXX(getBucketLeverInertia());
			chassis->GetSystem()->AddBody(m_bucketlever);

			// Create and Initialize the lift arm body
			m_liftarm = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
			m_liftarm->SetNameString(m_name + "_liftarm");
			m_liftarm->SetPos(points[LIFTARM]);
			m_liftarm->SetRot(frontendloader_to_abs.GetRot());
			m_liftarm->SetMass(getLiftArmMass());
			m_liftarm->SetInertiaXX(getLiftArmInertia());
			chassis->GetSystem()->AddBody(m_liftarm);

			// Create and Initialize the L shaped body
			m_lshaped = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
			m_lshaped->SetNameString(m_name + "_lshaped");
			m_lshaped->SetPos(points[LSHAPED]);
			m_lshaped->SetRot(frontendloader_to_abs.GetRot());
			m_lshaped->SetMass(getLShapedMass());
			m_lshaped->SetInertiaXX(getLShapedInertia());
			chassis->GetSystem()->AddBody(m_lshaped);

			// Create and Initialize the loader(bucket) body
			m_loader = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
			//m_loader = std::make_shared<ChBody>(m_parcollisionmodel); //Add 5/12
			//auto m_loader = std::make_shared<ChBody>(m_parcollisionmodel);
			m_loader->SetNameString(m_name + "_loader");
			m_loader->SetPos(points[LOADER]);
			m_loader->SetRot(frontendloader_to_abs.GetRot());
			m_loader->SetMass(getLoaderMass());
			m_loader->SetInertiaXX(getLoaderInertia());
			chassis->GetSystem()->AddBody(m_loader);
			//Define the collision behaviour for the loader(bucket) body::now it is a ChBody,
			//so I'll treat it with normal model functions-> afterwards they'll be changed to accomplish
			//a ChBodyEasyMesh bucket loader
			//collision::ChCollisionModelParallel* PARcolModel;//Add 5/12
			m_loader->GetCollisionModel()->ClearModel();
			//m_loader->ChangeCollisionModel(PARcolModel);//Add 5/12
			m_loader->GetCollisionModel()->AddBox(0.1/2.,0.5/2.,1.3/2.,ChVector<>(0.,0.5,2.0),ChQuaternion<>(1.,0.,0.,0.));
			m_loader->GetCollisionModel()->BuildModel();
			m_loader->SetCollide(true);



			// Create and Initialize the idle body
			m_idle = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
			m_idle->SetNameString(m_name + "_idle");
			m_idle->SetPos(points[IDLE]);
			m_idle->SetRot(frontendloader_to_abs.GetRot());
			m_idle->SetMass(getIdleMass());
			m_idle->SetInertiaXX(getIdleInertia());
			chassis->GetSystem()->AddBody(m_idle);


			// Create and initialize the revolute joint between chassis and idle body.
			// The z direction of the joint orientation matrix is dirs[VERTREV_AXIS], assumed
			// to be a unit vector.
			u = points[IDLE] - points[VERTREV];
			v = Vcross(dirs[VERTREV_AXIS], u);
			v.Normalize();
			u = Vcross(v, dirs[VERTREV_AXIS]);
			rot.Set_A_axis(u, v, dirs[VERTREV_AXIS]);

			m_revID_CH = std::make_shared<ChLinkEngine>();
			m_revID_CH->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
			m_revID_CH->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
			m_revID_CH->SetNameString(m_name + "_vertrev");
			m_revID_CH->Initialize(chassis, m_idle, ChCoordsys<>(points[VERTREV], rot.Get_A_quaternion()));
			chassis->GetSystem()->AddLink(m_revID_CH);


			// Create and initialize the pneumatic actuator between idle body and bucket lever.
			// The z direction of the joint orientation matrix is dirs[u](check in ChLinkPneumaticActuator class)
			u = points[PNEUMA1_ID] - points[PNEUMA1_BL];
			v = Vcross(dirs[VERTREV_AXIS], u);
			v.Normalize();
			u = Vcross(v, dirs[VERTREV_AXIS]);
			rot.Set_A_axis(dirs[VERTREV_AXIS], v, u); // This because z axis is the piston one

			m_pneumaBL_ID = std::make_shared<ChLinkPneumaticActuator>();
			m_pneumaBL_ID->SetNameString(m_name + "_pneumabucketlever");
			m_pneumaBL_ID->Initialize(m_idle, m_bucketlever, ChCoordsys<>(points[PNEUMA1_ID], rot.Get_A_quaternion()));
			m_pneumaBL_ID->Set_lin_offset(0.1);// const pneuma1_offset -I'll make this tunable in initializing
			m_pneumaBL_ID->Set_pneu_L(2.0);// const pneuma1_offset --I'll make this tunable in initializing

			chassis->GetSystem()->AddLink(m_pneumaBL_ID);


			// Create and initialize the pneumatic actuator between idle body and lift arm.
			// The z direction of the joint orientation matrix is dirs[u](check in ChLinkPneumaticActuator class)
			u = points[PNEUMA2_ID] - points[PNEUMA2_LA];
			v = Vcross(dirs[VERTREV_AXIS], u);
			v.Normalize();
			u = Vcross(v, dirs[VERTREV_AXIS]);
			rot.Set_A_axis(dirs[VERTREV_AXIS], v, u); // This because z axis is the piston one

			m_pneumaLA_ID = std::make_shared<ChLinkPneumaticActuator>();
			m_pneumaLA_ID->SetNameString(m_name + "_pneumaliftarm");
			m_pneumaLA_ID->Initialize(m_idle, m_bucketlever, ChCoordsys<>(points[PNEUMA2_ID], rot.Get_A_quaternion()));
			m_pneumaLA_ID->Set_lin_offset(0.1);// const pneuma1_offset -I'll make this tunable in initializing
			m_pneumaLA_ID->Set_pneu_L(2.0);// const pneuma1_offset --I'll make this tunable in initializing

			chassis->GetSystem()->AddLink(m_pneumaLA_ID);


			// Create and initialize the revolute joint between bucket lever and L-shaped body.
			// The z direction of the joint orientation matrix is dirs[????], assumed
			// to be a unit vector.
			u = points[BUCKETLEVER] - points[HORZREV_BL_LS];
			v = Vcross(dirs[HORZREV_AXIS], u);
			v.Normalize();
			u = Vcross(v, dirs[HORZREV_AXIS]);
			rot.Set_A_axis(u, v, dirs[HORZREV_AXIS]);

			m_revBL_LS = std::make_shared<ChLinkLockRevolute>();
			m_revBL_LS->SetNameString(m_name + "_horzrev_bl_ls");
			m_revBL_LS->Initialize(m_bucketlever, m_lshaped, ChCoordsys<>(points[HORZREV_BL_LS], rot.Get_A_quaternion()));
			chassis->GetSystem()->AddLink(m_revBL_LS);
			// Create and initialize the revolute joint between lift arm and L-shaped body.
			// The z direction of the joint orientation matrix is dirs[????], assumed
			// to be a unit vector.
			u = points[LIFTARM] - points[HORZREV_LA_LS];
			v = Vcross(dirs[HORZREV_AXIS], u);
			v.Normalize();
			u = Vcross(v, dirs[HORZREV_AXIS]);
			rot.Set_A_axis(u, v, dirs[HORZREV_AXIS]);

			m_revLA_LS = std::make_shared<ChLinkLockRevolute>();
			m_revLA_LS->SetNameString(m_name + "_horzrev_la_ls");
			m_revLA_LS->Initialize(m_liftarm, m_lshaped, ChCoordsys<>(points[HORZREV_LA_LS], rot.Get_A_quaternion()));
			chassis->GetSystem()->AddLink(m_revLA_LS);
			// Create and initialize the revolute joint between bucket lever and the bucket(BENNA).
			// The z direction of the joint orientation matrix is dirs[????], assumed
			// to be a unit vector.
			u = points[LOADER] - points[HORZREV_BL_B];
			v = Vcross(dirs[HORZREV_AXIS], u);
			v.Normalize();
			u = Vcross(v, dirs[HORZREV_AXIS]);
			rot.Set_A_axis(u, v, dirs[HORZREV_AXIS]);

			m_revBL_B = std::make_shared<ChLinkLockRevolute>();
			m_revBL_B->SetNameString(m_name + "_horzrev_bl_b");
			m_revBL_B->Initialize(m_bucketlever, m_loader, ChCoordsys<>(points[HORZREV_BL_B], rot.Get_A_quaternion()));
			chassis->GetSystem()->AddLink(m_revBL_B);
			// Create and initialize the revolute joint between lift arm and the bucket(BENNA).
			// The z direction of the joint orientation matrix is dirs[????], assumed
			// to be a unit vector
			u = points[LOADER] - points[HORZREV_LA_B];
			v = Vcross(dirs[HORZREV_AXIS], u);
			v.Normalize();
			u = Vcross(v, dirs[HORZREV_AXIS]);
			rot.Set_A_axis(u, v, dirs[HORZREV_AXIS]);

			m_revLA_B = std::make_shared<ChLinkLockRevolute>();
			m_revLA_B->SetNameString(m_name + "_horzrev_la_b");
			m_revLA_B->Initialize(m_liftarm, m_loader, ChCoordsys<>(points[HORZREV_LA_B], rot.Get_A_quaternion()));
			chassis->GetSystem()->AddLink(m_revLA_B);



			// Fill visualization points retrieving from bodies information
			m_bl1 = m_bucketlever->TransformPointParentToLocal(points[PNEUMA1_BL]);
			m_bl2 = m_bucketlever->TransformPointParentToLocal(points[HORZREV_BL_B]);

			m_la1 = m_liftarm->TransformPointParentToLocal(points[PNEUMA2_LA]);
			m_la2 = m_liftarm->TransformPointParentToLocal(points[HORZREV_LA_B]);
			
			m_ls1 = m_lshaped->TransformPointParentToLocal(points[HORZREV_BL_LS]);
			m_ls2 = m_lshaped->TransformPointParentToLocal(points[HORZREV_LA_LS]);

			m_i = m_idle->TransformPointParentToLocal(points[IDLE]);

			m_b = m_loader->TransformPointParentToLocal(points[LOADER]);




		}
		void ChFrontEndLoader::Synchronize(double time, double steering, double valve_inputs[4]) {
			
			// Function that makes steering the idle body wrt the chassis
			if (auto fun = std::dynamic_pointer_cast<ChFunction_Const>(m_revID_CH->Get_rot_funct()))
				fun->Set_yconst(getMaxAngle() * steering);
			// Function that sets valve command for both pneumatic actuator
			std::dynamic_pointer_cast<ChLinkPneumaticActuator>(m_pneumaBL_ID)->Set_ComA(*(valve_inputs + 0));
			std::dynamic_pointer_cast<ChLinkPneumaticActuator>(m_pneumaBL_ID)->Set_ComB(*(valve_inputs + 1));
			std::dynamic_pointer_cast<ChLinkPneumaticActuator>(m_pneumaLA_ID)->Set_ComA(*(valve_inputs + 2));
			std::dynamic_pointer_cast<ChLinkPneumaticActuator>(m_pneumaLA_ID)->Set_ComB(*(valve_inputs + 3));

		}
		// Get the total mass of the front end loader subsystem
		double ChFrontEndLoader::GetMass() const {
			return getBucketLeverMass() + getLiftArmMass() + getLoaderMass() + getLShapedMass() + getIdleMass();
		}
		// Visualization assets
		void ChFrontEndLoader::AddVisualizationAssets(VisualizationType vis) {
			if (vis == VisualizationType::NONE)
				return;

			// Visualization for bucket lever
			{
				auto cyl = std::make_shared<ChCylinderShape>();
				cyl->GetCylinderGeometry().p1 = m_bl1;
				cyl->GetCylinderGeometry().p2 = m_bl2;
				cyl->GetCylinderGeometry().rad = getBucketLeverRadius();
				m_bucketlever->AddAsset(cyl);
				
				auto col = std::make_shared<ChColorAsset>();
				col->SetColor(ChColor(0.2f, 0.7f, 0.7f));
				m_bucketlever->AddAsset(col);
				
			}

			// Visualization for lift arm
			{
				auto cyl = std::make_shared<ChCylinderShape>();
				cyl->GetCylinderGeometry().p1 = m_la1;
				cyl->GetCylinderGeometry().p2 = m_la2;
				cyl->GetCylinderGeometry().rad = getLiftArmRadius();
				m_liftarm->AddAsset(cyl);

				auto col = std::make_shared<ChColorAsset>();
				col->SetColor(ChColor(0.7f, 0.7f, 0.2f));//yellow color
				m_liftarm->AddAsset(col);
			}
			// Visualization for lshaped body
			{
				auto cyl = std::make_shared<ChCylinderShape>();
				cyl->GetCylinderGeometry().p1 = m_ls1;
				cyl->GetCylinderGeometry().p2 = m_ls2;
				cyl->GetCylinderGeometry().rad = getLShapedRadius();
				m_lshaped->AddAsset(cyl);

				auto col = std::make_shared<ChColorAsset>();
				col->SetColor(ChColor(0.7f, 0.0f, 0.0f));//red color
				m_lshaped->AddAsset(col);
			}
			// Visualization for loader BENNA
			{   //Alternative with ChBodyEasyMesh
				//..see demo_forklift.cpp

				//Define the visualization of the bucket
				auto m_loader_asset_assembly = std::make_shared<ChAssetLevel>();//asset
				m_loader_asset_assembly->GetFrame().SetPos(m_b);//asset
				//m_loader_asset_assembly->GetFrame().SetRot(Q_from_AngAxis(180*CH_C_DEG_TO_RAD,ChVector<>(1.0,0.0,1.0)));//asset

				m_loader->AddAsset(m_loader_asset_assembly);//asset
				auto m_loader_mesh = std::make_shared<ChObjShapeFile>();//mesh
				m_loader_mesh->SetFilename(GetChronoDataFile("forklift_body.obj"));//mesh
				m_loader_asset_assembly->AddAsset(m_loader_mesh);//mesh

				auto m_loader_texture = std::make_shared<ChTexture>(GetChronoDataFile("bucket.png"));//texture
				m_loader_asset_assembly->AddAsset(m_loader_texture);//texture

				auto col = std::make_shared<ChColorAsset>();//color
				col->SetColor(ChColor(0.7f, 0.7f, 0.2f));//color
				//m_loader->AddAsset(col);//color
			}
			// Visualization for idle body
			{
				auto mbox = std::make_shared<ChBoxShape>();
				mbox->GetBoxGeometry().Pos = m_i;
				mbox->GetBoxGeometry().Size = ChVector<>(0.12, 0.20, 0.1);
				m_idle->AddAsset(mbox);

				/*
				auto cyl = std::make_shared<ChAssetLevel>();
				cyl->GetFrame().SetPos(-IDLE);
				m_idle->AddAsset(cyl);
				auto idle_mesh = std::make_shared<ChObjShapeFile>();
				idle_mesh->SetFilename(GetChronoDataFile("cylinder.obj"));
				cyl->AddAsset(idle_mesh);
				auto idle_texture = std::make_shared<ChTexture>(GetChronoDataFile("tire_truck.png"));
				cyl->AddAsset(idle_texture);                     
*/
				auto col = std::make_shared<ChColorAsset>();
				col->SetColor(ChColor(0.7f, 0.7f, 0.2f));//
				m_idle->AddAsset(col);
			}

		}
		// Remove Visualization assets
		void ChFrontEndLoader::RemoveVisualizationAssets() {
			m_bucketlever->GetAssets().clear();
			m_liftarm->GetAssets().clear();
			m_lshaped->GetAssets().clear();
			m_loader->GetAssets().clear();
			m_idle->GetAssets().clear();
		}
		void ChFrontEndLoader::LogConstraintViolations() {
			// Revolute joint
			{
				ChMatrix<>* C = m_revID_CH->GetC();
				GetLog() << "Revolute              ";
				GetLog() << "  " << C->GetElement(0, 0) << "  ";
				GetLog() << "  " << C->GetElement(1, 0) << "  ";
				GetLog() << "  " << C->GetElement(2, 0) << "  ";
				GetLog() << "  " << C->GetElement(3, 0) << "  ";
				GetLog() << "  " << C->GetElement(4, 0) << "\n";
			}
		// function that must be studied and understood previously
		}

		// Set Valve command
		void ChFrontEndLoader::ValveCommand(std::shared_ptr<ChLinkPneumaticActuator> pneum, double a, double b) {
			pneum->Set_ComA(a);
			pneum->Set_ComB(b);
		}
		//Get valves command
		double * ChFrontEndLoader::GetValveCommand(std::shared_ptr<ChLinkPneumaticActuator> pneum){
			double comm[2];
			*(comm + 0) = pneum->Get_ComA();
			*(comm + 1) = pneum->Get_ComB();
			return comm;// is it volatile out of the scope?
		}
		//Get Pneumatic Force
		double ChFrontEndLoader::GetPneuForce(std::shared_ptr<ChLinkPneumaticActuator> pneum) {
			return pneum->Get_pneu_F();
		}


	} // end namespace vehicle
}     // end namespace chrono
