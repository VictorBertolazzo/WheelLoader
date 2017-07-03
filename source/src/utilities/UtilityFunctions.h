using namespace chrono;
using namespace chrono::collision;
using namespace postprocess;
// --------------------------------------------------------------------------------------------------------

#include "chrono/collision/ChCCollisionUtils.h"

// --------------------------------------------------------------------------------------------------------
double ComputeKineticEnergy(ChBody* body){

	double mass = body->GetMass();
	ChMatrix33<> I = body->GetInertia();
	ChVector <> xdot = body->GetPos_dt();
	ChVector <> omega = body->GetWvel_par();

	double kin = mass* xdot.Dot(xdot) + omega.Dot(I.Matr_x_Vect(omega));
	kin = kin / 2;	return kin;

}
void TimingHeader() {
	printf("    TIME    |");
	printf("    STEP |");
	printf("   BROAD |");
	printf("  NARROW |");
	printf("  SOLVER |");
	printf("  UPDATE |");
	printf("# BODIES |");
	printf("# CONTACT|");
	printf(" # ITERS |");
	printf("n\n");
}

void TimingOutput(chrono::ChSystem* mSys) {
	double TIME = mSys->GetChTime();
	double STEP = mSys->GetTimerStep();
	double BROD = mSys->GetTimerCollisionBroad();
	double NARR = mSys->GetTimerCollisionNarrow();
	double SOLVER = mSys->GetTimerSolver();
	double UPDT = mSys->GetTimerUpdate();
	int REQ_ITS = 0;
	int BODS = mSys->GetNbodies();
	int CNTC = mSys->GetNcontacts();
	if (chrono::ChSystemParallel* parallel_sys = dynamic_cast<chrono::ChSystemParallel*>(mSys)) {
		REQ_ITS = std::static_pointer_cast<chrono::ChIterativeSolverParallel>(mSys->GetSolver())->GetTotalIterations();
	}

	printf("   %8.5f | %7.4f | %7.4f | %7.4f | %7.4f | %7.4f | %7d | %7d | %7d | %7.4f |\n", TIME, STEP, BROD, NARR,
		SOLVER, UPDT, BODS, CNTC, REQ_ITS);
}
// Funnel utility Function
void AddWall(ChVector<> lower, ChVector<> upper, std::shared_ptr<ChBody> body, double r) {

	std::vector<ChVector<double>> cloud;
	double th = 0.5*r;// or halve an input
	// if statement only on lower 
	if (lower.x() != 0. && lower.y() == 0.){
		double ss = std::abs(lower.x());

		cloud.push_back(lower + ChVector<>(-th, ss, 0.));
		cloud.push_back(lower + ChVector<>(+th, ss, 0.));
		cloud.push_back(lower + ChVector<>(-th, -ss, 0.));
		cloud.push_back(lower + ChVector<>(+th, -ss, 0.));
		double uu = std::abs(upper.x());
		cloud.push_back(upper + ChVector<>(-th, uu, 0.));
		cloud.push_back(upper + ChVector<>(+th, uu, 0.));
		cloud.push_back(upper + ChVector<>(-th, -uu, 0.));
		cloud.push_back(upper + ChVector<>(+th, -uu, 0.));

	}

	if (lower.x() == 0. && lower.y() != 0.){
		double ss = std::abs(lower.y());
		cloud.push_back(lower + ChVector<>(ss, -th, 0.));
		cloud.push_back(lower + ChVector<>(ss, +th, 0.));
		cloud.push_back(lower + ChVector<>(-ss, -th, 0.));
		cloud.push_back(lower + ChVector<>(-ss, +th, 0.));
		double uu = std::abs(upper.y());
		cloud.push_back(upper + ChVector<>(uu, -th, 0.));
		cloud.push_back(upper + ChVector<>(uu, +th, 0.));
		cloud.push_back(upper + ChVector<>(-uu, -th, 0.));
		cloud.push_back(upper + ChVector<>(-uu, +th, 0.));
	}

	// Add a check on cloud.size()==8;


	body->GetCollisionModel()->AddConvexHull(cloud, ChVector<>(0, 0, 0), QUNIT);
	//body->GetCollisionModel()->BuildModel();

	auto shape = std::make_shared<ChTriangleMeshShape>();
	collision::ChConvexHullLibraryWrapper lh;
	lh.ComputeHull(cloud, shape->GetMesh());
	body->AddAsset(shape);

	body->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
}
// Particle Runtime Generation , case::DROP	
int SpawnParticles(utils::Generator* gen, double radius_g) {
	double dist = 2.3 * 1.01 * radius_g;

	////gen->createObjectsBox(utils::POISSON_DISK,
	////                     dist,
	////                     ChVector<>(9, 0, 3),
	////                     ChVector<>(0, 1, 0.5),
	////                     ChVector<>(-initVel, 0, 0));
	gen->createObjectsCylinderZ(utils::POISSON_DISK, dist, ChVector<>(0, 0, 0.15), 0.075f, 0, ChVector<>(0, 0, 0));
	std::cout << "  total bodies: " << gen->getTotalNumBodies() << std::endl;

	return gen->getTotalNumBodies();
}
// Funnel Generation , case::FUNNEL, 7.2r m/s speed.
void CreateFunnel(chrono::ChSystem* system, std::shared_ptr<ChBody> container, std::shared_ptr<ChMaterialSurface> material_terrain, ChMaterialSurface::ContactMethod method, double radius_g){
	auto funnel = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(funnel);
	funnel->SetIdentifier(-2);
	funnel->SetPos(ChVector<>(0., 0., 2 * radius_g));
	funnel->SetMass(1);
	funnel->SetBodyFixed(false);
	funnel->SetMaterialSurface(material_terrain);
	switch (method) {
		// Since it's not the contact btw funnel and particles what I'm interested in, I slow down mi-coefficient
	case ChMaterialSurface::SMC: {
		funnel->GetMaterialSurfaceSMC()->SetFriction(.1f);
		break;
	}
	case ChMaterialSurface::NSC: {
		funnel->GetMaterialSurfaceNSC()->SetFriction(.1f);
		break;
	}
	}
	funnel->SetCollide(true);
	funnel->GetCollisionModel()->ClearModel();
	// OpenGL does not accept more than ONE visualization shape per body
	double half_low_size = 6 * radius_g;
	double half_upp_size = 20 * radius_g;
	double base2base_height = 100 * radius_g;
	AddWall(ChVector<>(half_low_size, .0, .0), ChVector<>(half_upp_size, .0, base2base_height), funnel, radius_g);
	AddWall(ChVector<>(0., half_low_size, .0), ChVector<>(0., half_upp_size, base2base_height), funnel, radius_g);
	AddWall(ChVector<>(-half_low_size, .0, .0), ChVector<>(-half_upp_size, .0, base2base_height), funnel, radius_g);
	AddWall(ChVector<>(0., -half_low_size, .0), ChVector<>(0., -half_upp_size, base2base_height), funnel, radius_g);
	funnel->GetCollisionModel()->BuildModel();

	//----------------
	// Create the Funnel Movement: this should be a constant velocity trajectory in z direction
	// 
	// Linear actuator btw the container and the funnel, it simulates the raising of the latter which must be always closely over the sand heap top.
	auto cont2fun = std::make_shared<ChLinkLockPrismatic>();
	cont2fun->Initialize(funnel, container, false, ChCoordsys<>(funnel->GetPos(), QUNIT), ChCoordsys<>(container->GetPos(), QUNIT));
	system->AddLink(cont2fun);
	auto container2funnel = std::make_shared<ChLinkLinActuator>();
	container2funnel->Initialize(funnel, container, false, ChCoordsys<>(funnel->GetPos(), QUNIT), ChCoordsys<>(container->GetPos(), QUNIT));
	auto funnel_law = std::make_shared<ChFunction_Ramp>();
	funnel_law->Set_ang(7.2*radius_g);
	container2funnel->Set_lin_offset(Vlength(container->GetPos() - funnel->GetPos()));
	container2funnel->Set_dist_funct(funnel_law);
	system->AddLink(container2funnel);

}
// Hollowed Cylinder Generation , case::CASCADE
void CreateTube(chrono::ChSystem* system, std::shared_ptr<ChBody> container, std::shared_ptr<ChMaterialSurface> material_terrain, double time_hold){	// Create TUBE body
	auto tube = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(tube);
	tube->SetIdentifier(-2);
	tube->SetPos_dt(ChVector<>(.0, .0, 0.0));// Initial Value
	tube->SetMass(1.0);
	tube->SetBodyFixed(false);// true + actuator yields two bodies explode
	tube->SetCollide(true);
	tube->SetMaterialSurface(material_terrain);
	tube->GetCollisionModel()->ClearModel();
	ChQuaternion<> qtube;
	qtube.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(1, 0, 0));
	tube->SetPos(ChVector<>(0.0, .0, .0));
	tube->SetRot(qtube);
	// In original file measures fulfill to those proposed by the cited article, here the same are adapted to the TR requirements.
	for (int i = 0; i < 20; i++){
		utils::AddTorusGeometry(tube.get(), .150, .005, 5 * 20, 360, ChVector<>(0, 2 * i * 0.005, 0.), ChQuaternion<>(1.0, 0., 0., .0), true);
	}
	//utils::AddTorusGeometry(tube.get(), .133 / 2, .005, 20,360,ChVector<>(0,0,0),ChQuaternion<>(1.0,.0,.0,.0),true);
	tube->GetCollisionModel()->BuildModel();

	// Create a prismatic actuator btw CONTAINER and TUBE
	auto prismCT = std::make_shared<ChLinkLockPrismatic>();
	prismCT->Initialize(tube, container, ChCoordsys<>(ChVector<>(.0, .0, 0.0), QUNIT));
	system->AddLink(prismCT);
	auto linCT = std::make_shared<ChLinkLinActuator>();
	linCT->Initialize(tube, container, ChCoordsys<>(ChVector<>(.0, .0, 0.0), QUNIT));//m2 is the master
	linCT->Set_lin_offset(0.0);
	system->AddLink(linCT);
	auto legge1 = std::make_shared<ChFunction_Const>();
	legge1->Set_yconst(0.0);
	auto legge2 = std::make_shared<ChFunction_Ramp>();
	legge2->Set_ang(0.075);//.015 in origin
	auto sequence = std::make_shared<ChFunction_Sequence>();
	sequence->InsertFunct(legge1, time_hold, 1.0, true);
	sequence->InsertFunct(legge2, 300, 1.0, true);


	linCT->Set_dist_funct(sequence);
}

// -------------------TIME SERIES STRUCTURE----------------------------
struct TimeSeries {
	TimeSeries() {}
	TimeSeries(float t, float v)
		: mt(t), mv(v) {}
	float mt; float mv;
};
// -------------------READ FILE FUNCTION----------------------------
void ReadFile(const std::string& filename, std::vector<TimeSeries>& profile) {
	std::ifstream ifile(filename.c_str());
	std::string line;

	while (std::getline(ifile, line)) {
		std::istringstream iss(line);
		float ttime, vvalue;
		iss >> ttime >> vvalue;
		if (iss.fail())
			break;
		profile.push_back(TimeSeries(ttime, vvalue));
	}
	ifile.close();



}
// Class implemented to compute hydraulic force
class myHYDRforce {
public:
	virtual ~myHYDRforce() {}


	virtual double operator()(double time,
		double pressureH,
		double areaH)
	{
		double force = pressureH * areaH; //pH*aH-pR*aR, formula for piston
		return force;
	}

};
// Class implemented to set up a hydraulic/data input force between two markers
class myHYDRactuator : public ChLinkMarkers{

public:
	myHYDRactuator() : m_force(0), m_force_fun(NULL), m_pressureH(NULL), m_pressureR(NULL), m_areaH(5e-3), m_areaR(10) {};
	myHYDRactuator(const myHYDRactuator& other);
	virtual ~myHYDRactuator() {}


	void SetAreaH(double areaH){ m_areaH = areaH; }
	void SetAreaR(double areaR){ m_areaR = areaR; }
	double GetAreaH(){ return m_areaH; }
	double GetAreaR(){ return m_areaR; }

	void Set_HYDRforce(myHYDRforce* force) { m_force_fun = force; }
	void Set_PressureH(ChFunction* pressureH){ m_pressureH = pressureH; }
	void Set_PressureR(ChFunction* pressureR){ m_pressureR = pressureR; }


	void UpdateForces(double time)override{

		// Allow the base class to update itself (possibly adding its own forces)
		ChLinkMarkers::UpdateForces(time);

		// Get the pressure information
		double pressureH = m_pressureH->Get_y(time);
		// Invoke the provided functor to evaluate force
		m_force = m_force_fun ? (*m_force_fun)(time, pressureH, m_areaH) : 0;

		// Add to existing force.
		C_force += m_force * relM.pos.GetNormalized();

	}
protected:
	myHYDRforce* m_force_fun;  ///< functor for force calculation
	double m_areaH = 10.;
	double m_areaR = 9.;
	double m_force;  ///< force functor double value
	ChFunction* m_pressureH;
	ChFunction* m_pressureR;

};
