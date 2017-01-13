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
// Main driver function for the dwl2015 full model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>//Add 5/12
#include <iostream>//Add 5/12

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_parallel/physics/ChSystemParallel.h" // Add 5/12

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/dwl2015/DWL2015.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace dwl2015;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 2.5);
ChQuaternion<> initRot(1, 0, 0, 0);
// ChQuaternion<> initRot(0.866025, 0, 0, 0.5);
// ChQuaternion<> initRot(0.7071068, 0, 0, 0.7071068);
// ChQuaternion<> initRot(0.25882, 0, 0, 0.965926);
// ChQuaternion<> initRot(0, 0, 0, 1);

enum DriverMode { DEFAULT, RECORD, PLAYBACK };
DriverMode driver_mode = PLAYBACK;//DEFAULT

// Visualization type for chassis & wheels (PRIMITIVES, MESH, or NONE)
VisualizationType vis_type = VisualizationType::PRIMITIVES;
// Visualization type for suspensions (PRIMITIVES, MESH, or NONE)
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
// Visualization type for front end loader (PRIMITIVES, MESH, or NONE)
VisualizationType frontendloader_vis_type = VisualizationType::MESH;
// Visualization type for wheel (PRIMITIVES, MESH, or NONE)
VisualizationType wheel_vis_type = VisualizationType::PRIMITIVES;

VisualizationType steering_vis_type = VisualizationType::NONE;



// Type of powertrain model (SHAFTS, SIMPLE)
PowertrainModelType powertrain_model = PowertrainModelType::SHAFTS;

// Drive type (FWD, RWD, or AWD)
DrivelineType drive_type = DrivelineType::AWD;

// Type of tire model (RIGID, PACEJKA, LUGRE, FIALA)
TireModelType tire_model = TireModelType::RIGID;

// Rigid terrain
RigidTerrain::Type terrain_model = RigidTerrain::FLAT;

double terrainHeight = 0;      // terrain height (FLAT terrain only)
double terrainLength = 100.0;  // size in X direction
double terrainWidth = 100.0;   // size in Y direction

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
ChMaterialSurfaceBase::ContactMethod contact_method = ChMaterialSurfaceBase::DEM;
bool contact_vis = false;

// Simulation step sizes
double step_size = 0.001;
double tire_step_size = step_size;

// Simulation end time
double t_end = 1000;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = "../DWL2015";
const std::string pov_dir = out_dir + "/POVRAY";

// Debug logging
bool debug_output = false;
double debug_step_size = 1.0 / 1;  // FPS = 1

// POV-Ray output
bool povray_output = false;

struct Entry {
	Entry() {}
	Entry(double time, double valve_inputs[4])
		: m_time(time), m_valveinputs(valve_inputs){}
	double m_time;
	double* m_valveinputs;
};

static bool compare(const Entry& a, const Entry& b) { return a.m_time < b.m_time; }

std::string valve_input = vehicle::GetDataFile("dwl2015/frontendloader/valveinput/sample_valve_input.txt");

// Temporary function for getting valve inputs from file
double *GetValveInputsFromFile(double time, const std::string& filename) { //Add 5/12
	std::vector<Entry> m_data;
	double valve_inputs[4];
	std::ifstream ifile(filename.c_str());
	std::string line;
	//ifile.open("C:\\chrono\\data\\vehicle\\dwl2015\\frontendloader\\valveinput");
	if (ifile.good()) {
		while (std::getline(ifile, line)) {
			std::istringstream iss(line);

			double time, valve_inputs[4];

			iss >> time >> *(valve_inputs + 0) >> valve_inputs[1] >> valve_inputs[2] >> valve_inputs[3];
			//GetLog() << time << "\t" << valve_inputs[1] << "\n";

			if (iss.fail())
				break;

			m_data.push_back(Entry(time, valve_inputs));
		}

		ifile.close();
	}
	else
	{
		GetLog() << "File hasn't been opened " << strerror(errno) << "\n";
	}
	
//	if (!sorted)
		std::sort(m_data.begin(), m_data.end(), compare);
		if (time <= m_data[0].m_time) {
			valve_inputs[0] = m_data[0].m_valveinputs[0];
			valve_inputs[1] = m_data[0].m_valveinputs[1];
			valve_inputs[2] = m_data[0].m_valveinputs[2];
			valve_inputs[3] = m_data[0].m_valveinputs[3];
			return valve_inputs;
		}
		else if (time >= m_data.back().m_time) {
			valve_inputs[0] = m_data.back().m_valveinputs[0];
			valve_inputs[1] = m_data.back().m_valveinputs[1];
			valve_inputs[2] = m_data.back().m_valveinputs[2];
			valve_inputs[3] = m_data.back().m_valveinputs[3];
			return valve_inputs;
		}


	//GetLog() << "Loaded Valve Inputs file: " << filename.c_str() << "\n";
	double init_values[4] = { 0,0,0,0 };
	
	std::vector<Entry>::iterator right = std::lower_bound(m_data.begin(), m_data.end(), Entry(time, init_values), compare);

	std::vector<Entry>::iterator left = right - 1;

	double tbar = (time - left->m_time) / (right->m_time - left->m_time);

	valve_inputs[0] = left->m_valveinputs[0] + tbar * (right->m_valveinputs[0] - left->m_valveinputs[0]);
	valve_inputs[1] = left->m_valveinputs[1] + tbar * (right->m_valveinputs[1] - left->m_valveinputs[1]);
	valve_inputs[2] = left->m_valveinputs[2] + tbar * (right->m_valveinputs[2] - left->m_valveinputs[2]);
	valve_inputs[3] = left->m_valveinputs[3] + tbar * (right->m_valveinputs[3] - left->m_valveinputs[3]);

	return valve_inputs;
}//Add 5/12 //Add 5/12


// =============================================================================



int main(int argc, char* argv[]) {
	int threads = 8;
    
	
	// --------------
    // Create systems
    // --------------

    // Create the DWL2015 vehicle, set parameters, and initialize
    DWL2015_Full my_dwl2015;
	my_dwl2015.SetContactMethod(contact_method);
    my_dwl2015.SetChassisFixed(false);
    my_dwl2015.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    my_dwl2015.SetPowertrainType(powertrain_model);
    my_dwl2015.SetDriveType(drive_type);
    my_dwl2015.SetTireType(tire_model);
    my_dwl2015.SetTireStepSize(tire_step_size);
    my_dwl2015.Initialize();
	

	//First Initialize, then Visualize
	my_dwl2015.SetChassisVisualizationType(vis_type);
	my_dwl2015.SetWheelVisualizationType(wheel_vis_type);
	my_dwl2015.SetSuspensionVisualizationType(suspension_vis_type);
	my_dwl2015.SetSteeringVisualizationType(steering_vis_type);
	//my_dwl2015.SetTireVisualizationType(tire_vis_type);
	my_dwl2015.SetFrontEndLoaderVisualizationType(frontendloader_vis_type);
	
	/*
	VisualizationType tire_vis_type =
		(tire_model == TireModelType::RIGID_MESH) ? VisualizationType::MESH : VisualizationType::PRIMITIVES;
    */
	{
	// Set number of threads.
	int max_threads = CHOMPfunctions::GetNumProcs();
	if (threads > max_threads)
		threads = max_threads;
	my_dwl2015.GetSystem()->SetParallelThreadNumber(threads);
	int actual = my_dwl2015.GetSystem()->GetParallelThreadNumber();
	GetLog() << "threads: " << threads << ",	 max_threads: " << max_threads << ",	 actual_threads: " << actual  << "\n";
	CHOMPfunctions::SetNumThreads(threads);
	}

  // Create the terrain
    RigidTerrain terrain(my_dwl2015.GetSystem());
	terrain.SetContactFrictionCoefficient(0.9f);
	terrain.SetContactRestitutionCoefficient(0.01f);
	terrain.SetContactMaterialProperties(2e7f, 0.3f);
    //// terrain.SetContactMaterial(0.9f, 0.01f, 2e7f, 0.3f);  // old
    terrain.SetColor(ChColor(0.8f, 0.8f, 0.5f));
    switch (terrain_model) {
        case RigidTerrain::FLAT:
            terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            terrain.Initialize(terrainHeight, terrainLength, terrainWidth);
            break;
        case RigidTerrain::HEIGHT_MAP:
            terrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            terrain.Initialize(vehicle::GetDataFile("terrain/height_maps/test64.bmp"), "test64", 128, 128, 0, 4);
            break;
        case RigidTerrain::MESH:
            terrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            terrain.Initialize(vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
            break;
    }

    // Create the vehicle Irrlicht interface
    ChWheeledVehicleIrrApp app(&my_dwl2015.GetVehicle(), &my_dwl2015.GetPowertrain(), L"DWL2015 Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
	//app.AddTypicalCamera(irr::core::vector3df(0, 4, -6));
	app.SetChaseCamera(trackPoint, 12.0, 5.0);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

    // -----------------
    // Initialize output
    // -----------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (povray_output) {
        if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    std::string driver_file = out_dir + "/driver_inputs.txt";
    utils::CSV_writer driver_csv(" ");

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // If in playback mode, attach the data file to the driver system and
    // force it to playback the driver inputs.
    if (driver_mode == PLAYBACK) {
		std::string driver_input = vehicle::GetDataFile("generic/driver/Sample_Maneuver.txt");

		driver.SetInputDataFile(driver_input);//driver_file not found
        driver.SetInputMode(ChIrrGuiDriver::DATAFILE);
    }

    driver.Initialize();

    // ---------------
    // Simulation loop
    // ---------------

    if (debug_output) {
        GetLog() << "\n\n============ System Configuration ============\n";
        my_dwl2015.LogHardpointLocations();
    }

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int debug_steps = (int)std::ceil(debug_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    ChRealtimeStepTimer realtime_timer;
    int step_number = 0;
    int render_frame = 0;
    double time = 0;

    while (app.GetDevice()->run()) {
        time = my_dwl2015.GetSystem()->GetChTime();

        // End simulation
        if (time >= t_end)
            break;

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(my_dwl2015.GetSystem(), filename);
            }

            render_frame++;
        }

        // Debug logging
        if (debug_output && step_number % debug_steps == 0) {
            GetLog() << "\n\n============ System Information ============\n";
            GetLog() << "Time = " << time << "\n\n";
            my_dwl2015.DebugLog(OUT_SPRINGS | OUT_SHOCKS | OUT_CONSTRAINTS);
        }
		
        // Collect output data from modules (for inter-module communication)
        double throttle_input = driver.GetThrottle();
        double steering_input = driver.GetSteering();
        double braking_input = driver.GetBraking();


        // Driver output
        if (driver_mode == RECORD) {
            driver_csv << time << steering_input << throttle_input << braking_input << std::endl;
        }

		double * valve_inputs = GetValveInputsFromFile(time, valve_input); //{ 0,0,0,0 };// TEMPORARY//A ChDataDriver-like class will be implemented.
		/// Set Valve Command data
		my_dwl2015.SetValveCommand(DWL2015_Vehicle::M_PENUMA_BL_ID, *(valve_inputs+0),*(valve_inputs+1) );
		//my_dwl2015.SetValveCommand(DWL2015_Vehicle::M_PNEUMA_LA_ID, +step_number, -step_number);

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
// // I've overloaded the method, s.t. it works either with or without valve inputs		
        my_dwl2015.Synchronize(time, steering_input, braking_input, throttle_input, terrain, valve_inputs);
        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);

        terrain.Advance(step);
		my_dwl2015.Advance(step);
        app.Advance(step);

//		// Output pneumatic forces
		double pneumoforce = my_dwl2015.GetPneumaticForce(DWL2015_Vehicle::M_PENUMA_BL_ID);//I've not modified ChSubsysDefs.h
		//GetLog() << "Pneumatic = " << pneumoforce << " " << "Udm?" << "\n";//Working, numerical zero at the moment
//		// Output Valve commands
		double * command = my_dwl2015.GetValveCommand(DWL2015_Vehicle::M_PENUMA_BL_ID);
        //GetLog() << "Valves = " << command[0] << " , " << *(command + 1) << "Udm?" << "\n"; // Unable to read memory

// Increment frame number
        step_number++;
    }

    if (driver_mode == RECORD) {
        driver_csv.write_to_file(driver_file);
    }

    return 0;
}
