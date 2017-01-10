#=============================================================================
# CMake configuration file for Chrono Vehicle Models
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

message(STATUS "Models for the VEHICLE module...")

# ----------------------------------------------------------------------------
# List the files in the Chrono_vehicle_models library
# ----------------------------------------------------------------------------

set(CVM_COMMON_FILES
    ../ChApiModels.h
)
source_group("" FILES ${CVM_COMMON_FILES})

set(CVM_DWL2015_FILES
    dwl2015/DWL2015.cpp
    dwl2015/DWL2015.h
    dwl2015/DWL2015_BrakeSimple.cpp
    dwl2015/DWL2015_BrakeSimple.h
    dwl2015/DWL2015_Chassis.h
    dwl2015/DWL2015_Chassis.cpp
    dwl2015/DWL2015_DoubleWishbone.cpp
    dwl2015/DWL2015_DoubleWishbone.h
    dwl2015/DWL2015_DoubleWishboneReduced.cpp
    dwl2015/DWL2015_DoubleWishboneReduced.h
    dwl2015/DWL2015_Driveline2WD.cpp
    dwl2015/DWL2015_Driveline2WD.h
    dwl2015/DWL2015_Driveline4WD.cpp
    dwl2015/DWL2015_Driveline4WD.h
    dwl2015/DWL2015_FialaTire.cpp
    dwl2015/DWL2015_FialaTire.h
    dwl2015/DWL2015_LugreTire.cpp
    dwl2015/DWL2015_LugreTire.h
    dwl2015/DWL2015_PitmanArm.cpp
    dwl2015/DWL2015_PitmanArm.h
    dwl2015/DWL2015_Powertrain.cpp
    dwl2015/DWL2015_Powertrain.h
    dwl2015/DWL2015_RackPinion.cpp
    dwl2015/DWL2015_RackPinion.h
    dwl2015/DWL2015_RigidTire.cpp
    dwl2015/DWL2015_RigidTire.h
    dwl2015/DWL2015_SimpleDriveline.cpp
    dwl2015/DWL2015_SimpleDriveline.h
    dwl2015/DWL2015_SimplePowertrain.cpp
    dwl2015/DWL2015_SimplePowertrain.h
    dwl2015/DWL2015_Vehicle.h
    dwl2015/DWL2015_VehicleFull.cpp
    dwl2015/DWL2015_VehicleFull.h
    dwl2015/DWL2015_VehicleReduced.cpp
    dwl2015/DWL2015_VehicleReduced.h
    dwl2015/DWL2015_Wheel.cpp
    dwl2015/DWL2015_Wheel.h
)
if(ENABLE_MODULE_FEA)
  set(CVM_DWL2015_FEA_FILES
      dwl2015/DWL2015_ANCFTire.cpp
      dwl2015/DWL2015_ANCFTire.h
	  dwl2015/DWL2015_ReissnerTire.cpp
      dwl2015/DWL2015_ReissnerTire.h
  )
else()
  set(CVM_DWL2015_FEA_FILES "")
endif()
source_group("dwl2015" FILES ${CVM_DWL2015_FILES} ${CVM_DWL2015_FEA_FILES})

# ----------------------------------------------------------------------------
# Add the Chrono_vehicle_models library
# ----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LIBRARIES ChronoEngine ChronoEngine_vehicle)
if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_irrlicht)
    list(APPEND LIBRARIES ${CH_IRRKLANGLIB})
    include_directories(${CH_IRRLICHTINC})
endif()

if(ENABLE_MODULE_FEA)
    list(APPEND LIBRARIES ChronoEngine_fea)
endif()

add_library(ChronoModels_vehicle SHARED
    ${CVM_DWL2015_FILES}
    ${CVM_DWL2015_FEA_FILES}
    )

set_target_properties(ChronoModels_vehicle PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_MODELS")

target_link_libraries(ChronoModels_vehicle ${LIBRARIES})

install(TARGETS ChronoModels_vehicle
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_vehicle/models/vehicle
        FILES_MATCHING PATTERN "*.h")
