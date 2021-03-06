
SET(MODELS
#  cascading_packing_condition
#  central_pivot_mech
#  demo_IRR_fel
#  demo_PAR_friction
  demo_VEH_ArticulatedVehicle
  demo_PAR_WheelLoader
#  demo_VEH_wheelloader
  hydraulic_force
  iArmlinkage
  test_granular_PAR
#  test_granular_PAR_cyl
#  test_granular_PAR_funnel
#  test_granular_PAR_layer
  test_HYDR_actuator # future
  test_mesh
  test_normal
  test_normal_PAR
  test_PAR_rolling_friction
  test_SEQ_rolling_friction
  zBarlinkage
  zBarlinkage_PAR
  zBarSandpile
  )

  SET(MODEL_FILES
    subsystems/Articulated_Front.h
    subsystems/Articulated_Front.cpp
    subsystems/Articulated_Rear.h
    subsystems/Articulated_Rear.cpp
)
 SET(UTILITIES
 utilities/UtilityFunctions.h
 )

SOURCE_GROUP("utilities" FILES ${UTILITIES})
SOURCE_GROUP("subsystems" FILES ${MODEL_FILES})
SOURCE_GROUP("" FILES ${MODELS}.cpp ${MODEL_FILES})

#--------------------------------------------------------------
# Find the Chrono package with required components
#--------------------------------------------------------------

# Invoke find_package in CONFIG mode

find_package(Chrono
             COMPONENTS Irrlicht Opengl Parallel Vehicle
             CONFIG
)

# If Chrono and/or the required component(s) were not found, return now.

if(NOT Chrono_FOUND)
  message("Could not find requirements for Chrono tutorials")
  return()
endif()

include_directories(${CHRONO_INCLUDE_DIRS})

#--------------------------------------------------------------
# Append to the parent's list of DLLs (and make it visible up)
#--------------------------------------------------------------

list(APPEND ALL_DLLS "${CHRONO_DLLS}")
set(ALL_DLLS "${ALL_DLLS}" PARENT_SCOPE)

#--------------------------------------------------------------
# Compilation flags
#--------------------------------------------------------------

set(COMPILE_FLAGS ${CHRONO_CXX_FLAGS})

# Disable some warnings triggered by Irrlicht (Windows only)
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(COMPILE_FLAGS "${COMPILE_FLAGS} /wd4275")
endif()


foreach(PROGRAM ${MODELS})

  message(STATUS "...add ${PROGRAM}")

  add_executable(${PROGRAM}  "${PROGRAM}.cpp" ${MODEL_FILES} ${UTILITIES})
  source_group(""  FILES "${PROGRAM}.cpp")

  set_target_properties(${PROGRAM} PROPERTIES
    FOLDER demos
    COMPILE_FLAGS "${COMPILE_FLAGS}"
    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
    LINK_FLAGS "${COMPILE_FLAGS} ${CHRONO_LINKER_FLAGS}"
  )

  target_link_libraries(${PROGRAM} ${CHRONO_LIBRARIES})

endforeach(PROGRAM)
