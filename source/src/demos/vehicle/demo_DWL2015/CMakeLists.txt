#=============================================================================
# CMake configuration file for the DWL2015 demo.
# This example program requires Irrlicht run-time visualization
#=============================================================================

IF(NOT ENABLE_MODULE_IRRLICHT)
    RETURN()
ENDIF()

#--------------------------------------------------------------
# List all model files for this demo

SET(DEMO  demo_VEH_DWL2015)

SOURCE_GROUP("" FILES ${DEMO}.cpp)

#--------------------------------------------------------------
# Additional include directories

IF(ENABLE_MODULE_IRRLICHT)
  INCLUDE_DIRECTORIES(${CH_IRRLICHTINC})
ENDIF()

#--------------------------------------------------------------
# List of all required libraries

SET(LIBRARIES
    ChronoEngine
    ChronoEngine_vehicle
    ChronoModels_vehicle)

IF(ENABLE_MODULE_IRRLICHT)
  SET(LIBRARIES_IRR
      ChronoEngine_irrlicht
      ${CH_IRRLICHTLIB})
ELSE()
  SET(LIBRARIES_IRR "")
ENDIF()

#--------------------------------------------------------------
# Create the executable

MESSAGE(STATUS "...add ${DEMO}")

if(ENABLE_MODULE_IRRLICHT)
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
else()
  set(MY_CXX_FLAGS "${CH_CXX_FLAGS}")
endif()

ADD_EXECUTABLE(${DEMO} ${DEMO}.cpp ${MODEL_FILES})
SET_TARGET_PROPERTIES(${DEMO} PROPERTIES 
                      COMPILE_FLAGS "${MY_CXX_FLAGS}"
                      LINK_FLAGS "${LINKERFLAG_EXE}")
TARGET_LINK_LIBRARIES(${DEMO} ${LIBRARIES} ${LIBRARIES_IRR})
INSTALL(TARGETS ${DEMO} DESTINATION bin)
