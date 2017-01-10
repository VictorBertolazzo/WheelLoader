#=============================================================================
# CMake configuration file for Chrono Vehicle module
#
# Cannot be used stand-alone (it is loaded by parent CMake configuration file)
#=============================================================================

option(ENABLE_MODULE_VEHICLE "Enable the Chrono Vehicle module" OFF)

# Return now if this module is not enabled
if(NOT ENABLE_MODULE_VEHICLE)
   mark_as_advanced(FORCE ENABLE_IRRKLANG)
   mark_as_advanced(FORCE CH_IRRKLANG_SDKDIR)
   mark_as_advanced(FORCE CH_IRRKLANGLIB)
   return()
endif()

message(STATUS "==== Chrono Vehicle module ====")

mark_as_advanced(CLEAR ENABLE_IRRKLANG)
mark_as_advanced(CLEAR CH_IRRKLANG_SDKDIR)
mark_as_advanced(CLEAR CH_IRRKLANGLIB)

# If Irrlicht support was enabled, provide option to add Irrklang support.
#cmake_dependent_option(ENABLE_IRRKLANG "Enable Irrklang library for sound" OFF
#                       "ENABLE_MODULE_IRRLICHT" OFF)

# ----------------------------------------------------------------------------
# Find IrrKlang library
# ----------------------------------------------------------------------------
if(ENABLE_IRRKLANG)

  if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
      set(CH_IRRKLANG_SDKDIR ""  CACHE PATH   "Where is your IrrKlang SDK installed?")
      set(CH_IRRKLANGLIB   "" CACHE FILEPATH "Where is your Irrklank library?")
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
      FIND_PATH(CH_IRRKLANG_SDKDIR NAMES irrlicht.h PATHS "/usr/include/irrklang" "/usr/local/include/irrklang")
      FIND_LIBRARY(CH_IRRKLANGLIB NAMES Irrklang PATHS "/usr/local/lib" ${CH_IRRKLANG_SDKDIR}/lib/Linux)
  endif()

  if(EXISTS "${CH_IRRKLANG_SDKDIR}/include")
      set(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}/include")
  else()
      set(CH_IRRKLANGINC "${CH_IRRKLANG_SDKDIR}")
  endif()

  include_directories(${CH_IRRKLANGINC})

  mark_as_advanced(CLEAR CH_IRRKLANG_SDKDIR)
  mark_as_advanced(CLEAR CH_IRRKLANGLIB)

else()

  mark_as_advanced(FORCE CH_IRRKLANG_SDKDIR)
  mark_as_advanced(FORCE CH_IRRKLANGLIB)

endif()

# ----------------------------------------------------------------------------
# Generate and install configuration file
# ----------------------------------------------------------------------------

# Prepare replacement variables

if(ENABLE_IRRKLANG)
  set(CHRONO_IRRKLANG "#define CHRONO_IRRKLANG")
else()
  set(CHRONO_IRRKLANG "#undef CHRONO_IRRKLANG")
endif()

# ----------------------------------------------------------------------------
# List the files in the ChronoEngine_vehicle library
# ----------------------------------------------------------------------------

set(CV_WV_FRONTENDLOADER_FILES
    wheeled_vehicle/frontendloader/ChFrontEndLoader.h
    wheeled_vehicle/frontendloader/ChFrontEndLoader.cpp
)
source_group("wheeled_vehicle\\frontendloader" FILES ${CV_WV_FRONTENDLOADER_FILES})


# ----------------------------------------------------------------------------
# Add the ChronoEngine_vehicle library
# ----------------------------------------------------------------------------

set(CXX_FLAGS ${CH_CXX_FLAGS})
set(LINK_FLAGS ${CH_LINKERFLAG_SHARED})
set(LIBRARIES "ChronoEngine")

if(ENABLE_MODULE_IRRLICHT)
    set(CXX_FLAGS "${CXX_FLAGS} ${CH_IRRLICHT_CXX_FLAGS}")
    list(APPEND LIBRARIES ChronoEngine_irrlicht)
    list(APPEND LIBRARIES ${CH_IRRKLANGLIB})
    include_directories(${CH_IRRLICHTINC})
endif()

if(ENABLE_MODULE_FEA)
    list(APPEND LIBRARIES ChronoEngine_fea)
endif()

if(MPI_CXX_FOUND)
    set(CXX_FLAGS "${CXX_FLAGS} ${MPI_CXX_COMPILE_FLAGS}")
    set(LINK_FLAGS "${LINK_FLAGS} ${MPI_CXX_LINK_FLAGS}")
    include_directories(${MPI_CXX_INCLUDE_PATH})
    list(APPEND LIBRARIES ${MPI_CXX_LIBRARIES})
endif()

add_library(ChronoEngine_vehicle SHARED
#
    ${CV_WV_FRONTENDLOADER_FILES}
 )

set_target_properties(ChronoEngine_vehicle PROPERTIES
                      COMPILE_FLAGS "${CXX_FLAGS}"
                      LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
                      COMPILE_DEFINITIONS "CH_API_COMPILE_VEHICLE")

target_link_libraries(ChronoEngine_vehicle ${LIBRARIES})

install(TARGETS ChronoEngine_vehicle
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib64
        ARCHIVE DESTINATION lib64)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/chrono_vehicle
        FILES_MATCHING PATTERN "*.h")

#-------------------------------------------------------------------------------
# On Windows, copy the Irrklang DLL (if enabled)
#-------------------------------------------------------------------------------
if(${CMAKE_SYSTEM_NAME} MATCHES "Windows" AND ENBALE_IRRKLANG)
    if("${CH_COMPILER}" STREQUAL "COMPILER_MSVC")
        set(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Win32-VisualStudio/irrKlang.dll")
    elseif("${CH_COMPILER}" STREQUAL "COMPILER_MSVC_X64")
        set(CH_IRRKLANG_DLL "${CH_IRRKLANG_SDKDIR}/bin/Winx64-VisualStudio/irrKlang.dll")
    endif()

    add_custom_command(TARGET ChronoEngine_vehicle POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CH_IRRKLANG_DLL}"
        "${PROJECT_BINARY_DIR}/bin/$<CONFIGURATION>")
endif()
