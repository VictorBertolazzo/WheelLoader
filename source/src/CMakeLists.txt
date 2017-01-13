#=============================================================================
#
# CMake configuration file for all Chrono libraries, demos, and tests.
#
#=============================================================================


#------------------------------------------------------------
# Propagates the CMake build to other directories
#------------------------------------------------------------

# Let all modules have access to the headers of the main library
include_directories(${CH_INCLUDES})

# Add directories to build various optional modules.  Each module is supposed
# to provide an option for enabling that particular module.
add_subdirectory(chrono_vehicle)

# Add directories for model libraries
add_subdirectory(chrono_models)

# Add directories for demo programs, unit test programs, and additional test programs.
  add_subdirectory(demos)

#------------------------------------------------------------
# Generate and install the Chrono configuration header
#------------------------------------------------------------

# Prepare substitution variables for the modules that were enabled
# TODO: the following tests should be more precise (we should test
#       that a particular module was successfully built, not just
#       enabled)

if(ENABLE_UNIT_CASCADE)
  set(CHRONO_CASCADE "#define CHRONO_CASCADE")
else()
  set(CHRONO_CASCADE "#undef CHRONO_CASCADE")
endif()

if(ENABLE_MODULE_COSIMULATION)
  set(CHRONO_COSIMULATION "#define CHRONO_COSIMULATION")
else()
  set(CHRONO_COSIMULATION "#undef CHRONO_COSIMULATION")
endif()

if(ENABLE_MODULE_FEA)
  set(CHRONO_FEA "#define CHRONO_FEA")
else()
  set(CHRONO_FEA "#undef CHRONO_FEA")
endif()

if(ENABLE_MODULE_IRRLICHT)
  set(CHRONO_IRRLICHT "#define CHRONO_IRRLICHT")
else()
  set(CHRONO_IRRLICHT "#undef CHRONO_IRRLICHT")
endif()

if(ENABLE_MODULE_MATLAB)
  set(CHRONO_MATLAB "#define CHRONO_MATLAB")
else()
  set(CHRONO_MATLAB "#undef CHRONO_MATLAB")
endif()

if(ENABLE_MODULE_MKL)
  set(CHRONO_MKL "#define CHRONO_MKL")
else()
  set(CHRONO_MKL "#undef CHRONO_MKL")
endif()

if(ENABLE_MODULE_PARALLEL)
  set(CHRONO_PARALLEL "#define CHRONO_PARALLEL")
else()
  set(CHRONO_PARALLEL "#undef CHRONO_PARALLEL")
endif()

if(ENABLE_MODULE_OPENGL)
  set(CHRONO_OPENGL "#define CHRONO_OPENGL")
else()
  set(CHRONO_OPENGL "#undef CHRONO_OPENGL")
endif()

if(ENABLE_MODULE_OGRE)
  set(CHRONO_OGRE "#define CHRONO_OGRE")
else()
  set(CHRONO_OGRE "#undef CHRONO_OGRE")
endif()

if(ENABLE_MODULE_POSTPROCESS)
  set(CHRONO_POSTPROCESS "#define CHRONO_POSTPROCESS")
else()
  set(CHRONO_POSTPROCESS "#undef CHRONO_POSTPROCESS")
endif()

if(ENABLE_MODULE_PYTHON)
  set(CHRONO_PYTHON "#define CHRONO_PYTHON")
else()
  set(CHRONO_PYTHON "#undef CHRONO_PYTHON")
endif()

if(ENABLE_MODULE_VEHICLE)
  set(CHRONO_VEHICLE "#define CHRONO_VEHICLE")
else()
  set(CHRONO_VEHICLE "#undef CHRONO_VEHICLE")
endif()

if(ENABLE_MODULE_FSI)
  set(CHRONO_FSI "#define CHRONO_FSI")
else()
  set(CHRONO_FSI "#undef CHRONO_FSI")
endif()

if(ENABLE_OPENMP)
  set(CHRONO_OPENMP_ENABLED "#define CHRONO_OPENMP_ENABLED")
else()
  set(CHRONO_OPENMP_ENABLED "#undef CHRONO_OPENMP_ENABLED")
endif()

if(ENABLE_TBB)
  set(CHRONO_TBB_ENABLED "#define CHRONO_TBB_ENABLED")
else()
  set(CHRONO_TBB_ENABLED "#undef CHRONO_TBB_ENABLED")
endif()

