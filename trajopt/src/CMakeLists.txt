include_directories(
  ${TRAJOPT_SOURCE_DIR}/ext
  ${TRAJOPT_SOURCE_DIR}/src
)


add_definitions(-DEIGEN_DEFAULT_TO_ROW_MAJOR)

# gcc 4.7 workaround:
# execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
#                 OUTPUT_VARIABLE GCC_VERSION)
# if (GCC_VERSION VERSION_GREATER 4.6)
#   message("GCC > 4.6 detected")
#    add_definitions(-D_GLIBCXX_HAVE_GTHR_DEFAULT)
# endif()

find_package(GUROBI)
if (NOT GUROBI_FOUND)
  message(FATAL_ERROR "Gurobi not found. If you want to use Gurobi, set the GUROBI_HOME variable")
else()
  message("Gurobi found")
  include_directories(${GUROBI_INCLUDE_DIRS})
endif()

set(OpenRAVE_BOTH_LIBRARIES ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES})

add_subdirectory(sco)
add_subdirectory(trajopt)
add_subdirectory(utils)
if(BUILD_CLOUDPROC) 
	add_subdirectory(cloudproc)
endif()
if(BUILD_SANDBOX)
	add_subdirectory(sandbox)
endif()
add_subdirectory(osgviewer)
if(BUILD_HUMANOIDS)
  add_subdirectory(humanoids)
endif()
if(BUILD_SENSORSIM)
  add_subdirectory(sensorsim)
endif()

add_subdirectory(pclVisualizer)

if(NOT APPLE)
endif()
