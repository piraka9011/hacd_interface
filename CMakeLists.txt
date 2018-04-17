cmake_minimum_required(VERSION 2.8.3)
project(hacd_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
)

#find_package(OpenMP)
#if(OPENMP_FOUND)
#  set(CMAKE_CXX_FLAGS “${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}”)
#  set(CMAKE_C_FLAGS “${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}”)
#  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()
#set(LIB_TYPE "STATIC" CACHE STRING "STATIC, SHARED or MODULE?")
#message("[HACD] \t LIB_TYPE " ${LIB_TYPE})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  #CATKIN_DEPENDS
  #DEPENDS OpenMp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
set(PROJECT_CPP_FILES
  src/hacdGraph.cpp
  src/hacdHACD.cpp
  src/hacdICHull.cpp
  src/hacdManifoldMesh.cpp
  src/hacdMeshDecimator.cpp
  src/hacdMicroAllocator.cpp
  src/hacdRaycastMesh.cpp
)

# Create the actual HACD Libary
add_library(HACD
  ${PROJECT_CPP_FILES}
)
target_link_libraries(HACD
  ${catkin_LIBRARIES}
)

# Create the interface to the HACD Library
add_library(${PROJECT_NAME}
  src/hacdInterface.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  HACD
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
