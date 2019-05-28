cmake_minimum_required(VERSION 2.8.3)
project(test_pinocchio)

set(CMAKE_CXX_FLAGS "-std=c++11")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	urdf
	)

## System dependencies are found with CMake's conventions
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)
find_package(Eigen3 3.0.0 REQUIRED)
find_package(TinyXML 2.6 REQUIRED)
find_package(YamlCpp 0.5.2 REQUIRED) # Using a custom FindYamlcpp.cmake 

find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test_pinnochio
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

find_library(pinocchio NAMES libpinocchio.so PATHS /usr/local/lib REQUIRED)

###########
## Build ##
###########
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/cmake/config.h.cmake ${PROJECT_SOURCE_DIR}/include/Configuration.h)


## Specify additional locations of header files
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${TinyXML_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

## Your package locations should be listed before other locations
include_directories(
# include
/usr/local/include
${catkin_INCLUDE_DIRS}
)


add_executable(test_overview src/test_overview.cpp)
add_executable(test_urdf src/test_urdf.cpp)
add_executable(test_boost src/test_boost.cpp)


target_link_libraries(test_overview pinocchio ${catkin_LIBRARIES}) 
target_link_libraries(test_urdf pinocchio ${Boost_LIBRARIES} ${catkin_LIBRARIES}) 
target_link_libraries(test_boost pinocchio ${Boost_LIBRARIES} ${catkin_LIBRARIES}) 

