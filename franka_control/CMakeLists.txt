cmake_minimum_required(VERSION 3.0.2)
project(franka_control)
add_compile_options(-std=c++17 -Ofast)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(Eigen3 REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf 
 )

find_package(catkin REQUIRED)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} 
 CATKIN_DEPENDS 
 roscpp
 sensor_msgs
 std_msgs
)

include_directories(include )
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)

# Library
SET(${PROJECT_NAME}_HEADERS
  include/franka_control/qp_controller.hpp
  include/franka_control/robot.hpp
  )

SET(${PROJECT_NAME}_SOURCES
  src/robot.cpp
)


SET(CMAKE_BUILD_TYPE "Release")

ADD_LIBRARY(${PROJECT_NAME} SHARED  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC eiquadprog::eiquadprog pinocchio::pinocchio)

## Unit test - Math
add_executable(${PROJECT_NAME}_control src/qp_controller.cpp)
add_dependencies(${PROJECT_NAME}_control ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_control ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME}_control ${catkin_LIBRARIES})
