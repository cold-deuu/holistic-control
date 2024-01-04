cmake_minimum_required(VERSION 3.0.2)
project(rci_qp_controller)
add_compile_options(-std=c++17 -Ofast)

SET(CXX_DISABLE_WERROR TRUE)
SET(CMAKE_VERBOSE_MAKEFILE TRUE)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf 
 )


find_package(catkin REQUIRED)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} 
 CATKIN_DEPENDS 
)

include_directories(include )
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)


# Main Library
SET(${PROJECT_NAME}_MATH_HEADERS
  include/rci_qp_controller/math/fwd.hpp
  include/rci_qp_controller/math/util.hpp
  include/rci_qp_controller/math/constraint_base.hpp
  include/rci_qp_controller/math/constraint_equality.hpp
  include/rci_qp_controller/math/constraint_inequality.hpp
  include/rci_qp_controller/math/constraint_bound.hpp
  )

SET(${PROJECT_NAME}_TASKS_HEADERS
  include/rci_qp_controller/tasks/fwd.hpp
  include/rci_qp_controller/tasks/task_base.hpp
  include/rci_qp_controller/tasks/task_motion.hpp
  include/rci_qp_controller/tasks/task_se3_equality.hpp
  include/rci_qp_controller/tasks/task_joint_bound.hpp
  include/rci_qp_controller/tasks/task_joint_posture.hpp
  )

SET(${PROJECT_NAME}_TRAJECTORIES_HEADERS
  include/rci_qp_controller/trajectory/fwd.hpp
  include/rci_qp_controller/trajectory/trajectory_base.hpp
  include/rci_qp_controller/trajectory/trajectory_se3.hpp
  include/rci_qp_controller/trajectory/trajectory_euclidian.hpp
  )

SET(${PROJECT_NAME}_SOLVERS_HEADERS
  include/rci_qp_controller/solver/fwd.hpp
  include/rci_qp_controller/solver/util.hpp
  include/rci_qp_controller/solver/solver_HQP_output.hpp
  include/rci_qp_controller/solver/solver_HQP_base.hpp
  include/rci_qp_controller/solver/solver_HQP_factory.hpp
  include/rci_qp_controller/solver/solver_HQP_factory.hxx
  include/rci_qp_controller/solver/solver_HQP_eiquadprog.hpp
  include/rci_qp_controller/solver/solver_HQP_eiquadprog_rt.hpp
  include/rci_qp_controller/solver/solver_HQP_eiquadprog_rt.hxx
  include/rci_qp_controller/solver/solver_HQP_eiquadprog_fast.hpp
  )

SET(${PROJECT_NAME}_ROBOTS_HEADERS
  include/rci_qp_controller/robot/fwd.hpp
  include/rci_qp_controller/robot/robot_wrapper.hpp
  )

SET(${PROJECT_NAME}_FORMULATIONS_HEADERS
  include/rci_qp_controller/formulation/inverse_dynamics_formulation_base.hpp
  include/rci_qp_controller/formulation/inverse_dynamics_formulation_acc.hpp
  )

SET(${PROJECT_NAME}_HEADERS
  include/rci_qp_controller/config.hpp
  include/rci_qp_controller/util/statistics.hpp
  include/rci_qp_controller/util/stop_watch.hpp
  include/rci_qp_controller/util/Stdafx.hh
  ${${PROJECT_NAME}_MATH_HEADERS}
  ${${PROJECT_NAME}_TASKS_HEADERS}
  ${${PROJECT_NAME}_TRAJECTORIES_HEADERS}
  ${${PROJECT_NAME}_SOLVERS_HEADERS}
  ${${PROJECT_NAME}_ROBOTS_HEADERS}
  ${${PROJECT_NAME}_FORMULATIONS_HEADERS}
  )

LIST(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS)

SET(${PROJECT_NAME}_MATH_SOURCES
  src/math/constraint_base.cpp
  src/math/constraint_equality.cpp
  src/math/constraint_inequality.cpp
  src/math/constraint_bound.cpp
  src/math/util.cpp
  )

SET(${PROJECT_NAME}_TASKS_SOURCES
  src/tasks/task_base.cpp
  src/tasks/task_joint_bound.cpp
  src/tasks/task_joint_posture.cpp
  src/tasks/task_motion.cpp
  src/tasks/task_se3_equality.cpp
  )

SET(${PROJECT_NAME}_TRAJECTORIES_SOURCES
  src/trajectory/trajectory_se3.cpp
  src/trajectory/trajectory_euclidian.cpp
  )

SET(${PROJECT_NAME}_SOLVERS_SOURCES
  src/solver/solver_HQP_base.cpp
  src/solver/solver_HQP_factory.cpp
  src/solver/solver_HQP_eiquadprog.cpp
  src/solver/solver_HQP_eiquadprog_fast.cpp
  src/solver/util.cpp
  )

SET(${PROJECT_NAME}_ROBOTS_SOURCES
  src/robot/robot_wrapper.cpp
  )

SET(${PROJECT_NAME}_FORMULATIONS_SOURCES
  src/formulation/inverse_dynamics_formulation_base.cpp
  src/formulation/inverse_dynamics_formulation_acc.cpp
  )

SET(${PROJECT_NAME}_SOURCES
  src/util/statistics.cpp
  src/util/stop_watch.cpp
  ${${PROJECT_NAME}_MATH_SOURCES}
  ${${PROJECT_NAME}_ROBOTS_SOURCES}
  ${${PROJECT_NAME}_TRAJECTORIES_SOURCES}
  ${${PROJECT_NAME}_TASKS_SOURCES}
  ${${PROJECT_NAME}_SOLVERS_SOURCES}
  ${${PROJECT_NAME}_FORMULATIONS_SOURCES}
  )


SET(CMAKE_BUILD_TYPE "Release")

ADD_LIBRARY(${PROJECT_NAME} SHARED  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC eiquadprog::eiquadprog pinocchio::pinocchio)

## Unit test - Math
add_executable(${PROJECT_NAME}_math unittest/math.cpp)
add_dependencies(${PROJECT_NAME}_math ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_math ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME}_math ${catkin_LIBRARIES})

## Unit test - Constraint
add_executable(${PROJECT_NAME}_constraint unittest/constraint.cpp)
add_dependencies(${PROJECT_NAME}_constraint ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_constraint ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME}_constraint ${catkin_LIBRARIES})

## Unit test - Trajectory
add_executable(${PROJECT_NAME}_traj unittest/trajectory.cpp)
add_dependencies(${PROJECT_NAME}_traj ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_traj ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME}_traj ${catkin_LIBRARIES})

## Unit test - Robot
add_executable(${PROJECT_NAME}_robot unittest/robot-wrapper.cpp)
add_dependencies(${PROJECT_NAME}_robot ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_robot ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME}_robot ${catkin_LIBRARIES})

## Unit test - Task
add_executable(${PROJECT_NAME}_task unittest/task.cpp)
add_dependencies(${PROJECT_NAME}_task ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_task ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio)
target_link_libraries(${PROJECT_NAME}_task ${catkin_LIBRARIES})
