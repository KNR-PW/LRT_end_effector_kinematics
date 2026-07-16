cmake_minimum_required(VERSION 3.16)
project(multi_end_effector_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
option(ACTIVE_BENCHMARKS "Build benchmarks")

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/path_management/package_path.h.in"
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/path_management/package_path.h" @ONLY
)

##################
## Dependencies ##
##################

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  serialization
  log
  log_setup
  thread
  date_time
  regex
  chrono
  atomic
)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(THIS_PACKAGE_DEPENDS
  ocs2_robotic_tools 
  ocs2_pinocchio_interface
)

set(FLAGS
        ${OCS2_CXX_FLAGS}
        ${pinocchio_CFLAGS_OTHER}
        -Wno-ignored-attributes
        -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
        -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
        -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)


foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  src/Settings.cpp
  src/solvers/InverseSolverInterface.cpp
  src/solvers/gradient_based/GradientBasedSolver.cpp
  src/solvers/gradient_based/NewtonRaphson.cpp
  src/solvers/gradient_based/NewtonRaphsonAD.cpp
  src/solvers/gradient_based/QuIK.cpp
  src/MultiEndEffectorKinematics.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_DEPENDS})
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

add_executable(R6BotIkDemoNode
  src/R6BotIkDemoNode.cpp
)

target_include_directories(R6BotIkDemoNode
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

ament_target_dependencies(R6BotIkDemoNode
  rclcpp
  sensor_msgs
  visualization_msgs
  ament_index_cpp
  ${THIS_PACKAGE_DEPENDS}
)

target_link_libraries(R6BotIkDemoNode
  ${PROJECT_NAME}
  Eigen3::Eigen
)

target_compile_options(R6BotIkDemoNode PUBLIC ${FLAGS})

#########################
###   CLANG TOOLING   ###
#########################

find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS src include test
            CT_HEADER_DIRS include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  R6BotIkDemoNode
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY test/models/meldog
  DESTINATION share/${PROJECT_NAME}/models
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES test/models/r6bot/r6bot.urdf.xacro test/models/r6bot/r6bot.urdf
  DESTINATION share/${PROJECT_NAME}/models/r6bot
)


install(
  DIRECTORY test/models/r6bot/meshes test/models/r6bot/rviz test/models/r6bot/urdf test/models/r6bot/srdf
  DESTINATION share/${PROJECT_NAME}/models/r6bot
)

install(
  DIRECTORY test/models/r6bot/launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY benchmarks
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test
            test/NewtonRaphsonTest.cpp
            test/NewtonRaphsonADTest.cpp
            test/QuIKTest.cpp
            test/MultEndEffectorKinematicsTest.cpp
            
  )
  ament_target_dependencies(${PROJECT_NAME}_test ${THIS_PACKAGE_DEPENDS})
  target_include_directories(${PROJECT_NAME}_test PRIVATE test/include)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  target_compile_options(${PROJECT_NAME}_test PUBLIC ${FLAGS})

  if(ACTIVE_BENCHMARKS)
    find_package(ament_cmake_google_benchmark REQUIRED)
    ament_add_google_benchmark(${PROJECT_NAME}_benchmark 
        benchmarks/SolversBenchmark.cpp
    )
    target_include_directories(${PROJECT_NAME}_benchmark PRIVATE benchmarks/include)
    target_link_libraries(${PROJECT_NAME}_benchmark ${PROJECT_NAME})
    target_compile_options(${PROJECT_NAME}_benchmark PUBLIC ${FLAGS})
  endif()
endif()

ament_package()
