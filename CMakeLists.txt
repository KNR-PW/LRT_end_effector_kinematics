cmake_minimum_required(VERSION 3.8)
project(lrt_inverse_kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
# find dependencies
find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPENDS
  pinocchio
  eigen3_cmake_module
  Eigen3
  rclcpp 
  geometry_msgs
  trajectory_msgs
  control_msgs
  ocs2_pinocchio_interface
  ocs2_robotic_tools 
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include)

add_executable(EndEffectorKinematics
  src/solvers/InverseSolverInterface.cpp
  src/solvers/gradient_based/GradientBasedSolver.cpp
  src/solvers/gradient_based/NewtonRaphson.cpp
  # src/solvers/gradient_based/NewtonRaphsonAD.cpp
  src/InverseKinematics.cpp
  src/main.cpp
)

ament_target_dependencies(EndEffectorKinematics ${THIS_PACKAGE_DEPENDS})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  EndEffectorKinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES test/models/r6bot/r6bot.urdf.xacro
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

ament_package()
