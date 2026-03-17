cmake_minimum_required(VERSION 3.8)
project(maple_ros2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolicyStatus.msg"
  "msg/EnvStatus.msg"
  "msg/EvalProgress.msg"
  "msg/ActionArray.msg"
  "srv/ServePolicy.srv"
  "srv/ServeEnv.srv"
  "srv/ListPolicies.srv"
  "srv/ListEnvs.srv"
  "srv/ListTasks.srv"
  "action/RunEval.action"
  "action/RunEpisode.action"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()