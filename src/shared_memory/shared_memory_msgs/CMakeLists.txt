cmake_minimum_required(VERSION 3.8)
project(shared_memory_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GoalStartMsg.msg"
  "msg/GoalReachMsg.msg"
  "srv/GetRobotState.srv"
  "srv/GetInventoryStatus.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
