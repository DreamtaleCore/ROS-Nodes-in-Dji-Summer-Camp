cmake_minimum_required(VERSION 2.8.3)
project(uav_core_logic)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uav_core_logic
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(coreLogic src/recvInfo.cxx src/sendCmd.cxx src/main.cxx)
target_link_libraries(coreLogic ${catkin_LIBRARIES})
