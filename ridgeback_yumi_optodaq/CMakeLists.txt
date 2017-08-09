cmake_minimum_required(VERSION 2.8.3)
project(ridgeback_yumi_optodaq)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  optoforce_etherdaq_driver
  roscpp
  roslaunch
)

find_package(Boost REQUIRED COMPONENTS system thread program_options)


catkin_package(
  LIBRARIES etherdaq_driver
  CATKIN_DEPENDS diagnostic_msgs diagnostic_updater geometry_msgs roscpp optoforce_etherdaq_driver
)

include_directories(
 include ${catkin_INCLUDE_DIRS}
)


add_executable(left_etherdaq_node src/left_etherdaq_node.cpp)
target_link_libraries(left_etherdaq_node etherdaq_driver)

add_executable(right_etherdaq_node src/right_etherdaq_node.cpp)
target_link_libraries(right_etherdaq_node etherdaq_driver)


roslaunch_add_file_check(launch/both_arms.launch)
roslaunch_add_file_check(launch/left_arm.launch)
roslaunch_add_file_check(launch/right_arm.launch)
