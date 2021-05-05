cmake_minimum_required(VERSION 3.0.2)
project(canTest)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES canTest
#  CATKIN_DEPENDS rospy rpscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(canTest1 src/canTest1.cpp)
target_link_libraries(canTest1 ${catkin_LIBRARIES})

add_executable(canTest2 src/canTest2.cpp)
target_link_libraries(canTest2 ${catkin_LIBRARIES})

add_executable(canTest3 src/canTest3.cpp)
target_link_libraries(canTest3 ${catkin_LIBRARIES})

add_executable(canTest4 src/canTest4.cpp)
target_link_libraries(canTest4 ${catkin_LIBRARIES})

