cmake_minimum_required(VERSION 2.8.3)
project(fsm_bump_go)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(${PROJECT_NAME}
 # src/fsm_bump_go/bumpgo_Basic.cpp
  src/fsm_bump_go/bumpgo_Advanced.cpp
  src/fsm_bump_go/bumpgo_Laser.cpp
  src/fsm_bump_go/BumpGo.cpp 
)

# add_executable(bumpgo_Basic_node src/bumpgo_Basic_node.cpp)
add_executable(bumpgo_Advanced_node src/bumpgo_Advanced_node.cpp)
add_executable(bumpgo_Laser_node src/bumpgo_Laser_node.cpp)
add_executable(bumpgo_node src/bumpgo_node.cpp)

# target_link_libraries(bumpgo_Basic_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(bumpgo_Advanced_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(bumpgo_Laser_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(bumpgo_node ${catkin_LIBRARIES} ${PROJECT_NAME})

roslint_cpp(
 src/bumpgo_Basic_node.cpp
 src/bumpgo_Advanced_node.cpp
 src/bumpgo_Laser_node.cpp
 
 src/fsm_bump_go/bumpgo_Basic.cpp
 src/fsm_bump_go/bumpgo_Advanced.cpp
 src/fsm_bump_go/bumpgo_Laser.cpp
 
 include/fsm_bump_go/bumpgo_Basic.h
 include/fsm_bump_go/bumpgo_Advanced.h
 include/fsm_bump_go/bumpgo_Laser.h
)
