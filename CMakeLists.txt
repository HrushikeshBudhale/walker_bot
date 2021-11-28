cmake_minimum_required(VERSION 3.0.2)
project(walker_bot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(walker_bot_library src/walker_bot.cpp)

add_executable(${PROJECT_NAME}_node src/walker_bot_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  walker_bot_library
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_walker_bot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
