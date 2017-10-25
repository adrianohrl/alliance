cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(alliance)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  alliance_msgs
  roscpp
)

find_package(ruler REQUIRED COMPONENTS
  ruler_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}
    ruler
  CATKIN_DEPENDS
    alliance_msgs
    pluginlib
    roscpp
    ruler
    ruler_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ruler_INCLUDE_DIRS}
)

set(H_FILES
  include/alliance/acquiescence.h
  include/alliance/activity_suppression.h
  include/alliance/alliance.h
  include/alliance/behaved_robot.h
  include/alliance/behaviour_set.h
  include/alliance/behaviour_set_interface.h
  include/alliance/impatience.h
  include/alliance/impatience_reset.h
  include/alliance/inter_robot_communication.h
  include/alliance/layer.h
  include/alliance/layered_behaviour_set.h
  include/alliance/motivational_behaviour.h
  include/alliance/robot.h
  include/alliance/robot_interface.h
  include/alliance/sensor.h
  include/alliance/sensory_evaluator.h
  include/alliance/sensory_feedback.h
  include/alliance/task.h
  include/nodes/alliance_event.h
  include/nodes/high_level_node.h
  include/nodes/low_level_node.h
  include/nodes/alliance_observer.h
  include/nodes/alliance_node.h
  include/nodes/alliance_subject.h
  include/nodes/ros_sensor_message.h
)

set(CPP_FILES
  src/alliance/acquiescence.cpp
  src/alliance/activity_suppression.cpp
  src/alliance/behaved_robot.cpp
  src/alliance/behaviour_set.cpp
  src/alliance/impatience.cpp
  src/alliance/impatience_reset.cpp
  src/alliance/inter_robot_communication.cpp
  src/alliance/layer.cpp
  src/alliance/layered_behaviour_set.cpp
  src/alliance/motivational_behaviour.cpp
  src/alliance/robot.cpp
  src/alliance/sensor.cpp
  src/alliance/sensory_evaluator.cpp
  src/alliance/sensory_feedback.cpp
  src/alliance/task.cpp
  src/nodes/high_level_node.cpp
  src/nodes/low_level_node.cpp
)

add_library(${PROJECT_NAME}
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${ruler_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ruler_LIBRARIES}
)

add_executable(high_level
  src/high_level.cpp
  ${CPP_FILES}
)

add_dependencies(high_level
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${ruler_EXPORTED_TARGETS}
)

target_link_libraries(high_level
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ruler_LIBRARIES}
)

add_executable(low_level
  src/low_level.cpp
  ${CPP_FILES}
)

add_dependencies(low_level
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${ruler_EXPORTED_TARGETS}
)

target_link_libraries(low_level
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ruler_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}_utest
  test/utest.cpp
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}_utest
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${ruler_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_utest
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${ruler_LIBRARIES}
)
