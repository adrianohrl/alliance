cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

project(alliance)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  alliance_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    alliance_msgs
    pluginlib
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
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
  include/nodes/alliance_node.h
  include/nodes/alliance_observer.h
  include/nodes/alliance_subject.h
  include/nodes/high_level_node.h
  include/nodes/low_level_node.h
  include/nodes/motivation_publishers.h
  include/nodes/ros_sensor_message.h
  include/utilities/continuous_noisy_signal.h
  include/utilities/continuous_signal_type.h
  include/utilities/discrete_noisy_signal.h
  include/utilities/discrete_signal_type.h
  include/utilities/enum_converter.h
  include/utilities/exception.h
  include/utilities/event.h
  include/utilities/functions/buffered_function.h
  include/utilities/functions/exponential_function.h
  include/utilities/functions/function.h
  include/utilities/functions/functions.h
  include/utilities/functions/linear_function.h
  include/utilities/functions/probability_density_function.h
  include/utilities/functions/pulse_function.h
  include/utilities/functions/sample_holder.h
  include/utilities/functions/step_function.h
  include/utilities/functions/unary_sample_holder.h
  include/utilities/functions/value_change_event.h
  include/utilities/has_id.h
  include/utilities/has_name.h
  include/utilities/interval.h
  include/utilities/motivation_plot_generator.h
  include/utilities/noisy.h
  include/utilities/noisy_duration.h
  include/utilities/noisy_time.h
  include/utilities/observer.h
  include/utilities/ros_message_converter.h
  include/utilities/ros_node.h
  include/utilities/ros_service_server.h
  include/utilities/signal_type.h
  include/utilities/signal_types.h
  include/utilities/simulation.h
  include/utilities/subject.h
  include/utilities/toggle_event.h
  include/utilities/unary_signal_type.h
  include/utilities/utilities.h
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
  src/nodes/motivation_publishers.cpp
  src/utilities/continuous_noisy_signal.cpp
  src/utilities/continuous_signal_type.cpp
  src/utilities/discrete_noisy_signal.cpp
  src/utilities/discrete_signal_type.cpp
  src/utilities/exception.cpp
  src/utilities/event.cpp
  src/utilities/functions/probability_density_function.cpp
  src/utilities/functions/unary_sample_holder.cpp
  src/utilities/has_name.cpp
  src/utilities/motivation_plot_generator.cpp
  src/utilities/noisy_duration.cpp
  src/utilities/noisy_time.cpp
  src/utilities/observer.cpp
  src/utilities/ros_node.cpp
  src/utilities/signal_types.cpp
  src/utilities/simulation.cpp
  src/utilities/subject.cpp
  src/utilities/toggle_event.cpp
  src/utilities/unary_signal_type.cpp
)

add_library(${PROJECT_NAME}
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(high_level
  src/high_level.cpp
  ${CPP_FILES}
)

add_dependencies(high_level
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(high_level
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(low_level
  src/low_level.cpp
  ${CPP_FILES}
)

add_dependencies(low_level
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(low_level
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(${PROJECT_NAME}_utest
  test/utest.cpp
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}_utest
  ${Boost_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_utest
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
