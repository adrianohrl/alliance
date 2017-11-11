cmake_minimum_required(VERSION 2.8.3)

project(alliance_test)

find_package(catkin REQUIRED)

find_package(alliance REQUIRED COMPONENTS
  alliance_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    alliance
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    alliance
    alliance_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${alliance_INCLUDE_DIRS}
)

set(H_FILES
  include/alliance_test/layer.h
  include/alliance_test/border_protection.h
  include/alliance_test/odometry.h
  include/alliance_test/point_cloud.h
  include/alliance_test/report.h
  include/alliance_test/updated_sensory.h
  include/alliance_test/wander.h
)

set(CPP_FILES
  src/alliance_test/layer.cpp
  src/alliance_test/border_protection.cpp
  src/alliance_test/odometry.cpp
  src/alliance_test/point_cloud.cpp
  src/alliance_test/report.cpp
  src/alliance_test/updated_sensory.cpp
  src/alliance_test/wander.cpp
)

add_library(${PROJECT_NAME}
  ${CPP_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${alliance_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${alliance_LIBRARIES}
  ${catkin_LIBRARIES}
)
