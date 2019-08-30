cmake_minimum_required(VERSION 2.8.3)
project(app_move_pick)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vision_bridge
  pick_place_bridge
  hirop_msgs
  tf
  moveit_core
  moveit_ros_planning_interface
)

find_package(hirop_datamanager REQUIRED)
#find_package(hirop_nav REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp vision_bridge pick_place_bridge hirop_msgs tf
 moveit_core moveit_ros_planning_interface
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp src/auto_move_pick.cpp)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
 ${hirop_datamanager_LIBRARIES}
 hirop_nav
 ${catkin_LIBRARIES}
)

