cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  plan_ctrl
  onboard_detector 
  nav_msgs
  Eigen3
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching convex_corridor
 CATKIN_DEPENDS plan_env plan_ctrl
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library(convex_corridor
    src/convex_corridor.cpp
    )
target_link_libraries(convex_corridor
    ${catkin_LIBRARIES}
    )


add_library( path_searching
    src/astar.cpp
    src/kinodynamic_astar.cpp
    src/lfpc.cpp
    src/dynamic_risk_field.cpp
    src/dynamic_walking_corridor.cpp
    src/timed_trajectory_builder.cpp
    src/path_smoother.cpp
    src/mpc_controller.cpp
    src/kinematic_mppi_controller.cpp
    )
target_link_libraries( path_searching
    convex_corridor
    ${catkin_LIBRARIES} 
    )  

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_kinematic_mppi_controller
    test/test_kinematic_mppi_controller.cpp
  )
  target_link_libraries(test_kinematic_mppi_controller
    path_searching
    ${catkin_LIBRARIES}
  )
  catkin_add_gtest(test_dynamic_walking_corridor
    test/test_dynamic_walking_corridor.cpp
  )
  target_link_libraries(test_dynamic_walking_corridor
    path_searching
    ${catkin_LIBRARIES}
  )
  catkin_add_gtest(test_convex_corridor
    test/test_convex_corridor.cpp
  )
  target_link_libraries(test_convex_corridor
    convex_corridor
    ${catkin_LIBRARIES}
  )
  catkin_add_gtest(test_timed_trajectory
    test/test_timed_trajectory.cpp
  )
  target_link_libraries(test_timed_trajectory
    path_searching
    ${catkin_LIBRARIES}
  )
  catkin_add_gtest(test_timed_walking_corridor
    test/test_timed_walking_corridor.cpp
  )
  target_link_libraries(test_timed_walking_corridor
    path_searching
    ${catkin_LIBRARIES}
  )
  catkin_add_gtest(test_path_smoother
    test/test_path_smoother.cpp
  )
  target_link_libraries(test_path_smoother
    path_searching
    ${catkin_LIBRARIES}
  )
endif()
