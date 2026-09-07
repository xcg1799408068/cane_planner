cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

find_package(Eigen3 REQUIRED)
find_library(NLOPT_LIBRARY NAMES nlopt REQUIRED)
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
    src/corridor_failure_snapshot.cpp
    )
target_link_libraries(convex_corridor
    ${NLOPT_LIBRARY}
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

add_executable(convex_corridor_replay src/convex_corridor_replay.cpp)
target_link_libraries(convex_corridor_replay convex_corridor)

if(CATKIN_ENABLE_TESTING)
  # White-box target includes convex_corridor.cpp; do not link a second copy.
  catkin_add_gtest(test_mvie_roundoff test/test_mvie_roundoff.cpp src/corridor_failure_snapshot.cpp)
  target_link_libraries(test_mvie_roundoff ${NLOPT_LIBRARY} ${catkin_LIBRARIES})
  catkin_add_gtest(test_corridor_failure_snapshot test/test_corridor_failure_snapshot.cpp)
  target_link_libraries(test_corridor_failure_snapshot convex_corridor)
  catkin_add_gtest(test_astar_edges test/test_astar_edges.cpp)
  target_compile_definitions(test_astar_edges PRIVATE ASTAR_EDGE_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/corridor_failures/")
  target_link_libraries(test_astar_edges path_searching ${catkin_LIBRARIES})
  catkin_add_gtest(test_static_mpc test/test_static_mpc.cpp)
  target_link_libraries(test_static_mpc path_searching ${catkin_LIBRARIES})
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
    path_searching
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
