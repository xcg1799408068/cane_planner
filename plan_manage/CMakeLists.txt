cmake_minimum_required(VERSION 3.0.2)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 1. 查找依赖
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# 修复了这里的不可见字符，确保每一行都是标准干净的文本
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
  onboard_detector
  geometry_msgs
  gazebo_msgs
  plan_env
  path_searching
  bspline
  bspline_opt
)

# 2. 传递 PCL 的编译器宏定义（可选，防患未然）
add_definitions(${PCL_DEFINITIONS})

# 3. Catkin 包配置
# 移除了不存在的 LIBRARIES 导出，修复了 CATIKIN_DEPENDS 拼写错误
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs tf onboard_detector geometry_msgs gazebo_msgs plan_env path_searching bspline bspline_opt
)

# 4. 环境变量钩子
catkin_add_env_hooks(99.disable_ros1_eol_warnings
  SHELLS bash
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

###########
## Build ##
###########

# 头文件路径包含
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 定义可执行文件
add_executable(kin_replan_node
  src/kin_replan_node.cpp
  src/planner_manager.cpp
)

# 依赖确保（消息生成顺序）
add_dependencies(kin_replan_node 
  ${catkin_EXPORTED_TARGETS} 
)

# 目标链接库
# 补全了 OpenCV 和 Eigen3 的库链接，防止链接期报错
target_link_libraries(kin_replan_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_sim_odom_stop
    test/test_sim_odom_stop.cpp
    src/planner_manager.cpp
  )
  if(TARGET test_sim_odom_stop)
    add_dependencies(test_sim_odom_stop ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_sim_odom_stop
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OpenCV_LIBRARIES}
      Eigen3::Eigen
    )
  endif()
endif()

#############
## Install ##
#############

# 安装 资源/配置 文件夹
install(DIRECTORY launch urdf worlds scripts config maps models plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "*.py" EXCLUDE
)

# 安装 Python 脚本并赋予执行权限
catkin_install_python(PROGRAMS
  scripts/pedestrian_sim.py
  scripts/cmd_vel_to_state.py
  scripts/imu_from_state.py
  scripts/steering_bridge.py
  scripts/load_steering_controllers.py
  scripts/registered_map_accumulator.py
  scripts/model_pose_guard.py
  scripts/gazebo_pedestrian_truth_bridge.py
  scripts/mpc_event_monitor.py
  scripts/record_mpc_eval.sh
  scripts/record_lightweight_mpc_eval.sh
  scripts/analyze_mpc_eval.py
  scripts/summarize_mpc_eval.py
  scripts/publish_fixed_goal.py
  scripts/publish_lightweight_fixed_goal.py
  scripts/run_lightweight_mppi_compare.py
  scripts/global_path_corridor_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
