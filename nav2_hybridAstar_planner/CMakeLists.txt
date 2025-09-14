cmake_minimum_required(VERSION 3.8)
project(nav2_hybrid_astar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 找到所有依赖（补充缺失的依赖）
find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)  # 代码中使用 rclcpp::Node
find_package(tf2 REQUIRED)      # 代码中使用 tf2::getYaw
find_package(tf2_ros REQUIRED)  # 代码中使用 tf2_ros::Buffer
find_package(geometry_msgs REQUIRED)  # 代码中使用 PoseStamped
find_package(nav2_costmap_2d REQUIRED)  # 代码中使用 Costmap2DROS
find_package(nav_msgs REQUIRED)  # 代码中使用 Path

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 2. 包含头文件目录
include_directories(include)

# 3. 定义动态库名称（与 XML 中 library path 一致）
set(library_name ${PROJECT_NAME}_plugin)  # 结果：nav2_hybrid_astar_planner_plugin

# 4. 创建共享库（链接所有依赖）
add_library(${library_name} SHARED  
  src/nav2_hybridAstar_planner.cpp
)

# 5. 指定库的依赖关系（补充所有缺失的依赖）
ament_target_dependencies(${library_name}
  nav2_core
  pluginlib
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  nav2_costmap_2d
  nav_msgs
)

# 6. 安装库文件和头文件（无需修改）
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/ 
)

# 7. 导出插件描述文件（无需修改，确保 XML 文件名正确）
pluginlib_export_plugin_description_file(nav2_core hybridAstar_planner_plugin.xml)

ament_package()
