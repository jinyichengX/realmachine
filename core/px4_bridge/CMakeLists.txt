cmake_minimum_required(VERSION 3.8)
project(px4_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 导出编译数据库, 供 clangd 等 IDE 使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 上行: PX4 -> ROS 2 (本地位置/姿态 -> odom + TF)
add_executable(px4_odom_bridge src/px4_odom_bridge.cpp)
target_compile_features(px4_odom_bridge PUBLIC cxx_std_17)
ament_target_dependencies(px4_odom_bridge
  "rclcpp"
  "px4_msgs"
  "nav_msgs"
  "geometry_msgs"
  "tf2_ros"
)

# 下行: ROS 2 -> PX4 (Offboard 心跳/setpoint/模式与解锁指令)
add_executable(px4_offboard_control src/px4_offboard_control.cpp)
target_compile_features(px4_offboard_control PUBLIC cxx_std_17)
ament_target_dependencies(px4_offboard_control
  "rclcpp"
  "px4_msgs"
  "geometry_msgs"
)

# 台架测试: 伪造外部视觉位置, 让 EKF2 获得水平位置估计 (需显式确认, 仅限拆桨/系留)
add_executable(px4_fake_ev src/px4_fake_ev.cpp)
target_compile_features(px4_fake_ev PUBLIC cxx_std_17)
ament_target_dependencies(px4_fake_ev
  "rclcpp"
  "px4_msgs"
)

# 独立工具: 订阅 /fmu/out/vehicle_attitude, 打印欧拉角
add_executable(px4_attitude_print src/px4_attitude_print.cpp)
target_compile_features(px4_attitude_print PUBLIC cxx_std_17)
ament_target_dependencies(px4_attitude_print
  "rclcpp"
  "px4_msgs"
)

# EV 桥: LIO odom -> PX4 vehicle_visual_odometry
add_executable(px4_ev_bridge src/px4_ev_bridge.cpp)
target_compile_features(px4_ev_bridge PUBLIC cxx_std_17)
ament_target_dependencies(px4_ev_bridge
  "rclcpp"
  "px4_msgs"
  "nav_msgs"
)

install(TARGETS
  px4_odom_bridge
  px4_offboard_control
  px4_fake_ev
  px4_attitude_print
  px4_ev_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
