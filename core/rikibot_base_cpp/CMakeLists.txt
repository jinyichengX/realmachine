cmake_minimum_required(VERSION 3.8)
project(rikibot_base_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 导出编译数据库, 供 clangd 等 IDE 使用 (每次构建自动更新)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 可执行程序: base_node (协议/odom 测试主程序, 依赖 raw_serial.hpp)
# 注: src/rikibot_base_node.cpp 已废弃, 不再编译; src/raw_serial.cpp 已移除
add_executable(base_node src/base_node.cpp)
target_include_directories(base_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>)
target_compile_features(base_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  base_node
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "tf2_ros"
)

install(TARGETS base_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# udev 规则安装脚本 (把底盘 CH340 固定为 /dev/rikibase)
install(PROGRAMS scripts/create_udev_rules.sh
  DESTINATION share/${PROJECT_NAME}/scripts)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
