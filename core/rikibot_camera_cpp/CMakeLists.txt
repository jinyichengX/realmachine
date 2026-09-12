cmake_minimum_required(VERSION 3.8)
project(rikibot_camera_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(ASTRA_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk)

# 可执行程序: 相机 ROS2 节点
# (三层架构: main 入口 / CameraNode 组合驱动做 ROS 胶水 / VideoStreamDriver 只取流)
add_executable(rikibot_camera_node
  src/camera_node.cpp
  src/astra_camera_stream/astra_stream.cpp)
target_include_directories(rikibot_camera_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ASTRA_SDK_DIR}/include)
target_compile_features(rikibot_camera_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(rikibot_camera_node
  ${ASTRA_SDK_DIR}/lib/libastra.so
  ${ASTRA_SDK_DIR}/lib/libastra_core.so
  ${ASTRA_SDK_DIR}/lib/libastra_core_api.so)

set_target_properties(rikibot_camera_node PROPERTIES
  INSTALL_RPATH "$ORIGIN")
ament_target_dependencies(rikibot_camera_node
  "rclcpp"
  "sensor_msgs"
  "image_transport"
)

install(TARGETS rikibot_camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY sdk/lib/
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

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
