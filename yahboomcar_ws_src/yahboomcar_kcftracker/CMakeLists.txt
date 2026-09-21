cmake_minimum_required(VERSION 3.5)
project(yahboomcar_kcftracker)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED )
find_package(cv_bridge REQUIRED )


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_libraries("/lib/libopencv_world.so.3.4")
include_directories(include/yahboomcar_kcftracker)
if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# uncomment the line when a copyright and license is not present in all source files
#set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  ament_lint_auto_find_test_dependencies()
endif()
  
add_executable( KCF_Tracker_Node src/KCF_Tracker.cpp include/yahboomcar_kcftracker/kcftracker.cpp include/yahboomcar_kcftracker/fhog.cpp include/yahboomcar_kcftracker/PID.cpp)
ament_target_dependencies(KCF_Tracker_Node  rclcpp sensor_msgs geometry_msgs std_msgs cv_bridge)
  
install(
        TARGETS
        KCF_Tracker_Node  
        DESTINATION lib/${PROJECT_NAME}
        )
        
install(
	DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}
	)      
        
ament_package()
