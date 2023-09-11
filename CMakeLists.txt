cmake_minimum_required(VERSION 3.8)
project(anna_image_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    image_transport
    cv_bridge
    sensor_msgs
    std_msgs
    geometry_msgs  
    OpenCV
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

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

add_executable(line_image_processing src/line_image_processing.cpp)
ament_target_dependencies(line_image_processing rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)
 
install(TARGETS
line_image_processing
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
