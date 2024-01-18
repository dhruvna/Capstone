cmake_minimum_required(VERSION 3.8)
project(image_stream)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Declare a C++ executable for the image publisher
add_executable(image_publisher src/image_publisher.cpp)
ament_target_dependencies(image_publisher rclcpp sensor_msgs OpenCV cv_bridge)

# Declare a C++ executable for the image subscriber
add_executable(image_subscriber src/image_subscriber.cpp)
ament_target_dependencies(image_subscriber rclcpp sensor_msgs OpenCV cv_bridge)

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

# Install executables and resources
install(TARGETS
  image_publisher
  image_subscriber
  DESTINATION lib/${image_stream}
)

install(DIRECTORY resource/
  DESTINATION share/${image_stream}/resource
)

# Export dependencies
ament_package()
