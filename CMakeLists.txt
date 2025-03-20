cmake_minimum_required(VERSION 3.8)
project(max_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include(FindPkgConfig)
pkg_check_modules(CURLPP REQUIRED curlpp)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PathRequest.srv"
)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(apriltag_localization src/apriltag_localization.cpp)
ament_target_dependencies(apriltag_localization apriltag_msgs rclcpp std_msgs tf2 tf2_msgs tf2_geometry_msgs)
install(TARGETS
    apriltag_localization
  DESTINATION lib/${PROJECT_NAME})

add_executable(motor_control src/motor_control.cpp)
ament_target_dependencies(motor_control rclcpp std_msgs)
target_link_libraries(motor_control "${cpp_typesupport_target}" -lwiringPi jsoncpp_lib ${CURLPP_LDFLAGS})
install(TARGETS
    motor_control
  DESTINATION lib/${PROJECT_NAME})

add_executable(object_detection src/object_detection.cpp)
ament_target_dependencies(object_detection rclcpp std_msgs)
target_link_libraries(object_detection ${catkin_LIBRARIES} -lwiringPi)
install(TARGETS
    object_detection
  DESTINATION lib/${PROJECT_NAME})

add_executable(path_planning src/path_planning.cpp)
ament_target_dependencies(path_planning rclcpp)
target_link_libraries(path_planning "${cpp_typesupport_target}")
install(TARGETS
    path_planning
  DESTINATION lib/${PROJECT_NAME})

add_executable(webcam_reader src/webcam_reader.cpp)
ament_target_dependencies(webcam_reader cv_bridge image_transport OpenCV rclcpp std_msgs)
install(TARGETS
    webcam_reader
  DESTINATION lib/${PROJECT_NAME})

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
