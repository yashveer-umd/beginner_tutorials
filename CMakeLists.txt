cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



ament_export_dependencies(rosidl_default_runtime)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeString.srv"
)

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker 
                          rclcpp
                          std_msgs
                          geometry_msgs
                          tf2
                          tf2_ros
                       )

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# add_executable(server src/server.cpp)
# ament_target_dependencies(server rclcpp)

add_executable(server_client src/server_client.cpp)
ament_target_dependencies(server_client rclcpp)

install(TARGETS talker listener server_client
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

## This finds the relevant generated C++ code from ChangeString.srv 
# and allows your target to link against it.
# You may have noticed that this step was not necessary when the
# interfaces being used were from a package that was built separately. 
# ref : https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html

rosidl_target_interfaces(talker
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(server_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


if(BUILD_TESTING)

  set (THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rosidl_default_generators
  std_msgs
  )
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # the following lines skip linters
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()

  # Add a gtest executable
  set(TESTFILES
    test/basic_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  rosidl_target_interfaces(${PROJECT_NAME}_test
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  install(TARGETS ${PROJECT_NAME}_test
        DESTINATION lib/${PROJECT_NAME})


endif()

ament_package()
