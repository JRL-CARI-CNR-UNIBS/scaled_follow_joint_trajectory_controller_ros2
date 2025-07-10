cmake_minimum_required(VERSION 3.16)
project(scaled_follow_joint_trajectory_controller_ros2)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  joint_trajectory_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(${PROJECT_NAME} SHARED
  src/scaled_fjt_controller.cpp
  src/microinterpolator.cpp
)
target_compile_features((${PROJECT_NAME}  PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  joint_trajectory_controller::joint_trajectory_controller
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(controller_interface scaled_fjt_controller.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_trajectory test/test_trajectory.cpp)
  target_link_libraries(test_trajectory joint_trajectory_controller)
  target_link_libraries(test_trajectory ros2_control_test_assets::ros2_control_test_assets)
  target_compile_definitions(test_trajectory PRIVATE _USE_MATH_DEFINES)

  ament_add_gmock(test_tolerances test/test_tolerances.cpp)
  target_link_libraries(test_tolerances joint_trajectory_controller)
  target_link_libraries(test_tolerances ros2_control_test_assets::ros2_control_test_assets)
  target_compile_definitions(test_tolerances PRIVATE _USE_MATH_DEFINES)

  ament_add_gmock(test_trajectory_controller
    test/test_trajectory_controller.cpp)
  set_tests_properties(test_trajectory_controller PROPERTIES TIMEOUT 220)
  target_link_libraries(test_trajectory_controller
    joint_trajectory_controller
  )
  target_link_libraries(test_trajectory_controller ros2_control_test_assets::ros2_control_test_assets)
  target_compile_definitions(joint_trajectory_controller PRIVATE _USE_MATH_DEFINES)

  add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_add_gmock(test_load_joint_trajectory_controller
    test/test_load_joint_trajectory_controller.cpp
  )
  target_link_libraries(test_load_joint_trajectory_controller
    joint_trajectory_controller
    controller_manager::controller_manager
    ros2_control_test_assets::ros2_control_test_assets
  )

  ament_add_gmock(test_trajectory_actions
    test/test_trajectory_actions.cpp
  )
  set_tests_properties(test_trajectory_actions PROPERTIES TIMEOUT 300)
  target_link_libraries(test_trajectory_actions
    joint_trajectory_controller
    ros2_control_test_assets::ros2_control_test_assets)
endif()


install(
  DIRECTORY include/
  DESTINATION include/scaled_fjt_controller
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}))
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}

ament_package()
