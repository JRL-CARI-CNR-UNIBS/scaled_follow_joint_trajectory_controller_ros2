cmake_minimum_required(VERSION 3.8)
project(scaled_follow_joint_trajectory_controller_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(joint_trajectory_controller  REQUIRED)
find_package(controller_interface         REQUIRED)
find_package(hardware_interface           REQUIRED)
find_package(pluginlib                    REQUIRED)
find_package(rclcpp                       REQUIRED)
find_package(rclcpp_lifecycle             REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  joint_trajectory_controller
  controller_interface  
  hardware_interface  
  pluginlib            
  rclcpp            
  rclcpp_lifecycle  
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/scaled_fjt_controller.cpp
  src/microinterpolator.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

pluginlib_export_plugin_description_file(controller_interface scaled_fjt_controller.xml)


ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

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
## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION include/scaled_fjt_controller
FILES_MATCHING PATTERN "*.h*"
PATTERN ".svn" EXCLUDE)

ament_export_include_directories(
        include
)

ament_export_libraries(
        ${PROJECT_NAME}
)

ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)

ament_package()
