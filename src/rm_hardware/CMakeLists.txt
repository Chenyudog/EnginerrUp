cmake_minimum_required(VERSION 3.8)
project(rm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rmctrl_msgs REQUIRED)


include_directories(include)

add_executable(test_control src/test_control.cpp)
ament_target_dependencies(test_control
  rclcpp          
  rmctrl_msgs     
)


add_library(
  ${PROJECT_NAME}
  SHARED
  src/engineer_hardware_interface.cpp
)


ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  rclcpp_lifecycle 
  hardware_interface 
  pluginlib 
  rmctrl_msgs 

)


pluginlib_export_plugin_description_file(hardware_interface engieer_hardware_interface.xml)


install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  TARGETS test_control
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)


ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rmctrl_msgs
)

ament_package()
