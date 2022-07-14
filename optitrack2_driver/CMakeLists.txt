cmake_minimum_required(VERSION 3.5)

project(optitrack2_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
set(NATNET_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/NatNetSDK/) 
set(LIBNATNET_SDK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/NatNetSDK/lib/libNatNet.so)
set(natnet_include_directories
  ${NATNET_SDK_DIR}/include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(device_control_msgs REQUIRED)
find_package(device_control REQUIRED)

#find_package(NATNETSDK_CPP REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  mocap_msgs
  device_control
  device_control_msgs
)

include_directories(
  include
  ${natnet_include_directories}
)

add_library(
  ${PROJECT_NAME}
src/optitrack2_driver.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "OPTITRACK_BUILDING_LIBRARY")

add_executable(optitrack2_driver_main
  src/optitrack2_driver_main.cpp
)
ament_target_dependencies(optitrack2_driver_main ${dependencies})
target_link_libraries(optitrack2_driver_main
  ${PROJECT_NAME}
  ${LIBNATNET_SDK_LIBRARY}
)

ament_target_dependencies(optitrack2_driver ${dependencies})
target_link_libraries(optitrack2_driver ${LIBNATNET_SDK_LIBRARY})
#
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)
#
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  optitrack2_driver_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


#if(BUILD_TESTING)
#
#  find_package(ament_cmake_gtest REQUIRED)
#  find_package(ament_lint_auto REQUIRED)
#  find_package(ament_index_cpp REQUIRED)
#
#  ament_lint_auto_find_test_dependencies()
#
#  ament_add_gtest(test_optitrack2_driver test/test_optitrack2_driver.cpp)
#  target_link_libraries(test_optitrack2_driver ${PROJECT_NAME})
#
#endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
