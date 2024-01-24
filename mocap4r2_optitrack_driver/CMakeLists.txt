cmake_minimum_required(VERSION 3.5)

project(mocap4r2_optitrack_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(LIBNATNET_SDK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/NatNetSDK/lib/libNatNet.so)
set(NATNET_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/NatNetSDK/include
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_control_msgs REQUIRED)
find_package(mocap4r2_control REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  mocap4r2_msgs
  mocap4r2_control
  mocap4r2_control_msgs
)

include_directories(
  include
  ${NATNET_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/mocap4r2_optitrack_driver/mocap4r2_optitrack_driver.cpp)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "OPTITRACK_BUILDING_LIBRARY")

add_executable(mocap4r2_optitrack_driver_main
  src/mocap4r2_optitrack_driver_main.cpp
)
ament_target_dependencies(mocap4r2_optitrack_driver_main ${dependencies})
target_link_libraries(mocap4r2_optitrack_driver_main
  ${PROJECT_NAME}
  ${LIBNATNET_SDK_LIBRARY}
)

ament_target_dependencies(mocap4r2_optitrack_driver ${dependencies})
target_link_libraries(mocap4r2_optitrack_driver ${LIBNATNET_SDK_LIBRARY})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  mocap4r2_optitrack_driver_main
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)


  file(GLOB_RECURSE EXCLUDE_FILES NatNetSDK/*)

  ament_copyright(EXCLUDE ${EXCLUDE_FILES})
  ament_cppcheck(
    EXCLUDE ${EXCLUDE_FILES}
    LANGUAGE c++
  )
  ament_cpplint(EXCLUDE ${EXCLUDE_FILES})
  ament_lint_cmake()
  ament_uncrustify(
    EXCLUDE ${EXCLUDE_FILES}
    LANGUAGE c++
  )

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
