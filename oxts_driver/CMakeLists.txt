cmake_minimum_required(VERSION 3.8)
project(oxts_driver VERSION 0.1.0)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(Boost REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)

add_library(oxts_driver SHARED
               src/lib/NComRxC.c 
               src/lib/nav_conversions.cpp
               src/lib/parser_utils.cpp
               src/driver/driver.cpp)

ament_target_dependencies(oxts_driver 
                          rclcpp
                          std_msgs
                          sensor_msgs
                          geometry_msgs
                          nav_msgs
                          tf2
                          tf2_kdl)

target_include_directories(oxts_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PRIVATE ${Boost_INCLUDE_DIRS}
  $<INSTALL_INTERFACE:include>)

target_link_libraries(oxts_driver
  Boost::boost
  ${Boost_LIBRARIES}
)

install(TARGETS oxts_driver
        DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
