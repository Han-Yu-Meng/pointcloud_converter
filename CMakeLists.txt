cmake_minimum_required(VERSION 3.16)
project(pointcloud_converter)

find_package(PCL QUIET)
find_package(livox_ros_driver2 REQUIRED)

fins_add_node(${PROJECT_NAME} SHARED nodes.cpp)

fins_link_ros_dependencies(${PROJECT_NAME} sensor_msgs nav_msgs pcl_conversions)

target_compile_definitions(${PROJECT_NAME} PRIVATE BOOST_ALLOW_DEPRECATED_HEADERS)
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${PCL_LIBRARIES} 
    pcl_surface
    livox_ros_driver2::livox_interfaces2__rosidl_generator_cpp
    livox_ros_driver2::livox_interfaces2__rosidl_typesupport_cpp
)