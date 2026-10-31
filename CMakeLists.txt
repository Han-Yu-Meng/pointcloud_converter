cmake_minimum_required(VERSION 3.16)
project(pointcloud_converter)

find_package(PCL QUIET)
find_package(livox_ros_driver2 REQUIRED)

fins_add_node(${PROJECT_NAME} SHARED nodes.cpp)

if(TARGET livox_ros_driver2::livox_interfaces2__rosidl_generator_cpp)
    set(LIVOX_MSG_PREFIX livox_interfaces2)
    target_compile_definitions(${PROJECT_NAME} PRIVATE LIVOX_INTERFACES2)
else()
    set(LIVOX_MSG_PREFIX livox_ros_driver2)
endif()

fins_link_ros_dependencies(${PROJECT_NAME} sensor_msgs nav_msgs pcl_conversions)

target_compile_definitions(${PROJECT_NAME} PRIVATE BOOST_ALLOW_DEPRECATED_HEADERS)
target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${PCL_LIBRARIES} 
    pcl_surface
    livox_ros_driver2::${LIVOX_MSG_PREFIX}__rosidl_generator_cpp
    livox_ros_driver2::${LIVOX_MSG_PREFIX}__rosidl_typesupport_cpp
)