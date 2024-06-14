cmake_minimum_required(VERSION 3.8)
project(cnr_tf_named_object_loader)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-funroll-loops -Wall -Ofast)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS Core)

add_library(${PROJECT_NAME} SHARED
   src/${PROJECT_NAME}/cnr_tf_named_object_loader.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen yaml-cpp)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp moveit_msgs moveit_ros_planning_interface tf2_ros Eigen3)

add_executable(${PROJECT_NAME}_node src/cnr_tf_named_object_loader_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PROJECT_NAME} yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_node PUBLIC rclcpp moveit_msgs moveit_ros_planning_interface tf2_ros Eigen3)

#############
## Install ##
#############

# install the library
install(TARGETS
        ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include #gives the information, actual install happens below
    )

install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
    )

#install the headers
install(
      DIRECTORY include/
      DESTINATION include
    )
#install other folders
install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET) # exports the targets for CMake, to allow your library’s clients to use the target_link_libraries(client my_library::my_library) syntax.
ament_export_dependencies(rclcpp moveit_msgs moveit_ros_planning_interface tf2_ros Eigen3 yaml-cpp) # exports dependencies to downstream packages to avoid the the user to call find_package for those dependencies too.

ament_package()
