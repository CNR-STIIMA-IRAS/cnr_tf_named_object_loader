cmake_minimum_required(VERSION 3.8)
project(cnr_tf_named_object_loader)

# #################################
# Preamble
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-funroll-loops -Wall -Ofast)
endif()

### 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED COMPONENTS Core)
### 


add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/cnr_tf_named_object_loader.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${YAML_CPP_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp moveit_msgs moveit_ros_planning_interface tf2_ros)

add_executable(${PROJECT_NAME}_node src/cnr_tf_named_object_loader_node.cpp)
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PROJECT_NAME} Eigen3::Eigen ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_node PUBLIC rclcpp moveit_msgs moveit_ros_planning_interface tf2_ros)

#############
## Install ##
#############

install(TARGETS
        ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
    )

install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cnr_tf_named_object_loader.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

ament_package()
