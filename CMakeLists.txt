cmake_minimum_required(VERSION 3.8)
project(sensing_utils)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include headers from include/
include_directories(include)


# # In order to add the executable that can be used by ros2
# add_executable(pointcloud_handler_node
#   src/pointcloud_handler.cpp
# )
# ament_target_dependencies(pointcloud_handler_node
#   rclcpp std_msgs std_srvs sensor_msgs
#   pcl_conversions pcl_ros PCL
# )

# In order to add the library that can be used by ros2
set(library_name sensing_utils_lib)
add_library(${library_name} SHARED
  src/sensor_checker.cpp
)
ament_target_dependencies(sensing_utils_lib
  rclcpp
  sensor_msgs
)

# # In order to let ros2 find the executable
# install(TARGETS 
#   pointcloud_handler_node 
# 	DESTINATION lib/${PROJECT_NAME}
# )

# In order to let ros2 find the library
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# In order to let ros2 find the folders
install( 
  DIRECTORY include src
  DESTINATION share/${PROJECT_NAME}
)


# In order to link some needed libraries into this package
# target_link_libraries(pointcloud_handler_node
#   ${PCL_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
# )

# target_link_libraries(${library_name}
#   ${PCL_LIBRARIES}
# )


# In order to let the library be visible from other packages
install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()





# # Build the library from the .cpp file
# add_library(sensing_utils_lib
#   src/sensor_quick_check.cpp
# )

# ament_target_dependencies(sensing_utils_lib
#   rclcpp
#   sensor_msgs
# )

# target_include_directories(sensing_utils_lib PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# # Install library and headers
# install(TARGETS sensing_utils_lib
#   EXPORT sensing_utils_export
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# # Export targets and dependencies for downstream packages
# ament_export_targets(sensing_utils_export HAS_LIBRARY_TARGET)
# ament_export_include_directories(include)
# ament_export_dependencies(rclcpp sensor_msgs)

# ament_package()



