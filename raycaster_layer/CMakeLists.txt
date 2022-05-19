cmake_minimum_required(VERSION 2.8.3)
project(raycaster_layer)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
    angles
    costmap_2d
    dynamic_reconfigure
    geometry_msgs
    pluginlib
    roscpp
    rospy
    sensor_msgs
    tf2_geometry_msgs


    grid_map_core
    grid_map_ros
    grid_map_cv
    grid_map_filters
    grid_map_loader
    grid_map_msgs
    grid_map_octomap
    grid_map_rviz_plugin
    grid_map_visualization
    costmap_2d
    grid_map_costmap_2d

    geometry_msgs
    sensor_msgs
    cv_bridge
    octomap_msgs
    filters
)

# generate_dynamic_reconfigure_options(cfg/RangeSensorLayer.cfg)

catkin_package(
    CATKIN_DEPENDS
        angles
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        pluginlib
        roscpp
        rospy
        sensor_msgs
        tf2_geometry_msgs
    INCLUDE_DIRS inc
    LIBRARIES ${PROJECT_NAME}
)

include_directories(inc ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/raycaster_layer.cpp)
add_dependencies(${PROJECT_NAME} ${range_sensor_layer_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES costmap_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
# catkin_install_python(PROGRAMS scripts/send_message.py
#                       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
