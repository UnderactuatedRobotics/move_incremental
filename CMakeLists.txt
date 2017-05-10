cmake_minimum_required(VERSION 2.8.3)
project(move_incremental)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        costmap_2d
        geometry_msgs
        nav_core
        nav_msgs
        pcl_conversions
        pcl_ros
        pluginlib
        roscpp
        tf
        visualization_msgs
        message_generation
	)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})


# services
add_service_files(
    FILES
        MakeNavPlan.srv
        SetCostmap.srv
)

generate_messages(
    DEPENDENCIES
        std_msgs
        geometry_msgs
        nav_msgs
)

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        move_incremental
    CATKIN_DEPENDS
        nav_core
        roscpp
        pluginlib
)

add_library (move_incremental src/move_incremental.cpp src/move_incremental_ros.cpp include/move_incremental/move_incremental_ros.h include/move_incremental/move_incremental.h include/move_incremental/potarr_point.h src/move_incremental.cpp src/move_incremental_ros.cpp)
target_link_libraries(move_incremental
    ${catkin_LIBRARIES}
    )
add_dependencies(move_incremental ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

install(TARGETS move_incremental
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/move_incremental/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES move_incremental_plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
