message(" ")

message("CMAKE_PREFIX_PATH:  ${CMAKE_PREFIX_PATH}")
# ROS
find_package(catkin REQUIRED COMPONENTS
        #        cv_bridge
        image_transport
        roscpp
        sensor_msgs
        nav_msgs
        std_msgs
        tf2
        tf2_ros
)
catkin_package(
#        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp sensor_msgs
)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/../..
)
MESSAGE("Saiga PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
MESSAGE("Saiga CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

if(MODULE_CORE)
    add_subdirectory(core)
    message(STATUS "Module enabled:     Core")
endif()

if(MODULE_OPENGL)
    message(STATUS "Module enabled:     Opengl")
    add_subdirectory(opengl)
endif()

if(MODULE_VULKAN)
    message(STATUS "Module enabled:     Vulkan")
    add_subdirectory(vulkan)
endif()

if(MODULE_VISION)
    message(STATUS "Module enabled:     Vision")
    add_subdirectory(vision)
endif()

if(MODULE_CUDA)
    message(STATUS "Module enabled:     Cuda")
    add_subdirectory(cuda)
endif()


install(
    EXPORT ${LIB_EXPORT}
    FILE SaigaTargets.cmake
    NAMESPACE Saiga::
    DESTINATION ${INSTALL_SHARE}

    )
