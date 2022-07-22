cmake_minimum_required(VERSION 3.7)
project(coppelia_ros_contact)

set(PKG_DEPS
    roscpp
    gazebo_ros
    tactile_msgs
    cmake_modules)


find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

find_package(TinyXML REQUIRED)
find_package(TinyXML2 REQUIRED)

find_package(Eigen REQUIRED)

if(NOT LIBPLUGIN_DIR)
    if(DEFINED ENV{COPPELIASIM_ROOT_DIR})
        set(LIBPLUGIN_DIR $ENV{COPPELIASIM_ROOT_DIR}/programming/libPlugin)
    else()
        message(FATAL_ERROR "Environment variable COPPELIASIM_ROOT_DIR is not set")
    endif()
endif()

list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${LIBPLUGIN_DIR}/cmake)
find_package(CoppeliaSim 4.1.0.0 REQUIRED)
catkin_package(CATKIN_DEPENDS ${PKG_DEPS})


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
  ${TinyXML2_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
    )


coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/meta/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simROSContact.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/config.h)

coppeliasim_add_plugin(simExtROSContact SOURCES src/coppelia_ros_contact.cpp)
add_dependencies(simExtROSContact ${gazebo_msgs_EXPORTED_TARGETS})
target_link_libraries(simExtROSContact ${catkin_LIBRARIES}  ${TinyXML_LIBRARIES} ${TinyXML2_LIBRARIES} )
