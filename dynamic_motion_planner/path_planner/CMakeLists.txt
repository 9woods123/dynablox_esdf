

cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()



find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
    octomap_msgs
    interactive_markers
    
    )

    set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar/algorithm.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar/node4d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar/node3d.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar/collisiondetection.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar/path.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar/smoother.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar/visualize.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hybrid_astar/feasibleOpt.cpp

    )

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/algorithm.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/node4d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/node3d.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/collisiondetection.h
    # ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/planner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/path.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/smoother.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/visualize.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/helper.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/constants.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hybrid_astar/feasibleOpt.h

    )


## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

find_package(octomap_server REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
#cs_add_executable(my_node ${SOURCES})

# 添加库文件


if(NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif(NOT OMPL_FOUND)

    
cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

target_link_libraries(${PROJECT_NAME}
 ${catkin_LIBRARIES} 
${OMPL_LIBRARIES}  
${EIGEN3_LIBRARIES}
${OCTOMAP_LIBRARIES}
${OCTOMAP_SERVER_LIBRARIES}
${PCL_LIBRARIES}
)

cs_install()
cs_export()
    