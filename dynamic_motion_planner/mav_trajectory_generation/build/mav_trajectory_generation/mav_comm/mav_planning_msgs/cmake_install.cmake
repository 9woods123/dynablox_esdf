# Install script for directory: /home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs/msg" TYPE FILE FILES
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/msg/Point2D.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/msg/PointCloudWithPose.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/msg/Polygon2D.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/msg/PolygonWithHoles.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/msg/PolygonWithHolesStamped.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/msg/PolynomialSegment.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/msg/PolynomialTrajectory.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/msg/PolynomialSegment4D.msg"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/msg/PolynomialTrajectory4D.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs/srv" TYPE FILE FILES
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/srv/PlannerService.srv"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/srv/PolygonService.srv"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/srv/ChangeNameService.srv"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs/cmake" TYPE FILE FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/mav_trajectory_generation/mav_comm/mav_planning_msgs/catkin_generated/installspace/mav_planning_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/devel/include/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/devel/share/roseus/ros/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/devel/share/common-lisp/ros/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/devel/share/gennodejs/ros/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/devel/lib/python2.7/dist-packages/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/devel/lib/python2.7/dist-packages/mav_planning_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/mav_trajectory_generation/mav_comm/mav_planning_msgs/catkin_generated/installspace/mav_planning_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs/cmake" TYPE FILE FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/mav_trajectory_generation/mav_comm/mav_planning_msgs/catkin_generated/installspace/mav_planning_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs/cmake" TYPE FILE FILES
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/mav_trajectory_generation/mav_comm/mav_planning_msgs/catkin_generated/installspace/mav_planning_msgsConfig.cmake"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/mav_trajectory_generation/mav_comm/mav_planning_msgs/catkin_generated/installspace/mav_planning_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_planning_msgs" TYPE FILE FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_comm/mav_planning_msgs/package.xml")
endif()

