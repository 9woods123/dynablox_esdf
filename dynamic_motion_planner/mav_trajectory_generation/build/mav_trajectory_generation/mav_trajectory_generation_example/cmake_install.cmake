# Install script for directory: /home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_trajectory_generation_example

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_example")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_example"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example" TYPE EXECUTABLE FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/devel/lib/mav_trajectory_generation_example/trajectory_generation_example")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_example")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_example"
         OLD_RPATH "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/devel/lib:/home/woods/uuv/test_ws/devel/lib:/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_example")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_6dof_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_6dof_example")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_6dof_example"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example" TYPE EXECUTABLE FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/devel/lib/mav_trajectory_generation_example/trajectory_generation_6dof_example")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_6dof_example" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_6dof_example")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_6dof_example"
         OLD_RPATH "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/devel/lib:/home/woods/uuv/test_ws/devel/lib:/opt/ros/melodic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mav_trajectory_generation_example/trajectory_generation_6dof_example")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_trajectory_generation_example/include/" FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hpp$" REGEX "/\\.svn$" EXCLUDE)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_trajectory_generation_example/launch" TYPE DIRECTORY FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_trajectory_generation_example/launch/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/mav_trajectory_generation/mav_trajectory_generation_example/catkin_generated/installspace/mav_trajectory_generation_example.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_trajectory_generation_example/cmake" TYPE FILE FILES
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/mav_trajectory_generation/mav_trajectory_generation_example/catkin_generated/installspace/mav_trajectory_generation_exampleConfig.cmake"
    "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/build/mav_trajectory_generation/mav_trajectory_generation_example/catkin_generated/installspace/mav_trajectory_generation_exampleConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mav_trajectory_generation_example" TYPE FILE FILES "/home/woods/uuv/motion_planner_ws/src/mav_trajectory_generation/mav_trajectory_generation_example/package.xml")
endif()

