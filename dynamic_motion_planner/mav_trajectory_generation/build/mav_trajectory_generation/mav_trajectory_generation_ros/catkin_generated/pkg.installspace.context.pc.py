# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "eigen_catkin;mav_msgs;mav_trajectory_generation;mav_visualization;mav_planning_msgs;eigen_checks;roslib".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmav_trajectory_generation_ros".split(';') if "-lmav_trajectory_generation_ros" != "" else []
PROJECT_NAME = "mav_trajectory_generation_ros"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
