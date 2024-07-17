# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "cmake_modules;eigen_catkin;eigen_checks;glog_catkin;mav_msgs;nlopt".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lmav_trajectory_generation".split(';') if "-lmav_trajectory_generation" != "" else []
PROJECT_NAME = "mav_trajectory_generation"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
