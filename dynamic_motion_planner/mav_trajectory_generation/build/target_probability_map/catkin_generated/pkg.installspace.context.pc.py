# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "eigen_catkin;roscpp;rospy;std_msgs;sensor_msgs;detect_msgs;visualization_msgs;tf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltarget_probability_map".split(';') if "-ltarget_probability_map" != "" else []
PROJECT_NAME = "target_probability_map"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
