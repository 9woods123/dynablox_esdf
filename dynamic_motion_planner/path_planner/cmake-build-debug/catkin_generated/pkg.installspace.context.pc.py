# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "eigen_catkin;roscpp;rospy;std_msgs;visualization_msgs;voxblox_ros;octomap_msgs;uuv_control_msgs;tf;mav_trajectory_generation;mav_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lhybrid_astar".split(';') if "-lhybrid_astar" != "" else []
PROJECT_NAME = "hybrid_astar"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
