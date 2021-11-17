# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "costmap_2d;geometry_msgs;nav_core;navfn;nav_msgs;pluginlib;roscpp;tf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lompl_global_planner_lib".split(';') if "-lompl_global_planner_lib" != "" else []
PROJECT_NAME = "move_base_ompl"
PROJECT_SPACE_DIR = "/home/walle/catkin_ws/install"
PROJECT_VERSION = "0.0.1"
