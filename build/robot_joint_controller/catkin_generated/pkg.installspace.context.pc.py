# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "robot_msgs;controller_interface;hardware_interface;pluginlib;roscpp;realtime_tools".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lrobot_joint_controller".split(';') if "-lrobot_joint_controller" != "" else []
PROJECT_NAME = "robot_joint_controller"
PROJECT_SPACE_DIR = "/home/tim/ROS/rl_sar_visual/install"
PROJECT_VERSION = "3.0.0"
