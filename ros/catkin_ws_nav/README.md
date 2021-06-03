
// 编译teb_local_planner
rosdep install teb_local_planner
catkin_make  -DCATKIN_WHITELIST_PACKAGES="teb_local_planner"
