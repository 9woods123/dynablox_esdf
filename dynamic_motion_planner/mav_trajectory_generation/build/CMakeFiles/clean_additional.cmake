# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Debug")
  file(REMOVE_RECURSE
  "rviz-3d-nav-goal-tool/CMakeFiles/rviz_3d_nav_goal_tool_autogen.dir/AutogenUsed.txt"
  "rviz-3d-nav-goal-tool/CMakeFiles/rviz_3d_nav_goal_tool_autogen.dir/ParseCache.txt"
  "rviz-3d-nav-goal-tool/rviz_3d_nav_goal_tool_autogen"
  )
endif()
