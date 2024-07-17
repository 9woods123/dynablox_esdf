# RViz 3D Navigation Goal Tool

As technology continues to improve, the need for 3d navigation has become more and more important. Here is an old tool from Willow Garage which provides the support for 3d nav goal. The code is extracted from this [repository](https://github.com/HKUST-Aerial-Robotics/plan_utils), please visit for more information.  

## Building

This package is just like any other ROS pacakge, just clone it into your workspace and compile it accordingly.

ssh clone:  
```bash
git clone git@github.com:BruceChanJianLe/rviz-3d-nav-goal-tool.git
```

http clone:  
```bash
git clone https://github.com/BruceChanJianLe/rviz-3d-nav-goal-tool.git
```

building:  
```bash
catkin_make
```

## Using

To avoid conflicting with the 2D nav goal, please click the `minus` icon to remove it. Then click on the `plus` icon to choose the 3D nav goal. You may send the goal by left clicking and deciding the orientation of the goal. Without letting go of the mouse, right click to indicate the height of the goal.

## Reference

https://github.com/ros-planning/3d_navigation
