#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Geometry>
#include "hybrid_astar/constants.h"
#include "hybrid_astar/helper.h"
#include "hybrid_astar/collisiondetection.h"
#include "hybrid_astar/algorithm.h"
#include "hybrid_astar/node4d.h"


#include "hybrid_astar/path.h"
#include "hybrid_astar/smoother.h"
#include "hybrid_astar/visualize.h"
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_server/OctomapServer.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <uuv_control_msgs/Trajectory.h>

namespace HybridAStar {
/*!
   \brief A class that creates the interface for the hybrid A* algorithm.

    It inherits from `ros::nav_core::BaseGlobalPlanner` so that it can easily be used with the ROS navigation stack
   \todo make it actually inherit from nav_core::BaseGlobalPlanner
*/
class Planner {
 public:
  /// The default constructor
  Planner();

  /*!
     \brief Initializes the collision as well as heuristic lookup table
     \todo probably removed
  */
  void initializeLookups();

  /*!
     \brief Sets the map e.g. through a callback from a subscriber listening to map updates.
     \param map the map or occupancy grid
  */
  void setMap(const nav_msgs::OccupancyGrid::Ptr map);

  /*!
     \brief setStart
     \param start the start pose
  */
  void setStart(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& start);

  /*!
     \brief setGoal
     \param goal the goal pose
  */
  void setGoal(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& goal);

  /*!
     \brief The central function entry point making the necessary preparations to start the planning.
  */
  void plan();

  void pub_planned_traj(std::vector<Node4D> path_nodes) ;

 private:
 
  /// The node handle
  ros::NodeHandle n;

  ros::Publisher traj_pub;
  /// A publisher publishing the planned trajectory

  /// A subscriber for receiving goal updates
  ros::Subscriber subGoal;
  /// A subscriber for receiving start updates
  ros::Subscriber subStart;
  /// A listener that awaits transforms
  tf::TransformListener listener;
  /// A transform for moving start positions
  tf::StampedTransform transform;
  /// The path produced by the hybrid A* algorithm
  Path path;
  /// The smoother used for optimizing the path
  Smoother smoother;
  /// The path smoothed and ready for the controller
  Path smoothedPath = Path(true);
  /// The visualization used for search visualization
  Visualize visualization;
  /// The collission detection for testing specific configurations
  CollisionDetection configurationSpace;
  /// The voronoi diagram
  /// A pointer to the grid the planner runs on
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The start pose set through RViz
  geometry_msgs::PoseWithCovarianceStamped start;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;
  bool goal_setting_flag=false;
  bool start_setting_flag=false;



  octomap::ColorOcTree*  NaviMap;
  ros::Publisher octomap_pub;
  octomap_msgs::Octomap octomap_msg;
  uuv_control_msgs::Trajectory planned_traj;
  Eigen::Vector4d Start_from_InterMarker;
  Eigen::Vector4d Goal_from_InterMarker;

// ========woods======      read a octo map from  .ot  file. 


};
}
#endif // PLANNER_H
