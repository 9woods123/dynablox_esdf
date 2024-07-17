#ifndef VISUALIZE_H
#define VISUALIZE_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "hybrid_astar/node4d.h"
#include "hybrid_astar/node3d.h"
#include "hybrid_astar/algorithm.h"

namespace HybridAStar {
class Node4D;
class Node3D;

class Visualize {
 public:
  // ___________
  // CONSTRUCTOR
  /// The default constructor initializing the visualization object and setting publishers for the same.
  Visualize() {
    // _________________
    // TOPICS TO PUBLISH


    pubNode4D = n.advertise<geometry_msgs::PoseStamped>("/visualizeNodes4DPose", 100);
    pubNodes4D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes4DPoses", 100);
    pubNodes4Dreverse = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes4DPosesReverse", 100);
    pubNodes4DCosts = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes4DCosts", 100);

    pubNode3D = n.advertise<visualization_msgs::MarkerArray>("/visualizeNodes3DPose", 100);
    pubNodes3D = n.advertise<geometry_msgs::PoseArray>("/visualizeNodes3DPoses", 100);


  }

  // CLEAR VISUALIZATION
  /// Clears the entire visualization
  void clear();



  void publishNode3DPose(Node3D& node);
  /// Publishes all expanded nodes to RViz
  void publishNode3DPoses(Node3D& node);

  void publishNode4DPose(Node4D& node);
  /// Publishes all expanded nodes to RViz
  void publishNode4DPoses(Node4D& node);
  // PUBLISH THE COST FOR A 3D NODE TO RViz
  /// Publishes the minimum of the cost of all nodes in a 2D grid cell



 private:
  /// A handle to the ROS node
  ros::NodeHandle n;
  /// Publisher for a single 3D node

  ros::Publisher pubNode4D;
  ros::Publisher pubNodes4D;
  ros::Publisher pubNodes4Dreverse;
  ros::Publisher pubNodes4DCosts;

  ros::Publisher pubNode3D;
  ros::Publisher pubNodes3D;
  ros::Publisher pubNodes3Dreverse;
  ros::Publisher pubNodes3DCosts;

  geometry_msgs::PoseArray poses4D;
  geometry_msgs::PoseArray poses4Dreverse;

  geometry_msgs::PoseArray poses3D;

  visualization_msgs::MarkerArray CubeNode3D;
  visualization_msgs::MarkerArray CubeNodes3D;

};
}
#endif // VISUALIZE_H
