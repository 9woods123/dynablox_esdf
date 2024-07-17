#include "hybrid_astar/path.h"

using namespace HybridAStar;


//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
  Node4D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  addNode(node, 1);
  addVehicle(node, 1);
  publishPath();
  publishPathNodes();
  publishPathVehicles();
}


// TRACE 4DPATH
void Path::updatePath(const std::vector<Node4D>& nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 1;
  for (size_t i = 0; i < nodePath.size(); ++i) {
    addSegment(nodePath[i]);
    addNode(nodePath[i], k);
    addVehicle(nodePath[i], k);
    k++;
  }

  return;
}
// ___________
// ADD SEGMENT

void Path::addSegment(const Node4D& node) {
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = node.getZ() * Constants::cellSize;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path.poses.push_back(vertex);
}
// ________
// ADD NODE

void Path::addNode(const Node4D& node, int i) {
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 1) {
    pathNode.action=visualization_msgs::Marker::DELETEALL;
  }
  else{
      pathNode.action=visualization_msgs::Marker::ADD;
  }

  pathNode.lifetime = ros::Duration();  // 设置标记的持续时间为无限期
  pathNode.header.frame_id = "world";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNode.pose.position.y = node.getZ() * Constants::cellSize;
  pathNodes.markers.push_back(pathNode);
}


void Path::addVehicle(const Node4D& node, int i) {
    visualization_msgs::Marker pathVehicle;

     //delete all previous markersg
    if (i == 1) {
        pathVehicle.action = visualization_msgs::Marker::DELETEALL;
    }
    else{
        pathVehicle.action=visualization_msgs::Marker::ADD;
    }
    pathVehicle.lifetime = ros::Duration();  // 设置标记的持续时间为无限期
    pathVehicle.header.frame_id = "world";
    pathVehicle.header.stamp = ros::Time(0);
    pathVehicle.id = i;
    pathVehicle.type = visualization_msgs::Marker::CUBE;
    pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
    pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
    pathVehicle.scale.z = Constants::height- Constants::bloating * 2;
    pathVehicle.color.a =0.4;

    if (smoothed) {
        pathVehicle.color.r = Constants::orange.red;
        pathVehicle.color.g = Constants::orange.green;
        pathVehicle.color.b = Constants::orange.blue;
    } else {
        pathVehicle.color.r = Constants::teal.red;
        pathVehicle.color.g = Constants::teal.green;
        pathVehicle.color.b = Constants::teal.blue;
    }

    pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
    pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
    pathVehicle.pose.position.z = node.getZ() * Constants::cellSize;
    pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    pathVehicles.markers.push_back(pathVehicle);
}
