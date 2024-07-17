#include "hybrid_astar/visualize.h"
using namespace HybridAStar;
//###################################################
//                                CLEAR VISUALIZATION
//###################################################
void Visualize::clear() {

    // TODO check it is useful
    geometry_msgs::PoseStamped emptyPose;
    emptyPose.header.frame_id = "world";
    emptyPose.header.stamp = ros::Time::now();
    emptyPose.header.seq = 0;
    pubNode4D.publish(emptyPose);  // 发布空的pose消息
    pubNodes4Dreverse.publish(emptyPose);
    pubNodes3D.publish(emptyPose);

    poses3D.poses.clear();
    poses4D.poses.clear();
    poses4Dreverse.poses.clear();

}


//###################################################
//                                    CURRENT 4D NODE
//###################################################
void Visualize::publishNode4DPose(Node4D& node) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp = ros::Time::now();
  pose.header.seq = 0;
  pose.pose.position.x = node.getX() * Constants::cellSize;
  pose.pose.position.y = node.getY() * Constants::cellSize;
  pose.pose.position.z = node.getZ() * Constants::cellSize;
  //FORWARD
  if (node.getPrim() < 5) {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  }
  //REVERSE
  else {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
  }
  // PUBLISH THE POSE
  pubNode4D.publish(pose);
}


//###################################################
//                              ALL EXPANDED 4D NODES
//###################################################
void Visualize::publishNode4DPoses(Node4D& node) {
  geometry_msgs::Pose pose;
  pose.position.x = node.getX() * Constants::cellSize;
  pose.position.y = node.getY() * Constants::cellSize;
  pose.position.z = node.getZ() * Constants::cellSize;

  //FORWARD
  if (node.getPrim() < 5) {
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    poses4D.poses.push_back(pose);
    poses4D.header.stamp = ros::Time::now();
    poses4D.header.frame_id="world";
    // PUBLISH THE POSEARRAY
    pubNodes4D.publish(poses4D);
  }
  //REVERSE
  else {
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
    poses4Dreverse.poses.push_back(pose);
    poses4Dreverse.header.stamp = ros::Time::now();
    poses4Dreverse.header.frame_id="world";
      // PUBLISH THE POSEARRAY
    pubNodes4Dreverse.publish(poses4Dreverse);
  }

}

void Visualize::publishNode3DPose(HybridAStar::Node3D &node) {


    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "nodes";
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = node.getX() * 5;
    marker.pose.position.y = node.getY() *5;
    marker.pose.position.z = node.getZ() * 5;
    marker.pose.orientation.w = 1.0;  // Default orientation

    // Set the scale of the marker (adjust as needed)
    marker.scale.x =5;
    marker.scale.y =5;
    marker.scale.z =5;

    // Set the color of the marker
    marker.color.a = 0.3;  // Alpha (transparency)
    marker.color.r = 1.0;  // Red
    marker.color.g = 0.0;  // Green
    marker.color.b = 0;

    CubeNode3D.markers.push_back(marker);
}

void Visualize::publishNode3DPoses(Node3D& node) {

    float res=5;

    Eigen::Vector3d pt(node.getX(),node.getY(),node.getZ());
    Eigen::Vector3d  origin_(0,0,0);
    float inv_resolution_=1/res;
    Eigen::Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();


    geometry_msgs::Pose pose;
    pose.position.x = idx[0]*res;
    pose.position.y = idx[1]*res;
    pose.position.z = idx[2]*res;


    poses3D.poses.push_back(pose);
    poses3D.header.stamp = ros::Time::now();
    poses3D.header.frame_id="world";
        // PUBLISH THE POSEARRAY
    pubNodes3D.publish(poses3D);


}
