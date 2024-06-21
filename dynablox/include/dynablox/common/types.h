#ifndef DYNABLOX_COMMON_TYPES_H_
#define DYNABLOX_COMMON_TYPES_H_

#include <utility>
#include <vector>

#include <pcl_ros/point_cloud.h>
#include <voxblox/core/block.h>
#include <voxblox/core/common.h>
#include <voxblox/core/layer.h>
#include <voxblox/core/voxel.h>
#include <Eigen/Dense>

namespace dynablox {

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

using VoxelIndex = voxblox::VoxelIndex;
using BlockIndex = voxblox::BlockIndex;
using TsdfVoxel = voxblox::TsdfVoxel;
using TsdfBlock = voxblox::Block<TsdfVoxel>;
using TsdfLayer = voxblox::Layer<TsdfVoxel>;

// Additional information stored for every point in the cloud.
struct PointInfo {
  // Include this point when computing performance metrics.
  bool ready_for_evaluation = false;

  // Set to true if the point falls into a voxel labeled ever-free.
  bool ever_free_level_dynamic = false;

  // Set to true if the point belongs to a cluster labeled dynamic.
  bool cluster_level_dynamic = false;

  // Set to true if the point belongs to a tracked object.
  bool object_level_dynamic = false;

  // Distance of the point to the sensor.
  double distance_to_sensor = -1.0;

  // Ground truth label if available.
  bool ground_truth_dynamic = false;
};

// Additional information for a point cloud.
struct CloudInfo {
  bool has_labels = false;
  std::uint64_t timestamp;
  Point sensor_position;
  std::vector<PointInfo> points;
};

// Maps each voxel in a block to all point cloud indices that fall into in it.
using VoxelToPointMap = voxblox::HierarchicalIndexIntMap;

// Map of block indices to voxel indices and point indices of the cloud.
using BlockToPointMap = voxblox::AnyIndexHashMapType<VoxelToPointMap>::type;

// Simple axis-aligned bounding box.
struct BoundingBox {
  Point min_corner;
  Point max_corner;

  bool intersects(const BoundingBox& other, const float margin = 0.f) const {
    if (min_corner.x - margin > other.max_corner.x) {
      return false;
    }
    if (min_corner.y - margin > other.max_corner.y) {
      return false;
    }
    if (min_corner.z - margin > other.max_corner.x) {
      return false;
    }
    if (max_corner.x + margin < other.min_corner.x) {
      return false;
    }
    if (max_corner.y + margin < other.min_corner.y) {
      return false;
    }
    if (max_corner.z + margin < other.min_corner.z) {
      return false;
    }
    return true;
  }

  float extent() const {
    return (max_corner.getVector3fMap() - min_corner.getVector3fMap()).norm();
  }
};

// Indices of all points in the cloud belonging to this cluster.
struct Cluster {
  int id = -1;                // ID of the cluster set during tracking.
  int track_length = 0;       // Frames this cluster has been tracked.
  float delta_distance=0;    //   Distance between two frames within the same cluster.
  Eigen::Vector3f position=Eigen::Vector3f(0.0f, 0.0f, 0.0f); 
  Eigen::Vector3f velocity=Eigen::Vector3f(0.0f, 0.0f, 0.0f);//  Velocity calculated under linear assumption
  Eigen::MatrixXd covariance_matrix;

  bool valid = false;         // Whether the cluster has met all cluster checks.
  BoundingBox aabb;           // Axis-aligned bounding box of the cluster.
  std::vector<int> points;    // Indices of points in cloud.
  std::vector<Point> voxels;  // Center points of voxels in this cluster.
  
  Cluster()
      : covariance_matrix(6, 6) {  // Initialize covariance_matrix with dimensions

        covariance_matrix <<
            0.1, 0,   0,   0,   0,   0,
            0,   0.1, 0,   0,   0,   0,
            0,   0,   0.1, 0,   0,   0,
            0,   0,   0,   0.1, 0,   0,
            0,   0,   0,   0,   0.1, 0,
            0,   0,   0,   0,   0,   0.1;
 
  }
};

using Clusters = std::vector<Cluster>;

}  // namespace dynablox

#endif  // DYNABLOX_COMMON_TYPES_H_
