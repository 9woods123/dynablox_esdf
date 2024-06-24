#include "dynablox/processing/tracking.h"

namespace dynablox {

void Tracking::Config::checkParams() const {}

void Tracking::Config::setupParamsAndPrinting() {
  setupParam("min_track_duration", &min_track_duration, "frames");
  setupParam("max_tracking_distance", &max_tracking_distance, "m");
}

Tracking::Tracking(const Config& config) : config_(config.checkValid()) {

}



void Tracking::track(const Cloud& cloud, Clusters& clusters,
                     CloudInfo& cloud_info) {

  // Associate current to previous cluster ids.
  trackClusterIDs(cloud, clusters);
  calculateTrackDuration(cloud_info.timestamp);


  // Label the cloud info.
  for (Cluster& cluster : clusters) {
    if (cluster.track_length >= config_.min_track_duration) {
      cluster.valid = true;
      for (int idx : cluster.points) {
        cloud_info.points[idx].object_level_dynamic = true;
      }
    }
  }
}


void Tracking::calculateTrackDuration(std::uint64_t time_stamp){
  // The duration is the time difference between two frames, 
  //but I didn't process the first frame, so it is subtracted by a default initialized value.
  
  track_duration= time_stamp-last_track_timestamp;
  last_track_timestamp= time_stamp;
  // std::cout<<"track_duration: "<<float(track_duration/1e9f)<<"  s"<<std::endl;

}



void Tracking::trackClusterIDs(const Cloud& cloud, Clusters& clusters) {
  
  
  // std::cout<<"=============trackClusterIDs=============="<<std::endl;

  std::vector<voxblox::Point> centroids(clusters.size());

  size_t i = 0;
  
  for (const Cluster& cluster : clusters) {
    voxblox::Point centroid = {0, 0, 0};
    for (int index : cluster.points) {
      const Point& point = cloud[index];
      centroid = centroid + voxblox::Point(point.x, point.y, point.z);
    }
    centroids[i] = centroid / cluster.points.size();
    ++i;
  }

  // Compute the distances of all clusters. [previous][current]->dist
  struct Association {
    float distance;
    int previous_id;
    int current_id;
    // Eigen::Vector3f delta_distance_vector;  
    
  };

  std::vector<std::vector<Association>> distances(previous_centroids_.size());
  for (size_t i = 0; i < previous_centroids_.size(); ++i) {
    std::vector<Association>& d = distances[i];
    d.reserve(centroids.size());
    for (size_t j = 0; j < centroids.size(); ++j) {
      Association association;
      association.distance = (previous_centroids_[i] - centroids[j]).norm();
      // association.delta_distance_vector = centroids[j]-previous_centroids_[i] ;
      association.previous_id = i;
      association.current_id = j;
      d.push_back(association);
    }
  }

  // Associate all previous ids until no more minimum distances exist.
  std::unordered_set<int> reused_ids;
  std::unordered_set<int> current_ids;
  
  while (true) {
    // Find the minimum distance and IDs (exhaustively).
    float min = std::numeric_limits<float>::max();
    int prev_id = 0;
    int curr_id = 0;
    int erase_i = 0;
    int erase_j = 0;
    // Eigen::Vector3f delta_distance_vector(0.0f,0.0f,0.0f);  

    for (size_t i = 0u; i < distances.size(); ++i) {
      for (size_t j = 0u; j < distances[i].size(); ++j) {
        const Association& association = distances[i][j];
        if (association.distance < min) {
          min = association.distance;
          // delta_distance_vector= association.delta_distance_vector;
          curr_id = association.current_id;
          prev_id = association.previous_id;
          erase_i = i;
          erase_j = j;

        }
      }
    }

    if (min > config_.max_tracking_distance) {
      // no more good fits.
      break;
    }

    // Update traked cluster and remove that match to search for next best.

    clusters[curr_id].id = previous_ids_[prev_id];
    clusters[curr_id].track_length = previous_track_lengths_[prev_id] + 1;
    clusters[curr_id].delta_distance = min;

    Eigen::Vector3f curr_centroid_position = centroids[curr_id];
    Eigen::Vector3f curr_centroid_velocity  = Eigen::Vector3f(
      (centroids[curr_id]-previous_centroids_[prev_id] )/static_cast<float>(track_duration / 1e9f));


    if(clusters[curr_id].track_length > config_.min_track_duration)
    {

      clusters[curr_id].velocity=curr_centroid_velocity;
      clusters[curr_id].position=curr_centroid_position;
      // if reach min track duration , start kalman fliter based tracker
      FilterTracker(clusters[curr_id].id,curr_centroid_position,curr_centroid_velocity);

    }
    else{
      clusters[curr_id].velocity=Eigen::Vector3f::Zero();
      clusters[curr_id].position=curr_centroid_position;
    }

    // std::cout<<"clus id: "<<clusters[curr_id].id<<"  track_length: "<< clusters[curr_id].track_length 
    // <<"   delta_dist: "<<clusters[curr_id].delta_distance<<std::endl;
    

    reused_ids.insert(previous_ids_[prev_id]);
    current_ids.insert(clusters[curr_id].id);

    // remove elements which has been found as the same object;
    distances.erase(distances.begin() + erase_i);
    for (auto& vec : distances) {
      vec.erase(vec.begin() + erase_j);
    }
  }

  // mark the disappeared clusters, and delete these corrosponding trakcer filiters
  for (int id : previous_ids_) {
      if (current_ids.find(id) == current_ids.end()) {
          // track_filter_manager.markClusterAsDisappeared(id);
          track_filter_manager.removeTracker(id);
      }
  }
  
  track_filter_manager.debugManager();


  // Fill in all remaining ids and track data.
  previous_centroids_ = centroids;
  previous_ids_.clear();
  previous_ids_.reserve(clusters.size());
  previous_track_lengths_.clear();
  previous_ids_.reserve(clusters.size());

  int id_counter = 0;
  for (Cluster& cluster : clusters) {
    if (cluster.id == -1) {
      // id==-1 for initial condition
      // We need to replace it.
      while (reused_ids.find(id_counter) != reused_ids.end()) {
        // if id_counter is in the reused_id, then 
        id_counter++;
      }
      cluster.id = id_counter;
      id_counter++;
    }
    previous_ids_.push_back(cluster.id);
    previous_track_lengths_.push_back(cluster.track_length);

  }

}

void Tracking::FilterTracker(int cluster_id, Eigen::Vector3f curr_centroid_position,
Eigen::Vector3f curr_centroid_velocity)
{


float dt=static_cast<float>(track_duration / 1e9f);

Eigen::MatrixXd A(2, 2);
A << 1,dt,
      0, 1;

Eigen::MatrixXd B(2, 1); 
B << 0, 0;   // B 0,0 for no control input 
// x_t+1= A * x_t + B u
// P_t+1 = A* P * AT + Q

Eigen::MatrixXd Q(2, 2);
Q << 0.1, 0,
      0, 0.1; 
// Q: motion model Gaussian error

Eigen::MatrixXd P(2, 2);
P << 1, 0, 
      0, 1;
//  prrior error distrubtion

// y=H *x + gaussion_error
// gaussion_error ~ N(0,R)
Eigen::MatrixXd H(2, 2);
H << 1, 0, 
      0, 1;

Eigen::MatrixXd R(2, 2);
R << 0.25, 0,
      0, 0.25;


Eigen::Vector3d curr_position = curr_centroid_position.cast<double>();  // 将 Eigen::Vector3f 转换为 Eigen::VectorXd
Eigen::Vector3d curr_velocity = curr_centroid_velocity.cast<double>();  // 将 Eigen::Vector3f 转换为 Eigen::VectorXd

// std::cout<<"curr_position:"<< curr_position << std::endl;

bool is_new_cluster=track_filter_manager.addTracker(cluster_id,dt, A, B, H, Q, R, P);


if (is_new_cluster)
  {
    // t0= 0 s;
    std::cout<<"=========is_new_cluster: "<<is_new_cluster<<std::endl;

    track_filter_manager.init_filiter_tracker(cluster_id,0,curr_position);

  }

// pdateTracker(int cluster_id, const Eigen::VectorXd& y, const Eigen::VectorXd& u) {

// track_filter_manager.updateTracker(cluster_id,)


}

}  // namespace dynablox
