#ifndef DYNABLOX_PROCESSING_TRACKING_H_
#define DYNABLOX_PROCESSING_TRACKING_H_

#include "dynablox/3rd_party/config_utilities.hpp"
#include "dynablox/common/types.h"
#include "dynablox/kalman_filter/filter_manager.h"


namespace dynablox {



class Tracking {
 public:
  // Config.
  struct Config : public config_utilities::Config<Config> {
    // Numbers of frames a cluster needs to be tracked to be considered dynamic.
    int min_track_duration = 0;

    // Maximum distance a cluster may have moved to be considered a track [m].
    float max_tracking_distance = 1.f;

    Config() { setConfigName("Tracking"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Constructor.
  Tracking(const Config& config);

  /**
   * @brief Track all clusters w.r.t. the previous clsuters. Denote the object
   * level points dynamic in the cloud_info.
   *
   * @param cloud Current lidar point cloud.
   * @param clusters Current detected clusters.
   * @param cloud_info Cloud info to denote moving points.
   */
  void track(const Cloud& cloud, Clusters& clusters, CloudInfo& cloud_info);
  void addFilterTracker(int cluster_id);
  void calculateTrackDuration(std::uint64_t time_stamp);

  // void initializeKalmanFilter(double dt, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
  //                               const Eigen::MatrixXd& H, const Eigen::MatrixXd& Q,
  //                               const Eigen::MatrixXd& R, const Eigen::MatrixXd& P) {
  //       kalman_filter = KalmanFilter(dt, A, B, H, Q, R, P);
  //   };


  // void kalmanTracking(Eigen::Vector3f centroid_position, Eigen::Vector3f velocity);

  // void setKalmanFilter();

 private:
  const Config config_;
  std::uint64_t last_track_timestamp;
  std::uint64_t track_duration;  // Nsec

  // store the filter coorsponding to each object 
  TrackerFilterManager track_filter_manager;

  // Tracking data w.r.t. previous observation.
  std::vector<voxblox::Point> previous_centroids_;
  std::vector<int> previous_ids_;
  std::vector<int> previous_track_lengths_;

  /**
   * @brief Simple closest association tracking for now.
   *
   * @param cloud Lidar point cloud.
   * @param clusters Current clusters to be tracked.
   */
  void trackClusterIDs(const Cloud& cloud, Clusters& clusters);
};

}  // namespace dynablox

#endif  // DYNABLOX_PROCESSING_TRACKING_H_
