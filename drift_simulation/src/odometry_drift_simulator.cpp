#include "drift_simulation/odometry_drift_simulator.h"

#include <chrono>
#include <fstream>
#include <thread>

#include <eigen_conversions/eigen_msg.h>
#include <minkindr_conversions/kindr_msg.h>

namespace unreal_airsim {
OdometryDriftSimulator::OdometryDriftSimulator(
    Config config, ros::NodeHandle nh, const ros::NodeHandle& nh_private)
    : config_(config.checkValid()),
      nh_private_(nh_private),
      started_publishing_(false),
      velocity_noise_sampling_period_(1.f / config.velocity_noise_frequency_hz),
      velocity_noise_(config.velocity_noise),
      pose_noise_(config.pose_noise) {
  // Get path to write drift values.
  nh_private_.param<std::string>("output_drifted_file_name",
                                 output_drifted_file_name_, "");
  nh_private_.param<std::string>("output_gt_file_name", output_gt_file_name_,
                                 "");
  nh_private_.param<std::string>("global_frame_name", global_frame_name_,
                                 global_frame_name_);
  nh_private_.param<std::string>("sensor_frame_name", sensor_frame_name_,
                                 sensor_frame_name_);

  // Ensure params are set.
  if (output_drifted_file_name_.empty()) {
    LOG(FATAL) << "Parameter 'output_drifted_file_name' must be set!";
  }
  if (output_gt_file_name_.empty()) {
    LOG(WARNING) << "Parameter 'output_gt_file_name' is not set, not writing "
                    "the ground truth values.";
  }

  reset();

  pointcloud_sub_ = nh.subscribe("pointcloud", 100,
                                 &OdometryDriftSimulator::poseCallback, this);
  LOG(INFO) << "Initialized drifting odometry simulator, with config:\n"
            << config_;
}

void OdometryDriftSimulator::poseCallback(
    const sensor_msgs::PointCloud2& pointcloud_msg) {
  tf::StampedTransform transform;
  const int max_tries = 1000;
  int tries = 0;
  std::string except;
  while (tries < max_tries) {
    try {
      tf_listener_.lookupTransform(sensor_frame_name_, global_frame_name_,
                                   pointcloud_msg.header.stamp, transform);
      break;
    } catch (tf::TransformException& ex) {
      tries++;
      except = ex.what();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  if (tries == max_tries) {
    LOG(FATAL) << "Failed to get transform: " << except;
  }
  geometry_msgs::TransformStamped ground_truth_pose_msg;
  ground_truth_pose_msg.header.stamp = transform.stamp_;
  tf::transformTFToMsg(transform, ground_truth_pose_msg.transform);

  this->tick(ground_truth_pose_msg);

  geometry_msgs::TransformStamped simulated_pose_msg;
  simulated_pose_msg =
      this->convertGroundTruthToDriftedPoseMsg(ground_truth_pose_msg);

  std::fstream fout, fout_truth;

  // Write the data.
  fout.open(output_drifted_file_name_, std::ios::out | std::ios::app);
  fout << simulated_pose_msg.transform.translation.x << ", "
       << simulated_pose_msg.transform.translation.y << ", "
       << simulated_pose_msg.transform.translation.z << ", "
       << simulated_pose_msg.transform.rotation.x << ", "
       << simulated_pose_msg.transform.rotation.y << ", "
       << simulated_pose_msg.transform.rotation.z << ", "
       << simulated_pose_msg.transform.rotation.w << std::endl;
  fout.close();
  LOG_FIRST_N(INFO, 1) << "Writing drift data to '" << output_drifted_file_name_
                       << "'.";

  if (!output_gt_file_name_.empty()) {
    fout_truth.open(output_gt_file_name_, std::ios::out | std::ios::app);
    fout_truth << ground_truth_pose_msg.transform.translation.x << ", "
               << ground_truth_pose_msg.transform.translation.y << ", "
               << ground_truth_pose_msg.transform.translation.z << ", "
               << ground_truth_pose_msg.transform.rotation.x << ", "
               << ground_truth_pose_msg.transform.rotation.y << ", "
               << ground_truth_pose_msg.transform.rotation.z << ", "
               << ground_truth_pose_msg.transform.rotation.w << std::endl;
    fout_truth.close();
    LOG_FIRST_N(INFO, 1) << "Writing gt data to '" << output_drifted_file_name_
                         << "'.";
  }
}

void OdometryDriftSimulator::reset() {
  last_velocity_noise_sampling_time_ = ros::Time();
  current_linear_velocity_noise_sample_W_.setZero();
  current_angular_velocity_noise_sample_W_.setZero();
  integrated_pose_drift_.setIdentity();
  current_pose_noise_.setIdentity();
  current_simulated_pose_.setIdentity();
  last_ground_truth_pose_msg_ = geometry_msgs::TransformStamped();
}

void OdometryDriftSimulator::tick(
    const geometry_msgs::TransformStamped& ground_truth_pose_msg) {
  // Compute the time delta
  const ros::Time& current_timestamp = ground_truth_pose_msg.header.stamp;
  double delta_t = 0.0;
  if (!last_ground_truth_pose_msg_.header.stamp.isZero()) {
    delta_t =
        (current_timestamp - last_ground_truth_pose_msg_.header.stamp).toSec();
  }
  last_ground_truth_pose_msg_ = ground_truth_pose_msg;
  if (delta_t < 0.0) {
    LOG(WARNING) << "Time difference between current and last received pose "
                    "msg is negative. Skipping.";
    return;
  }

  // Get the ground truth pose
  Transformation ground_truth_pose;
  tf::transformMsgToKindr(ground_truth_pose_msg.transform, &ground_truth_pose);

  // Draw a random velocity noise sample, used to simulate drift
  if (last_velocity_noise_sampling_time_ + velocity_noise_sampling_period_ <
      current_timestamp) {
    last_velocity_noise_sampling_time_ = current_timestamp;

    // Sample the linear velocity noise in body frame
    const Transformation::Vector3 current_linear_velocity_noise_sample_B_ = {
        velocity_noise_.x(), velocity_noise_.y(), velocity_noise_.z()};
    current_linear_velocity_noise_sample_W_ =
        ground_truth_pose.getRotation().rotate(
            current_linear_velocity_noise_sample_B_);

    // Sample the angular velocity noise directly in world frame,
    // since we only want to simulate drift on world frame yaw
    current_angular_velocity_noise_sample_W_ = {0.0, 0.0,
                                                velocity_noise_.yaw()};
  }

  // Integrate the drift
  Transformation::Vector6 drift_delta_W_vec;
  drift_delta_W_vec << current_linear_velocity_noise_sample_W_,
      current_angular_velocity_noise_sample_W_;
  drift_delta_W_vec *= delta_t;
  const Transformation drift_delta_W = Transformation::exp(drift_delta_W_vec);
  integrated_pose_drift_ = drift_delta_W * integrated_pose_drift_;

  // Draw a random pose noise sample
  Transformation::Vector6 pose_noise_B_vec;
  pose_noise_B_vec << pose_noise_.x(), pose_noise_.y(), pose_noise_.z(),
      pose_noise_.roll(), pose_noise_.pitch(), pose_noise_.yaw();
  current_pose_noise_ = Transformation::exp(pose_noise_B_vec);

  // Update the current simulated pose
  current_simulated_pose_ =
      integrated_pose_drift_ * ground_truth_pose * current_pose_noise_;
}

OdometryDriftSimulator::Transformation
OdometryDriftSimulator::getGroundTruthPose() const {
  Transformation ground_truth_pose;
  tf::transformMsgToKindr(last_ground_truth_pose_msg_.transform,
                          &ground_truth_pose);
  return ground_truth_pose;
}

geometry_msgs::TransformStamped OdometryDriftSimulator::getSimulatedPoseMsg()
    const {
  geometry_msgs::TransformStamped simulated_pose_msg =
      last_ground_truth_pose_msg_;
  tf::transformKindrToMsg(current_simulated_pose_,
                          &simulated_pose_msg.transform);
  return simulated_pose_msg;
}

OdometryDriftSimulator::Transformation
OdometryDriftSimulator::convertDriftedToGroundTruthPose(
    const OdometryDriftSimulator::Transformation& simulated_pose) const {
  return integrated_pose_drift_.inverse() * simulated_pose *
         current_pose_noise_.inverse();
}

OdometryDriftSimulator::Transformation
OdometryDriftSimulator::convertGroundTruthToDriftedPose(
    const OdometryDriftSimulator::Transformation& ground_truth_pose) const {
  return integrated_pose_drift_ * ground_truth_pose * current_pose_noise_;
}

geometry_msgs::TransformStamped
OdometryDriftSimulator::convertDriftedToGroundTruthPoseMsg(
    const geometry_msgs::TransformStamped& simulated_pose_msg) const {
  Transformation simulated_pose;
  tf::transformMsgToKindr(simulated_pose_msg.transform, &simulated_pose);

  const Transformation ground_truth_pose =
      convertDriftedToGroundTruthPose(simulated_pose);

  geometry_msgs::TransformStamped ground_truth_pose_msg = simulated_pose_msg;
  tf::transformKindrToMsg(ground_truth_pose, &ground_truth_pose_msg.transform);
  return ground_truth_pose_msg;
}

geometry_msgs::TransformStamped
OdometryDriftSimulator::convertGroundTruthToDriftedPoseMsg(
    const geometry_msgs::TransformStamped& ground_truth_pose_msg) const {
  Transformation ground_truth_pose;
  tf::transformMsgToKindr(ground_truth_pose_msg.transform, &ground_truth_pose);

  const Transformation simulated_pose =
      convertGroundTruthToDriftedPose(ground_truth_pose);

  geometry_msgs::TransformStamped simulated_pose_msg = ground_truth_pose_msg;
  tf::transformKindrToMsg(simulated_pose, &simulated_pose_msg.transform);
  return simulated_pose_msg;
}

OdometryDriftSimulator::VelocityNoiseDistributions::VelocityNoiseDistributions(
    const OdometryDriftSimulator::Config::NoiseConfigMap&
        velocity_noise_configs)
    : x(velocity_noise_configs.at("x")),
      y(velocity_noise_configs.at("y")),
      z(velocity_noise_configs.at("z")),
      yaw(velocity_noise_configs.at("yaw")) {}

OdometryDriftSimulator::PoseNoiseDistributions::PoseNoiseDistributions(
    const OdometryDriftSimulator::Config::NoiseConfigMap& pose_noise_configs)
    : x(pose_noise_configs.at("x")),
      y(pose_noise_configs.at("y")),
      z(pose_noise_configs.at("z")),
      yaw(pose_noise_configs.at("yaw")),
      pitch(pose_noise_configs.at("pitch")),
      roll(pose_noise_configs.at("roll")) {}

OdometryDriftSimulator::Config OdometryDriftSimulator::Config::fromRosParams(
    const ros::NodeHandle& nh) {
  Config config;

  nh.param("publish_ground_truth_pose", config.publish_ground_truth_pose,
           config.publish_ground_truth_pose);

  nh.param("ground_truth_frame_suffix", config.ground_truth_frame_suffix,
           config.ground_truth_frame_suffix);

  nh.param("velocity_noise_frequency_hz", config.velocity_noise_frequency_hz,
           config.velocity_noise_frequency_hz);

  for (auto& kv : config.velocity_noise) {
    kv.second = NormalDistribution::Config::fromRosParams(
        ros::NodeHandle(nh, "velocity_noise/" + kv.first));
  }
  for (auto& kv : config.pose_noise) {
    kv.second = NormalDistribution::Config::fromRosParams(
        ros::NodeHandle(nh, "position_noise/" + kv.first));
  }

  return config;
}

bool OdometryDriftSimulator::Config::isValid() const {
  bool is_valid = true;

  if (ground_truth_frame_suffix.empty()) {
    LOG(WARNING)
        << "The ground_truth_frame_suffix should be a non-empty string";
    is_valid = false;
  }

  if (velocity_noise_frequency_hz <= 0.f) {
    LOG(WARNING)
        << "The velocity_noise_frequency_hz should be a positive float";
    is_valid = false;
  }

  for (auto& kv : velocity_noise) {
    if (!kv.second.isValid("velocity_noise/" + kv.first + "/")) {
      is_valid = false;
    }
  }
  for (auto& kv : pose_noise) {
    if (!kv.second.isValid("position_noise/" + kv.first + "/")) {
      is_valid = false;
    }
  }

  return is_valid;
}

std::ostream& operator<<(std::ostream& os,
                         const OdometryDriftSimulator::Config& config) {
  os << "-- publish_ground_truth_pose: "
     << (config.publish_ground_truth_pose ? "true" : "false") << "\n"
     << "-- ground_truth_frame_suffix: " << config.ground_truth_frame_suffix
     << "\n"
     << "-- velocity_noise_frequency_hz: " << config.velocity_noise_frequency_hz
     << "\n";

  os << "-- velocity_noise/\n";
  for (auto& kv : config.velocity_noise) {
    os << "---- " << kv.first << ": {" << kv.second << "}\n";
  }
  os << "-- position_noise/\n";
  for (auto& kv : config.pose_noise) {
    os << "---- " << kv.first << ": {" << kv.second << "}\n";
  }

  return os;
}
}  // namespace unreal_airsim
