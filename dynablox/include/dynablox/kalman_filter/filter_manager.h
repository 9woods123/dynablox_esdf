#ifndef TRACKER_MANAGER_H
#define TRACKER_MANAGER_H

#include <unordered_map>
#include <memory>
#include "kalman_filter.h"

class TrackerFilterManager {
public:
    TrackerFilterManager(){}

    void addTracker(int cluster_id,double dt, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                 const Eigen::MatrixXd& H, const Eigen::MatrixXd& Q,
                 const Eigen::MatrixXd& R, const Eigen::MatrixXd& P)
    {
        if (tracker_filters.find(cluster_id) == tracker_filters.end()) {
            tracker_filters[cluster_id] = std::make_shared<KalmanFilter>(dt, A, B, H, Q, R, P);
        }
    }


    void removeTracker(int cluster_id) {
        tracker_filters.erase(cluster_id);
    }


    std::shared_ptr<KalmanFilter> getTracker(int cluster_id) {
        auto it = tracker_filters.find(cluster_id);
        if (it != tracker_filters.end()) {
            return it->second;
        }
        return nullptr;
    }


    void updateTracker(int cluster_id, const Eigen::VectorXd& y, const Eigen::VectorXd& u) {
        std::shared_ptr<KalmanFilter> filter = getTracker(cluster_id);
        if (filter) {
            filter->update(y, u);
        }
    }

    void clear(){
        tracker_filters.clear();
    }

    Eigen::VectorXd getTrackerState(int cluster_id) {
        std::shared_ptr<KalmanFilter> filter = getTracker(cluster_id);
        if (filter) {
            return filter->state();
        }
        throw std::runtime_error("Tracker not found");
    }

private:
    std::unordered_map<int, std::shared_ptr<KalmanFilter>> tracker_filters;

};

#endif // TRACKER_MANAGER_H
