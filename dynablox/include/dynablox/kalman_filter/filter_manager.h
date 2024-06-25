#ifndef TRACKER_MANAGER_H
#define TRACKER_MANAGER_H

#include <unordered_map>
#include <memory>
#include "kalman_filter.h"
#include <chrono>

class TrackerFilterManager {
public:
    TrackerFilterManager(){
        tracker_filters.clear();
        disappeared_cluster_ids.clear();
    }

    void printMatrixInfo(const Eigen::MatrixXd& mat, const std::string& name) {
        std::cout << name << " size: " << mat.rows() << "x" << mat.cols() << std::endl;
    }

    bool addTracker(int cluster_id,double dt, const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                const Eigen::MatrixXd& H, const Eigen::MatrixXd& Q,
                const Eigen::MatrixXd& R, const Eigen::MatrixXd& P)
    {   
        
        if (tracker_filters.find(cluster_id) == tracker_filters.end()) {

        std::shared_ptr<KalmanFilter> insert_filter = std::make_shared<KalmanFilter>(dt, A, B, H, Q, R, P);
        tracker_filters.emplace(cluster_id,insert_filter);

        return true;
        }
        else { 

            return false; 
        }
    }

    void removeTracker(int cluster_id) {
        if (tracker_filters.find(cluster_id) != tracker_filters.end()) {
        tracker_filters.erase(cluster_id);

        }
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
    

    void init_filiter_tracker(int cluster_id,double t0, const Eigen::VectorXd& x0) {
        
        std::shared_ptr<KalmanFilter> filter = getTracker(cluster_id);
        if (filter) {
            filter->init(t0, x0);
        }

    }

    void clearDisappearedClusterIds() {
        disappeared_cluster_ids.clear();
    }

    void markClusterAsDisappeared(int cluster_id) {
        disappeared_cluster_ids.insert(cluster_id);
    }


    void debugManager()
    {
        
        std::cout<<"===========debugManager======== s"<<std::endl;

        for (const auto& pair : tracker_filters) {
            int cluster_id = pair.first;
            const auto& filter = pair.second;
            std::cout << "Cluster ID: " << cluster_id << ", Filter state: " << filter->state().transpose() << std::endl;
        }
        std::cout<<"===========debugManager======== e"<<std::endl;

    }


private:
    std::unordered_map<int, std::shared_ptr<KalmanFilter>> tracker_filters;
    std::unordered_set<int> disappeared_cluster_ids;

};

#endif // TRACKER_MANAGER_H
