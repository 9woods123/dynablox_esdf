

#ifndef ONLINE_MOTION_PLANNER
#define ONLINE_MOTION_PLANNER


#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <std_srvs/Trigger.h>
#include <uuv_control_msgs/Hold.h>
#include <uuv_control_msgs/Trajectory.h>

#include <nav_msgs/Odometry.h>
#include <chrono>


#include "hybrid_astar/constants.h"
#include "hybrid_astar/helper.h"
#include "hybrid_astar/collisiondetection.h"
#include "hybrid_astar/algorithm.h"
#include "hybrid_astar/node4d.h"
#include "hybrid_astar/path.h"
#include "hybrid_astar/smoother.h"
#include "hybrid_astar/visualize.h"

#include "voxblox_ros/esdf_server.h"
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

enum PlanningState{
     UNINIT,
     NEWPLAN,
     REPLAN,
     EXECUTE,
     GETGOAL
};


class online_motion_planner{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher  planned_traj_pub;
    ros::Publisher  model_state_pub;
    ros::Publisher  traj_pub;
    ros::Subscriber point_cloud_sub;
    ros::Subscriber robot_pose_sub;
    ros::Subscriber ctrl_ref_sub;
    ros::Subscriber goal_sub;

    ros::ServiceClient holdOnClient;

    ros::Timer local_planning_timer;
    ros::Rate rate_;

    geometry_msgs::PoseStamped current_pose;
    visualization_msgs::Marker ctrl_ref;
    geometry_msgs::PoseStamped ctrl_ref_pose;


    Eigen::Vector4d start;
    Eigen::Vector4d goal;  //destination
    Eigen::Vector4d traj_end;  //destination


    PlanningState planning_state_;

//========================important object======================================

    HybridAStar::Path path;
    /// The smoother used for optimizing the path
    HybridAStar::Smoother smoother;
    /// The path smoothed and ready for the controller
    HybridAStar::Path smoothedPath = HybridAStar::Path(true);
    /// The visualization used for search visualization
    HybridAStar::Visualize visualization;
    /// The collission detection for testing specific configurations
//    std::shared_ptr<HybridAStar::CollisionDetection> configurationSpace;
    HybridAStar::CollisionDetection configurationSpace;
    HybridAStar::Algorithm hybridAstarPlanner;

    ros::Time  traj_start_time;
    ros::Time  last_plan_time;

    uuv_control_msgs::Trajectory planned_traj;


    geometry_msgs::PoseStamped new_goal_pose;
    geometry_msgs::PoseStamped next_start;
    bool GoalRecevied,StartRecevied;
    bool new_goal=false;

    std::shared_ptr<voxblox::EsdfServer> esdf_server_;

//===================================Parameters===================================
    std::string robot_name_;
    float planning_horizon_;
    float replan_horizon_;
    float replan_time_horizon_;  // second
    float planning_resolution_;
    float radius_min_;
    int   astar_iteration_;
    float step_min_;
    bool is_z_axis_considered_goal_check_;

    float bound_box_length_;
    float bound_box_width_;
    float bound_box_height_;
    float collision_distance_;
    float v_max_;
    float a_max_;
    float dt_;

// data record
    std::ofstream record_file;
//===================================private function===================================

    void initPlanner();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void refCallback(const visualization_msgs::MarkerConstPtr & msg);
    void planning_loop(const ros::TimerEvent& event);
    bool trajCollisionDetection();
    bool ReachGoal();
    bool trajReplanHorizon();
    void setStart(geometry_msgs::PoseStamped point);
    void setGoal(geometry_msgs::PoseStamped point);
    void pub_planned_traj(std::vector<HybridAStar::Node4D> path_nodes,ros::Time traj_start_time,float curr_vel);
    void holdOn();
    geometry_msgs::PoseStamped getTrajPointFromTime(ros::Time current_time);
    double getVelFromTime(ros::Time current_time);
    
    bool planMotion(bool isReplan=false);



    void writeToCSV( const double& costs);
public:
    online_motion_planner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~online_motion_planner(){
        record_file.close();
        std::cout<<"~~~online_motion_planner"<<std::endl;
    }
    void closeCSV(){
        record_file.close();
    }

};


#endif //ONLINE_MOTION_PLANNER

