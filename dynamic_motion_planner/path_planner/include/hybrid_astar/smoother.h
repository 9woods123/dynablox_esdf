#ifndef SMOOTHER_H
#define SMOOTHER_H

#include <cmath>
#include <vector>

#include "hybrid_astar/node4d.h"
#include "hybrid_astar/helper.h"
#include "hybrid_astar/constants.h"
#include "hybrid_astar/collisiondetection.h"

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include <mav_msgs/eigen_mav_msgs.h>

#define SQ(x) ((x)*(x))

namespace HybridAStar {
/*!
   \brief This class takes a path object and smoothes the nodes of the path.

   It also uses the Voronoi diagram as well as the configuration space.
*/
    class Smoother {
    public:
        Smoother() {}


        /// obstacleCost - pushes the path away from obstacles
        Eigen::Vector3d obstacleTerm(Eigen::Vector3d xi,const HybridAStar::CollisionDetection *configurationSpace
        ,double& obstacle_cost);

        Eigen::Vector3d targetobstacleTerm(Eigen::Vector3d xi,const HybridAStar::CollisionDetection *configurationSpace);

        /// curvatureCost - forces a maximum curvature of 1/R along the path ensuring drivability
        Eigen::Vector3d curvatureTerm();

        /// smoothnessCost - attempts to spread nodes equidistantly and with the same orientation
        Eigen::Vector3d smoothnessTerm(Eigen::Vector3d xim3,Eigen::Vector3d xim2, Eigen::Vector3d xim1,
                                       Eigen::Vector3d xi,
                                       Eigen::Vector3d xip1, Eigen::Vector3d xip2,Eigen::Vector3d xip3, double& smooth_cost);

        float FeasibleTerm(Eigen::Vector3d xim2, Eigen::Vector3d xim1,
                                       Eigen::Vector3d xi,
                                       Eigen::Vector3d xip1, Eigen::Vector3d xip2,
                                       float t_im2,float t_im1,
                                       float t_i,
                                       float t_ip1,float t_ip2
                                       );

        std::vector<double> adjustPathTime(std::vector<Node4D> &path, double v_init , double v_max, double a_max);

        bool smooth(const HybridAStar::CollisionDetection *configurationSpace,double d_t,double v_init ,double v_max, double a_max);

        bool getPolynomialTraj(const mav_msgs::EigenTrajectoryPoint::Vector& waypoints,
                               mav_trajectory_generation::Trajectory* trajectory, std::vector<double> segment_times) ;

        std::vector<double>  getSegmentTimes(std::vector<Node4D>* path, double v_init , double v_max, double a_max);

        void tracePath(const Node4D* node, int i = 0, std::vector<Node4D> path = std::vector<Node4D>());

        /// returns the path of the smoother object
        const std::vector<Node4D>& get4DPath() {return fourDpath;}

        void debugPath();

        inline float getDistance(Eigen::Vector3d from,Eigen::Vector3d to);
        inline void setSafeDistance(float distance){
            collision_safe_distance=distance;}


    private:

        std::vector<Node4D> fourDpath;

        float collision_safe_distance=3.5;
        float alpha=0.1;
        float wObstacle = 0.05;
        float wtargetObstacle = 0.00;

        float wCurvature = 0.01;

        float wAccSmooth = 0.0;
        float wVelSmooth = 0.5;

    };
}
#endif // SMOOTHER_H
