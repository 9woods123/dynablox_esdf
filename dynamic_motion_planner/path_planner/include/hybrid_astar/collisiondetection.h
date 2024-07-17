#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "hybrid_astar/constants.h"
#include "hybrid_astar/node4d.h"
#include "hybrid_astar/node3d.h"

#include <Eigen/Geometry>
#include "voxblox_ros/esdf_server.h"


using namespace Eigen;
using namespace octomap;


namespace HybridAStar {

/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/

        enum VoxelState {
            unknown,
            occupied,
            free
        };

        class CollisionDetection {

        private:


            std::shared_ptr<voxblox::EsdfServer> esdf_server_;
            // params
            float bound_box_length_;
            float bound_box_width_;
            float bound_box_height_;
            float collision_distance_;

            std::vector<Eigen::Vector3d> obstacle_list;
            float obstacle_raduis;

        public:
            /// Constructor
            CollisionDetection();
            void initConfiguration(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                                         const double map_resolution, const double occur_threshold,
                                         const double free_threshold);
            void initObstacleList(const HybridAStar::Node4D *start, float search_horizon);
            bool isTraversable(const Node4D *node);
            bool isTraversable(const Node3D *node);

            void updateMapByOctomapPtr(octomap::ColorOcTree *map);

            void SetMapPtr(voxblox::EsdfServer *esdf_server);

            void getMapSize();

            bool getDistanceAndGradientAtPosition
            (const Eigen::Vector3d &point,double *distance,Eigen::Vector3d *gradient) const;

            bool getTargetObstacleGradient
                    (const Eigen::Vector3d &point,Eigen::Vector3d *gradient) const;

            VoxelState checkVoxelState(Eigen::Vector3d point);


            void setBoundBox(float lenght,float width,float height){
                bound_box_length_=lenght;
                bound_box_width_=width;
                bound_box_height_=height;}
            void setCollisionDistance(float collision_distance){
                collision_distance_=collision_distance;
            }

        };
    }
#endif // COLLISIONDETECTION_H
