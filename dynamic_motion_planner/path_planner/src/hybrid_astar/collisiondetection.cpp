#include "hybrid_astar/collisiondetection.h"

using namespace HybridAStar;



CollisionDetection::CollisionDetection() {
    esdf_server_= nullptr;
}

void CollisionDetection::initConfiguration(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                                       const double map_resolution, const double occur_threshold,
                                       const double free_threshold) {

    esdf_server_.reset(new voxblox::EsdfServer(nh, nh_private));   // begin an esdf ros process
    esdf_server_->setTraversabilityRadius(0.2);
    // tp_server_.reset(new tp_map::tp_map_server(nh,nh_private,0.1,0.25,0.85));
}


bool CollisionDetection::getDistanceAndGradientAtPosition(
        const Eigen::Vector3d &point, double *distance,Eigen::Vector3d *gradient) const{

    return esdf_server_->getEsdfMapPtr()->
    getDistanceAndGradientAtPosition(point,distance,gradient);

}

bool CollisionDetection::getTargetObstacleGradient(const Eigen::Vector3d &point, Eigen::Vector3d *gradient) const {

    *gradient = Eigen::Vector3d::Zero();
    float d_max=10;

    for (auto obstacle_position : obstacle_list) {

        // 计算障碍物对于输入点的影响
        double obstacle_distance = (point - obstacle_position).norm();

        if(obstacle_distance>d_max)
        {
            continue;
        }


        // 计算障碍物对梯度的贡献
        Eigen::Vector3d obstacle_gradient = (point - obstacle_position).normalized();
        obstacle_gradient *= 2*(obstacle_distance-d_max);

        // 将障碍物的梯度贡献累加到总梯度中
        *gradient += -obstacle_gradient;
    }
    return true;
}

void CollisionDetection::SetMapPtr(voxblox::EsdfServer *esdf_server) {
    esdf_server_.reset(esdf_server);
}

void CollisionDetection::getMapSize() {

    std::cout<<"block_size()==="<<
    esdf_server_->getEsdfMapPtr()->block_size()
    <<std::endl
    <<"NumberOfAllocatedBlocks()==="<<
    esdf_server_->getEsdfMapPtr()->getEsdfLayer().getNumberOfAllocatedBlocks()
    <<std::endl;

}

VoxelState CollisionDetection::checkVoxelState(Eigen::Vector3d point) {

    double distance = 0.0;
    double c_voxel_distance=collision_distance_;

    //    auto start = std::chrono::high_resolution_clock::now();

//    auto end = std::chrono::high_resolution_clock::now();
//    std::chrono::duration<double, std::milli> elapsed = end - start;
//    std::cout <<" o list="<<obstacle_list.size()<< " checkVoxelState  cost==" << elapsed.count() << " ms" << std::endl;

    if (esdf_server_->getEsdfMapPtr()->getDistanceAtPosition(point, &distance))
    {
        // This means the voxel is observed
        if (distance < c_voxel_distance)
        {
            return VoxelState::occupied;
        }
        else
        {
            return VoxelState::free;
        }
    }
    else
    {
        return VoxelState::unknown;
    }

}



bool CollisionDetection::isTraversable(const HybridAStar::Node4D *node){

    //  this function from chatgpt is error that it provides a solution calculating
    //  a bound box using octomap->begin_leafs_bbx(min,max);
    //  the bound box axis is fixed, two point are given to determine a cuboid
    //  but the min and max point are opposite, when rotation happens, the
    //  bound box are smaller than we imagined.


    float length=bound_box_length_;
    float width =bound_box_width_;
    float height=bound_box_height_;   //  TODO,   get params from launch


    float current_x=node->getX();
    float current_y=node->getY();
    float current_z=node->getZ();
    float current_t=node->getT();



    // Convert the yaw to a rotation matrix
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = Eigen::Vector3d(current_x, current_y, current_z);
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(current_t, Eigen::Vector3d::UnitZ());
    transform.linear() = rot;

    float resolution=esdf_server_->getEsdfMapPtr()->voxel_size();

    for (float x= -length/2.0; x<= length/2.0; x+=resolution)
    {

        for(float y= -width/2.0; y<=width/2.0; y+=resolution)
        {

            for (float z= -height/2.0; z<= height/2.0; z+=resolution)
            {
             Eigen::Vector3d point_in_bound_box =transform * Eigen::Vector3d(x, y, z);
             VoxelState point_state=checkVoxelState(point_in_bound_box);

             if(point_state==VoxelState::occupied)
             {
                 return false;
             }

            }

        }
    }


    return true;

}

bool CollisionDetection::isTraversable(const HybridAStar::Node3D *node){

    //  this function from chatgpt is error that it provides a solution calculating
    //  a bound box using octomap->begin_leafs_bbx(min,max);
    //  the bound box axis is fixed, two point are given to determine a cuboid
    //  but the min and max point are opposite, when rotation happens, the
    //  bound box are smaller than we imagined.


    float length=2;
    float width =1;
    float height=1;   //  TODO,   get params from launch


    float current_x=node->getX();
    float current_y=node->getY();
    float current_z=node->getZ();



    // Convert the yaw to a rotation matrix
    Eigen::Affine3d transform = Eigen::Affine3d::Identity();
    transform.translation() = Eigen::Vector3d(current_x, current_y, current_z);
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    transform.linear() = rot;

    float resolution=0.5;
    for (float x= -length/2.0; x<= length/2.0; x+=resolution)
    {

        for(float y= -width/2.0; y<=width/2.0; y+=resolution)
        {

            for (float z= -height/2.0; z<= height/2.0; z+=resolution)
            {
                Eigen::Vector3d point_in_bound_box =transform * Eigen::Vector3d(x, y, z);
                VoxelState point_state=checkVoxelState(point_in_bound_box);

                if(point_state==VoxelState::occupied)
                {
                    return false;
                }

            }

        }
    }

    return true;

}