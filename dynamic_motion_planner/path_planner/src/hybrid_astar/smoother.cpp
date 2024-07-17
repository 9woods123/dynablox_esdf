#include "hybrid_astar/smoother.h"

using namespace HybridAStar;


void Smoother::debugPath() {

    for(auto point:fourDpath)
    {
        std::cout<<"point== ("<<point.getX()<<","<<point.getY()<<","
        <<point.getZ()<<")  "<<std::endl;
    }

}

Eigen::Vector3d
Smoother::targetobstacleTerm(Eigen::Vector3d xi,
                             const HybridAStar::CollisionDetection *configurationSpace) {

    Eigen::Vector3d targetobstacleGradient;
    if(configurationSpace->getTargetObstacleGradient(xi,&targetobstacleGradient))
    {
        return -wtargetObstacle*targetobstacleGradient;
    }
    return Eigen::Vector3d(0,0,0);
}


Eigen::Vector3d Smoother::obstacleTerm(Eigen::Vector3d xi,
                                       const HybridAStar::CollisionDetection *configurationSpace, double& obstacle_cost){

    // TODO add a gradient calculated by considering the targets.
    obstacle_cost=0;
    Eigen::Vector3d obstacleGradient;
    double d_threshold=collision_safe_distance;
    double distance;
    if(configurationSpace->
            getDistanceAndGradientAtPosition(xi, &distance,&obstacleGradient))
    {
        if(distance<d_threshold)
        {
            obstacle_cost+=(distance-d_threshold)*(distance-d_threshold);
            return -wObstacle*obstacleGradient;
        }

    }

    return Eigen::Vector3d(0,0,0);
}



Eigen::Vector3d Smoother::smoothnessTerm(Eigen::Vector3d xim3, Eigen::Vector3d xim2, Eigen::Vector3d xim1,
                                         Eigen::Vector3d xi, Eigen::Vector3d xip1, Eigen::Vector3d xip2,
                                         Eigen::Vector3d xip3, double& smooth_cost) {

    Eigen::Vector3d gradient= wAccSmooth * (-xim3+ 6*xim2 - 15*xim1 + 20*xi - 15*xip1 + 6*xip2 -xip3)+
                              wVelSmooth * (xim2 - 4 * xim1 + 6 * xi - 4 * xip1 + xip2);
                              
    smooth_cost=((xip2-xip1)-(xip1-xi)).norm();

    return gradient;

}



void  Smoother::tracePath(const Node4D* node, int i, std::vector<Node4D> path )
{
    if (node == nullptr) {
        std::reverse(path.begin(),path.end());
        this->fourDpath =path;
        return;
    }

    i++;
    path.push_back(*node);
    tracePath(node->getPred(), i, path);
}


bool Smoother::getPolynomialTraj(const mav_msgs::EigenTrajectoryPoint::Vector& waypoints, 
    mav_trajectory_generation::Trajectory* trajectory,std::vector<double> segment_times) 
 {

 if (waypoints.size() < 2) {
    return false;
  }

  constexpr int N = 10;
  constexpr int D = 3;
  mav_trajectory_generation::PolynomialOptimization<N> poly_opt(D);

  int num_vertices = waypoints.size();

  int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::ACCELERATION;

  mav_trajectory_generation::Vertex::Vector vertices(
      num_vertices, mav_trajectory_generation::Vertex(D));


  vertices.front().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      waypoints.front().position_W);
  vertices.front().addConstraint(
            mav_trajectory_generation::derivative_order::VELOCITY,
            waypoints.front().velocity_W);
 //   TODO the  VELOCITY should be  

  vertices.back().makeStartOrEnd(0, derivative_to_optimize);
  vertices.back().addConstraint(
      mav_trajectory_generation::derivative_order::POSITION,
      waypoints.back().position_W);

  // Now do the middle bits.
  size_t j = 1;
  for (size_t i = 1; i < waypoints.size() - 1; i += 1) {
    vertices[j].addConstraint(
        mav_trajectory_generation::derivative_order::POSITION,
        waypoints[i].position_W);
    j++;
  }


  poly_opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  if (poly_opt.solveLinear()) {
    poly_opt.getTrajectory(trajectory);
  } else {
    return false;
  }
  return true;

 }



std::vector<double> Smoother:: getSegmentTimes(std::vector<Node4D>* path, double v_init,double v_max, double a_max)
{
        // Calculate the segments times by assuming that  
        // accelerate at maximum acceleration, and  maintain a constant speed 
        // at v_max
//        std::cout<<"v_init="<<v_init<<" v_max="<<v_max<<" a_max="<<a_max<<std::endl;
//        std::cout<<"delta_times== ";

        std::vector<double> times;
        double v_last=v_init;
        for (int i=0; i<path->size()-1;i++)
        {
            double delta_time=0;
            double min_acc_time( (v_max-v_last)/a_max  );
            double distance_acc= 
            v_last*min_acc_time+0.5* a_max*min_acc_time*min_acc_time;
            double step_distance=std::sqrt(
                                  SQ(path->at(i+1).getX()-path->at(i).getX())
                                 +SQ(path->at(i+1).getY()-path->at(i).getY())
                                 +SQ(path->at(i+1).getZ()-path->at(i).getZ()));


            if (distance_acc<step_distance)
            {
                delta_time=min_acc_time+(step_distance-distance_acc)/v_max;
                v_last=v_max;
                //index= (index==0) ? i : index;
            }
            else
            {
                double a=a_max/2; double b=v_last; double c=-step_distance;
                delta_time= (-b+std::sqrt(b*b-4*a*c) )/ (2 * a);
                v_last=v_last+delta_time*a_max;
                if(v_last>v_max){v_last=v_max;}
            }
//            std::cout<<" d_time: "<<delta_time<<std::endl;
            times.push_back(delta_time);
        }

    return times;
}

 bool Smoother::smooth(const HybridAStar::CollisionDetection *configurationSpace,double d_t,
                       double v_init ,double v_max, double a_max) {

     auto start = std::chrono::high_resolution_clock::now();

     int iterations = 0;
     // the maximum iterations for the gd smoother
     int maxIterations = 50;
     // the lenght of the path in number of nodes
     int pathLength = fourDpath.size();
     if(pathLength<5){ return false;}
     std::vector<Node4D> newPath = fourDpath;

     // ================== an  optimization calculating by adjust the wapypoint===================
     float totalWeight = wAccSmooth + wVelSmooth +  wObstacle + wtargetObstacle;
     std::vector<Node4D> temp_path=newPath;
     float total_cost=0;
    float delat_cost=99999;

     while (iterations < maxIterations ) {
         // choose the first three nodes of the path
        double smooth_cost=0;
        double obstacle_cost=0;
        total_cost=0;
         for (int i = 3; i < pathLength - 3; ++i) {
             // x i minus 2 x_(i+2)

             Eigen::Vector3d xim3(newPath[i - 3].getX(), newPath[i - 3].getY(), newPath[i - 3].getZ());
             Eigen::Vector3d xim2(newPath[i - 2].getX(), newPath[i - 2].getY(),newPath[i - 2].getZ());
             Eigen::Vector3d xim1(newPath[i - 1].getX(), newPath[i - 1].getY(),newPath[i - 1].getZ());
             Eigen::Vector3d xi(newPath[i].getX(), newPath[i].getY(),newPath[i].getZ());
             Eigen::Vector3d xip1(newPath[i + 1].getX(), newPath[i + 1].getY(),newPath[i + 1].getZ());
             Eigen::Vector3d xip2(newPath[i + 2].getX(), newPath[i + 2].getY(),newPath[i + 2].getZ());
             Eigen::Vector3d xip3(newPath[i + 3].getX(), newPath[i + 3].getY(), newPath[i + 3].getZ());

            Eigen::Vector3d correction(0,0,0);
            correction = correction + smoothnessTerm(xim3, xim2, xim1, xi, xip1, xip2, xip3,smooth_cost);
            correction = correction + obstacleTerm(xi,configurationSpace,obstacle_cost);
            //  correction = correction + targetobstacleTerm(xi,configurationSpace);

             //  gradient descent
             xi = xi - alpha * correction/totalWeight;
             total_cost+= smooth_cost/totalWeight+obstacle_cost/totalWeight;

             temp_path[i].setX(xi[0]);
             temp_path[i].setY(xi[1]);
             temp_path[i].setZ(xi[2]);
         }

        // std::cout<<"total_cost= "<<total_cost<<std::endl;
         newPath=temp_path;
         iterations++;
     }
     auto end_opt = std::chrono::high_resolution_clock::now();

     std::chrono::duration<double, std::milli> opt_cost = end_opt - start;
    //  std::cout << " optimer cost==" << opt_cost.count() << " ms" << std::endl;

     //================== an  optimization calculating by adjust the wapypoint===================


     //====================== using  minimum snap to get a Polynomial Traj .======================
     std::vector<double> segment_times= Smoother::getSegmentTimes(&newPath,  v_init, v_max,  a_max);
//     std::vector<double> segment_times= Smoother::adjustPathTime(newPath,  v_init, v_max,  a_max);


     mav_msgs::EigenTrajectoryPoint::Vector waypoints;
     for(auto point:newPath)
     {
         mav_msgs::EigenTrajectoryPoint wp;
         wp.position_W<<point.getX(),point.getY(),point.getZ();
         waypoints.push_back(wp);
     }

     mav_trajectory_generation::Trajectory  trajectory;
     bool success=getPolynomialTraj(waypoints,&trajectory,segment_times);
     mav_msgs::EigenTrajectoryPointVector path;
     if (success) {
         mav_trajectory_generation::sampleWholeTrajectory(trajectory, d_t, &path);
     }
    //====================== using  minimum snap to get a Polynomial Traj .======================

    //=======================  adjust trajectory yaw ==============================
    std::vector<Node4D> finalPath;
    for(auto wp:path)
    {
        Node4D newnode;
        newnode.setX(wp.position_W[0]);
        newnode.setY(wp.position_W[1]);
        newnode.setZ(wp.position_W[2]);
        finalPath.push_back(newnode);
    }

    finalPath.back().setT(fourDpath.back().getT());

    for (int i = 0; i < finalPath.size()-1 ; ++i)
    {
        Eigen::Vector3d xi(finalPath[i].getX(), finalPath[i].getY(), finalPath[i].getZ());
        Eigen::Vector3d xip1(finalPath[i+1].getX(), finalPath[i+1].getY(), finalPath[i+1].getZ());
        Eigen::Vector3d Dxi = xip1 - xi;
        finalPath[i].setT(std::atan2(Dxi.y(), Dxi.x()));
    }
     fourDpath = finalPath;
    //=======================  adjust trajectory yaw ==============================

     auto end = std::chrono::high_resolution_clock::now();
     std::chrono::duration<double, std::milli> elapsed = end - start;
     std::cout << "trajectory optimer cost==" << elapsed.count() << " ms" << std::endl;
     return true;

 }

inline float  Smoother::getDistance(Eigen::Vector3d from, Eigen::Vector3d to) {
    return (from-to).norm();
}
