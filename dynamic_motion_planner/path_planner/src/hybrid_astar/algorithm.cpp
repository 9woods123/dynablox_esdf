#include "hybrid_astar/algorithm.h"



using namespace HybridAStar;



bool Algorithm::GotGoal(Node4D& current, const Node4D& goal){


    Eigen::Vector4i goal_id=NodeToIndex(goal);
    Eigen::Vector4i curr_id=NodeToIndex(current);
    double e_dis;

    if(is_z_axis_considered_goal_check)
    {
      int dist_x=abs(goal_id[0]-curr_id[0]);
      int dist_y=abs(goal_id[1]-curr_id[1]);
      int dist_z=abs(goal_id[2]-curr_id[2]);
      double yaw_dis=std::abs(goal_id[3]-curr_id[3]);
      yaw_dis=std::min(yaw_dis,(2*M_PI-yaw_dis));
    
      int dist_threold=3;

      if((dist_x<=dist_threold)&&
      (dist_y<=dist_threold)&&
      (dist_z<=dist_threold)&&
      (yaw_dis<=0)
      ){
        std::cout<<"HYbridAstar GotGoal"<<std::endl;
        return true;
      }
      else{
        return false;
      }

    }
    else
    {
      int dist_x=abs(goal_id[0]-curr_id[0]);
      int dist_y=abs(goal_id[1]-curr_id[1]);
      int dist_z=abs(goal_id[2]-curr_id[2]);
      int dist_threold=3;

      if((dist_x<=dist_threold)&&(dist_y<=dist_threold)){
        std::cout<<"HYbridAstar GotGoal"<<std::endl;
        return true;
      }
      else{
        return false;
      }

    }


//    std::cout<<"goal=="<<goal.getX()<<", "<<goal.getY()<<", "<<goal.getZ()<<", "<<goal.getT()/M_PI<<" pi"<<std::endl;
//    std::cout<<"current=="<<current.getX()<<", "<<current.getY()<<", "<<current.getZ()<<", "<<current.getT()/M_PI<<" pi"<<std::endl;
//    std::cout<<"current. G=="<<current.getG()<<"    current. H=="<<current.getH();
//    std::cout<<"    e_dis=="<<e_dis<<"  yaw_dis=="<<yaw_dis/M_PI<<" pi "<<std::endl;
// debug goal 1.99 pi, curr 0.05 pi, the yaw dis=1.94pi , however, we exc



//     if(is_z_axis_considered_goal_check)
//     {
//         e_dis=std::sqrt(SQ(goal_id[0]-curr_id[0])+SQ(goal_id[1]-curr_id[1])+SQ(goal_id[2]-curr_id[2]));
//     }
//     else
//     {
//         e_dis=std::sqrt(SQ(goal_id[0]-curr_id[0])+SQ(goal_id[1]-curr_id[1]));
//     }

//     double yaw_dis=std::abs(goal_id[3]-curr_id[3]);
//     yaw_dis=std::min(yaw_dis,(2*M_PI-yaw_dis));

// //    std::cout<<"goal=="<<goal.getX()<<", "<<goal.getY()<<", "<<goal.getZ()<<", "<<goal.getT()/M_PI<<" pi"<<std::endl;
// //    std::cout<<"current=="<<current.getX()<<", "<<current.getY()<<", "<<current.getZ()<<", "<<current.getT()/M_PI<<" pi"<<std::endl;
// //    std::cout<<"current. G=="<<current.getG()<<"    current. H=="<<current.getH();
// //    std::cout<<"    e_dis=="<<e_dis<<"  yaw_dis=="<<yaw_dis/M_PI<<" pi "<<std::endl;
// // debug goal 1.99 pi, curr 0.05 pi, the yaw dis=1.94pi , however, we exc



//     if(e_dis>4*planning_step/planning_resolution||yaw_dis>0.25*M_PI)
//     {
//         return false;
//     }
//     else
//     {
//         std::cout<<"HYbridAstar GotGoal"<<std::endl;
//         return true;
//     }

}

void debug_open(boost::heap::binomial_heap<Node4D*,
        boost::heap::compare<CompareNodes>> open)
{
    std::cout << "========debug openlist========"<< std::endl;

    std::cout <<"open size=="<<open.size()<<std::endl;

    for (auto it = open.begin(); it != open.end(); ++it)
    {
        Node4D* node = *it;
        // 在此处处理每个节点
        // 可以访问节点的成员变量或调用节点的成员函数
        // 例如，输出节点的状态信息
//        std::cout << "Node= ("<<node->getX()<<","<<node->getY()<<","<<node->getZ()<<")   "<<"Node getC=="
//        << node->getC() << std::endl;
    }
}


void debug_map(std::unordered_map<Eigen::Vector3i, Node4D*, matrix_hash<Eigen::Vector3i>> e_nodes)
{
    std::cout<<" ============debug map==============="<<std::endl;
    std::cout<<" ==e_nodes size"<<e_nodes.size()<<std::endl;

//    for (auto node:e_nodes)
//    {
////    std::cout<<"  "<<node.first<<"        "<<node.second<<""<<std::endl;
//    std::cout<<"  "<<node.second->getC()<<""<<std::endl;
//
//    }

}

Eigen::Vector4i Algorithm::NodeToIndex(Node4D node)
{
  Eigen::Vector4d pt(node.getX(),node.getY(),node.getZ(),node.getT());
  Eigen::Vector4d  origin_(0,0,0,0);
  float inv_resolution_=1/planning_resolution;
 
  pt.z()=pt.z()*5;   //  the z-axis should be more dense, as the primitive motion along z-axis is smaller
                                  //  sometimes cant cross through the index with a big resolution.
  // pt.w()=pt.w()*5;
  // =================woods====================
  Eigen::Vector4i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  return idx;
}


Eigen::Vector3i Algorithm::NodeToIndex3d(HybridAStar::Node3D node) {

    int res=5;
    Eigen::Vector3d pt(node.getX(),node.getY(),node.getZ());
    Eigen::Vector3d  origin_(0,0,0);
    float inv_resolution_=0.2;
    Eigen::Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
    return idx;

}

Node4D* Algorithm::hybridAStar(Node4D& start,
                             const Node4D& goal,
                             CollisionDetection& configurationSpace,
                             Visualize& visualization)
{

  // update local obstacle list of configurationSpace
  // save computing cost by avoiding
  // search obstacle multi times.
//   configurationSpace.initObstacleList(&start,search_horizon);
  /////////////////////////////////////////


  // PREDECESSOR AND SUCCESSOR INDEX
  Eigen::Vector4i iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = Constants::reverse ? (2*Node4D::dir) : Node4D::dir;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;

  // VISUALIZATION DELAY
  ros::Duration d(0.2);

  // OPEN LIST AS BOOST IMPLEMENTATION

  Open.clear();
  extend_nodes.clear();
  extend_nodes3d.clear();
  //  std::unordered_map<Eigen::Vector3i, Node4D> extend_nodes2;
  updateH(start, goal,configurationSpace, visualization);

  // INIT mark start as open  and push on priority queue aka open list. ==============
  start.open();
  Open.push(&start);
  extend_nodes.emplace(NodeToIndex(start), &start);

  // NODE POINTER
  Node4D* nPred;
  Node4D* nSucc;

  // float max = 0.f;

  // continue until O empty
  while (!Open.empty()) {


    iterations++;
    nPred = Open.top();
    iPred = NodeToIndex(*nPred);


    if (Constants::visualization) {
          visualization.publishNode4DPoses(*nPred);
          // visualization.publishNode4DPose(*nPred);
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if ( extend_nodes.find(iPred)->second->isClosed())
    {
      // pop node from the open list and start with a fresh node
      Open.pop();
      continue;
    }

      // _________________
    // EXPANSION OF NODE
    else if (extend_nodes.find(iPred)->second->isOpen()) {
      

      //std::cout<<"===EXPANSION OF NODE=="<<std::endl;
      // remove node from open list
      Open.pop();
      // add node to closed list
      extend_nodes.find(iPred)->second->close();

      // ================================================================================

      bool reach_horizon = nPred->getG()>=search_horizon;

      if (*nPred == goal || GotGoal (*nPred,goal)|| iterations > Constants::iterations||reach_horizon) {
          return nPred;
      }
      else {   

    //  std::vector<Node4D*> neibors=nPred->generateNeibors();

    // for (int i =0;i<dir;i++)
    //     {
        // std::cout<<" ============Node4D::generateNeibors()=============="<<std::endl;

        for (int i =0;i<dir;i++)
        {
          nSucc=nPred->generateNeibors(i);
        // nSucc=nPred->createSuccessor(i);
        // nSucc->updateG();

          // std::cout<<" Cost:"<< nSucc->getC()<<std::endl;
        // for( auto& nSucc : neibors ){
// 
          // set index of the successor
          iSucc = NodeToIndex(*nSucc);
          // ensure successor is on grid and traversable
          if (configurationSpace.isTraversable(nSucc)) {
          // ensure successor is not on closed list or it has the same index as the predecessor

            if (    (extend_nodes.find(iSucc)==extend_nodes.end())  // TRUE: the node is not found in extend nodes
                ||  !extend_nodes.find(iSucc)->second->isClosed()   //  means that it is a fresh node
                ||   iPred == iSucc)
            {

              extend_nodes.emplace(iSucc, nSucc);  // the unextended node become extended.

              // nSucc->updateG();// calculate new G value

              newG = nSucc->getG();
              // if successor not on open list or found a shorter way to the cell
              if (! extend_nodes.find(iSucc)->second->isOpen()
                 || newG < extend_nodes.find(iSucc)->second->getG()
                 || iPred == iSucc)
              {
                  // calculate H value
                updateH(*nSucc, goal,configurationSpace,visualization);

                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                  delete nSucc;
                  continue;
                }

                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                    nSucc->setPred(nPred->getPred());  //in the createSuccessor(), nSucc->pred are set to be nPred.
                }

                if (nSucc->getPred() == nSucc) {
                  std::cout << "looping";
                }

                // put successor on open list
                nSucc->open();
                extend_nodes.find(iSucc)->second=nSucc;
                Open.push(nSucc);


              } else {
                  
                  delete nSucc;
              }
            } else {

                delete nSucc;
                }
          } else {

               delete nSucc;
          }
        }
      }
    }
  }

  if (Open.empty()) {
    // std::cout<<" Open is empty"<<std::endl;
    //TDOO test
    return nullptr;

  }

  return nullptr;
}


void Algorithm::updateH(Node4D& start, const Node4D& goal,CollisionDetection& configurationSpace, Visualize& visualization)
{

  // double distance=sqrt((start.getX()-goal.getX())*(start.getX()-goal.getX())+(start.getY()-goal.getY())*(start.getY()-goal.getY())+
  // (start.getZ()-goal.getZ())*(start.getZ()-goal.getZ()));

  // start.setH(distance);
  // return;

  double dubinsCost = 0;
  double reedsSheppCost = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (!Constants::reverse&&Constants::dubins) {

    ompl::base::DubinsStateSpace dubinsPath(radius_min_);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();

    double dbStart_yaw = start.getT(); // 设置航向角
    dbStart->setXY(start.getX(),start.getY());
    dbStart->setYaw(dbStart_yaw);

    double dbEnd_yaw = goal.getT(); // 设置航向角
    dbEnd->setXY(goal.getX(),goal.getY());
    dbEnd->setYaw(dbEnd_yaw);
    float twoDdubinsCost = dubinsPath.distance(dbStart, dbEnd);
    dubinsCost=sqrt(twoDdubinsCost*twoDdubinsCost +(start.getZ()-goal.getZ())*(start.getZ()-goal.getZ()) );
    // dubinsCost+=double(std::abs(start.getZ()-goal.getZ()));
  }

  // if reversing is active use a

  if (Constants::reverse && !Constants::dubins) {

      ompl::base::ReedsSheppStateSpace reedsSheppPath(radius_min_);
      State* rsStart = (State*)reedsSheppPath.allocState();
      State* rsEnd = (State*)reedsSheppPath.allocState();
      rsStart->setXY(start.getX(), start.getY());
      rsStart->setYaw(start.getT());
      rsEnd->setXY(goal.getX(), goal.getY());
      rsEnd->setYaw(goal.getT());
      reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
      reedsSheppCost+=std::abs(start.getZ()-goal.getZ());

  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, dubinsCost));
  // std::cout<<"dubinsCost:  "<<dubinsCost<<std::endl;
}


