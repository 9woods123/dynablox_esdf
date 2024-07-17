#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;
typedef ompl::base::SE3StateSpace::StateType SE3State;
typedef ompl::base::SO3StateSpace::StateType SO3State;

#include "hybrid_astar/node4d.h"
#include "hybrid_astar/node3d.h"

#include "hybrid_astar/visualize.h"
#include "hybrid_astar/collisiondetection.h"
#include <unordered_map>
#include <boost/heap/binomial_heap.hpp>
#define SQ(x) ((x)*(x))


namespace HybridAStar {
class Node4D;
class Node3D;

class Visualize;


struct CompareNodes {
        /// Sorting 3D nodes by increasing C value - the total estimated cost
bool operator()(const Node4D* lhs, const Node4D* rhs) const {
    return lhs->getC() > rhs->getC();}

bool operator()(const Node3D* lhs, const Node3D* rhs) const {
        return lhs->getC() > rhs->getC();}

};


template <typename T> struct matrix_hash : std::unary_function<T, size_t> {
        std::size_t operator()(T const& matrix) const {
            size_t seed = 0;
            for (size_t i = 0; i < matrix.size(); ++i) {
                auto elem = *(matrix.data() + i);
                seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
                        (seed >> 2);
            }
            return seed;
        }
    };



typedef boost::heap::binomial_heap<Node4D*,boost::heap::compare<CompareNodes>> priorityQueue;
typedef boost::heap::binomial_heap<Node3D*,boost::heap::compare<CompareNodes>> priorityQueue3d;



class Algorithm {



private:

    float search_horizon=45;
    float radius_min_=10;
    float planning_resolution=5;
    float search_horizon3d=45;
    bool  threeD_hybrid=false;
    float planning_step;
    bool  is_z_axis_considered_goal_check=true;

    priorityQueue Open;         // This list realize the compared insertion
    // update h value
    std::unordered_map<Eigen::Vector4i, Node4D*, matrix_hash<Eigen::Vector4i>> extend_nodes;


    priorityQueue3d Open3d;         // This list realize the compared insertion
    // update h value
    std::unordered_map<Eigen::Vector3i, Node3D*, matrix_hash<Eigen::Vector3i>> extend_nodes3d;

public:
  Algorithm() {}



  Node4D* hybridAStar(Node4D& start,
                             const Node4D& goal,
                             CollisionDetection& configurationSpace,
                             Visualize& visualization);

  void updateH(Node4D& start, const Node4D& goal,CollisionDetection& configurationSpace,
               Visualize& visualization);


  bool GotGoal(Node4D& current, const Node4D& goal);


  Eigen::Vector4i NodeToIndex(Node4D node);

  Eigen::Vector3i NodeToIndex3d(Node3D node);
  void setResolution(float resolution){planning_resolution=resolution;}
  void setSearchHorizon(float horizon){search_horizon=horizon;}
  void setMinStep(float step){planning_step=step;  HybridAStar::Node4D::setMinStep(step);}
  void setMinRadius(float r){ radius_min_=r; HybridAStar::Node4D::setMinRadius(radius_min_);}
  void setGoalCheck(bool signal){is_z_axis_considered_goal_check=signal;}

};
}
#endif // ALGORITHM_H
