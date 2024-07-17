#ifndef NODE4D_H
#define NODE4D_H

#include <cmath>
#include <iostream>
#include <vector>

#include "hybrid_astar/constants.h"
#include "hybrid_astar/helper.h"

namespace HybridAStar {
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.

   Each node has a unique configuration (x, y,z, theta) in the configuration space C.
*/
enum class Direction {
    Forward = 0,
    Reverse = 1,
};

class Node4D {

private:
    /// the x position
    float x;
    /// the y position
    float y;
    /// the z position
    float z;
    /// the heading theta
    float t;
    /// the cost-so-far
    double g;
    /// the cost-to-go
    double h;
    /// the index of the node in the 3D array
    int idx;
    /// the open value
    bool o;
    /// the closed value
    bool c;
    /// the motion primitive of the node
    int prim;
    /// the predecessor pointer
    Direction  dircetion;
    
    const Node4D* pred;



 public:

  /// The default constructor for 3D array initialization
  Node4D(): Node4D(0, 0, 0,0, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  Node4D(float x, float y, float z,float t, double g, double h, const Node4D* pred, int prim = 0, Direction dir=Direction::Forward) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->t =Helper::normalizeHeadingRad(t);
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->prim = prim;
    this->dircetion=dir;
  }

  // GETTER METHODS
  /// get the x position
  float getX() const { return x; }
  /// get the y position
  float getY() const { return y; }
  /// get the z position
  float getZ() const { return z; }
  /// get the heading theta
  float getT() const { return t; }
  /// get the cost-so-far (real value)
  double getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  double getH() const { return h; }
  /// get the total estimated cost
  double getC() const { return g + h; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx; }
  /// get the number associated with the motion primitive of the node
  int getPrim() const { return prim; }
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  /// determine whether the node is open
  const Node4D* getPred() const { return pred; }

  // SETTER METHODS
  /// set the x position
  void setX(const float& x) { this->x = x; }
  /// set the y position
  void setY(const float& y) { this->y = y; }
  /// set the z position
  void setZ(const float& z) { this->z = z; }
  /// set the heading theta
  void setT(const float& t) { this->t = Helper::normalizeHeadingRad(t);; }
  /// set the cost-so-far (real value)
  void setG(const double& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const double& h) { this->h = h; }
  /// open the node
  void open() { o = true; c = false;}
  /// close the node
  void close() { c = true; o = false; }
  /// set a pointer to the predecessor of the node
  void setPred(const Node4D* pred) { this->pred = pred; }

  // UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
  void updateG();
  double calculateG(float delta_x, float delta_y, float delta_z);
  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
  bool operator == (const Node4D& rhs) const;


  // SUCCESSOR CREATION
  /// Creates a successor in the continous space.
  Node4D* createSuccessor(const int i);

  Node4D*generateNeibors(const int input_index);

  static float radius_min;
  static float step_min;

  /// Number of possible directions
  static const int dir;
  /// Possible movements in the x direction
  static  float dx[];
  /// Possible movements in the y direction
  static  float dy[];
  /// Possible movements in the z direction
  static  float dz[];
  /// Possible movements regarding heading theta
  static  float dt[];

  static void setMinRadius(float radius_min_) {// 静态成员函数用于设置参数值
        radius_min = radius_min_;}
  static void setMotionPrims() ;
  static void setMinStep(float step){step_min=step;
  std::cout <<"step_min=="<<step_min<<std::endl;}

};
}
#endif // NODE4D_H
