//
// Created by woods on 2023/6/8.
//

#ifndef LOCAL_PLANNER_WS_NODE3D_H
#define LOCAL_PLANNER_WS_NODE3D_H



#include <cmath>
#include "iostream"

#include "constants.h"
namespace HybridAStar {

/*!
   \brief A two dimensional node class used for the holonomic with obstacles heuristic.

   Each node has a unique discrete position (x,y).
*/
    class Node3D {
    public:
        /// The default constructor for 2D array initialization.
        Node3D(): Node3D(0, 0, 0,0, 0, nullptr) {


    }
        /// Constructor for a node with the given arguments
        Node3D(int x, int y, int z,float g, float h, Node3D* pred) {
            this->x = x;
            this->y = y;
            this->z=  z;
            this->g = g;
            this->h = h;
            this->pred = pred;
            this->o = false;
            this->c = false;
            this->d = false;
            this->idx = -1;
        }
        // GETTER METHODS
        /// get the x position
        int getX() const { return x; }
        /// get the y position
        int getY() const { return y; }

        int getZ() const { return z; }

        /// get the cost-so-far (real value)
        float getG() const { return g; }
        /// get the cost-to-come (heuristic value)
        float getH() const { return h; }
        /// get the total estimated cost
        float getC() const { return g + h; }
        /// get the index of the node in the 2D array
        int getIdx() const { return idx; }
        /// determine whether the node is open
        bool  isOpen() const { return o; }
        /// determine whether the node is closed
        bool  isClosed() const { return c; }
        /// determine whether the node is discovered
        bool  isDiscovered() const { return d; }
        /// get a pointer to the predecessor
        Node3D* getPred() const { return pred; }

        // SETTER METHODS
        /// set the x position
        void setX(const int& x) { this->x = x; }
        /// set the y position
        void setY(const int& y) { this->y = y; }
        /// set the z position
        void setZ(const int& z) { this->z = z; }
        /// set the cost-so-far (real value)
        void setG(const float& g) { this->g = g; }
        /// set the cost-to-come (heuristic value)
        void setH(const float& h) { this->h = h; }
        /// open the node
        void open() { o = true; c = false; }
        /// close the node
        void close() { c = true; o = false; }
        /// set the node neither open nor closed
        void reset() { c = false; o = false; }
        /// discover the node
        void discover() { d = true; }
        /// set a pointer to the predecessor of the node
        void setPred(Node3D* pred) { this->pred = pred; }

        // UPDATE METHODS
        /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
        void updateG() { g += movementCost(*pred); d = true; }
        /// Updates the cost-to-go for the node x' to the goal node.
        void updateH(const Node3D& goal) { h = movementCost(goal); }
        /// The heuristic as well as the cost measure.
        float movementCost(const Node3D& node) const {
            return sqrt((x - node.x) * (x - node.x) +
            (y - node.y) * (y - node.y)+
            (z - node.z) * (z - node.z)); }

        // CUSTOM OPERATORS
        /// Custom operator to compare nodes. Nodes are equal if their x and y position is the same.
        bool operator == (const Node3D& rhs) const;

        // GRID CHECKING
        /// Validity check to test, whether the node is in the 2D array.
        bool isOnGrid(const int width, const int height) const;

        // SUCCESSOR CREATION
        /// Creates a successor on a eight-connected grid.
        Node3D* createSuccessor(const int i);

        // CONSTANT VALUES
        /// Number of possible directions
        static const int dir;
        /// Possible movements in the x direction
        static const int dx[];
        /// Possible movements in the y direction
        static const int dy[];
        /// Possible movements in the z direction
        static const int dz[];
    private:
        /// the x position
        int x;
        /// the y position
        int y;
        /// the z position
        int z;
        /// the cost-so-far
        float g;
        /// the cost-to-go
        float h;
        /// the index of the node in the 2D array
        int idx;
        /// the open value
        bool o;
        /// the closed value
        bool c;
        /// the discovered value
        bool d;
        /// the predecessor pointer
        Node3D* pred;
    };
}











#endif //LOCAL_PLANNER_WS_NODE3D_H
