//
// Created by woods on 2023/6/8.
//

#include "hybrid_astar/node3d.h"


using namespace HybridAStar;

const float step_size=2;
// possible directions
const int Node3D::dir = 27;
// possible movements

const int Node3D::dx[] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,0,0,
                          0,0,0,0,0,0,0,1,1,1,1,
                          1,1,1,1,1 };

const int Node3D::dy[] = { -1 ,-1 ,-1 ,0 ,0,0,1,1,1,-1,-1,
                          -1,0 ,0 ,0,1,1,1,-1,-1,-1,
                          0 ,0 ,0 ,1,1,1};

const int Node3D::dz[] = { -1, 0,  1,  -1,0,  1,-1,0,  1, -1,0,
                           1, -1,0, 1,-1,0,1,-1,0,1, -1,
                           0, 1, -1,0,1};


//###################################################
//                                   CREATE SUCCESSOR
//###################################################
Node3D* Node3D::createSuccessor(const int i) {

    int xSucc = x + step_size*Node3D::dx[i];
    int ySucc = y + step_size*Node3D::dy[i];
    int zSucc = z + step_size*Node3D::dz[i];

    return new Node3D(xSucc, ySucc,zSucc, g, 0, this);
}

//###################################################
//                                 2D NODE COMPARISON
//###################################################
bool Node3D::operator == (const Node3D& rhs) const {
    return x == rhs.x && y == rhs.y && z==rhs.z;
}
