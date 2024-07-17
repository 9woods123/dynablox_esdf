#include "hybrid_astar/node4d.h"

using namespace HybridAStar;
const int Node4D::dir = 9;
float Node4D::radius_min = 0;
float Node4D::step_min = 1;
float Node4D::dx[dir] = {};
float Node4D::dy[dir] = {};
float Node4D::dz[dir] = {};
float Node4D::dt[dir] = {};


void copy_array(float a[], float b[], int size)
{
  for (int i = 0; i < size; ++i)
  {
    a[i] = b[i];
  }
}

void Node4D::setMotionPrims()
{

  float delta_x=Node4D::step_min;
  float delta_t= asin(delta_x/Node4D::radius_min);
  float delta_y= (1- cos(delta_t))*Node4D::radius_min;
  float delta_distance=delta_t*Node4D::radius_min;
  float dy_array[]=   { 0,              -delta_y,   delta_y,  0 ,         0               };
  float dx_array[]  = { delta_distance, delta_x,    delta_x,  delta_x ,   delta_x         };
  float dz_array[]  = { 0,              0,          0,        delta_x/4,  -delta_x/4      };
  float dt_array[]  = { 0,              -delta_t,   delta_t,  0,          0               };

  copy_array(dx,dx_array,dir);
  copy_array(dy,dy_array,dir);
  copy_array(dz,dz_array,dir);
  copy_array(dt,dt_array,dir);

}

Node4D*Node4D::generateNeibors(const int input_index){

  // using dynamic model to generate primitives,  by woods 2024- 05-08

  // std::cout<<"  input_index:  "<<input_index<<std::endl;

  float input_turning_rate_max = Node4D::step_min/Node4D::radius_min;

  float input_vel_z_max =Node4D::step_min/6;
  float input_vel_forward = Node4D::step_min;

  float curr_x=this->x;
  float curr_y=this->y;
  float curr_z=this->z;
  float curr_theta=this->t;

  int dir_number=3;
  float delat_time=1.0;

  int i = input_index/dir_number;
  int j = input_index%dir_number;
  int auto_index;

  // if (i==0){
  //   auto_index=0;
  // }else if(i==1)
  // {
  //   auto_index=-1;
  // }else if (i==2)
  // {
  //   auto_index=1;
  // }
    double u_rate =  (i-1) * input_turning_rate_max / double((dir_number-1)/2) + 0.000001;
  // float u_vel_z = (j - 1) * input_vel_z_max / float((dir_number-1)/2);
    double u_vel_z =(j-1) * input_vel_z_max / double((dir_number-1)/2);

// if (input_index==0){
//   u_rate=  0.000001;
//   u_vel_z =0 ;

//   //向前进

// }else if (input_index==1){


//   u_rate=   -1* input_turning_rate_max + 0.000001;
//   u_vel_z =0 ;
//   //向右拐
// }

// else if (input_index==2){

//   u_rate=   1* input_turning_rate_max + 0.000001;
//   u_vel_z =0 ;

//   //向左拐
// }

// else if (input_index==3){

//   u_rate=   + 0.000001;
//   u_vel_z =0.5 ;

//   //向左拐
// }
// else if (input_index==4){

//   u_rate=   + 0.000001;
//   u_vel_z =- 0.5 ;

//   //向下
// }


  // std::cout<<"u_rate:"<<u_rate<<std::endl;

  // float delta_x = input_vel_forward * cos(curr_theta) *delat_time;
  // float delta_y = input_vel_forward * sin(curr_theta)* delat_time;
  // float delta_z = u_vel_z* delat_time;
  // float delta_t =  u_rate* delat_time;


// 直接使用 v cos theta 和  v sin theta 來計算 delta ,会发现,无论u rate 如何,
//即delta theta 如何, deltax和deltay 是一样的,虽然在计算node2ID时会考虑theta作为第四纬变量,
//但是其密度和三维空间的密度并不能相提并论,或者说并不能算是线性相关.
//本规划问题,考虑的四维空间变量,但是在计算node2index时,如果过分强调theta这个纬度上的区分,
//这会使节点拓展过程中,产生许多空间坐标接近,但角度存在微小差异的点.这降低了搜索效率.
// delta x=v∫cos(wt) dt ( t_curr  -> t_curr+delta_t)= (v/w)sin(wt) ( t_curr  -> t_curr+delta_t)


  double delta_z = u_vel_z* delat_time;

  double delta_t =  u_rate* delat_time;

  double delta_x =   input_vel_forward/u_rate*( sin(u_rate*delat_time+curr_theta)-sin(curr_theta));
  double delta_y =  -input_vel_forward/u_rate*( cos(u_rate*delat_time+curr_theta)-cos(curr_theta));

  // if (delta_z!=0)
  // {
  //  delta_x =   0.95*input_vel_forward/u_rate*( sin(u_rate*delat_time+curr_theta)-sin(curr_theta));
  //  delta_y =  -0.95*input_vel_forward/u_rate*( cos(u_rate*delat_time+curr_theta)-cos(curr_theta));
  // }

  // std::cout<<"delta_x:"<<delta_x<< "   delta_y:"<<delta_y<<"   delta_z:"<<delta_z;

  double xSucc = curr_x + delta_x;
  double ySucc = curr_y + delta_y;
  double zSucc = curr_z + delta_z;
  double tSucc = Helper::normalizeHeadingRad(curr_theta + delta_t);

  double newG=calculateG(delta_x,delta_y,delta_z);
  // std::cout<<"   newG:"<<newG<< std::endl;


  // std::cout<<"xSucc:"<<xSucc<< "   ySucc:"<<ySucc<<" zSucc:"<<zSucc<<"  tSucc:"<<tSucc<<std::endl;

//   Node4D* neibor=new Node4D(xSucc, ySucc, zSucc, tSucc, newG, 0, this, i, Direction::Forward);

return new Node4D(xSucc, ySucc, zSucc, tSucc, newG, 0, this, i, Direction::Forward);
// return neibor;
}

double Node4D::calculateG(float delta_x, float delta_y, float delta_z){

// motion along z-axis is  punisied by multipied 10 
return this->g+sqrt(delta_x*delta_x+ delta_y*delta_y + delta_z*delta_z);

}

Node4D *Node4D::createSuccessor(const int i)
{

  float xSucc;
  float ySucc;
  float zSucc;
  float tSucc;

  // calculate successor positions forward
  if (i < Node4D::dir)
  { // =====woods=====这里是5，因为考虑了z轴运动
    xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
    ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
    zSucc = z + dz[i]; //=========woods======= 这里没有考虑伏仰角度
    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
  }
  // backwards
  else
  {
    xSucc = x - dx[i - Node4D::dir] * cos(t) - dy[i - Node4D::dir] * sin(t);
    ySucc = y - dx[i - Node4D::dir] * sin(t) + dy[i - Node4D::dir] * cos(t);
    zSucc = z - dz[i - Node4D::dir]; //=========woods======= 这里没有考虑伏仰角度
    tSucc = Helper::normalizeHeadingRad(t - dt[i - Node4D::dir]);
  }
  return new Node4D(xSucc, ySucc, zSucc, tSucc, g, 0, this, i);
}


//                                      MOVEMENT COST
void Node4D::updateG()
{
  // forward driving
  if (prim < Node4D::dir)
  {
    // penalize turning
    if (pred->prim != prim)
    {
      // penalize change of direction
      if (pred->prim > Node4D::dir - 1)
      { //=======woods======前一个节点是倒车的化
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
      }
      else
      {
        g += sqrt(dx[0] * dx[0] + 10 * dz[prim] * dz[prim]);
      }
    }
    else
    {

      g += sqrt(dx[0] * dx[0] + 10 * dz[prim] * dz[prim]);
    }
  }
  // reverse driving
  else
  {
    // penalize turning and reversing
    if (pred->prim != prim)
    {
      // penalize change of direction
      if (pred->prim < Node4D::dir)
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      }
      else
      {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    }
    else
    {

      g += sqrt(dx[0] * dx[0] * Constants::penaltyReversing + dz[prim] * dz[prim]);
    }
  }
}

//                                 4D NODE COMPARISON
bool Node4D::operator==(const Node4D &rhs) const
{
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (int)z == (int)rhs.z &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}
