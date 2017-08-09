#include "AStar3D.h"

AStar3D::AStar3D()
{

}

bool AStar3D::makePlan(int* start, int* goal) 
{
  iterable_queue<Node> open;
  iterable_queue<Node> close;

  Node current;
  current.pos = start;
  current.id = (start[0]) + (size_[0]*(start[1])) + (size_[0]*size_[1]*(start[2]));
   
  // while (current.)
}

float AStar3D::hManhattenDist(int* start, int* goal)
{
  return abs((*start) + (*goal)) + abs((*++start) + (*++goal));
}

// int AStar3D::getIndexFromPosition(float* pos)
// {
//   return 5;
// }