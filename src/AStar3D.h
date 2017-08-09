/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <angles/angles.h>

#include <stdio.h>      /* printf */
#include <stdlib.h>     /* abs */
#include <queue>
#include <deque>
#include <iostream>

#ifndef ASTAR_3D_CPP
#define ASTAR_3D_CPP

struct Node {
  int id;
  int* pos;
  float h;
  float g;
};


template<typename T, typename Container=std::deque<T> >
class iterable_queue : public std::queue<T,Container>
{
public:
  bool contains(Node& node){
    for(iterator iter = this->c.begin();iter != this->c.end();iter++)
      if ((*iter).id == node.id)
        return true;
    return false;
  }

  typedef typename Container::iterator iterator;
  typedef typename Container::const_iterator const_iterator;

  iterator begin() { return this->c.begin(); }
  iterator end() { return this->c.end(); }
  const_iterator begin() const { return this->c.begin(); }
  const_iterator end() const { return this->c.end(); }
};


class AStar3D {

  public:

    AStar3D();
    // AStar3D(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /** overridden classes from interface nav_core::BaseGlobalPlanner **/
    // void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    // bool makePlan(const geometry_msgs::PoseStamped& start,
    //               const geometry_msgs::PoseStamped& goal,
    //               std::vector<geometry_msgs::PoseStamped>& plan
    //              );
    bool makePlan(int* start, int* goal);


  private:
    // Heuristics
    float hManhattenDist(int* start, int* goal);

    // Variables
    costmap_2d::Costmap2DROS* costmap_ros_;
    std::vector<int(*)[4]> costmap_exp_;
    int size_[3];
    




};

#endif
