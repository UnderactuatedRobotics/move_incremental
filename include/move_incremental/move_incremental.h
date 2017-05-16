/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, MIT Cognitive Robotics.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
* 
* Author: Fei Sun, Yijiang Huang
*********************************************************************/

#ifndef MOVE_INCREMENTAL_H
#define MOVE_INCREMENTAL_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <math.h>
#include <stack>
#include <queue>
#include <list>
#include <ext/hash_map>

namespace move_incremental
{
using namespace std;
using namespace __gnu_cxx;

/*
*
*/
class state
{
 public:
  int x;
  int y;
  int theta;
  pair<double, double> k;

  bool operator==(const state &s2) const
  {
    return ((x == s2.x) && (y == s2.y));
  }

  bool operator!=(const state &s2) const
  {
    return ((x != s2.x) || (y != s2.y));
  }

  bool operator>(const state &s2) const
  {
    if (k.first - 0.00001 > s2.k.first) { return true; }
    else if (k.first < s2.k.first - 0.00001) { return false; }
    return k.second > s2.k.second;
  }

  bool operator<=(const state &s2) const
  {
    if (k.first < s2.k.first) { return true; }
    else if (k.first > s2.k.first) { return false; }
    return k.second < s2.k.second + 0.00001;
  }

  bool operator<(const state &s2) const
  {
    if (k.first + 0.000001 < s2.k.first) { return true; }
    else if (k.first - 0.000001 > s2.k.first) { return false; }
    return k.second < s2.k.second;
  }

};

struct ipoint2
{
  int x, y;
};

struct cellInfo
{
  double g;
  double rhs;
  double cost;
};

// hasher function
class state_hash
{
 public:
  size_t operator()(const state &s) const
  {
    return s.x + 34245 * s.y;
  }
};

//! ds_pq: Priority Queue U in D* Lite, smallest element will appear at the top (OpenList)
typedef std::priority_queue < state, std::vector<state>, std::greater<state> >   ds_pq;

//! ds_ch: cell Hash in D* Lite
typedef hash_map < state, cellInfo, state_hash, equal_to<state> > ds_ch;

//! ds_oh: Open Hash in D* Lite
typedef hash_map < state, float, state_hash, equal_to<state> >    ds_oh;

/**
 * @class MoveIncremental
 * @brief Navigation function class. Holds buffers for costmap, MoveIncremental map. Maps are pixel-based. Origin is upper left, x is right, y is down.
 */
class MoveIncremental
{
 public:
  /**
   * @brief  Constructs the planner
   * @param nx The x size of the map
   * @param ny The y size of the map
   */
  MoveIncremental(int nx, int ny);    // size of map

  ~MoveIncremental();

  void init(int sX, int sY, int gX, int gY);
  void updateCell(int x, int y, double val);
  void updateStart(int x, int y);
  void updateGoal(int x, int y);
  bool replan();

  list <state> getPath();

 private:

  bool close(double x, double y);
  void makeNewCell(state u);

  double  getG(state u);
  double  getRHS(state u);
  void    getSucc(state u, list <state> &s);
  void    getPred(state u, list <state> &s);

  void    setG(state u, double g);
  double  setRHS(state u, double rhs);

  int   computeShortestPath();

  void  updateVertex(state u);
  void  insert(state u);
  void  remove(state u);

  double eightCondist(state a, state b);
  double trueDist(state a, state b);
  double heuristic(state a, state b);

  state calculateKey(state u);

  double  cost(state a, state b);
  bool    occupied(state u);
  bool    isValid(state u);
  float   keyHashCode(state u);

 private:
  list <move_incremental::state> path;

  //! cost of an unseen cell
  double C1;

  double k_m;

  move_incremental::state s_start, s_goal, s_last;

  //! maximum number of nodes expanded before giving up
  int maxSteps;

  move_incremental::ds_pq openList;

  /*!
   * @brief a state-cost hash table
   */
  move_incremental::ds_ch cellHash;
  move_incremental::ds_oh openHash;
};
};// namspace move_incremental

#endif //MOVE_INCREMENTAL_H