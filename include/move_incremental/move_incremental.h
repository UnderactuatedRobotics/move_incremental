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
* Author: Fei Sun
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

// cost defs
#define COST_UNKNOWN_ROS 255 // 255 is unknown cost
#define COST_OBS 254         // 254 for forbidden regions
#define COST_OBS_ROS 253     // ROS values of 253 are obstacles

// MoveIncremental cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.
// Incoming costmap cost values are in the range 0 to 252.
// With COST_NEUTRAL of 50, the COST_FACTOR needs to be about 0.8 to
// ensure the input values are spread evenly over the output range, 50
// to 253.  If COST_FACTOR is higher, cost values will have a plateau
// around obstacles and the planner will then treat (for example) the
// whole width of a narrow hallway as equally undesirable and thus
// will not plan paths down the center.

#define COST_NEUTRAL 50        // Set this to "open space" value
#define COST_FACTOR 0.8        // Used for translating costs in MoveIncremental::setCostmap()

// Define the cost type in the case that it is not set. However, this allows
// clients to modify it without changing the file. Arguably, it is better to require it to
// be defined by a user explicitly
#ifndef COSTTYPE
#define COSTTYPE unsigned char    // Whatever is used...
#endif

// potential defs
#define POT_HIGH 1.0e10        // unassigned cell potential

// priority buffers
#define PRIORITYBUFSIZE 10000

using namespace std;
using namespace __gnu_cxx;

namespace move_incremental
{
/*
* D* Lite Addition
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

class state_hash
{
 public:
  size_t operator()(const state &s) const
  {
    return s.x + 34245 * s.y;
  }
};

typedef priority_queue < state, vector< state >, greater< state > > ds_pq;
typedef hash_map < state, cellInfo, state_hash, equal_to< state > > ds_ch;
typedef hash_map < state, float, state_hash, equal_to< state > > ds_oh;

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
  list <state> path;

  double C1;
  double k_m;
  state s_start, s_goal, s_last;
  int maxSteps;

  ds_pq openList;
  ds_ch cellHash;
  ds_oh openHash;

  bool close(double x, double y);
  void makeNewCell(state u);
  double getG(state u);
  double getRHS(state u);
  void setG(state u, double g);
  double setRHS(state u, double rhs);
  double eightCondist(state a, state b);
  int computeShortestPath();
  void updateVertex(state u);
  void insert(state u);
  void remove(state u);
  double trueDist(state a, state b);
  double heuristic(state a, state b);
  state calculateKey(state u);
  void getSucc(state u, list <state> &s);
  void getPred(state u, list <state> &s);
  double cost(state a, state b);
  bool occupied(state u);
  bool isValid(state u);
  float keyHashCode(state u);

};
};// namspace move_incremental

#endif //MOVE_INCREMENTAL_H