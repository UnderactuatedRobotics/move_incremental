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

//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
//
#ifndef PROJECT_MOVE_INCREMENTAL_H
#define PROJECT_MOVE_INCREMENTAL_H

#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include <math.h>
#include <stack>
#include <queue>
#include <list>
#include <ext/hash_map>

// cost defs
#define COST_UNKNOWN_ROS 255        // 255 is unknown cost
#define COST_OBS 254        // 254 for forbidden regions
#define COST_OBS_ROS 253    // ROS values of 253 are obstacles

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
    class state {
    public:
        int x;
        int y;
        int theta;
        pair<double,double> k;

        bool operator == (const state &s2) const {
        return ((x == s2.x) && (y == s2.y));
        }

        bool operator != (const state &s2) const {
        return ((x != s2.x) || (y != s2.y));
        }

        bool operator > (const state &s2) const {
        if (k.first-0.00001 > s2.k.first) return true;
        else if (k.first < s2.k.first-0.00001) return false;
        return k.second > s2.k.second;
        }

        bool operator <= (const state &s2) const {
        if (k.first < s2.k.first) return true;
        else if (k.first > s2.k.first) return false;
        return k.second < s2.k.second + 0.00001;
        }


        bool operator < (const state &s2) const {
        if (k.first + 0.000001 < s2.k.first) return true;
        else if (k.first - 0.000001 > s2.k.first) return false;
        return k.second < s2.k.second;
        }

    };

    struct ipoint2 {
        int x,y;
    };

    struct cellInfo {
        double g;
        double rhs;
        double cost;
        bool cost_changed;
    };

    class state_hash {
    public:
        size_t operator()(const state &s) const {
            return s.x + 34245*s.y;
        }
    };

    typedef priority_queue<state, vector<state>, greater<state> > ds_pq;
    typedef hash_map<state,cellInfo, state_hash, equal_to<state> > ds_ch;
    typedef hash_map<state, float, state_hash, equal_to<state> > ds_oh;

    /**
     * @class MoveIncremental
     * @brief Navigation function class. Holds buffers for costmap, MoveIncremental map. Maps are pixel-based. Origin is upper left, x is right, y is down.
     */
    class MoveIncremental {
    public:
        /**
         * @brief  Constructs the planner
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        MoveIncremental(int nx, int ny);    // size of map

        ~MoveIncremental();

        /**
         * @brief  Sets or resets the size of the map
         * @param nx The x size of the map
         * @param ny The y size of the map
         */
        void setNavArr(int nx, int ny); /**< sets or resets the size of the map */
        int nx, ny, ns;        /**< size of grid, in pixels */

        /**
         * @brief  Set up the cost array for the planner, usually from ROS
         * @param cmap The costmap
         * @param isROS Whether or not the costmap is coming in in ROS format
         * @param allow_unknown Whether or not the planner should be allowed to plan through unknown space
         */
        void
        setCostmap(const COSTTYPE *cmap, bool isROS = true, bool allow_unknown = true); /**< sets up the cost map */

        /**
         * @brief  Calculates a plan using the A* heuristic, returns true if one is found
         * @return True if a plan is found, false otherwise
         */
        bool calcMoveIncrementalAstar();    /**< calculates a plan, returns true if found */

        /**
         * @brief Caclulates the full navigation function using Dijkstra
         */
        bool calcMoveIncrementalDijkstra(bool atStart = false);    /**< calculates the full navigation function */

        /**
         * @brief  Accessor for the x-coordinates of a path
         * @return The x-coordinates of a path
         */
        float *getPathX();        /**< x-coordinates of path */

        /**
         * @brief  Accessor for the y-coordinates of a path
         * @return The y-coordinates of a path
         */
        float *getPathY();        /**< y-coordinates of path */

        /**
         * @brief  Accessor for the length of a path
         * @return The length of a path
         */
        int getPathLen();        /**< length of path, 0 if not found */

        /**
         * @brief  Gets the cost of the path found the last time a navigation function was computed
         * @return The cost of the last path found
         */
        float getLastPathCost();      /**< Return cost of path found the last time A* was called */

        /** cell arrays */
        COSTTYPE *costarr;        /**< cost array in 2D configuration space */
        float *potarr;        /**< potential array, navigation function potential */
        bool *pending;        /**< pending cells during propagation */
        int nobs;            /**< number of obstacle cells */

        /** block priority buffers */
        int *pb1, *pb2, *pb3;        /**< storage buffers for priority blocks */
        int *curP, *nextP, *overP;    /**< priority buffer block ptrs */
        int curPe, nextPe, overPe; /**< end points of arrays */

        /** block priority thresholds */
        float curT;            /**< current threshold */
        float priInc;            /**< priority threshold increment */

        /** goal and start positions */
        /**
         * @brief  Sets the goal position for the planner. Note: the navigation cost field computed gives the cost to get to a given point from the goal, not from the start.
         * @param goal the goal position
         */
        void setGoal(int *goal);

        /**
         * @brief  Sets the start position for the planner. Note: the navigation cost field computed gives the cost to get to a given point from the goal, not from the start.
         * @param start the start position
         */
        void setStart(int *start);

        int goal[2];
        int start[2];

        /**
         * @brief  Initialize cell k with cost v for propagation
         * @param k the cell to initialize
         * @param v the cost to give to the cell
         */
        void initCost(int k, float v); /**< initialize cell <k> with cost <v>, for propagation */

        /** simple obstacle for testing */
        void setObs();

        /** propagation */

        /**
         * @brief  Updates the cell at index n
         * @param n The index to update
         */
        void updateCell(int n);    /**< updates the cell at index <n> */

        /**
         * @brief  Updates the cell at index n using the A* heuristic
         * @param n The index to update
         */
        void updateCellAstar(int n);    /**< updates the cell at index <n>, uses A* heuristic */

        void setupMoveIncremental(bool keepit = false); /**< resets all nav fn arrays for propagation */

        /**
         * @brief  Run propagation for <cycles> iterations, or until start is reached using breadth-first Dijkstra method
         * @param cycles The maximum number of iterations to run for
         * @param atStart Whether or not to stop when the start point is reached
         * @return true if the start point is reached
         */
        bool propMoveIncrementalDijkstra(int cycles, bool atStart = false); /**< returns true if start point found or full prop */
        /**
         * @brief  Run propagation for <cycles> iterations, or until start is reached using the best-first A* method with Euclidean distance heuristic
         * @param cycles The maximum number of iterations to run for
         * @return true if the start point is reached
         */
        bool propMoveIncrementalAstar(int cycles); /**< returns true if start point found */

        /** gradient and paths */
        float *gradx, *grady;        /**< gradient arrays, size of potential array */
        float *pathx, *pathy;        /**< path points, as subpixel cell coordinates */
        int npath;            /**< number of path points */
        int npathbuf;            /**< size of pathx, pathy buffers */

        float last_path_cost_; /**< Holds the cost of the path found the last time A* was called */


        /**
         * @brief  Calculates the path for at mose <n> cycles
         * @param n The maximum number of cycles to run for
         * @return The lenght of the path found
         */
        int
        calcPath(int n, int *st = NULL); /**< calculates path for at most <n> cycles, returns path length, 0 if none */

        float gradCell(int n);    /**< calculates gradient at cell <n>, returns norm */
        float pathStep;        /**< step size for following gradient */

        /** display callback */
        void display(void fn(MoveIncremental *nav), int n = 100); /**< <n> is the number of cycles between updates  */
        int displayInt;        /**< save second argument of display() above */
        void (*displayFn)(MoveIncremental *nav); /**< display function itself */

        /** save costmap */
        void savemap(const char *fname); /**< write out costmap and start/goal states as fname.pgm and fname.txt */

        /********************
         * D* Lite
         ********************
         */

        /**
         * @brief  Calculates a plan using the A* heuristic, returns true if one is found
         * @return True if a plan is found, false otherwise
         */
        bool calcMoveIncrementalDstar();    /**< calculates a plan, returns true if found */

        /**
         * @brief  Updates the cell at index n using the A* heuristic
         * @param n The index to update
         */
        void updateCellDstar(int n);    /**< updates the cell at index <n>, uses D* Lite heuristic */

        /**
         * @brief  Run propagation for <cycles> iterations, or until start is reached using the best-first D* method with Euclidean distance heuristic
         * @param cycles The maximum number of iterations to run for
         * @return true if the start point is reached
         */
        bool propMoveIncrementalDstar(int cycles); /**< returns true if start point found */

        void   init(int sX, int sY, int gX, int gY);
        void   updateCell(int x, int y, double val);
        void   updateStart(int x, int y);
        void   updateGoal(int x, int y);
        bool   replan();

        list<state> getPath();

        ds_ch getCellHash();

    private:
        list<state> path;

        double C1;
        double k_m;
        state s_start, s_goal, s_last;
        int maxSteps;  

        ds_pq openList;
        ds_ch cellHash;
        ds_oh openHash;

        void   initialize();
        bool   close(double x, double y);
        void   makeNewCell(state u);
        double getG(state u);
        double getRHS(state u);
        void   setG(state u, double g);
        double setRHS(state u, double rhs);
        double eightCondist(state a, state b);
        int    computeShortestPath();
        void   updateVertex(state u);
        void   insert(state u);
        void   remove(state u);
        double trueDist(state a, state b);
        double heuristic(state a, state b);
        state  calculateKey(state u);
        void   getSucc(state u, list<state> &s);
        void   getPred(state u, list<state> &s);
        double cost(state a, state b); 
        bool   occupied(state u);
        bool   isValid(state u);
        float keyHashCode(state u);

    };
};// namspace move_incremental

#endif //PROJECT_MOVE_INCREMENTAL_H