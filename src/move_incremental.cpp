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
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
//
// Path calc has sanity check that it succeeded
//

#include <ros/console.h>
#include <move_incremental/move_incremental.h>

namespace move_incremental
{
    //
    // create nav fn buffers
    //
    MoveIncremental::MoveIncremental(int xs, int ys)
    {
        // D* Lite
        maxSteps = 80000;  // node expansions before we give up
        C1 = 1; // cost of an unseen cell

        // create cell arrays
        costarr = NULL;
        potarr = NULL;
        pending = NULL;
        gradx = grady = NULL;
        setNavArr(xs,ys);

        // priority buffers
        pb1 = new int[PRIORITYBUFSIZE];
        pb2 = new int[PRIORITYBUFSIZE];
        pb3 = new int[PRIORITYBUFSIZE];

        // for Dijkstra (breadth-first), set to COST_NEUTRAL
        // for A* (best-first), set to COST_NEUTRAL
        priInc = 2*COST_NEUTRAL;

        // goal and start
        goal[0] = goal[1] = 0;
        start[0] = start[1] = 0;

        // display function
        displayFn = NULL;
        displayInt = 0;

        // path buffers
        npathbuf = npath = 0;
        pathx = pathy = NULL;
        pathStep = 0.5;
    }


    MoveIncremental::~MoveIncremental()
    {
        if(costarr)
            delete[] costarr;
        if(potarr)
            delete[] potarr;
        if(pending)
            delete[] pending;
        if(gradx)
            delete[] gradx;
        if(grady)
            delete[] grady;
        if(pathx)
            delete[] pathx;
        if(pathy)
            delete[] pathy;
        if(pb1)
            delete[] pb1;
        if(pb2)
            delete[] pb2;
        if(pb3)
            delete[] pb3;
    }


    //
    // set goal, start positions for the nav fn
    //

    void
    MoveIncremental::setGoal(int *g)
    {
        goal[0] = g[0];
        goal[1] = g[1];
        ROS_INFO("[MoveIncremental] Setting goal to %d,%d\n", goal[0], goal[1]);
    }

    void
    MoveIncremental::setStart(int *g)
    {
        start[0] = g[0];
        start[1] = g[1];
        ROS_INFO("[MoveIncremental] Setting start to %d,%d\n", start[0], start[1]);
    }

    //
    // Set/Reset map size
    //

    void
    MoveIncremental::setNavArr(int xs, int ys)
    {
        ROS_INFO("[MoveIncremental] Array is %d x %d\n", xs, ys);

        nx = xs;
        ny = ys;
        ns = nx*ny;

        if(costarr)
            delete[] costarr;
        if(potarr)
            delete[] potarr;
        if(pending)
            delete[] pending;

        if(gradx)
            delete[] gradx;
        if(grady)
            delete[] grady;

        costarr = new COSTTYPE[ns]; // cost array, 2d config space
        memset(costarr, 0, ns*sizeof(COSTTYPE));
        potarr = new float[ns];	// navigation potential array
        pending = new bool[ns];
        memset(pending, 0, ns*sizeof(bool));
        gradx = new float[ns];
        grady = new float[ns];
    }


    //
    // set up cost array, usually from ROS
    //

    void
    MoveIncremental::setCostmap(const COSTTYPE *cmap, bool isROS, bool allow_unknown)
    {
        COSTTYPE *cm = costarr;
        if (isROS)			// ROS-type cost array
        {
            for (int i=0; i<ny; i++)
            {
                int k=i*nx;
                for (int j=0; j<nx; j++, k++, cmap++, cm++)
                {
                    // This transforms the incoming cost values:
                    // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")
                    // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated obstacle")
                    // values in range 0 to 252 -> values from COST_NEUTRAL to COST_OBS_ROS.
                    *cm = COST_OBS;
                    int v = *cmap;
                    if (v < COST_OBS_ROS)
                    {
                        v = COST_NEUTRAL+COST_FACTOR*v;
                        if (v >= COST_OBS)
                            v = COST_OBS-1;
                        *cm = v;
                    }
                    else if(v == COST_UNKNOWN_ROS && allow_unknown)
                    {
                        v = COST_OBS-1;
                        *cm = v;
                    }
                }
            }
        }

        else				// not a ROS map, just a PGM
        {
            for (int i=0; i<ny; i++)
            {
                int k=i*nx;
                for (int j=0; j<nx; j++, k++, cmap++, cm++)
                {
                    *cm = COST_OBS;
                    if (i<7 || i > ny-8 || j<7 || j > nx-8)
                        continue;	// don't do borders
                    int v = *cmap;
                    if (v < COST_OBS_ROS)
                    {
                        v = COST_NEUTRAL+COST_FACTOR*v;
                        if (v >= COST_OBS)
                            v = COST_OBS-1;
                        *cm = v;
                    }
                    else if(v == COST_UNKNOWN_ROS)
                    {
                        v = COST_OBS-1;
                        *cm = v;
                    }
                }
            }

        }
    }

    bool
    MoveIncremental::calcMoveIncrementalDijkstra(bool atStart)
    {
    }


    //
    // calculate navigation function, given a costmap, goal, and start
    //

    bool
    MoveIncremental::calcMoveIncrementalAstar()
    {
    }


    //
    // returning values
    //

    float *MoveIncremental::getPathX() { return pathx; }
    float *MoveIncremental::getPathY() { return pathy; }
    int    MoveIncremental::getPathLen() { return npath; }


    //
    // simple obstacle setup for tests
    //

    void
    MoveIncremental::setObs()
    {
#if 0
        // set up a simple obstacle
      ROS_INFO("[MoveIncremental] Setting simple obstacle\n");
      int xx = nx/3;
      for (int i=ny/3; i<ny; i++)
        costarr[i*nx + xx] = COST_OBS;
      xx = 2*nx/3;
      for (int i=ny/3; i<ny; i++)
        costarr[i*nx + xx] = COST_OBS;
      xx = nx/4;
      for (int i=20; i<ny-ny/3; i++)
        costarr[i*nx + xx] = COST_OBS;
      xx = nx/2;
      for (int i=20; i<ny-ny/3; i++)
        costarr[i*nx + xx] = COST_OBS;
      xx = 3*nx/4;
      for (int i=20; i<ny-ny/3; i++)
        costarr[i*nx + xx] = COST_OBS;
#endif
    }


    // inserting onto the priority blocks
#define push_cur(n)  { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && curPe<PRIORITYBUFSIZE) \
  { curP[curPe++]=n; pending[n]=true; }}
#define push_next(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && nextPe<PRIORITYBUFSIZE) \
  { nextP[nextPe++]=n; pending[n]=true; }}
#define push_over(n) { if (n>=0 && n<ns && !pending[n] && \
    costarr[n]<COST_OBS && overPe<PRIORITYBUFSIZE) \
  { overP[overPe++]=n; pending[n]=true; }}


    // Set up navigation potential arrays for new propagation

    void
    MoveIncremental::setupMoveIncremental(bool keepit)
    {

    }


    // initialize a goal-type cost for starting propagation

    void
    MoveIncremental::initCost(int k, float v)
    {

    }


    //
    // Critical function: calculate updated potential value of a cell,
    //   given its neighbors' values
    // Planar-wave update calculation from two lowest neighbors in a 4-grid
    // Quadratic approximation to the interpolated value
    // No checking of bounds here, this function should be fast
    //

#define INVSQRT2 0.707106781

    inline void
    MoveIncremental::updateCell(int n)
    {
    }


    //
    // Use A* method for setting priorities
    // Critical function: calculate updated potential value of a cell,
    //   given its neighbors' values
    // Planar-wave update calculation from two lowest neighbors in a 4-grid
    // Quadratic approximation to the interpolated value
    // No checking of bounds here, this function should be fast
    //

#define INVSQRT2 0.707106781

    inline void
    MoveIncremental::updateCellAstar(int n)
    {
    }



    //
    // main propagation function
    // Dijkstra method, breadth-first
    // runs for a specified number of cycles,
    //   or until it runs out of cells to update,
    //   or until the Start cell is found (atStart = true)
    //

    bool
    MoveIncremental::propMoveIncrementalDijkstra(int cycles, bool atStart)
    {
    }


    //
    // main propagation function
    // A* method, best-first
    // uses Euclidean distance heuristic
    // runs for a specified number of cycles,
    //   or until it runs out of cells to update,
    //   or until the Start cell is found (atStart = true)
    //

    bool
    MoveIncremental::propMoveIncrementalAstar(int cycles)
    {
    }


    float MoveIncremental::getLastPathCost()
    {
        return last_path_cost_;
    }


    //
    // Path construction
    // Find gradient at array points, interpolate path
    // Use step size of pathStep, usually 0.5 pixel
    //
    // Some sanity checks:
    //  1. Stuck at same index position
    //  2. Doesn't get near goal
    //  3. Surrounded by high potentials
    //

    int
    MoveIncremental::calcPath(int n, int *st)
    {
        //  return npath;			// out of cycles, return failure
        ROS_INFO("[PathCalc] No path found, path too long");
        //savemap("MoveIncremental_pathlong");
        return 0;			// out of cycles, return failure
    }


    //
    // gradient calculations
    //

    // calculate gradient at a cell
    // positive value are to the right and down
    float
    MoveIncremental::gradCell(int n)
    {
        if (gradx[n]+grady[n] > 0.0)	// check this cell
            return 1.0;

        if (n < nx || n > ns-nx)	// would be out of bounds
            return 0.0;

        float cv = potarr[n];
        float dx = 0.0;
        float dy = 0.0;

        // check for in an obstacle
        if (cv >= POT_HIGH)
        {
            if (potarr[n-1] < POT_HIGH)
                dx = -COST_OBS;
            else if (potarr[n+1] < POT_HIGH)
                dx = COST_OBS;

            if (potarr[n-nx] < POT_HIGH)
                dy = -COST_OBS;
            else if (potarr[nx+1] < POT_HIGH)
                dy = COST_OBS;
        }

        else				// not in an obstacle
        {
            // dx calc, average to sides
            if (potarr[n-1] < POT_HIGH)
                dx += potarr[n-1]- cv;
            if (potarr[n+1] < POT_HIGH)
                dx += cv - potarr[n+1];

            // dy calc, average to sides
            if (potarr[n-nx] < POT_HIGH)
                dy += potarr[n-nx]- cv;
            if (potarr[n+nx] < POT_HIGH)
                dy += cv - potarr[n+nx];
        }

        // normalize
        float norm = hypot(dx, dy);
        if (norm > 0)
        {
            norm = 1.0/norm;
            gradx[n] = norm*dx;
            grady[n] = norm*dy;
        }
        return norm;
    }


    //
    // display function setup
    // <n> is the number of cycles to wait before displaying,
    //     use 0 to turn it off

    void
    MoveIncremental::display(void fn(MoveIncremental *nav), int n)
    {
        displayFn = fn;
        displayInt = n;
    }


    //
    // debug writes
    // saves costmap and start/goal
    //

    void
    MoveIncremental::savemap(const char *fname)
    {
    }

    /********************
     * D* Lite
     ********************
     */

    void MoveIncremental::setCostmap2D(costmap_2d::Costmap2D* costmap) {
      cellCostmap = costmap;
      cellCostGrid = cellCostmap->getCharMap();
    }

    /********************
     * D* Lite
     ********************
     */

/* float Dstar::keyHashCode(state u) 
 * -------------------------- 
 * Returns the key hash code for the state u, this is used to compare
 * a state that have been updated
 */
float MoveIncremental::keyHashCode(state u) {

  return (float)(u.k.first + 1193*u.k.second);

}

/* bool Dstar::isValid(state u) 
 * --------------------------
 * Returns true if state u is on the open list or not by checking if
 * it is in the hash table.
 */
bool MoveIncremental::isValid(state u) {
  
  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return false;
  if (!close(keyHashCode(u), cur->second)) return false;
  return true;
  
}

/* void Dstar::getPath() 
 * --------------------------
 * Returns the path created by replan()
 */
list<state> MoveIncremental::getPath() {
  return path;
}

/* void Dstar::getPath() 
 * --------------------------
 * Returns the path created by replan()
 */
ds_ch MoveIncremental::getCellHash() {
  return cellHash;
}

/* bool Dstar::occupied(state u)
 * --------------------------
 * returns true if the cell is occupied (non-traversable), false
 * otherwise. non-traversable are marked with a cost < 0.
 */
bool MoveIncremental::occupied(state u) {
  
  ds_ch::iterator cur = cellHash.find(u);
  
  if (cur == cellHash.end()) return false;
  return (cur->second.cost < 0);
}

/* void Dstar::init(int sX, int sY, int gX, int gY)
 * --------------------------
 * Init dstar with start and goal coordinates, rest is as per
 * [S. Koenig, 2002]
 */
void MoveIncremental::init(int sX, int sY, int gX, int gY) {
  
  cellHash.clear();
  path.clear();
  openHash.clear();
  while(!openList.empty()) openList.pop();

  k_m = 0;
  
  s_start.x = sX;
  s_start.y = sY;
  s_goal.x  = gX;
  s_goal.y  = gY;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;

}

/* void Dstar::initialize()
 * --------------------------
 * Init dstar with start and goal coordinates, rest is as per
 * [S. Koenig, 2002]
 */
void MoveIncremental::initialize() {
  s_last = s_start;

  cellHash.clear();
  cellHash.resize(160000);
  path.clear();
  // U = empty
  openHash.clear();
  while(!openList.empty()) openList.pop();

  // k_m = 0
  k_m = 0;
  
  // for all s in S, rhs(s)=g(s)=INFINITY
  // do this in makeCell when push new cell to cellHash

  // rhs(s_goal)=0
  // U.Insert(s_goal, CalculateKey(s_goal))
  cellInfo tmp;
  tmp.g = INFINITY;
  tmp.rhs =  0;
  tmp.cost = C1;
  tmp.cost_changed = false;

  insert(calculateKey(s_goal));
  cellHash[s_goal] = tmp;

  // ComputeShortestPath()
  int res = computeShortestPath();
  if (res < 0) {
    fprintf(stderr, "NO PATH TO GOAL\n");
  }
}


/* void Dstar::makeNewCell(state u)
 * --------------------------
 * Checks if a cell is in the hash table, if not it adds it in.
 */
void MoveIncremental::makeNewCell(state u) {
  
  if (cellHash.find(u) != cellHash.end()) return;

  int x = u.x;
  int y = u.y;
  int index = cellCostmap->getIndex(x,y);

  cellInfo tmp;
  tmp.g       = tmp.rhs = INFINITY;
  tmp.cost    = (double)cellCostGrid[index];
  tmp.cost_changed = false;

  cellHash[u] = tmp;  
}

/* double Dstar::getG(state u)
 * --------------------------
 * Returns the G value for state u.
 */
double MoveIncremental::getG(state u) {

  if (cellHash.find(u) == cellHash.end()) 
    //return heuristic(u,s_goal);
    return INFINITY;
  return cellHash[u].g;
  
}

/* double Dstar::getRHS(state u)
 * --------------------------
 * Returns the rhs value for state u.
 */
double MoveIncremental::getRHS(state u) {

  if (u == s_goal) return 0;  

  if (cellHash.find(u) == cellHash.end()) 
    //return heuristic(u,s_goal);
    return INFINITY;
  return cellHash[u].rhs;
  
}

/* void Dstar::setG(state u, double g)
 * --------------------------
 * Sets the G value for state u
 */
void MoveIncremental::setG(state u, double g) {
  
  //makeNewCell(u);  
  cellHash[u].g = g; 
}

/* void Dstar::setRHS(state u, double rhs)
 * --------------------------
 * Sets the rhs value for state u
 */
double MoveIncremental::setRHS(state u, double rhs) {
  
  //makeNewCell(u);
  cellHash[u].rhs = rhs;

}

/* double Dstar::eightCondist(state a, state b) 
 * --------------------------
 * Returns the 8-way distance between state a and state b.
 */
double MoveIncremental::eightCondist(state a, state b) {
  double temp;
  double min = abs(a.x - b.x);
  double max = abs(a.y - b.y);
  if (min > max) {
    double temp = min;
    min = max;
    max = temp;
  }
  return ((M_SQRT2-1.0)*min + max);
}

/* int Dstar::computeShortestPath()
 * --------------------------
 * As per [S. Koenig, 2002] except for 2 main modifications:
 * 1. We stop planning after a number of steps, 'maxsteps' we do this
 *    because this algorithm can plan forever if the start is
 *    surrounded by obstacles. 
 * 2. We lazily remove states from the open list so we never have to
 *    iterate through it.
 */
// openList is the priority queue U

// while(U.TopKey()<CalculateKey(s_start) OR rhs(s_start)!=g(s_start))
//  k_old = U.TopKey()
//  u = U.Pop()
//  if(k_old<CalculateKey(u))
//    U.Insert(u, CalculateKey(u))
//  else if(g(u)>rhs(u))
//    g(u) = rhs(u)
//    for all s in Pred(u) UpdateVertex(s)
//  else
//    g(u) = inf
//    for all s in (Pred(u) and u) UpdateVertex(s)
// DCHECK
int MoveIncremental::computeShortestPath() {
  
  list<state> s;
  list<state>::iterator i;

  if (openList.empty()) return 1;

  int k=0;
  // while(U.TopKey()<CalculateKey(s_start) OR rhs(s_start)!=g(s_start))
  while ((!openList.empty()) && 
         (openList.top() < (s_start = calculateKey(s_start))) || 
         (getRHS(s_start) != getG(s_start))) {

    if (k++ > maxSteps) {
      fprintf(stderr, "At maxsteps\n");
      return -1;
    }

    // k_old = U.TopKey()
    // u = U.Pop()
    
    state u;
    // lazy remove
    bool test = (getRHS(s_start) != getG(s_start));
    while(1) { 
      if (openList.empty()) return 1;
      u = openList.top();
      openList.pop();
      
      if (!isValid(u)) continue;
      if (!(u < s_start) && (!test)) return 2;
      break;
    }
    ds_oh::iterator cur = openHash.find(u);
    openHash.erase(cur);

    state k_old = u;

    // if(k_old<CalculateKey(u))
    if (k_old < calculateKey(u)) { // u is out of date
      // U.Insert(u, CalculateKey(u))
      insert(u);
    }
    // else if(g(u)>rhs(u))
    else if (getG(u) > getRHS(u)) { // needs update (got better)
      // g(u) = rhs(u)
      setG(u,getRHS(u));
      // for all s in Pred(u) UpdateVertex(s)
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
    }
    // else
    else {   // g <= rhs, state has got worse
      // g(u) = inf
      setG(u,INFINITY);
      // for all s in (Pred(u) and u) UpdateVertex(s)
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
      updateVertex(u);
    }
  }
  return 0;
}

/* bool Dstar::close(double x, double y) 
 * --------------------------
 * Returns true if x and y are within 10E-5, false otherwise
 */
bool MoveIncremental::close(double x, double y) {
    
  if (std::isinf(x) && std::isinf(y)) return true;
  return (fabs(x-y) < 0.00001);
  
}

/* void Dstar::updateVertex(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
// DCHECK
void MoveIncremental::updateVertex(state u) {

  list<state> s;
  list<state>::iterator i;

  // if(u!=s_goal) rhs(u)= min (s in Succ(u)) (c(u,s')+g(s'))
  if (u != s_goal) {
    getSucc(u,s);
    double tmp = INFINITY;
    double tmp2;

    for (i=s.begin();i != s.end(); i++) {
      tmp2 = getG(*i) + cost(u,*i);
      if (tmp2 < tmp) tmp = tmp2;
    }
    if (!close(getRHS(u),tmp)) setRHS(u,tmp);
  }

  // if (u in U) U.remove(u)
  // if(g(u)!=rhs(u)) U.insert(u, CalculateKey(u))
  if (!close(getG(u),getRHS(u))) insert(u);
  // Note: duplicate is removed later in lazy remove
}

/* void Dstar::insert(state u) 
 * --------------------------
 * Inserts state u into openList and openHash.
 */
void MoveIncremental::insert(state u) {
  
  ds_oh::iterator cur;
  float csum;

  u    = calculateKey(u);
  cur  = openHash.find(u);
  csum = keyHashCode(u);
  // return if cell is already in list. TODO: this should be
  // uncommented except it introduces a bug, I suspect that there is a
  // bug somewhere else and having duplicates in the openList queue
  // hides the problem...
  //if ((cur != openHash.end()) && (close(csum,cur->second))) return;
  
  openHash[u] = csum;
  openList.push(u);
} 

/* void Dstar::remove(state u)
 * --------------------------
 * Removes state u from openHash. The state is removed from the
 * openList lazilily (in replan) to save computation.
 */
void MoveIncremental::remove(state u) {
  
  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return;
  openHash.erase(cur);
}


/* double Dstar::trueDist(state a, state b) 
 * --------------------------
 * Euclidean cost between state a and state b.
 */
double MoveIncremental::trueDist(state a, state b) {
  
  float x = a.x-b.x;
  float y = a.y-b.y;
  return sqrt(x*x + y*y);
  
}

/* double Dstar::heuristic(state a, state b)
 * --------------------------
 * Pretty self explanitory, the heristic we use is the 8-way distance
 * scaled by a constant C1 (should be set to <= min cost).
 */
double MoveIncremental::heuristic(state a, state b) {
  return eightCondist(a,b)*C1;
}

/* state Dstar::calculateKey(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
// DCHECK
state MoveIncremental::calculateKey(state u) {
  
  double val = fmin(getRHS(u),getG(u));
  
  u.k.first  = val + heuristic(u,s_start) + k_m;
  u.k.second = val;

  return u;
}

/* double Dstar::cost(state a, state b)
 * --------------------------
 * Returns the cost of moving from state a to state b. This could be
 * either the cost of moving off state a or onto state b, we went with
 * the former. This is also the 8-way cost.
 */
double MoveIncremental::cost(state a, state b) {

  int xd = abs(a.x-b.x);
  int yd = abs(a.y-b.y);
  double scale = 1;

  if (xd+yd>1) scale = M_SQRT2;

  if (cellHash.count(a) == 0) return scale*C1;
  return scale*cellHash[a].cost;

}
/* void Dstar::updateCell(int x, int y, double val)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void MoveIncremental::updateCell(int x, int y, double val) {
  
  state u;
  
  u.x = x;
  u.y = y;

  //if ((u == s_start) || (u == s_goal)) return;

  //makeNewCell(u); 

  if(cellHash[u].cost == val) {
    cellHash[u].cost_changed = false;
  } else {
    cellHash[u].cost_changed = true;
  }

  cellHash[u].cost = val;

  //updateVertex(u);
}
  
/* void Dstar::getSucc(state u,list<state> &s)
 * --------------------------
 * Returns a list of successor states for state u, since this is an
 * 8-way graph this list contains all of a cells neighbours. Unless
 * the cell is occupied in which case it has no successors. 
 */
void MoveIncremental::getSucc(state u,list<state> &s) {
  
  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  if (occupied(u)) return;

  makeNewCell(u);

  u.x += 1;
  s.push_front(u);
  makeNewCell(u);
  u.y += 1;
  s.push_front(u);
  makeNewCell(u);
  u.x -= 1;
  s.push_front(u);
  makeNewCell(u);
  u.x -= 1;
  s.push_front(u);
  makeNewCell(u);
  u.y -= 1;
  s.push_front(u);
  makeNewCell(u);
  u.y -= 1;
  s.push_front(u);
  makeNewCell(u);
  u.x += 1;
  s.push_front(u);
  makeNewCell(u);
  u.x += 1;
  s.push_front(u);
  makeNewCell(u);

}

/* void Dstar::getPred(state u,list<state> &s)
 * --------------------------
 * Returns a list of all the predecessor states for state u. Since
 * this is for an 8-way connected graph the list contails all the
 * neighbours for state u. Occupied neighbours are not added to the
 * list.
 */
void MoveIncremental::getPred(state u,list<state> &s) {
  
  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  state ua, ub;
  // u.x += 1;
  // if (!occupied(u)) s.push_front(u);
  // u.y += 1; // TODO
  // if (!occupied(u)) s.push_front(u);
  // u.x -= 1;
  // if (!occupied(u)) s.push_front(u);
  // u.x -= 1; // TODO
  // if (!occupied(u)) s.push_front(u);
  // u.y -= 1;
  // if (!occupied(u)) s.push_front(u);
  // u.y -= 1; // TODO
  // if (!occupied(u)) s.push_front(u);
  // u.x += 1; 
  // if (!occupied(u)) s.push_front(u);
  // u.x += 1; // TODO
  // if (!occupied(u)) s.push_front(u);

  makeNewCell(u);

  u.x += 1;
  if (!occupied(u)) {
    s.push_front(u);
    makeNewCell(u);
  }
  u.y += 1; // Case 1

  ua.x = u.x-1;
  ua.y = u.y;
  ub.x = u.x;
  ub.y = u.y-1;
  if (!occupied(u) && !occupied(ua) && !occupied(ub) ) {
    s.push_front(u);
    makeNewCell(u);
  }

  u.x -= 1;
  if (!occupied(u)) {
    s.push_front(u);
    makeNewCell(u);
  }
  
  u.x -= 1; // Case 2
  ua.x = u.x+1;
  ua.y = u.y;
  ub.x = u.x;
  ub.y = u.y-1;
  if (!occupied(u) && !occupied(ua) && !occupied(ub)) {
    s.push_front(u);
    makeNewCell(u);
  }

  u.y -= 1;
  if (!occupied(u)) {
    s.push_front(u);
    makeNewCell(u);
  }
  
  u.y -= 1; // Case 3
  ua.x = u.x;
  ua.y = u.y+1;
  ub.x = u.x+1;
  ub.y = u.y;
  if (!occupied(u) && !occupied(ua) && !occupied(ub)) {
    s.push_front(u);
    makeNewCell(u);
  }
  
  u.x += 1; 
  if (!occupied(u)) {
    s.push_front(u);
    makeNewCell(u);
  }
  
  u.x += 1; // Case 4
  ua.x = u.x;
  ua.y = u.y+1;
  ub.x = u.x-1;
  ub.y = u.y;
  if (!occupied(u) && !occupied(ua) && !occupied(ub)) {
    s.push_front(u);
    makeNewCell(u);
  }
  
}

/* void Dstar::updateStart(int x, int y)
 * --------------------------
 * Update the position of the robot, this does not force a replan.
 */
void MoveIncremental::updateStart(int x, int y) {

  s_start.x = x;
  s_start.y = y;
  
  // // comment out
  // k_m += heuristic(s_last,s_start);

  // s_start = calculateKey(s_start);
  // s_last  = s_start;
  
}

/* void Dstar::updateGoal(int x, int y)
 * --------------------------
 * This is somewhat of a hack, to change the position of the goal we
 * first save all of the non-empty on the map, clear the map, move the
 * goal, and re-add all of non-empty cells. Since most of these cells
 * are not between the start and goal this does not seem to hurt
 * performance too much. Also it free's up a good deal of memory we
 * likely no longer use.
 */
void MoveIncremental::updateGoal(int x, int y) {
  s_goal.x  = x;
  s_goal.y  = y;

  // list< pair<ipoint2, double> > toAdd;
  // pair<ipoint2, double> tp;
  
  // ds_ch::iterator i;
  // list< pair<ipoint2, double> >::iterator kk;
  
  // for(i=cellHash.begin(); i!=cellHash.end(); i++) {
  //   if (!close(i->second.cost, C1)) {
  //     tp.first.x = i->first.x;
  //     tp.first.y = i->first.y;
  //     tp.second = i->second.cost;
  //     toAdd.push_back(tp);
  //   }
  // }

  // cellHash.clear();
  // openHash.clear();

  // while(!openList.empty())
  //   openList.pop();
  
  // k_m = 0;
  
  // s_goal.x  = x;
  // s_goal.y  = y;

  // cellInfo tmp;
  // tmp.g = tmp.rhs =  0;
  // tmp.cost = C1;

  // cellHash[s_goal] = tmp;

  // tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  // tmp.cost = C1;
  // cellHash[s_start] = tmp;
  // s_start = calculateKey(s_start);

  // s_last = s_start;    

  // for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
  //   //updateCell(kk->first.x, kk->first.y, kk->second);

  //   state u;
    
  //   u.x = x;
  //   u.y = y;

  //   if ((u == s_start) || (u == s_goal)) return;

  //   makeNewCell(u); 

  //   if(cellHash[u].cost == kk->second) {
  //     cellHash[u].cost_changed = false;
  //   } else {
  //     cellHash[u].cost_changed = true;
  //   }

  //   cellHash[u].cost = kk->second;

  //   updateVertex(u);

  // }
  

}

/* bool Dstar::replan()
 * --------------------------
 * Updates the costs for all cells and computes the shortest path to
 * goal. Returns true if a path is found, false otherwise. The path is
 * computed by doing a greedy search over the cost+g values in each
 * cells. In order to get around the problem of the robot taking a
 * path that is near a 45 degree angle to goal we break ties based on
 *  the metric euclidean(state, goal) + euclidean(state,start). 
 */
// Main algorithm: procedure Main()
// 
// s_last = s_start
// Initialize()
// ComputeShortestPath()
// while (s_start != s_goal)
//  if(g_start == inf) then there is no path
//  s_start = arg min (s' in Succ(s_start)) (c(s_start, s')+g(s'))
//  move to s_start
//  scan graph for changed edge costs
//  if any edge costs changed
//      k_m = k_m + h(s_last, s_start)
//      s_last = s_start
//      for all directed edge (u, v) with changed edge costs
//          update the edge cost c(u, v)
//          UpdateVertex(u)
//      ComputeShortestPath() 
bool MoveIncremental::replan() {
  list<state> n;
  list<state>::iterator i;

  path.clear();

  // The following only gets called once in ROS:makeplan()
  // s_last = s_start
  // Initialize()
  // ComputeShortestPath()

  // while (s_start != s_goal)
  while(s_start != s_goal) {
    // if(g_start == inf) then there is no path
    if (std::isinf(getG(s_start))) {
      fprintf(stderr, "NO PATH TO GOAL\n");
      return false;
    }
    // s_start = arg min (s' in Succ(s_start)) (c(s_start, s')+g(s'))
    getSucc(s_start, n);    // n is a list of Succ of cur
    if (n.empty()) {
      fprintf(stderr, "NO PATH TO GOAL\n");
      return false;
    }    

    double cmin = INFINITY;
    double tmin;
    state smin;
    for (i=n.begin(); i!=n.end(); i++) {
      //if (occupied(*i)) continue;
      double val  = cost(s_start,*i);
      double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i); // (Euclidean) cost to goal + cost to pred
      val += getG(*i);

      if (close(val,cmin)) {
        if (tmin > val2) {
          tmin = val2;
          cmin = val;
          smin = *i;
        }
      } else if (val < cmin) {
        tmin = val2;
        cmin = val;
        smin = *i;
      }
    }
    s_start = smin;
    n.clear();

    // move to s_start
    path.push_back(s_start);
 
    ds_ch::iterator j;
    for(j=cellHash.begin(); j!=cellHash.end(); j++) {
      // scan graph for changed edge costs
      int x = j->first.x;
      int y = j->first.y;
      int index = cellCostmap->getIndex(x,y);
      double c = (double)cellCostGrid[index];

      if( c >= COST_POSSIBLY_CIRCUMSCRIBED)
          updateCell(x, y, -1);
      else if (c == costmap_2d::FREE_SPACE){
          updateCell(x, y, 1);
      }else
      {
          updateCell(x, y, c);
      }      
      // if any edge costs changed   
      if(j->second.cost_changed) {
        // k_m = k_m + h(s_last, s_start)
        k_m = k_m + heuristic(s_last, s_start);
        // s_last = s_start
        s_last = s_start;
        // for all directed edge (u, v) with changed edge costs
        //    update the edge cost c(u, v)
        //    UpdateVertex(u)
        //    ComputeShortestPath()
        updateVertex(j->first);
        computeShortestPath();
      }
    }

  }

  path.push_back(s_goal);
  return true;
}

};// namspace move_incremental