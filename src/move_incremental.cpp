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
#if 0
        static char costmap_filename[1000];
      static int file_number = 0;
      snprintf( costmap_filename, 1000, "MoveIncremental-dijkstra-costmap-%04d", file_number++ );
      savemap( costmap_filename );
#endif
        setupMoveIncremental(true);

        // calculate the nav fn and path
        propMoveIncrementalDijkstra(std::max(nx*ny/20,nx+ny),atStart);

        // path
        int len = calcPath(nx*ny/2);

        if (len > 0)			// found plan
        {
            ROS_INFO("[MoveIncremental] Path found, %d steps\n", len);
            return true;
        }
        else
        {
            ROS_INFO("[MoveIncremental] No path found\n");
            return false;
        }

    }


    //
    // calculate navigation function, given a costmap, goal, and start
    //

    bool
    MoveIncremental::calcMoveIncrementalAstar()
    {
        setupMoveIncremental(true);

        // calculate the nav fn and path
        propMoveIncrementalAstar(std::max(nx*ny/20,nx+ny));

        // path
        int len = calcPath(nx*4);

        if (len > 0)			// found plan
        {
            ROS_INFO("[MoveIncremental] Path found, %d steps\n", len);
            return true;
        }
        else
        {
            ROS_INFO("[MoveIncremental] No path found\n");
            return false;
        }
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
        // reset values in propagation arrays
        for (int i=0; i<ns; i++)
        {
            potarr[i] = POT_HIGH;
            if (!keepit) costarr[i] = COST_NEUTRAL;
            gradx[i] = grady[i] = 0.0;
        }

        // outer bounds of cost array
        COSTTYPE *pc;
        pc = costarr;
        for (int i=0; i<nx; i++)
            *pc++ = COST_OBS;
        pc = costarr + (ny-1)*nx;
        for (int i=0; i<nx; i++)
            *pc++ = COST_OBS;
        pc = costarr;
        for (int i=0; i<ny; i++, pc+=nx)
            *pc = COST_OBS;
        pc = costarr + nx - 1;
        for (int i=0; i<ny; i++, pc+=nx)
            *pc = COST_OBS;

        // priority buffers
        curT = COST_OBS;
        curP = pb1;
        curPe = 0;
        nextP = pb2;
        nextPe = 0;
        overP = pb3;
        overPe = 0;
        memset(pending, 0, ns*sizeof(bool));

        // set goal
        int k = goal[0] + goal[1]*nx;
        initCost(k,0);

        // find # of obstacle cells
        pc = costarr;
        int ntot = 0;
        for (int i=0; i<ns; i++, pc++)
        {
            if (*pc >= COST_OBS)
                ntot++;			// number of cells that are obstacles
        }
        nobs = ntot;
    }


    // initialize a goal-type cost for starting propagation

    void
    MoveIncremental::initCost(int k, float v)
    {
        potarr[k] = v;
        push_cur(k+1);
        push_cur(k-1);
        push_cur(k-nx);
        push_cur(k+nx);
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
        // get neighbors
        float u,d,l,r;
        l = potarr[n-1];
        r = potarr[n+1];
        u = potarr[n-nx];
        d = potarr[n+nx];
        //  ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
        //	 potarr[n], l, r, u, d);
        //  ROS_INFO("[Update] cost: %d\n", costarr[n]);

        // find lowest, and its lowest neighbor
        float ta, tc;
        if (l<r) tc=l; else tc=r;
        if (u<d) ta=u; else ta=d;

        // do planar wave update
        if (costarr[n] < COST_OBS)	// don't propagate into obstacles
        {
            float hf = (float)costarr[n]; // traversability factor
            float dc = tc-ta;		// relative cost between ta,tc
            if (dc < 0) 		// ta is lowest
            {
                dc = -dc;
                ta = tc;
            }

            // calculate new potential
            float pot;
            if (dc >= hf)		// if too large, use ta-only update
                pot = ta+hf;
            else			// two-neighbor interpolation update
            {
                // use quadratic approximation
                // might speed this up through table lookup, but still have to
                //   do the divide
                float d = dc/hf;
                float v = -0.2301*d*d + 0.5307*d + 0.7040;
                pot = ta + hf*v;
            }

            //      ROS_INFO("[Update] new pot: %d\n", costarr[n]);

            // now add affected neighbors to priority blocks
            if (pot < potarr[n])
            {
                float le = INVSQRT2*(float)costarr[n-1];
                float re = INVSQRT2*(float)costarr[n+1];
                float ue = INVSQRT2*(float)costarr[n-nx];
                float de = INVSQRT2*(float)costarr[n+nx];
                potarr[n] = pot;
                if (pot < curT)	// low-cost buffer block
                {
                    if (l > pot+le) push_next(n-1);
                    if (r > pot+re) push_next(n+1);
                    if (u > pot+ue) push_next(n-nx);
                    if (d > pot+de) push_next(n+nx);
                }
                else			// overflow block
                {
                    if (l > pot+le) push_over(n-1);
                    if (r > pot+re) push_over(n+1);
                    if (u > pot+ue) push_over(n-nx);
                    if (d > pot+de) push_over(n+nx);
                }
            }

        }

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
        // get neighbors
        float u,d,l,r;
        l = potarr[n-1];
        r = potarr[n+1];
        u = potarr[n-nx];
        d = potarr[n+nx];
        //ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
        //	 potarr[n], l, r, u, d);
        // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

        // find lowest, and its lowest neighbor
        float ta, tc;
        if (l<r) tc=l; else tc=r;
        if (u<d) ta=u; else ta=d;

        // do planar wave update
        if (costarr[n] < COST_OBS)	// don't propagate into obstacles
        {
            float hf = (float)costarr[n]; // traversability factor
            float dc = tc-ta;		// relative cost between ta,tc
            if (dc < 0) 		// ta is lowest
            {
                dc = -dc;
                ta = tc;
            }

            // calculate new potential
            float pot;
            if (dc >= hf)		// if too large, use ta-only update
                pot = ta+hf;
            else			// two-neighbor interpolation update
            {
                // use quadratic approximation
                // might speed this up through table lookup, but still have to
                //   do the divide
                float d = dc/hf;
                float v = -0.2301*d*d + 0.5307*d + 0.7040;
                pot = ta + hf*v;
            }

            //ROS_INFO("[Update] new pot: %d\n", costarr[n]);

            // now add affected neighbors to priority blocks
            if (pot < potarr[n])
            {
                float le = INVSQRT2*(float)costarr[n-1];
                float re = INVSQRT2*(float)costarr[n+1];
                float ue = INVSQRT2*(float)costarr[n-nx];
                float de = INVSQRT2*(float)costarr[n+nx];

                // calculate distance
                int x = n%nx;
                int y = n/nx;
                float dist = hypot(x-start[0], y-start[1])*(float)COST_NEUTRAL;

                potarr[n] = pot;
                pot += dist;
                if (pot < curT)	// low-cost buffer block
                {
                    if (l > pot+le) push_next(n-1);
                    if (r > pot+re) push_next(n+1);
                    if (u > pot+ue) push_next(n-nx);
                    if (d > pot+de) push_next(n+nx);
                }
                else
                {
                    if (l > pot+le) push_over(n-1);
                    if (r > pot+re) push_over(n+1);
                    if (u > pot+ue) push_over(n-nx);
                    if (d > pot+de) push_over(n+nx);
                }
            }

        }

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
        int nwv = 0;			// max priority block size
        int nc = 0;			// number of cells put into priority blocks
        int cycle = 0;		// which cycle we're on

        // set up start cell
        int startCell = start[1]*nx + start[0];

        for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
        {
            //
            if (curPe == 0 && nextPe == 0) // priority blocks empty
                break;

            // stats
            nc += curPe;
            if (curPe > nwv)
                nwv = curPe;

            // reset pending flags on current priority buffer
            int *pb = curP;
            int i = curPe;
            while (i-- > 0)
                pending[*(pb++)] = false;

            // process current priority buffer
            pb = curP;
            i = curPe;
            while (i-- > 0)
                updateCell(*pb++);

            if (displayInt > 0 &&  (cycle % displayInt) == 0)
                displayFn(this);

            // swap priority blocks curP <=> nextP
            curPe = nextPe;
            nextPe = 0;
            pb = curP;		// swap buffers
            curP = nextP;
            nextP = pb;

            // see if we're done with this priority level
            if (curPe == 0)
            {
                curT += priInc;	// increment priority threshold
                curPe = overPe;	// set current to overflow block
                overPe = 0;
                pb = curP;		// swap buffers
                curP = overP;
                overP = pb;
            }

            // check if we've hit the Start cell
            if (atStart)
                if (potarr[startCell] < POT_HIGH)
                    break;
        }

        ROS_INFO("[MoveIncremental] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
                  cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);

        if (cycle < cycles) return true; // finished up here
        else return false;
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
        int nwv = 0;			// max priority block size
        int nc = 0;			// number of cells put into priority blocks
        int cycle = 0;		// which cycle we're on

        // set initial threshold, based on distance
        float dist = hypot(goal[0]-start[0], goal[1]-start[1])*(float)COST_NEUTRAL;
        curT = dist + curT;

        // set up start cell
        int startCell = start[1]*nx + start[0];

        // do main cycle
        for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
        {
            //
            if (curPe == 0 && nextPe == 0) // priority blocks empty
                break;

            // stats
            nc += curPe;
            if (curPe > nwv)
                nwv = curPe;

            // reset pending flags on current priority buffer
            int *pb = curP;
            int i = curPe;
            while (i-- > 0)
                pending[*(pb++)] = false;

            // process current priority buffer
            pb = curP;
            i = curPe;
            while (i-- > 0)
                updateCellAstar(*pb++);

            if (displayInt > 0 &&  (cycle % displayInt) == 0)
                displayFn(this);

            // swap priority blocks curP <=> nextP
            curPe = nextPe;
            nextPe = 0;
            pb = curP;		// swap buffers
            curP = nextP;
            nextP = pb;

            // see if we're done with this priority level
            if (curPe == 0)
            {
                curT += priInc;	// increment priority threshold
                curPe = overPe;	// set current to overflow block
                overPe = 0;
                pb = curP;		// swap buffers
                curP = overP;
                overP = pb;
            }

            // check if we've hit the Start cell
            if (potarr[startCell] < POT_HIGH)
                break;

        }

        last_path_cost_ = potarr[startCell];

        ROS_INFO("[MoveIncremental] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
                  cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);


        if (potarr[startCell] < POT_HIGH) return true; // finished up here
        else return false;
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
        // test write
        //savemap("test");

        // check path arrays
        if (npathbuf < n)
        {
            if (pathx) delete [] pathx;
            if (pathy) delete [] pathy;
            pathx = new float[n];
            pathy = new float[n];
            npathbuf = n;
        }

        // set up start position at cell
        // st is always upper left corner for 4-point bilinear interpolation
        if (st == NULL) st = start;
        int stc = st[1]*nx + st[0];

        // set up offset
        float dx=0;
        float dy=0;
        npath = 0;

        // go for <n> cycles at most
        for (int i=0; i<n; i++)
        {
            // check if near goal
            int nearest_point=std::max(0,std::min(nx*ny-1,stc+(int)round(dx)+(int)(nx*round(dy))));
            if (potarr[nearest_point] < COST_NEUTRAL)
            {
                pathx[npath] = (float)goal[0];
                pathy[npath] = (float)goal[1];
                return ++npath;	// done!
            }

            if (stc < nx || stc > ns-nx) // would be out of bounds
            {
                ROS_INFO("[PathCalc] Out of bounds");
                return 0;
            }

            // add to path
            pathx[npath] = stc%nx + dx;
            pathy[npath] = stc/nx + dy;
            npath++;

            bool oscillation_detected = false;
            if( npath > 2 &&
                pathx[npath-1] == pathx[npath-3] &&
                pathy[npath-1] == pathy[npath-3] )
            {
                ROS_INFO("[PathCalc] oscillation detected, attempting fix.");
                oscillation_detected = true;
            }

            int stcnx = stc+nx;
            int stcpx = stc-nx;

            // check for potentials at eight positions near cell
            if (potarr[stc] >= POT_HIGH ||
                potarr[stc+1] >= POT_HIGH ||
                potarr[stc-1] >= POT_HIGH ||
                potarr[stcnx] >= POT_HIGH ||
                potarr[stcnx+1] >= POT_HIGH ||
                potarr[stcnx-1] >= POT_HIGH ||
                potarr[stcpx] >= POT_HIGH ||
                potarr[stcpx+1] >= POT_HIGH ||
                potarr[stcpx-1] >= POT_HIGH ||
                oscillation_detected)
            {
                ROS_INFO("[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);
                // check eight neighbors to find the lowest
                int minc = stc;
                int minp = potarr[stc];
                int st = stcpx - 1;
                if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
                st++;
                if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
                st++;
                if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
                st = stc-1;
                if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
                st = stc+1;
                if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
                st = stcnx-1;
                if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
                st++;
                if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
                st++;
                if (potarr[st] < minp) {minp = potarr[st]; minc = st; }
                stc = minc;
                dx = 0;
                dy = 0;

                ROS_INFO("[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
                          potarr[stc], pathx[npath-1], pathy[npath-1]);

                if (potarr[stc] >= POT_HIGH)
                {
                    ROS_INFO("[PathCalc] No path found, high potential");
                    //savemap("MoveIncremental_highpot");
                    return 0;
                }
            }

                // have a good gradient here
            else
            {

                // get grad at four positions near cell
                gradCell(stc);
                gradCell(stc+1);
                gradCell(stcnx);
                gradCell(stcnx+1);


                // get interpolated gradient
                float x1 = (1.0-dx)*gradx[stc] + dx*gradx[stc+1];
                float x2 = (1.0-dx)*gradx[stcnx] + dx*gradx[stcnx+1];
                float x = (1.0-dy)*x1 + dy*x2; // interpolated x
                float y1 = (1.0-dx)*grady[stc] + dx*grady[stc+1];
                float y2 = (1.0-dx)*grady[stcnx] + dx*grady[stcnx+1];
                float y = (1.0-dy)*y1 + dy*y2; // interpolated y

                // show gradients
                ROS_INFO("[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
                          gradx[stc], grady[stc], gradx[stc+1], grady[stc+1],
                          gradx[stcnx], grady[stcnx], gradx[stcnx+1], grady[stcnx+1],
                          x, y);

                // check for zero gradient, failed
                if (x == 0.0 && y == 0.0)
                {
                    ROS_INFO("[PathCalc] Zero gradient");
                    return 0;
                }

                // move in the right direction
                float ss = pathStep/hypot(x, y);
                dx += x*ss;
                dy += y*ss;

                // check for overflow
                if (dx > 1.0) { stc++; dx -= 1.0; }
                if (dx < -1.0) { stc--; dx += 1.0; }
                if (dy > 1.0) { stc+=nx; dy -= 1.0; }
                if (dy < -1.0) { stc-=nx; dy += 1.0; }

            }

            //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
            //	     potarr[stc], x, y, pathx[npath-1], pathy[npath-1]);
        }

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
        char fn[4096];

        ROS_INFO("[MoveIncremental] Saving costmap and start/goal points");
        // write start and goal points
        sprintf(fn,"%s.txt",fname);
        FILE *fp = fopen(fn,"w");
        if (!fp)
        {
            ROS_WARN("Can't open file %s", fn);
            return;
        }
        fprintf(fp,"Goal: %d %d\nStart: %d %d\n",goal[0],goal[1],start[0],start[1]);
        fclose(fp);

        // write cost array
        if (!costarr) return;
        sprintf(fn,"%s.pgm",fname);
        fp = fopen(fn,"wb");
        if (!fp)
        {
            ROS_WARN("Can't open file %s", fn);
            return;
        }
        fprintf(fp,"P5\n%d\n%d\n%d\n", nx, ny, 0xff);
        fwrite(costarr,1,nx*ny,fp);
        fclose(fp);
    }

    /********************
     * D* Lite
     ********************
     */

    //
    // calculate navigation function, given a costmap, goal, and start
    //

    bool
    MoveIncremental::calcMoveIncrementalDstar()
    {
        setupMoveIncremental(true);

        // calculate the nav fn and path
        propMoveIncrementalDstar(std::max(nx*ny/20,nx+ny));

        // path
        int len = calcPath(nx*4);

        if (len > 0)            // found plan
        {
            ROS_INFO("[MoveIncremental] Path found, %d steps\n", len);
            return true;
        }
        else
        {
            ROS_INFO("[MoveIncremental] No path found\n");
            return false;
        }
    }

    //
    // Use D* method for setting priorities
    // Critical function: calculate updated potential value of a cell,
    //   given its neighbors' values
    // Planar-wave update calculation from two lowest neighbors in a 4-grid
    // Quadratic approximation to the interpolated value
    // No checking of bounds here, this function should be fast
    //

#define INVSQRT2 0.707106781

    inline void
    MoveIncremental::updateCellDstar(int n)
    {
        // get neighbors
        float u,d,l,r;
        l = potarr[n-1];
        r = potarr[n+1];
        u = potarr[n-nx];
        d = potarr[n+nx];
        //ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
        //   potarr[n], l, r, u, d);
        // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

        // find lowest, and its lowest neighbor
        float ta, tc;
        if (l<r) tc=l; else tc=r;
        if (u<d) ta=u; else ta=d;

        // do planar wave update
        if (costarr[n] < COST_OBS)  // don't propagate into obstacles
        {
            float hf = (float)costarr[n]; // traversability factor
            float dc = tc-ta;       // relative cost between ta,tc
            if (dc < 0)         // ta is lowest
            {
                dc = -dc;
                ta = tc;
            }

            // calculate new potential
            float pot;
            if (dc >= hf)       // if too large, use ta-only update
                pot = ta+hf;
            else            // two-neighbor interpolation update
            {
                // use quadratic approximation
                // might speed this up through table lookup, but still have to
                //   do the divide
                float d = dc/hf;
                float v = -0.2301*d*d + 0.5307*d + 0.7040;
                pot = ta + hf*v;
            }

            //ROS_INFO("[Update] new pot: %d\n", costarr[n]);

            // now add affected neighbors to priority blocks
            if (pot < potarr[n])
            {
                float le = INVSQRT2*(float)costarr[n-1];
                float re = INVSQRT2*(float)costarr[n+1];
                float ue = INVSQRT2*(float)costarr[n-nx];
                float de = INVSQRT2*(float)costarr[n+nx];

                // calculate distance
                int x = n%nx;
                int y = n/nx;
                float dist = hypot(x-start[0], y-start[1])*(float)COST_NEUTRAL;

                potarr[n] = pot;
                pot += dist;
                if (pot < curT) // low-cost buffer block
                {
                    if (l > pot+le) push_next(n-1);
                    if (r > pot+re) push_next(n+1);
                    if (u > pot+ue) push_next(n-nx);
                    if (d > pot+de) push_next(n+nx);
                }
                else
                {
                    if (l > pot+le) push_over(n-1);
                    if (r > pot+re) push_over(n+1);
                    if (u > pot+ue) push_over(n-nx);
                    if (d > pot+de) push_over(n+nx);
                }
            }

        }

    }

    //
    // main propagation function
    // D* method, best-first
    // uses Euclidean distance heuristic
    // runs for a specified number of cycles,
    //   or until it runs out of cells to update,
    //   or until the Start cell is found (atStart = true)
    //

    bool
    MoveIncremental::propMoveIncrementalDstar(int cycles)
    {
        int nwv = 0;            // max priority block size
        int nc = 0;         // number of cells put into priority blocks
        int cycle = 0;      // which cycle we're on

        // set initial threshold, based on distance
        float dist = hypot(goal[0]-start[0], goal[1]-start[1])*(float)COST_NEUTRAL;
        curT = dist + curT;

        // set up start cell
        int startCell = start[1]*nx + start[0];

        // do main cycle
        for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
        {
            //
            if (curPe == 0 && nextPe == 0) // priority blocks empty
                break;

            // stats
            nc += curPe;
            if (curPe > nwv)
                nwv = curPe;

            // reset pending flags on current priority buffer
            int *pb = curP;
            int i = curPe;
            while (i-- > 0)
                pending[*(pb++)] = false;

            // process current priority buffer
            pb = curP;
            i = curPe;
            while (i-- > 0)
                updateCellDstar(*pb++);

            if (displayInt > 0 &&  (cycle % displayInt) == 0)
                displayFn(this);

            // swap priority blocks curP <=> nextP
            curPe = nextPe;
            nextPe = 0;
            pb = curP;      // swap buffers
            curP = nextP;
            nextP = pb;

            // see if we're done with this priority level
            if (curPe == 0)
            {
                curT += priInc; // increment priority threshold
                curPe = overPe; // set current to overflow block
                overPe = 0;
                pb = curP;      // swap buffers
                curP = overP;
                overP = pb;
            }

            // check if we've hit the Start cell
            if (potarr[startCell] < POT_HIGH)
                break;

        }

        last_path_cost_ = potarr[startCell];

        ROS_INFO("[MoveIncremental] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
                  cycle,nc,(int)((nc*100.0)/(ns-nobs)),nwv);


        if (potarr[startCell] < POT_HIGH) return true; // finished up here
        else return false;
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


/* void Dstar::makeNewCell(state u)
 * --------------------------
 * Checks if a cell is in the hash table, if not it adds it in.
 */
void MoveIncremental::makeNewCell(state u) {
  
  if (cellHash.find(u) != cellHash.end()) return;

  cellInfo tmp;
  tmp.g       = tmp.rhs = heuristic(u,s_goal);
  tmp.cost    = C1;
  cellHash[u] = tmp;
  
}

/* double Dstar::getG(state u)
 * --------------------------
 * Returns the G value for state u.
 */
double MoveIncremental::getG(state u) {

  if (cellHash.find(u) == cellHash.end()) 
    return heuristic(u,s_goal);
  return cellHash[u].g;
  
}

/* double Dstar::getRHS(state u)
 * --------------------------
 * Returns the rhs value for state u.
 */
double MoveIncremental::getRHS(state u) {

  if (u == s_goal) return 0;  

  if (cellHash.find(u) == cellHash.end()) 
    return heuristic(u,s_goal);
  return cellHash[u].rhs;
  
}

/* void Dstar::setG(state u, double g)
 * --------------------------
 * Sets the G value for state u
 */
void MoveIncremental::setG(state u, double g) {
  
  makeNewCell(u);  
  cellHash[u].g = g; 
}

/* void Dstar::setRHS(state u, double rhs)
 * --------------------------
 * Sets the rhs value for state u
 */
double MoveIncremental::setRHS(state u, double rhs) {
  
  makeNewCell(u);
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
int MoveIncremental::computeShortestPath() {
  
  list<state> s;
  list<state>::iterator i;

  if (openList.empty()) return 1;

  int k=0;
  while ((!openList.empty()) && 
         (openList.top() < (s_start = calculateKey(s_start))) || 
         (getRHS(s_start) != getG(s_start))) {

    if (k++ > maxSteps) {
      fprintf(stderr, "At maxsteps\n");
      return -1;
    }


    state u;

    bool test = (getRHS(s_start) != getG(s_start));
    
    // lazy remove
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

    if (k_old < calculateKey(u)) { // u is out of date
      insert(u);
    } else if (getG(u) > getRHS(u)) { // needs update (got better)
      setG(u,getRHS(u));
      getPred(u,s);
      for (i=s.begin();i != s.end(); i++) {
        updateVertex(*i);
      }
    } else {   // g <= rhs, state has got worse
      setG(u,INFINITY);
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
void MoveIncremental::updateVertex(state u) {

  list<state> s;
  list<state>::iterator i;
 
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

  if (!close(getG(u),getRHS(u))) insert(u);
  
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

  if ((u == s_start) || (u == s_goal)) return;

  makeNewCell(u); 
  cellHash[u].cost = val;

  updateVertex(u);
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

  u.x += 1;
  s.push_front(u);
  u.y += 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.x -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.y -= 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);
  u.x += 1;
  s.push_front(u);

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

  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.y += 1; // Case 1

  ua.x = u.x-1;
  ua.y = u.y;
  ub.x = u.x;
  ub.y = u.y-1;
  if (!occupied(u) && !occupied(ua) && !occupied(ub) ) 
      s.push_front(u);


  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  
  u.x -= 1; // Case 2
  ua.x = u.x+1;
  ua.y = u.y;
  ub.x = u.x;
  ub.y = u.y-1;
  if (!occupied(u) && !occupied(ua) && !occupied(ub)) 
    s.push_front(u);

  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  
  u.y -= 1; // Case 3
  ua.x = u.x;
  ua.y = u.y+1;
  ub.x = u.x+1;
  ub.y = u.y;
  if (!occupied(u) && !occupied(ua) && !occupied(ub)) 
    s.push_front(u);
  
  u.x += 1; 
  if (!occupied(u)) s.push_front(u);
  
  u.x += 1; // Case 4
  ua.x = u.x;
  ua.y = u.y+1;
  ub.x = u.x-1;
  ub.y = u.y;
  if (!occupied(u) && !occupied(ua) && !occupied(ub)) 
    s.push_front(u);
  
}

/* void Dstar::updateStart(int x, int y)
 * --------------------------
 * Update the position of the robot, this does not force a replan.
 */
void MoveIncremental::updateStart(int x, int y) {

  s_start.x = x;
  s_start.y = y;
  
  k_m += heuristic(s_last,s_start);

  s_start = calculateKey(s_start);
  s_last  = s_start;
  
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
   
  list< pair<ipoint2, double> > toAdd;
  pair<ipoint2, double> tp;
  
  ds_ch::iterator i;
  list< pair<ipoint2, double> >::iterator kk;
  
  for(i=cellHash.begin(); i!=cellHash.end(); i++) {
    if (!close(i->second.cost, C1)) {
      tp.first.x = i->first.x;
      tp.first.y = i->first.y;
      tp.second = i->second.cost;
      toAdd.push_back(tp);
    }
  }

  cellHash.clear();
  openHash.clear();

  while(!openList.empty())
    openList.pop();
  
  k_m = 0;
  
  s_goal.x  = x;
  s_goal.y  = y;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;    

  for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
    updateCell(kk->first.x, kk->first.y, kk->second);
  }
  

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
bool MoveIncremental::replan() {

  path.clear();

  int res = computeShortestPath();
  //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
  if (res < 0) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }
  list<state> n;
  list<state>::iterator i;

  state cur = s_start; 

  if (std::isinf(getG(s_start))) {
    fprintf(stderr, "NO PATH TO GOAL\n");
    return false;
  }
  
  while(cur != s_goal) {
    
    path.push_back(cur);
    getSucc(cur, n);

    if (n.empty()) {
      fprintf(stderr, "NO PATH TO GOAL\n");
      return false;
    }

    double cmin = INFINITY;
    double tmin;
    state smin;
    
    for (i=n.begin(); i!=n.end(); i++) {
  
      //if (occupied(*i)) continue;
      double val  = cost(cur,*i);
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
    n.clear();
    cur = smin;
  }
  path.push_back(s_goal);
  return true;
}

};// namspace move_incremental