/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017 MIT Cognitive Robotics
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
* Author: Yijiang Huang, Fei Sun
*********************************************************************/
#ifndef PROJECT_MOVE_INCREMENTAL_ROS_H
#define PROJECT_MOVE_INCREMENTAL_ROS_H

#include <vector>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

#include <pcl_ros/publisher.h>

#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>

#include <move_incremental/move_incremental.h>
#include <move_incremental/potarr_point.h>

#define COST_POSSIBLY_CIRCUMSCRIBED 128

namespace move_incremental {
    /**
     * @class MoveIncrementalROS
     * @brief Provides a ROS wrapper for the MoveIncremental planner which runs D* Lite navigation function on a costmap.
     */
    class MoveIncrementalROS : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the MoveIncrementalROS object
         */
        MoveIncrementalROS();

        /**
         * @brief  Constructor for the MoveIncrementalROS object
         * @param  name The name of this planner
         * @param  costmap A pointer to the ROS wrapper of the costmap to use
         */
        MoveIncrementalROS(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        /**
         * @brief  Constructor for the MoveIncrementalROS object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use
         * @param  global_frame The global frame of the costmap
         */
        MoveIncrementalROS(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);

        /**
         * @brief  Destructor
         */
        ~MoveIncrementalROS() {}

        /**
         * BaseGlobalPlanner interface - initialize, with default global_frame
         * @brief  Initialization function for the MoveIncrementalROS object
         * @param  name The name of this planner
         * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        /**
         * BaseGlobalPlanner interface - initialize
         * @brief  Initialization function for the MoveIncrementalROS object
         * @param  name The name of this planner
         * @param  costmap A pointer to the costmap to use for planning
         * @param  global_frame The global frame of the costmap
         */
        void initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame);

        /**
         * BaseGlobalPlanner interface - makePlan, with default tolerance
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector <geometry_msgs::PoseStamped> &plan);

        /**
         * BaseGlobalPlanner interface - makePlan
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param tolerance The tolerance on the goal point for the planner
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      double tolerance,
                      std::vector <geometry_msgs::PoseStamped> &plan);

        /**
         * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
         * @param goal The goal pose to create a plan to
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool
        getPlanFromPotential(const geometry_msgs::PoseStamped &goal, std::vector <geometry_msgs::PoseStamped> &plan);

        /**
         * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
         * @param world_point The point to get the potential for
         * @return The navigation function's value at that point in the world
         */
        double getPointPotential(const geometry_msgs::Point &world_point);

        /**
         * @brief  Publish a path for visualization purposes
         */
        void publishPlan(const std::vector <geometry_msgs::PoseStamped> &path, double r, double g, double b, double a);


        bool makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);

        /**
         * Main plan function to invoke D* Lite
         * @param grid_plan
         * @param start
         * @param goal
         * @return
         */
        int plan(std::vector <geometry_msgs::PoseStamped> &grid_plan, geometry_msgs::PoseStamped &start,
                 geometry_msgs::PoseStamped &goal);

    private:

        inline double sq_distance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2) {
            double dx = p1.pose.position.x - p2.pose.position.x;
            double dy = p1.pose.position.y - p2.pose.position.y;
            return dx * dx + dy * dy;
        }

        void mapToWorld(double mx, double my, double &wx, double &wy);

        void clearRobotCell(const tf::Stamped <tf::Pose> &global_pose, unsigned int mx, unsigned int my);

    protected:

        //! Store a copy of the current costmap in \a costmap.  Called by makePlan.
        costmap_2d::Costmap2D* costmap_;

        //! D* Lite planner
        boost::shared_ptr<MoveIncremental> planner_;

        //! msg publisher
        ros::Publisher                   plan_pub_;
        pcl_ros::Publisher<PotarrPoint>  potarr_pub_;

        bool initialized_;
        bool allow_unknown_;
        bool visualize_potential_;

    private:

        ros::ServiceServer make_plan_srv_;

        double planner_window_x_, planner_window_y_, default_tolerance_;

        boost::mutex mutex_;

        std::string tf_prefix_;
        std::string global_frame_;
    };
};// namspace move_incremental

#endif //PROJECT_MOVE_INCREMENTAL_ROS_H
