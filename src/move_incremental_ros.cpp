/*********************************************************************
*
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
* Author: Yijiang Huang, Fei Sun
*********************************************************************/

#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <pcl_conversions/pcl_conversions.h>

#include <move_incremental/move_incremental_ros.h>

//register this planner as a BaseGlobalPlanner plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(MoveIncremental, MoveIncrementalROS,
    move_incremental::MoveIncrementalROS, nav_core::BaseGlobalPlanner
)

namespace move_incremental
{

//! constructor
MoveIncrementalROS::MoveIncrementalROS()
    : costmap_(NULL), planner_(), initialized_(false), allow_unknown_(true) {}

//! constructor
MoveIncrementalROS::MoveIncrementalROS(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_(NULL), planner_(), initialized_(false), allow_unknown_(true)
{
  //initialize the planner
  initialize(name, costmap_ros);
}

//! constructor
MoveIncrementalROS::MoveIncrementalROS(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
    : costmap_(NULL), planner_(), initialized_(false), allow_unknown_(true)
{
  //initialize the planner
  initialize(name, costmap, global_frame);
}

void MoveIncrementalROS::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string global_frame)
{
  if (!initialized_)
  {
    ros::NodeHandle private_nh("~/" + name);

    try
    {
      costmap_ = costmap;

      global_frame_ = global_frame;

      planner_ = boost::shared_ptr<MoveIncremental>(
          new MoveIncremental(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));

      planner_->init(0, 0, 10, 10); //First initialization

    }
    catch (exception &e)
    {
      ROS_ERROR("An exception occurred. Exception Nr. %s", e.what());

    }

    private_nh.param("allow_unknown", allow_unknown_, true);
    private_nh.param("planner_window_x", planner_window_x_, 0.0);
    private_nh.param("planner_window_y", planner_window_y_, 0.0);
    private_nh.param("default_tolerance", default_tolerance_, 0.0);

    //get the tf prefix
    ros::NodeHandle prefix_nh;
    tf_prefix_ = tf::getPrefixParam(prefix_nh);

    ROS_INFO("[MoveIncremental] Parameters input finished");

    // setup publishers
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveIncrementalROS::makePlanService, this);
    private_nh.param("visualize_potential", visualize_potential_, false);

    //if we're going to visualize the potential array we need to advertise
    if (visualize_potential_)
    {
      potarr_pub_.advertise(private_nh, "potential", 1);
    }

    ROS_INFO("[MoveIncremental] msgs & service published");

    initialized_ = true;
  }
  else
  {
    ROS_WARN("[MoveIncremental] MoveIncremental Planner not intialized. Need reboot to try again.");
  }
}

void MoveIncrementalROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}

void MoveIncrementalROS::clearRobotCell(const tf::Stamped <tf::Pose> &global_pose, unsigned int mx, unsigned int my)
{
  if (!initialized_)
  {
    ROS_ERROR(
        "[MoveIncremental] This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  //set the associated costs in the cost map to be free
  costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool MoveIncrementalROS::makePlanService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
{
  makePlan(req.start, req.goal, resp.plan.poses);

  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = global_frame_;

  return true;
}

void MoveIncrementalROS::mapToWorld(double mx, double my, double &wx, double &wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

bool MoveIncrementalROS::makePlan(const geometry_msgs::PoseStamped &start,
                                  const geometry_msgs::PoseStamped &goal,
                                  std::vector <geometry_msgs::PoseStamped> &plan)
{
  return makePlan(start, goal, default_tolerance_, plan);
}

//! BaseGlobalPlanner Interface
bool MoveIncrementalROS::makePlan(const geometry_msgs::PoseStamped &start,
                                  const geometry_msgs::PoseStamped &goal, double tolerance,
                                  std::vector <geometry_msgs::PoseStamped> &plan)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (!initialized_)
  {
    ROS_ERROR(
        "[MoveIncremental] This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }

  //clear the plan, just in case
  plan.clear();

  ros::NodeHandle n;

  //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
  if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame_))
  {
    ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
              tf::resolve(tf_prefix_, global_frame_).c_str(),
              tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
    return false;
  }

  if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame_))
  {
    ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
              tf::resolve(tf_prefix_, global_frame_).c_str(),
              tf::resolve(tf_prefix_, start.header.frame_id).c_str());
    return false;
  }

  double wx = start.pose.position.x;
  double wy = start.pose.position.y;

  unsigned int mx, my;
  if (!costmap_->worldToMap(wx, wy, mx, my))
  {
    ROS_WARN(
        "[MoveIncremental] The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
    return false;
  }

  //clear the starting cell within the costmap because we know it can't be an obstacle
  tf::Stamped <tf::Pose> start_pose;
  tf::poseStampedMsgToTF(start, start_pose);
  clearRobotCell(start_pose, mx, my);

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.pose.position.x;
  wy = goal.pose.position.y;

  if (!costmap_->worldToMap(wx, wy, mx, my))
  {
    if (tolerance <= 0.0)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[MoveIncremental] The goal sent to the MoveIncremental planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }
    mx = 0;
    my = 0;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // TODO: shouldn't init (clean up & erase) everytime! This is anti-incremental.
//  // D* Lite, initialize start and goal
//  planner_->init(map_start[0], map_start[1], map_goal[0], map_goal[1]);

  geometry_msgs::PoseStamped s = start;
  geometry_msgs::PoseStamped g = goal;

  std::vector <geometry_msgs::PoseStamped> grid_plan;
  ROS_INFO("[MoveIncremental] Start To plan");

  // main plan call, first plan
  if (this->plan(grid_plan, s, g))
  {
    plan.clear();
    //cnt_no_plan_= 0;
    //cnt_make_plan_++ ;

    for (size_t i = 0; i < grid_plan.size(); i++)
    {

      geometry_msgs::PoseStamped posei;
      //posei.header.seq = cnt_make_plan_;
      posei.header.stamp = ros::Time::now();
      posei.header.frame_id = global_frame_; /// Check in which frame to publish
      posei.pose.position.x = grid_plan[i].pose.position.x;
      posei.pose.position.y = grid_plan[i].pose.position.y;
      posei.pose.position.z = 0.0;
      posei.pose.orientation.x = 0.0;
      posei.pose.orientation.y = 0.0;
      posei.pose.orientation.z = 0.0;
      posei.pose.orientation.w = 1.0;

      // we are not using orientation computed from planner now
      // posei.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, grid_plan[i].pose.position.z);

      plan.push_back(posei);
    }

    ROS_INFO("[MoveIncremental] SRL_DSTAR_LITE Path found");
    return true;
  }
  else
  {
    //cnt_no_plan_++;
    ROS_WARN("[MoveIncremental] NO PATH FOUND FROM THE D* Lite PLANNER");
    return false;
  }
}

void
MoveIncrementalROS::publishPlan(const std::vector <geometry_msgs::PoseStamped> &path, double r, double g, double b,
                                double a)
{
  if (!initialized_)
  {
    ROS_ERROR(
        "[MoveIncremental] This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }

  //create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());

  if (!path.empty())
  {
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = path[0].header.stamp;
  }

  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i = 0; i < path.size(); i++)
  {
    gui_path.poses[i] = path[i];
  }

  plan_pub_.publish(gui_path);
}

int MoveIncrementalROS::plan(std::vector <geometry_msgs::PoseStamped> &grid_plan,
                             geometry_msgs::PoseStamped &start, geometry_msgs::PoseStamped &goal)
{
  /// TODO plan using the D* Lite Object

  /// 0. Setting Start and Goal points
  /// start
  unsigned int start_mx;
  unsigned int start_my;
  double start_x = start.pose.position.x;
  double start_y = start.pose.position.y;
  costmap_->worldToMap(start_x, start_y, start_mx, start_my);
  ROS_INFO("[MoveIncremental] Update Start Point %f %f to %d %d", start_x, start_y, start_mx, start_my);
  planner_->updateStart(start_mx, start_my);

  /// goal
  unsigned int goal_mx;
  unsigned int goal_my;
  double goal_x = goal.pose.position.x;
  double goal_y = goal.pose.position.y;
  costmap_->worldToMap(goal_x, goal_y, goal_mx, goal_my);
  ROS_INFO("[MoveIncremental] Update Goal Point %f %f to %d %d", goal_x, goal_y, goal_mx, goal_my);
  planner_->updateGoal(goal_mx, goal_my);

  /// 1.Update Planner costs
  // TODO: cleanup to cut down running time, major bottleneck
  // proposal: scan over map, and only update vertex that has changed cost

  int nx_cells, ny_cells;
  nx_cells = costmap_->getSizeInCellsX();
  ny_cells = costmap_->getSizeInCellsY();
  ROS_INFO("[MoveIncremental] Update cell costs");

  unsigned char *grid = costmap_->getCharMap();
  for (int x = 0; x < (int) costmap_->getSizeInCellsX(); x++)
  {
    for (int y = 0; y < (int) costmap_->getSizeInCellsY(); y++)
    {
      int index = costmap_->getIndex(x, y);

      double c = (double) grid[index];
      //ROS_INFO_STREAM("[MoveIncremental]" << grid_plan[index]);

      if (c >= COST_POSSIBLY_CIRCUMSCRIBED)
      {
        // cost = -1 -> untraversable
        planner_->updateCell(x, y, -1);
      }
      else
      {
        if (c == costmap_2d::FREE_SPACE)
        {
          planner_->updateCell(x, y, 1);
        }
        else
        {
          planner_->updateCell(x, y, c);
        }
      }
    }
  }

  ROS_INFO("[MoveIncremental] Replan");
  /// 2. Plannig using D* Lite
  planner_->replan();

  ROS_INFO("[MoveIncremental] Get Path");
  /// 3. Get Path
  list <state> path_to_shortcut = planner_->getPath();

  list <state> path;

  // no shortcut
  path = path_to_shortcut;

  /// 4. Returning the path generated
  grid_plan.clear();
  grid_plan.push_back(start);

  // no smoothing
  double costmap_resolution = costmap_->getResolution();
  double origin_costmap_x = costmap_->getOriginX();
  double origin_costmap_y = costmap_->getOriginY();

  std::list<state>::const_iterator iterator;

  for (iterator = path.begin(); iterator != path.end(); ++iterator)
  {
    state node = *iterator;

    geometry_msgs::PoseStamped next_node;
    //next_node.header.seq = cnt_make_plan_;
    next_node.header.stamp = ros::Time::now();
    next_node.header.frame_id = global_frame_;

    next_node.pose.position.x = (node.x + 0.5) * costmap_resolution + origin_costmap_x;
    next_node.pose.position.y = (node.y + 0.5) * costmap_resolution + origin_costmap_y;
    next_node.pose.position.z = 0.0;

    next_node.pose.orientation.x = 0.0;
    next_node.pose.orientation.y = 0.0;
    next_node.pose.orientation.z = 0.0;
    next_node.pose.orientation.w = 1.0;

    grid_plan.push_back(next_node);
  }

  if (path.size() > 0)
  {
    // publishPath(grid_plan);
    publishPlan(grid_plan, 0.0, 1.0, 0.0, 0.0);
    return true;
  }
  else
  {
    return false;
  }
}

};// namspace move_incremental
