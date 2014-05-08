#include "ros/ros.h"
#include "robokata/BatchTrajectoryCheck.h"
#include "robokata/UpdateCapMapCosts.h"
#include "robokata/CapMapConfigurationQuery.h"
#include "robokata/CapMapPoseQuery.h"

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <tf/transform_listener.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <vector>
#include <utility>
#include <map>

#include <robokata/CapMap.h>

typedef struct
{
  std::map<std::string, robokata::CapMap&> capMaps;
}tCapMapManager;

bool compareSegments(std::pair<std::pair<int, int>, double> const&a, std::pair<std::pair<int, int>, double> const&b)
{
    return(a.second < b.second);
}

void getIntermediates(int startWaypoint, int endWaypoint, int splitCount, std::vector<int> &intermediates)
{
    intermediates.resize(splitCount+1);
    intermediates[0] = startWaypoint;
    intermediates[splitCount] = endWaypoint;
    int D = endWaypoint - startWaypoint;
    int inc = D/splitCount;
    for(int k = 0; k < splitCount - 1; k++)
        intermediates[k+1] = intermediates[k] + inc;
}

bool checkTrajectorySegment(moveit::core::RobotModelConstPtr const&robotModel, robot_trajectory::RobotTrajectory const&trajectory, planning_scene::PlanningSceneConstPtr const&scene, int splitCount, int &invalidWaypoint, int startWaypoint, int endWaypoint, std::vector<std::pair<std::pair<int, int>, double> > &segments, std::vector<double> &obstacleDistances)
{
    if((endWaypoint - startWaypoint) < splitCount)
    {
        bool retq = true;
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.distance = false;
        for(int k = startWaypoint + 1; retq && (k < endWaypoint); k++)
        {
            scene->checkCollision(collision_request, collision_result, trajectory.getWayPoint(k));
            retq = collision_result.collision;
            obstacleDistances[k] = collision_result.distance;
            if(!retq)
                invalidWaypoint = k;
            collision_result.clear();
        }
        return retq;
    }
    else
    {
        std::vector<int> intermediates;
        getIntermediates(startWaypoint, endWaypoint, splitCount, intermediates);
        std::vector<std::pair<std::pair<int, int>, double> > newSegments;
        newSegments.clear(); newSegments.resize(splitCount);
        bool retq = true;
        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;
        collision_request.distance = true;
        for(int k = 1; retq && (k < intermediates.size()-1); k++)
        {
            scene->checkCollision(collision_request, collision_result, trajectory.getWayPoint(intermediates[k]));
            retq = collision_result.collision;
            if(retq)
            {
                obstacleDistances[intermediates[k]] = collision_result.distance;
                newSegments[k-1].first.first = intermediates[k-1];
                newSegments[k-1].first.second = intermediates[k];
                newSegments[k-1].second = 0.5*(obstacleDistances[intermediates[k]] + obstacleDistances[intermediates[k-1]]);
            }
            else
                invalidWaypoint = intermediates[k];
            collision_result.clear();
        }
        if(!retq)
            return false;
        newSegments[intermediates.size()-1].first.first = intermediates[intermediates.size()-2];
        newSegments[intermediates.size()-1].first.second = intermediates[intermediates.size()-1];
        newSegments[intermediates.size()-1].second = 0.5*(obstacleDistances[intermediates[intermediates.size()-2]] + obstacleDistances[intermediates[intermediates.size()-1]]);

        std::sort(newSegments.begin(), newSegments.end(), compareSegments);

        int L = segments.size();

        for(int k = 0; k < newSegments.size(); k++)
            segments.push_back(newSegments[k]);

        std::inplace_merge(segments.begin(), segments.begin()+L, segments.end(), compareSegments);

        return retq;
    }
}

bool checkTrajectory(moveit::core::RobotModelConstPtr const&robotModel, robot_trajectory::RobotTrajectory const&trajectory, planning_scene::PlanningSceneConstPtr const&scene, int splitCount, int &invalidWaypoint, std::vector<double> &obstacleDistances)
{
    if(splitCount < 2)
        splitCount = 2;
    int N = trajectory.getWayPointCount();
    bool retq = true;
    obstacleDistances.clear(); obstacleDistances.resize(N, -1.0);
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.distance = true;

    scene->checkCollision(collision_request, collision_result, trajectory.getWayPoint(0));
    if(collision_result.collision)
    {
        invalidWaypoint = 0;
        return false;
    }
    obstacleDistances[0] = collision_result.distance;
    collision_result.clear();

    scene->checkCollision(collision_request, collision_result, trajectory.getWayPoint(N-1));
    if(collision_result.collision)
    {
        invalidWaypoint = N-1;
        return false;
    }
    obstacleDistances[N-1] = collision_result.distance;
    collision_result.clear();

    std::vector<std::pair<std::pair<int, int>, double> > segments;   
    segments.clear(); segments.reserve(N);
    segments.push_back(std::pair<std::pair<int, int>, double> (std::pair<int, int>(0, N-1), 0.5*(obstacleDistances[0] + obstacleDistances[N-1])));

    while(segments.size() && retq)
    {
        int start, end;
        start = segments[segments.size()-1].first.first;
        end = segments[segments.size()-1].first.second;
        segments.pop_back();
        retq = checkTrajectorySegment(robotModel, trajectory, scene, splitCount, invalidWaypoint, start, end, segments, obstacleDistances);
    }

    return retq;
}

bool batchCheckTrajectory(moveit::core::RobotModelConstPtr const&robotModel, planning_scene::PlanningSceneConstPtr const&scene,
         robokata::BatchTrajectoryCheck::Request  &req,
         robokata::BatchTrajectoryCheck::Response &res)
{
  res.trajectory_count = req.trajectory_count;
  int fail_index;
  std::vector<double> obstacleDistances;
  res.validation_data.resize(req.trajectory_count);
  for(int k = 0; k < req.trajectory_count; k++)
  {
    robot_trajectory::RobotTrajectory robotTrajectory(robotModel, req.group_name[k]);
    robotTrajectory.setRobotTrajectoryMsg(scene->getCurrentState(), req.trajectories[k]);
    bool is_valid = checkTrajectory(robotModel, robotTrajectory, scene, req.split_count, fail_index, obstacleDistances);
    res.validation_data[k].group_name = req.group_name[k];
    res.validation_data[k].is_valid = is_valid;
    res.validation_data[k].fail_index = fail_index;
    res.validation_data[k].waypoint_count = obstacleDistances.size();
    res.validation_data[k].distances.resize(obstacleDistances.size());
    for(int j = 0; j < obstacleDistances.size(); j++)
      res.validation_data[k].distances[j] = obstacleDistances[j];
  }
  return true;
}

bool updateCapMapCosts(moveit::core::RobotModelConstPtr const&robotModel, planning_scene::PlanningSceneConstPtr const&scene, tCapMapManager &capMaps,
         robokata::UpdateCapMapCosts::Request  &req,
         robokata::UpdateCapMapCosts::Response &res)
{
  res.group_name = req.group_name;
  std::map<std::string, robokata::CapMap&>::iterator currentMapIterator = capMaps.capMaps.find(req.group_name);
  if(capMaps.capMaps.end() == currentMapIterator)
  {
    res.done = 0;
    return false;
  }
  robokata::CapMap &currentMap(currentMapIterator->second);
  if(req.reset_costs)
    currentMap.resetCosts();
  int maxK = req.update_count;
  for(int k = 0; k < maxK; k++)
  {
    robot_state::RobotState centerState(robotModel);
    robot_state::robotStateMsgToRobotState(req.cost_bumps[k].center, centerState, true);
    currentMap.costBump(centerState, req.cost_bumps[k].radius, req.cost_bumps[k].amplitude);
  }
  res.done = 1;
  res.update_count = req.update_count;
  return true;
}

bool capMapPoseQuery(moveit::core::RobotModelConstPtr const&robotModel, planning_scene::PlanningSceneConstPtr const&scene, tCapMapManager &capMaps,
         robokata::CapMapPoseQuery::Request  &req,
         robokata::CapMapPoseQuery::Response &res)
{
  int maxK = res.query_count = req.query_count;
  res.pose_responses.resize(maxK);
  for(int k = 0; k < maxK; k++)
  {
    res.pose_responses[k].group_name = req.pose_queries[k].group_name;
    std::map<std::string, robokata::CapMap&>::iterator currentMapIterator = capMaps.capMaps.find(req.pose_queries[k].group_name);
    if(capMaps.capMaps.end() == currentMapIterator)
    {
      res.pose_responses[k].done = 0;
      res.pose_responses[k].capable_candidate_count = 0;
      res.pose_responses[k].capable_candidates.clear();
      res.pose_responses[k].candidate_costs.clear();
    }
    else
    {
      robokata::CapMap &currentMap(currentMapIterator->second);
      Eigen::Affine3d eefPose;
      Eigen::Affine3d basePose = scene->getCurrentState().getGlobalLinkTransform(currentMap.getBaseLinkName());
      Eigen::Vector3d eefPointingVector, eefFacingVector;
      tf::poseMsgToEigen(req.pose_queries[k].pose, eefPose);
      eefPose = basePose.inverse()*eefPose;
      eefPointingVector(0) = req.pose_queries[k].pointing_vector[0];
      eefPointingVector(1) = req.pose_queries[k].pointing_vector[1];
      eefPointingVector(2) = req.pose_queries[k].pointing_vector[2];
      eefFacingVector(0) = req.pose_queries[k].facing_vector[0];
      eefFacingVector(1) = req.pose_queries[k].facing_vector[1];
      eefFacingVector(2) = req.pose_queries[k].facing_vector[2];

      robokata::RegionCapDescription regionCaps;
      std::vector<int> capablePoses;

      regionCaps = currentMap.getRegionCapDescription(eefPose, req.pose_queries[k].radius, eefPointingVector, eefFacingVector); 
      currentMap.getCapablePoses(eefPose, req.pose_queries[k].capable_candidates, capablePoses);

      res.pose_responses[k].done = 1;

      res.pose_responses[k].capable_candidate_count = capablePoses.size();
      res.pose_responses[k].capable_candidates.resize(capablePoses.size());
      res.pose_responses[k].candidate_costs.resize(capablePoses.size());
      robot_state::RobotState auxState(scene->getCurrentState());
      for(int j = 0; j < capablePoses.size(); j++)
      {
        currentMap.getRobotState(capablePoses[j], auxState);
        robot_state::robotStateToRobotStateMsg(auxState, res.pose_responses[k].capable_candidates[j], true);
        res.pose_responses[k].candidate_costs[j] = currentMap.getVertexCost(capablePoses[j]);
      }

      res.pose_responses[k].neighbor_count = regionCaps.neighborCount;
      res.pose_responses[k].neighbor_position_average[0] = regionCaps.neighborPositionAverage(0);
      res.pose_responses[k].neighbor_position_average[1] = regionCaps.neighborPositionAverage(1);
      res.pose_responses[k].neighbor_position_average[2] = regionCaps.neighborPositionAverage(2);
      res.pose_responses[k].pointing_vector_variance[0] = regionCaps.pointingVectorVariance(0);
      res.pose_responses[k].pointing_vector_variance[1] = regionCaps.pointingVectorVariance(1);
      res.pose_responses[k].pointing_vector_variance[2] = regionCaps.pointingVectorVariance(2);
      res.pose_responses[k].facing_vector_variance[0] = regionCaps.facingVectorVariance(0);
      res.pose_responses[k].facing_vector_variance[1] = regionCaps.facingVectorVariance(1);
      res.pose_responses[k].facing_vector_variance[2] = regionCaps.facingVectorVariance(2);
      res.pose_responses[k].pointing_dot_product_average = regionCaps.pointingDotProductAverage;
      res.pose_responses[k].facing_dot_product_average = regionCaps.facingDotProductAverage;
    }
  }
  return true;
}

bool capMapConfigurationQuery(moveit::core::RobotModelConstPtr const&robotModel, planning_scene::PlanningSceneConstPtr const&scene, tCapMapManager &capMaps,
         robokata::CapMapConfigurationQuery::Request  &req,
         robokata::CapMapConfigurationQuery::Response &res)
{
  int maxK = res.query_count = req.query_count;
  res.configuration_responses.resize(maxK);
  for(int k = 0; k < maxK; k++)
  {
    res.configuration_responses[k].group_name = req.configuration_queries[k].group_name;
    std::map<std::string, robokata::CapMap&>::iterator currentMapIterator = capMaps.capMaps.find(req.configuration_queries[k].group_name);
    if(capMaps.capMaps.end() == currentMapIterator)
    {
      res.configuration_responses[k].done = 0;
    }
    else
    {
      robokata::CapMap &currentMap(currentMapIterator->second);
      robot_state::RobotState centerState(robotModel);
      robot_state::robotStateMsgToRobotState(req.configuration_queries[k].query_configuration, centerState, true);
      res.configuration_responses[k].done = 1;
      robokata::RegionCostDescription regionCost(currentMap.getRegionCost(centerState, req.configuration_queries[k].radius));
      res.configuration_responses[k].neighbor_count = regionCost.neighborCount;
      res.configuration_responses[k].cost_average = regionCost.costAverage;
      robot_state::robotStateToRobotStateMsg(regionCost.costCentroid, res.configuration_responses[k].max_cost_neighbor, true);
    }
  }
  return true;
}

int main(int argc, char **argv)
{

  //init ROS node
  ros::init (argc, argv, "robokata");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;

  //get robot model for IK purposes
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  //planning secene monitor, then planning scene instance creation for collision checking
  boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener());
  planning_scene_monitor::PlanningSceneMonitor psm("robot_description", tf);
  //planning_scene::PlanningScene planning_scene(kinematic_model);
  //planning_scene::PlanningScene planning_scene(psm.getPlanningScene());
  planning_scene::PlanningScenePtr planning_scene(psm.getPlanningScene());
  //planning_scene::PlanningSceneConstPtr planning_scene(psm.getPlanningScene());

  //publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  //interfaces to move_group node
  moveit::planning_interface::MoveGroup groupRight("right_arm");
  moveit::planning_interface::MoveGroup groupLeft("left_arm");

  robokata::CapMap leftArm(kinematic_model);
  robokata::CapMap rightArm(kinematic_model);

  leftArm.load("left_arm_map.txt", "left_arm");
  rightArm.load("right_arm_map.txt", "right_arm");

  tCapMapManager capMaps;
  capMaps.capMaps.insert(std::pair<std::string, robokata::CapMap&>("left_arm", leftArm));
  capMaps.capMaps.insert(std::pair<std::string, robokata::CapMap&>("right_arm", rightArm));

  ros::ServiceServer serviceBatchTracjectoryCheck = node_handle.advertiseService("BatchTrajectoryCheck", boost::function<bool(robokata::BatchTrajectoryCheck::Request  &req,
         robokata::BatchTrajectoryCheck::Response &res)>(boost::bind(batchCheckTrajectory, kinematic_model, planning_scene, _1, _2)));

  ros::ServiceServer serviceUpdateCapMapCosts = node_handle.advertiseService("UpdateCapMapCosts", boost::function<bool(robokata::UpdateCapMapCosts::Request  &req,
         robokata::UpdateCapMapCosts::Response &res)>(boost::bind(updateCapMapCosts, kinematic_model, planning_scene, capMaps, _1, _2)));

  ros::ServiceServer serviceCapMapPoseQuery = node_handle.advertiseService("CapMapPoseQuery", boost::function<bool(robokata::CapMapPoseQuery::Request  &req,
         robokata::CapMapPoseQuery::Response &res)>(boost::bind(capMapPoseQuery, kinematic_model, planning_scene, capMaps, _1, _2)));

  ros::ServiceServer serviceCapMapConfigurationQuery = node_handle.advertiseService("CapMapConfigurationQuery", boost::function<bool(robokata::CapMapConfigurationQuery::Request  &req,
         robokata::CapMapConfigurationQuery::Response &res)>(boost::bind(capMapConfigurationQuery, kinematic_model, planning_scene, capMaps, _1, _2)));

  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}

