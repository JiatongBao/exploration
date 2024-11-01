/**************************************************************************
dual_state_graph.cpp
Implementation of dual_state graph. Create local and global graph according
to the new node from dynamic rrt.

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/

#include "dsvplanner/dual_state_graph.h"

#include <graph_utils.h>
#include <misc_utils/misc_utils.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "lkh_tsp_solver/lkh_interface.h"

namespace dsvplanner_ns
{
  bool DualStateGraph::readParameters()
  {
    nh_private_.getParam("/graph/world_frame_id", world_frame_id_);
    nh_private_.getParam("/graph/pub_local_graph_topic", pub_local_graph_topic_);
    nh_private_.getParam("/graph/pub_global_graph_topic", pub_global_graph_topic_);
    nh_private_.getParam("/graph/pub_global_points_topic", pub_global_points_topic_);
    nh_private_.getParam("/graph/sub_keypose_topic", sub_keypose_topic_);
    nh_private_.getParam("/graph/sub_path_topic", sub_path_topic_);
    nh_private_.getParam("/graph/sub_graph_planner_status_topic", sub_graph_planner_status_topic_);
    nh_private_.getParam("/graph/kCropPathWithTerrain", kCropPathWithTerrain);
    nh_private_.getParam("/graph/kConnectVertexDistMax", kConnectVertexDistMax);
    nh_private_.getParam("/graph/kDistBadEdge", kDistBadEdge);
    nh_private_.getParam("/graph/kDegressiveCoeff", kDegressiveCoeff);
    nh_private_.getParam("/graph/kDirectionCoeff", kDirectionCoeff);
    nh_private_.getParam("/graph/kExistingPathRatioThresholdGlobal", kExistingPathRatioThresholdGlobal);
    nh_private_.getParam("/graph/kExistingPathRatioThreshold", kExistingPathRatioThreshold);
    nh_private_.getParam("/graph/kLongEdgePenaltyMultiplier", kLongEdgePenaltyMultiplier);
    nh_private_.getParam("/graph/kMaxLongEdgeDist", kMaxLongEdgeDist);
    nh_private_.getParam("/graph/kMaxVertexAngleAlongZ", kMaxVertexAngleAlongZ);
    nh_private_.getParam("/graph/kMaxVertexDiffAlongZ", kMaxVertexDiffAlongZ);
    nh_private_.getParam("/graph/kMaxDistToPrunedRoot", kMaxDistToPrunedRoot);
    nh_private_.getParam("/graph/kMaxPrunedNodeDist", kMaxPrunedNodeDist);
    nh_private_.getParam("/graph/kMinVertexDist", kMinVertexDist);
    nh_private_.getParam("/graph/kSurroundRange", kSurroundRange);
    nh_private_.getParam("/graph/kMinGainRange", kMinGainRange);
    nh_private_.getParam("/graph/kMinDistanceToRobotToCheck", kMinDistanceToRobotToCheck);
    nh_private_.getParam("/rm/kBoundX", robot_bounding[0]);
    nh_private_.getParam("/rm/kBoundY", robot_bounding[1]);
    nh_private_.getParam("/rm/kBoundZ", robot_bounding[2]);

    // jtbao
    nh_private_.getParam("/graph/pub_local_frontier_cluster_topic_", pub_local_frontier_cluster_topic_);
    nh_private_.getParam("/graph/pub_local_candidate_goal_topic_", pub_local_candidate_goal_topic_);
    nh_private_.getParam("/graph/pub_candidate_goal_points_topic", pub_candidate_goal_points_topic_);
    nh_private_.getParam("tsp_dir", tsp_dir_);

    return true;
  }

  void DualStateGraph::setParams(Params params)
  {
    params_ = params;
  }

  //发布局部图，具体是两种，
  // local_graph_pub_发布的是graph_utils::TopologicalGraph形式的局部图，
  // graph_points_pub_发布的是pointcloud2形式的局部图
  void DualStateGraph::publishLocalGraph()
  {
    // Publish the current local graph
    local_graph_.header.stamp = ros::Time::now();
    local_graph_.header.frame_id = world_frame_id_;
    local_graph_pub_.publish(local_graph_);

    // graph_point is used to detect frontiers
    graph_point_cloud_->points.clear();
    for (int i = 0; i < local_graph_.vertices.size(); i++)
    {
      pcl::PointXYZ p1;
      p1.x = local_graph_.vertices[i].location.x;
      p1.y = local_graph_.vertices[i].location.y;
      p1.z = local_graph_.vertices[i].location.z;
      graph_point_cloud_->points.push_back(p1);
    }
    sensor_msgs::PointCloud2 graph_pc;
    pcl::toROSMsg(*graph_point_cloud_, graph_pc);
    graph_pc.header.frame_id = "map";
    graph_points_pub_.publish(graph_pc);
  }

  // jtbao add
  //发布局部图中增益不为零的节点以及局部目标点
  void DualStateGraph::publishLocalGraphVertexHasGain()
  {
    
    local_candidate_goal_pc_->points.clear();
    for (int i = 0; i < cluster_centroids_.size(); i++)
    {
      pcl::PointXYZ p1;
      p1.x = cluster_centroids_[i][0];
      p1.y = cluster_centroids_[i][1];
      p1.z = cluster_centroids_[i][2];
      local_candidate_goal_pc_->points.push_back(p1);
    }
    sensor_msgs::PointCloud2 goal_pc;
    pcl::toROSMsg(*local_candidate_goal_pc_, goal_pc);
    goal_pc.header.frame_id = "map";
    local_candidate_goal_pub_.publish(goal_pc);
    
    local_frontier_cluster_pc_->points.clear();
    for (int i = 0; i < local_graph_.vertices.size(); i++)
    {
      if(local_graph_.vertices[i].information_gain>0)
      {
        pcl::PointXYZ p1;
        p1.x = local_graph_.vertices[i].location.x;
        p1.y = local_graph_.vertices[i].location.y;
        p1.z = local_graph_.vertices[i].location.z;
        local_frontier_cluster_pc_->points.push_back(p1);
      }
    }
    sensor_msgs::PointCloud2 cluster_pc;
    pcl::toROSMsg(*local_frontier_cluster_pc_, cluster_pc);
    cluster_pc.header.frame_id = "map";
    local_frontier_cluster_pub_.publish(cluster_pc);
  }

  void DualStateGraph::setLastLocalGoal(StateVec goal)
  {
    last_local_goal_ = goal;
  }

  StateVec DualStateGraph::getLastLocalGoal()
  {
    return last_local_goal_;
  }

  void DualStateGraph::addAsCleanedGoal(StateVec goal)
  {
    if(!isCleanedGoal(goal))
      cleaned_goals_.push_back(goal);
  }

  bool DualStateGraph::isCleanedGoal(StateVec goal)
  {
    for(int i=0;i<cleaned_goals_.size();i++){
      StateVec qry = cleaned_goals_[i];
      double dist = sqrt(SQ(goal[0] - qry[0]) + SQ(goal[1] - qry[1]) + SQ(goal[2] - qry[2]));
      if(dist<1.0)
        return true;
    }
    return false;
  }

  void DualStateGraph::cleanLastLocalGoal()
  {
    addAsCleanedGoal(last_local_goal_);
  }

  double DualStateGraph::getDistFromNearestLocalGraphPointTo(const StateVec &goal)
  {
    double dist;
    geometry_msgs::Point goalPoint;
    goalPoint.x = goal[0];
    goalPoint.y = goal[1];
    goalPoint.z = goal[2];
    int vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(local_graph_, goalPoint);
    if(vertex_idx==-1) return 0;

    StateVec pnt;
    pnt[0] = local_graph_.vertices[vertex_idx].location.x;
    pnt[1] = local_graph_.vertices[vertex_idx].location.y;
    pnt[2] = local_graph_.vertices[vertex_idx].location.z;
    dist = sqrt(SQ(goal[0] - pnt[0]) + SQ(goal[1] - pnt[1]));
    return dist;
  }

  int DualStateGraph::getSafetyVertexIdxToPoint(const StateVec& pnt)
  {
    int best_idx = -1;
    double best_distance = INFINITY;
    for (int i = local_graph_.vertices.size() - 1; i >= 0; i--)
    {
      StateVec p1;
      p1[0] = local_graph_.vertices[i].location.x;
      p1[1] = local_graph_.vertices[i].location.y;
      p1[2] = local_graph_.vertices[i].location.z;
      double length = sqrt(SQ(p1[0] - pnt[0]) + SQ(p1[1] - pnt[1]));
      if (length > best_distance || fabs(p1[2] - pnt[2]) > params_.kMaxExtensionAlongZ)
      {
        continue;
      }
      if (volumetric_mapping::OctomapManager::CellStatus::kOccupied ==
          manager_->getLineStatusBoundingBox(p1, pnt, params_.boundingBox))
      {
        continue;
      }
      best_distance = length;
      best_idx = i;
    }

    if(best_idx==-1){
      int start_vertex_id = 0;
      geometry_msgs::Point goalPoint;
      goalPoint.x = pnt[0];
      goalPoint.y = pnt[1];
      goalPoint.z = pnt[2];
      int goal_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(local_graph_, goalPoint);
      std::vector<int> path;
      graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, start_vertex_id, goal_vertex_idx);
      if(path.size()>0)
        best_idx = path[path.size()/2];
      else
        best_idx = goal_vertex_idx;
    }
    
    return best_idx;
  }

  bool DualStateGraph::checkLocalGraphHasGainInLargerRange(StateVec &goal)
  {
    int maxGain = 0;
    int maxGainIdx = -1;
    int nodeNum = local_graph_.vertices.size();
    if(nodeNum==0) return false;
    
    for (int i = nodeNum-1; i >nodeNum/2 ; i--)
    {
      StateVec state;
      state[0] = local_graph_.vertices[i].location.x;
      state[1] = local_graph_.vertices[i].location.y;
      state[2]= local_graph_.vertices[i].location.z;

      const double disc = 0.5;
      const double searchRadius = params_.kGainRange * 3;
      int gain = 0;

      StateVec origin(state[0], state[1], state[2]);
      StateVec vec;
      double rangeSq = pow(searchRadius, 2.0);

      // Iterate over all nodes within the allowed distance
      for (vec[0] = state[0] - searchRadius; vec[0] < state[0] + searchRadius; vec[0] += disc)
      {
        for (vec[1] = state[1] - searchRadius; vec[1] < state[1] + searchRadius; vec[1] += disc)
        {
          for (vec[2] = state[2] - 0.4; vec[2] < state[2] + 0.4; vec[2] += disc)
          {
            StateVec dir = vec - origin;
            // Skip if distance is too large
            if (dir.transpose().dot(dir) > rangeSq)
            {
              continue;
            }
            bool insideAFieldOfView = false;
            // Check that voxel center is inside the field of view. This check is
            // for velodyne.
            if (fabs(dir[2] < sqrt(dir[0] * dir[0] + dir[1] * dir[1]) * tan(M_PI * params_.sensorVerticalView / 360)))
            {
              insideAFieldOfView = true;
            }
            if (!insideAFieldOfView)
            {
              continue;
            }

            // Check cell status and add to the gain considering the corresponding
            // factor.
            double probability;
            volumetric_mapping::OctomapManager::CellStatus node = manager_->getCellProbabilityPoint(vec, &probability);
            if (node == volumetric_mapping::OctomapManager::CellStatus::kUnknown)
            {
              if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
                  this->manager_->getVisibility(origin, vec, false))
              {
                gain += params_.kGainUnknown;
              }
            }
            else if (node == volumetric_mapping::OctomapManager::CellStatus::kOccupied)
            {
              if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
                  this->manager_->getVisibility(origin, vec, false))
              {
                gain += params_.kGainOccupied;
              }
            }
            else
            {
              if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
                  this->manager_->getVisibility(origin, vec, false))
              {
                gain += params_.kGainFree;
              }
            }
          }
        }
      }
      if(gain > params_.kMinEffectiveGain && !isOnTrajectory(state)){
        if(gain>maxGain){
          maxGain = gain;
          maxGainIdx = i;
        }
      }
    }
    if(maxGainIdx>=0){
      goal[0] = local_graph_.vertices[maxGainIdx].location.x;
      goal[1] = local_graph_.vertices[maxGainIdx].location.y;
      goal[2] = local_graph_.vertices[maxGainIdx].location.z;
      return true;
    }
    return false;
  }

  void DualStateGraph::removeCandidateGoalPoint(const StateVec &point)
  {
    bool updated = false;
    while(1){
      bool exist = false;
      int num = candidate_goals_.size();
      for (int i = 0; i < num; i++)
      {
        StateVec pt;
        pt[0] = candidate_goals_[i][0];
        pt[1] = candidate_goals_[i][1];
        pt[2] = candidate_goals_[i][2];

        StateVec direction;
        direction[0] = point[0] - pt[0];
        direction[1] = point[1] - pt[1];
        direction[2] = point[2] - pt[2];
        if (direction.norm() < CLUSTERING_EPSILON)
        {
          // remove from the list
          candidate_goals_.erase(candidate_goals_.begin() + i);
          updated = true;
          exist = true;
          break;
        }
      }
      if(!exist) break;
    }
    
    if (updated)
      publishCandidateGoals();
  }

  void DualStateGraph::publishCandidateGoals()
  {
    candidate_goal_point_cloud_->points.clear();
    for (int i = 0; i < candidate_goals_.size(); i++)
    {
      pcl::PointXYZ p1;
      p1.x = candidate_goals_[i][0];
      p1.y = candidate_goals_[i][1];
      p1.z = candidate_goals_[i][2];
      candidate_goal_point_cloud_->points.push_back(p1);
    }
    sensor_msgs::PointCloud2 gain_pc;
    pcl::toROSMsg(*candidate_goal_point_cloud_, gain_pc);
    gain_pc.header.frame_id = "map";
    candidate_goal_points_pub_.publish(gain_pc);
  }

  // jtbao add
  int DualStateGraph::clusteringLocalGraphVertexHasGain()
  {
    for (int i = 0; i < graph_gain_point_cloud_clusters_.size(); i++)
    {
      graph_gain_point_cloud_clusters_[i].clear();
    }
    graph_gain_point_cloud_clusters_.clear();

    int clusterNum = 0;
    for (int i = 0; i < local_graph_.vertices.size(); i++)
    {
      if (local_graph_.vertices[i].information_gain > 0)
      {
        Eigen::Vector4d pt;
        pt[0] = local_graph_.vertices[i].location.x;
        pt[1] = local_graph_.vertices[i].location.y;
        pt[2] = local_graph_.vertices[i].location.z;
        pt[3] = local_graph_.vertices[i].information_gain;

        if (clusterNum == 0)
        {
          std::vector<Eigen::Vector4d> pc;
          pc.push_back(pt);
          graph_gain_point_cloud_clusters_.push_back(pc);
          clusterNum++;
          continue;
        }

        bool found = false;
        for (int c = 0; c < clusterNum; c++)
        {
          for (int k = 0; k < graph_gain_point_cloud_clusters_[c].size(); k++)
          {
            Eigen:Vector4d refPoint = graph_gain_point_cloud_clusters_[c][k];
            float dist = sqrt(SQ(pt[0] - refPoint[0]) + SQ(pt[1] - refPoint[1]) + SQ(pt[2] - refPoint[2]));
            if (dist < CLUSTERING_EPSILON)
            {
              graph_gain_point_cloud_clusters_[c].push_back(pt);
              found = true;
              break;
            }
          }
          if (found)
            break;
        }
        if (!found)
        {
          std::vector<Eigen::Vector4d> pc;
          pc.push_back(pt);
          graph_gain_point_cloud_clusters_.push_back(pc);
          clusterNum++;
        }
      }
    }

    // remove cluster has no more than 3 points
    while (1)
    {
      bool exist = false;
      for (int i = 0; i < graph_gain_point_cloud_clusters_.size(); i++)
      {
        int clusterPointNum = graph_gain_point_cloud_clusters_[i].size();
        if (clusterPointNum <= 3)
        {
          graph_gain_point_cloud_clusters_[i].clear();
          graph_gain_point_cloud_clusters_.erase(graph_gain_point_cloud_clusters_.begin() + i);
          exist = true;
          break;
        }
      }
      if (!exist)
        break;
    }


    // calculate candidate goals from each cluster
    calcLocalCandidateGoals();

    publishLocalGraphVertexHasGain();

    return cluster_centroids_.size();
  }

  bool DualStateGraph::isClusterCentroid(StateVec pt)
  {
    for(int i=0;i<cluster_centroids_.size();i++){
      StateVec qry = cluster_centroids_[i];
      double dist = sqrt(SQ(pt[0] - qry[0]) + SQ(pt[1] - qry[1]) + SQ(pt[2] - qry[2]));
      if(dist<1.0){
        return true;
      }
    }
    return false;
  }

  void DualStateGraph::clearCandidateGoalsInLocalBoundary()
  {
    // get robot position
    if (local_graph_.vertices.size() == 0)
      return;

    StateVec robotPos;
    robotPos[0] = local_graph_.vertices[0].location.x;
    robotPos[1] = local_graph_.vertices[0].location.y;
    robotPos[2] = local_graph_.vertices[0].location.z;

    std::vector<StateVec> listPoints;
    for (int i = 0; i < candidate_goals_.size(); i++)
    {
      if (abs(candidate_goals_[i][0] - robotPos[0]) <= params_.kMaxXLocalBound && abs(candidate_goals_[i][1] - robotPos[1]) <= params_.kMaxYLocalBound)
      {
        listPoints.push_back(candidate_goals_[i]);
      }
    }
    for (int i = 0; i < listPoints.size(); i++)
    {
      removeCandidateGoalPoint(listPoints[i]);
    }
    listPoints.clear();
  }

  void DualStateGraph::addGlobalClusterIntoLocalClusters()
  {
    // get robot position
    if (local_graph_.vertices.size() == 0)
      return;

    StateVec robotPos;
    robotPos[0] = local_graph_.vertices[0].location.x;
    robotPos[1] = local_graph_.vertices[0].location.y;
    robotPos[2] = local_graph_.vertices[0].location.z;

    for (int i = 0; i < candidate_goals_.size(); i++)
    {
      if (abs(candidate_goals_[i][0] - robotPos[0]) <= params_.kMaxXLocalBound && abs(candidate_goals_[i][1] - robotPos[1]) <= params_.kMaxYLocalBound)
      {
        if(!isClusterCentroid(candidate_goals_[i]))
          cluster_centroids_.push_back(candidate_goals_[i]);
      }
    }
  }

  // remove cluster centroids that lies on the robot trajectory
  bool DualStateGraph::isOnTrajectory(const StateVec &point)
  {
    int cnt = 0;
    for(int i=robot_trajectory_.size()-1; i>=0; i--){
      if(cnt>50) return false; //only consider recent 50m
      double dist = sqrt(SQ(robot_trajectory_[i][0] - point[0]) + SQ(robot_trajectory_[i][1] - point[1]) + SQ(robot_trajectory_[i][2] - point[2]));
      if(dist<CLUSTERING_EPSILON)
        return true;
      cnt++;
    }
    return false;
  }

  StateVec DualStateGraph::calcMean(std::vector<StateVec> cluster)
  {
    int pointNum = cluster.size();
    StateVec mean;
    for (int k = 0; k < pointNum; k++)
    {
      mean[0] += cluster[k][0];
      mean[1] += cluster[k][1];
      mean[2] += cluster[k][2];
    }
    mean[0] /= pointNum;
    mean[1] /= pointNum;
    mean[2] /= pointNum;
    return mean;
  }

  StateVec DualStateGraph::findMaxGainPoint(std::vector<Eigen::Vector4d> cluster)
  {
    StateVec res;
    res[0] = 0, res[1] = 0, res[2] = 0;
    int pointNum = cluster.size();
    if(pointNum==0){
      return res;
    }
    double maxGain = 0.00001;
    int maxGainK = -1;
    for (int k = 0; k < pointNum; k++)
    {
      Eigen::Vector4d refPoint = cluster[k];
      if(refPoint[3]>maxGain){
        maxGain = refPoint[3];
        maxGainK = k;
      }
    }
    res[0] = cluster[maxGainK][0];
    res[1] = cluster[maxGainK][1];
    res[2] = cluster[maxGainK][2];
    return res;
  }

  StateVec DualStateGraph::adjustCentroid(std::vector<StateVec> cluster, StateVec robotPos)
  {
    int pointNum = cluster.size();
    StateVec centroid;
    for (int k = 0; k < pointNum; k++)
    {
      centroid[0] += cluster[k][0];
      centroid[1] += cluster[k][1];
      centroid[2] += cluster[k][2];
    }
    centroid[0] /= pointNum;
    centroid[1] /= pointNum;
    centroid[2] /= pointNum;

    StateVec moveDir;
    moveDir[0] = centroid[0] - robotPos[0];
    moveDir[1] = centroid[1] - robotPos[1];
    moveDir[2] = 0;

    double maxDist = 0.00001;
    int maxDistK = -1;
    for (int k = 0; k < pointNum; k++)
    {
      StateVec refPoint = cluster[k];
      StateVec dir;
      dir[0] = refPoint[0] - robotPos[0];
      dir[1] = refPoint[1] - robotPos[1];
      dir[2] = 0;

      double cosAng = moveDir.normalized().dot(dir.normalized());
      int angle = acos(cosAng) / M_PI * 180;

      if(angle<5){
        double dist = sqrt(SQ(robotPos[0] - refPoint[0]) + SQ(robotPos[1] - refPoint[1]) + SQ(robotPos[2] - refPoint[2]));
        if(dist>maxDist){
          maxDist = dist;
          maxDistK = k;
        }
      }
    }
    if(maxDistK>=0){
      centroid[0] = cluster[maxDistK][0];
      centroid[1] = cluster[maxDistK][1];
      centroid[2] = cluster[maxDistK][2];
    }
    return centroid;
  }

  void DualStateGraph::calcLocalCandidateGoals()
  {
    StateVec robotPos;
    robotPos[0] = local_graph_.vertices[0].location.x;
    robotPos[1] = local_graph_.vertices[0].location.y;
    robotPos[2] = local_graph_.vertices[0].location.z;

    cluster_centroids_.clear();
    for (int c = 0; c < graph_gain_point_cloud_clusters_.size(); c++)
    {
      int clusterPointNum = graph_gain_point_cloud_clusters_[c].size();

      StateVec centroid;
      centroid[0] = 0, centroid[1] = 0, centroid[2] = 0;
      
      
      for (int k = 0; k < clusterPointNum; k++)
      {
        Eigen::Vector4d refPoint = graph_gain_point_cloud_clusters_[c][k];
        centroid[0] += refPoint[0];
        centroid[1] += refPoint[1];
        centroid[2] += refPoint[2];
      }
      centroid[0] /= clusterPointNum;
      centroid[1] /= clusterPointNum;
      centroid[2] /= clusterPointNum;

      if(clusterPointNum<=3){
        cluster_centroids_.push_back(centroid);
        continue;
      }

      Eigen::Vector2d mean;
      mean[0] = centroid[0];
      mean[1] = centroid[1];

      bool needSplit = false;
      #ifdef SPLIT_CLUSTER
      for (int k = 0; k < clusterPointNum; k++)
      {
        Eigen::Vector4d refPoint = graph_gain_point_cloud_clusters_[c][k];
        Eigen::Vector2d point;
        point[0] = refPoint[0];
        point[1] = refPoint[1];
        if((point - mean).norm() > CLUSTERING_SPLIT_THRESHOLD)
          needSplit = true;
      }
      #endif
      if(needSplit){
        //compute principal component
        Eigen::Matrix2d cov;
        cov.setZero();
        for (int k = 0; k < clusterPointNum; k++)
        {
          Eigen::Vector4d refPoint = graph_gain_point_cloud_clusters_[c][k];
          Eigen::Vector2d point;
          point[0] = refPoint[0];
          point[1] = refPoint[1];
          Eigen::Vector2d diff = point - mean;
          cov += diff * diff.transpose();
        }
        cov /= double(clusterPointNum);
        Eigen::EigenSolver<Eigen::Matrix2d> es(cov);
        auto values = es.eigenvalues().real();
        auto vectors = es.eigenvectors().real();
        int max_idx;
        double max_eigenvalue = -1000000;
        for (int i = 0; i < values.rows(); ++i) {
          if (values[i] > max_eigenvalue) {
            max_idx = i;
            max_eigenvalue = values[i];
          }
        }
        std::vector<Eigen::Vector3d> cls1, cls2;
        Eigen::Vector2d first_pc = vectors.col(max_idx);
        for (int k = 0; k < clusterPointNum; k++){
          Eigen::Vector4d refPoint = graph_gain_point_cloud_clusters_[c][k];
          Eigen::Vector2d point2d;
          StateVec point3d;
          point2d[0] = refPoint[0];
          point2d[1] = refPoint[1];
          point3d[0] = refPoint[0];
          point3d[1] = refPoint[1];
          point3d[2] = refPoint[2];
          if ((point2d - mean).dot(first_pc) >= 0)
            cls1.push_back(point3d);
          else
            cls2.push_back(point3d);
        }
        StateVec mean1 = adjustCentroid(cls1, robotPos);
        StateVec mean2 = adjustCentroid(cls2, robotPos);
        if(!isOnTrajectory(mean1) && !isCleanedGoal(mean1))
          cluster_centroids_.push_back(mean1);
        if(!isOnTrajectory(mean2) && !isCleanedGoal(mean2))
          cluster_centroids_.push_back(mean2);
      }
      else{
        //adjust centroid
        StateVec moveDir;
        moveDir[0] = centroid[0] - robotPos[0];
        moveDir[1] = centroid[1] - robotPos[1];
        moveDir[2] = 0;

        double maxDist = 0.00001;
        int maxDistK = -1;
        for (int k = 0; k < clusterPointNum; k++)
        {
          Eigen::Vector4d refPoint = graph_gain_point_cloud_clusters_[c][k];
          StateVec dir;
          dir[0] = refPoint[0] - robotPos[0];
          dir[1] = refPoint[1] - robotPos[1];
          dir[2] = 0;

          double cosAng = moveDir.normalized().dot(dir.normalized());
          int angle = acos(cosAng) / M_PI * 180;

          if(angle<5){
            double dist = sqrt(SQ(robotPos[0] - refPoint[0]) + SQ(robotPos[1] - refPoint[1]) + SQ(robotPos[2] - refPoint[2]));
            if(dist>maxDist){
              maxDist = dist;
              maxDistK = k;
            }
          }
        }
        if(maxDistK>=0){
          centroid[0] = graph_gain_point_cloud_clusters_[c][maxDistK][0];
          centroid[1] = graph_gain_point_cloud_clusters_[c][maxDistK][1];
          centroid[2] = graph_gain_point_cloud_clusters_[c][maxDistK][2];
        }
        if(!isOnTrajectory(centroid) && !isCleanedGoal(centroid))
          cluster_centroids_.push_back(centroid);
      }
      
      
      

      /*
      //possible method 1 : gain biased centroid
      double sumGain = 0;
      for (int k = 0; k < clusterPointNum; k++)
      {
        Eigen::Vector4d refPoint = graph_gain_point_cloud_clusters_[c][k];
        sumGain += refPoint[3];
        centroid[0] += refPoint[0] * refPoint[3];
        centroid[1] += refPoint[1] * refPoint[3];
        centroid[2] += refPoint[2] * refPoint[3];
      }
      centroid[0] /= sumGain;
      centroid[1] /= sumGain;
      centroid[2] /= sumGain;
      */

      /*
      //possible mehtod 2: max gain point
      double maxGain = 0.00001;
      int maxGainK = -1;
      double maxDist = 0.00001;
      for (int k = 0; k < clusterPointNum; k++)
      {
        Eigen::Vector4d refPoint = graph_gain_point_cloud_clusters_[c][k];
        double dist = sqrt(SQ(robotPos[0] - refPoint[0]) + SQ(robotPos[1] - refPoint[1]) + SQ(robotPos[2] - refPoint[2]));
        if(refPoint[3]>maxGain || (refPoint[3]==maxGain && dist>maxDist)){
          maxDist = dist;
          maxGain = refPoint[3];
          maxGainK = k;
        };
      }
      centroid[0] = graph_gain_point_cloud_clusters_[c][maxGainK][0];
      centroid[1] = graph_gain_point_cloud_clusters_[c][maxGainK][1];
      centroid[2] = graph_gain_point_cloud_clusters_[c][maxGainK][2];
      */

      /*
      //possible method 3: furthest distance point
      double maxDist = 0.00001;
      int maxDistK = -1;
      StateVec robotPos;
      robotPos[0] = local_graph_.vertices[0].location.x;
      robotPos[1] = local_graph_.vertices[0].location.y;
      robotPos[2] = local_graph_.vertices[0].location.z;
      for (int k = 0; k < clusterPointNum; k++)
      {
        Eigen::Vector4d refPoint = graph_gain_point_cloud_clusters_[c][k];
        double dist = sqrt(SQ(robotPos[0] - refPoint[0]) + SQ(robotPos[1] - refPoint[1]) + SQ(robotPos[2] - refPoint[2]));
        if(dist>maxDist){
          maxDist = dist;
          maxDistK = k;
        }
      }
      centroid[0] = graph_gain_point_cloud_clusters_[c][maxDistK][0];
      centroid[1] = graph_gain_point_cloud_clusters_[c][maxDistK][1];
      centroid[2] = graph_gain_point_cloud_clusters_[c][maxDistK][2];
      */

      // if(!isOnTrajectory(centroid) && !isCleanedGoal(centroid))
      //   cluster_centroids_.push_back(centroid);
    }
  }

  void DualStateGraph::getFullCostMatrix(const Vector3d &cur_pos, Eigen::MatrixXd &mat)
  {
    int cluster_num = cluster_centroids_.size();
    if (cluster_num <= 0)
      return;

    std::vector<StateVec> places;
    places.push_back(cur_pos);
    for(int i=0;i<cluster_num;i++)
    {
      places.push_back(cluster_centroids_[i]);
    }
    StateVec home;
    home[0]=0, home[1]=0, home[2]=0;

    int dimen = places.size();
    mat.resize(dimen, dimen);
    for (int r = 0; r < dimen; r++)
      mat(r, r) = 0;

    //fill block between places
    for (int r = 1; r < dimen; r++)
    {
      for (int c = 1; c < r; c++)
      {
        double cost = 0;
        geometry_msgs::Point locationFrom, locationTo;
        locationFrom.x = places[r][0];
        locationFrom.y = places[r][1];
        locationFrom.z = places[r][2];
        locationTo.x = places[c][0];
        locationTo.y = places[c][1];
        locationTo.z = places[c][2];

        int start_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(local_graph_, locationFrom);
        int end_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(local_graph_, locationTo);
        if(start_vertex_idx==-1 || end_vertex_idx==-1){
          std::cout << "Warning: using local_graph_, GetClosestVertexIdxToPoint failed." << std::endl;
          StateVec moveDir;
          moveDir[0] = locationTo.x - locationFrom.x;
          moveDir[1] = locationTo.y - locationFrom.y;
          moveDir[2] = locationTo.z - locationFrom.z;
          cost = moveDir.norm();
        }
        else{
          std::vector<int> path;
          graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, start_vertex_idx, end_vertex_idx);
          if (path.empty())
            cost = 9999;
          else
            cost = graph_utils_ns::PathLength(path, local_graph_);
        }

        mat(r, c) = cost;
        mat(c, r) = mat(r, c);
      }
    }

    //fill block from current pos to the places
    geometry_msgs::Point locationFrom;
    locationFrom.x = places[0][0];
    locationFrom.y = places[0][1];
    locationFrom.z = places[0][2];
    for (int c = 1; c < dimen; c++)
    {
      double cost = 0;
      geometry_msgs::Point locationTo;
      locationTo.x = places[c][0];
      locationTo.y = places[c][1];
      locationTo.z = places[c][2];

      StateVec robotDir = explore_direction_;
      StateVec moveDir;
      moveDir[0] = locationTo.x - locationFrom.x;
      moveDir[1] = locationTo.y - locationFrom.y;
      moveDir[2] = 0;
      
      double cosAng = robotDir.normalized().dot(moveDir.normalized());
      cost = acos(cosAng) / M_PI * 20; //20 denotes the maximum cost

      int start_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(local_graph_, locationFrom);
      int end_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(local_graph_, locationTo);
      if(start_vertex_idx==-1 || end_vertex_idx==-1){
        std::cout << "Warning: using local_graph_, GetClosestVertexIdxToPoint failed." << std::endl;
        StateVec moveDir;
        moveDir[0] = locationTo.x - locationFrom.x;
        moveDir[1] = locationTo.y - locationFrom.y;
        moveDir[2] = locationTo.z - locationFrom.z;
        cost += moveDir.norm();
      }
      else{
        std::vector<int> path;
        graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, start_vertex_idx, end_vertex_idx);
        if (path.empty())
          cost = 9999;
        else
          cost += graph_utils_ns::PathLength(path, local_graph_);
      }

      mat(0, c) = cost;

      //standard TSP
      //mat(c, 0) = cost;
      mat(c, 0) = 0;
    }

    /*
    //with considering global home position
    // Fill block from clusters to home
    mat.leftCols<1>().setZero();
    geometry_msgs::Point locationTo;
    locationTo.x = home[0];
    locationTo.y = home[1];
    locationTo.z = home[2];
    int end_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(global_graph_, locationTo);
    if(end_vertex_idx==-1)
        std::cout << "Error: using global_graph_, GetClosestVertexIdxToPoint (Home) failed." << std::endl;

    for(int r=1;r<dimen;r++){
      geometry_msgs::Point locationFrom;
      locationFrom.x = places[r][0];
      locationFrom.y = places[r][1];
      locationFrom.z = places[r][2];
      int start_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(global_graph_, locationFrom);
      double cost;
      if(start_vertex_idx==-1){
          std::cout << "Warning: using global_graph_, GetClosestVertexIdxToPoint failed." << std::endl;
          cost = 0;
      }
      else{
        std::vector<int> path;
        graph_utils_ns::ShortestPathBtwVertex(path, global_graph_, start_vertex_idx, end_vertex_idx);
        if (path.empty())
          cost = 9999;
        else
          cost = graph_utils_ns::PathLength(path, global_graph_);
      }
      mat(r,0) = cost;
    }
    */

    places.clear();
  }

  void DualStateGraph::getGlobalFullCostMatrix(const Vector3d &cur_pos, const std::vector<StateVec> selected_goals, Eigen::MatrixXd &mat)
  {
    int cluster_num = selected_goals.size();
    if (cluster_num <= 0)
      return;

    std::vector<StateVec> places;
    places.push_back(cur_pos);
    for(int i=0;i<cluster_num;i++)
    {
      places.push_back(selected_goals[i]);
    }
    StateVec home;
    home[0]=0, home[1]=0, home[2]=0;

    int dimen = places.size();
    mat.resize(dimen, dimen);
    for (int r = 0; r < dimen; r++)
      mat(r, r) = 0;

    for (int r = 1; r < dimen; r++)
    {
      for (int c = 0; c < r; c++)
      {
        double cost = 0;
        geometry_msgs::Point locationFrom, locationTo;
        locationFrom.x = places[r][0];
        locationFrom.y = places[r][1];
        locationFrom.z = places[r][2];
        locationTo.x = places[c][0];
        locationTo.y = places[c][1];
        locationTo.z = places[c][2];

        int start_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(global_graph_, locationFrom);
        int end_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(global_graph_, locationTo);
        if(start_vertex_idx==-1 || end_vertex_idx==-1){
          std::cout << "Warning: using global_graph_, GetClosestVertexIdxToPoint failed." << std::endl;
          cost = 0;
        }
        else{
          std::vector<int> path;
          graph_utils_ns::ShortestPathBtwVertex(path, global_graph_, start_vertex_idx, end_vertex_idx);
          if (path.empty())
            cost = 9999;
          else
            cost = graph_utils_ns::PathLength(path, global_graph_);
        }

        mat(r, c) = cost;
        mat(c, r) = mat(r, c);
      }
    }

    // Fill block from current state to clusters
    mat.leftCols<1>().setZero();
    // Fill block from clusters to current state (cosidering returning home)
    mat.leftCols<1>().setZero();
    geometry_msgs::Point locationTo;
    locationTo.x = home[0];
    locationTo.y = home[1];
    locationTo.z = home[2];
    int end_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(global_graph_, locationTo);
    if(end_vertex_idx==-1)
        std::cout << "Error: using global_graph_, GetClosestVertexIdxToPoint (Home) failed." << std::endl;

    for(int r=1;r<dimen;r++){
      geometry_msgs::Point locationFrom;
      locationFrom.x = places[r][0];
      locationFrom.y = places[r][1];
      locationFrom.z = places[r][2];
      double cost;
      int start_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(global_graph_, locationFrom);
      if(start_vertex_idx==-1){
          std::cout << "Warning: using global_graph_, GetClosestVertexIdxToPoint failed." << std::endl;
          cost = 0;
      }
      else{
        std::vector<int> path;
        graph_utils_ns::ShortestPathBtwVertex(path, global_graph_, start_vertex_idx, end_vertex_idx);
        if (path.empty())
          cost = 9999;
        else
          cost = graph_utils_ns::PathLength(path, global_graph_);
      }
      mat(r,0) = cost;
    }

    places.clear();
  }

  void DualStateGraph::addCandidateGoalPoint(const StateVec &point)
  {
    bool exist = false;
    for (int i = 0; i < candidate_goals_.size(); i++)
    {
      StateVec dir;
      dir[0] = candidate_goals_[i][0] - point[0];
      dir[1] = candidate_goals_[i][1] - point[1];
      dir[2] = candidate_goals_[i][2] - point[2];

      if (dir.norm() < 2.0)
      {
        exist = true;
      }
    }
    if (!exist)
    {
      candidate_goals_.push_back(point);
    }
  }

  void DualStateGraph::findLocalTour(const Vector3d &cur_pos, std::vector<StateVec> &visits)
  {
    visits.clear();
    if (cluster_centroids_.size() == 0)
      return;
    if (cluster_centroids_.size() == 1)
    {
      visits.push_back(cluster_centroids_[0]);
      return;
    }

    std::vector<int> indices;

    /*
    std::ofstream debug_file(tsp_dir_ + "/debug.txt");
    debug_file << cur_pos[0] << "," << cur_pos[1] << "," << cur_pos[2] << "\n";
    for (int i = 0; i < cluster_centroids_.size(); i++)
    {
      debug_file << i << ":" << cluster_centroids_[i][0] << "," << cluster_centroids_[i][1] << "," << cluster_centroids_[i][2] << "\n";
    }
    debug_file.close();
    */

    // Get cost matrix for current state and clusters
    getFullCostMatrix(cur_pos, local_cost_mat_);
    
    const int dimension = local_cost_mat_.rows();

    // Write params and cost matrix to problem file
    std::ofstream prob_file(tsp_dir_ + "/single.tsp");
    // Problem specification part, follow the format of TSPLIB

    std::string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + std::to_string(dimension) +
                            "\nEDGE_WEIGHT_TYPE : "
                            "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

    // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
    //     "\nEDGE_WEIGHT_TYPE : "
    //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

    prob_file << prob_spec;
    // prob_file << "TYPE : TSP\n";
    // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
    // Problem data part
    const int scale = 1;
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        int int_cost = local_cost_mat_(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
    prob_file << "EOF";
    prob_file.close();

    // Call LKH TSP solver
    solveTSPLKH((tsp_dir_ + "/single.par").c_str());

    // Read optimal tour from the tour section of result file
    std::ifstream res_file(tsp_dir_ + "/single.txt");
    std::string res;
    while (getline(res_file, res))
    {
      // Go to tour section
      if (res.compare("TOUR_SECTION") == 0)
        break;
    }

    // Read path for ATSP formulation
    while (getline(res_file, res))
    {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1) // Ignore the current state
        continue;
      //if(id==dimension) //jtbao: Ignore the home state
      //  continue;
      if (id == -1)
        break;
      indices.push_back(id - 2); // Idx of solver-2 == Idx of cluster
    }

    res_file.close();

    for (int i = 0; i < indices.size(); i++)
    {
      visits.push_back(cluster_centroids_[indices[i]]);
    }
  }

  void DualStateGraph::selectCandidateGoalsForGlobalPlanning(std::vector<StateVec> &res)
  {
    std::vector<std::vector<Eigen::Vector4d>> clusters;
    
    geometry_msgs::Point locationHome, locationTo;
    locationHome.x = 0;
    locationHome.y = 0;
    locationHome.z = 0;
    int start_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(global_graph_, locationHome);

    int clusterNum = 0;
    for (int i = 0; i < candidate_goals_.size(); i++)
    {
      Eigen::Vector4d pt;
      pt[0] = candidate_goals_[i][0];
      pt[1] = candidate_goals_[i][1];
      pt[2] = candidate_goals_[i][2];

      locationTo.x = pt[0];
      locationTo.y = pt[1];
      locationTo.z = pt[2];
      double cost = 0;
      int end_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(global_graph_, locationTo);
      if(end_vertex_idx==-1){
        std::cout << "Warning: using global_graph_, GetClosestVertexIdxToPoint failed." << std::endl;
        cost = 0;
      }
      else{
        std::vector<int> path;
        graph_utils_ns::ShortestPathBtwVertex(path, global_graph_, start_vertex_idx, end_vertex_idx);
        if (path.empty())
          cost = 9999;
        else
          cost = graph_utils_ns::PathLength(path, global_graph_);
      }

      pt[3] = cost;

      if (clusterNum == 0)
      {
        std::vector<Eigen::Vector4d> pc;
        pc.push_back(pt);
        clusters.push_back(pc);
        clusterNum++;
        continue;
      }

      bool found = false;
      for (int c = 0; c < clusterNum; c++)
      {
        for (int k = 0; k < clusters[c].size(); k++)
        {
          Eigen:Vector4d refPoint = clusters[c][k];
          float dist = fabs(pt[3] - refPoint[3]);
          if (dist < 10.0)
          {
            clusters[c].push_back(pt);
            found = true;
            break;
          }
        }
        if (found)
          break;
      }
      if (!found)
      {
        std::vector<Eigen::Vector4d> pc;
        pc.push_back(pt);
        clusters.push_back(pc);
        clusterNum++;
      }
    }

    res.clear();
    for(int i=0;i<clusters.size();i++){
      res.push_back(findMaxGainPoint(clusters[i]));
      clusters[i].clear();
    }
    clusters.clear();
  }

  void DualStateGraph::findGlobalTour(const Vector3d &cur_pos, std::vector<StateVec> &visits)
  {
    visits.clear();
    if (candidate_goals_.size() == 0)
      return;
    if (candidate_goals_.size() == 1)
    {
      visits.push_back(candidate_goals_[0]);
      return;
    }

    //cluster and select candidate goals for global tour planning
    std::vector<StateVec> selected_goals;
    if(candidate_goals_.size() > 40){
      selectCandidateGoalsForGlobalPlanning(selected_goals);
    }
    else{
      for(int i=0;i<candidate_goals_.size();i++){
        StateVec goal;
        goal[0] = candidate_goals_[i][0];
        goal[1] = candidate_goals_[i][1];
        goal[2] = candidate_goals_[i][2];
        selected_goals.push_back(goal);
      }
    }
  
   /*
   // do not cluster the candidate goals
   for(int i=0;i<candidate_goals_.size();i++){
        StateVec goal;
        goal[0] = candidate_goals_[i][0];
        goal[1] = candidate_goals_[i][1];
        goal[2] = candidate_goals_[i][2];
        selected_goals.push_back(goal);
    }
    */

    std::vector<int> indices;

    /*
    std::ofstream debug_file(tsp_dir_ + "/debug.txt");
    debug_file << cur_pos[0] << "," << cur_pos[1] << "," << cur_pos[2] << "\n";
    for (int i = 0; i < candidate_goals_.size(); i++)
    {
      debug_file << i << ":" << candidate_goals_[i][0] << "," << candidate_goals_[i][1] << "," << candidate_goals_[i][2] << "\n";
    }
    debug_file.close();
    */

    // Get cost matrix for current state and clusters
    Eigen::MatrixXd cost_mat;
    getGlobalFullCostMatrix(cur_pos, selected_goals, cost_mat);
    const int dimension = cost_mat.rows();

    // Write params and cost matrix to problem file
    std::ofstream prob_file(tsp_dir_ + "/single.tsp");
    // Problem specification part, follow the format of TSPLIB

    std::string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + std::to_string(dimension) +
                            "\nEDGE_WEIGHT_TYPE : "
                            "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

    // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
    //     "\nEDGE_WEIGHT_TYPE : "
    //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

    prob_file << prob_spec;
    // prob_file << "TYPE : TSP\n";
    // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
    // Problem data part
    const int scale = 1;
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
    prob_file << "EOF";
    prob_file.close();

    //std::chrono::steady_clock::time_point solve_tsp_start_ = std::chrono::steady_clock::now();
  
    // Call LKH TSP solver
    solveTSPLKH((tsp_dir_ + "/single.par").c_str());

    //record the time of solving atsp
    //std::chrono::steady_clock::time_point solve_tsp_over_ = std::chrono::steady_clock::now();
    //std::chrono::steady_clock::duration time_span = solve_tsp_over_ - solve_tsp_start_;
    //double tsp_solve_time = double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
    //FILE *fdebug = fopen("/home/ralab/debug.txt", "a+");
    //fprintf(fdebug, "%d %d %f\n", candidate_goals_.size(), selected_goals.size(), tsp_solve_time);
    //fflush(fdebug);
    //fclose(fdebug);

    // Read optimal tour from the tour section of result file
    std::ifstream res_file(tsp_dir_ + "/single.txt");
    std::string res;
    while (getline(res_file, res))
    {
      // Go to tour section
      if (res.compare("TOUR_SECTION") == 0)
        break;
    }

    // Read path for ATSP formulation
    while (getline(res_file, res))
    {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1) // Ignore the current state
        continue;
      //if(id==dimension) //jtbao: Ignore the home state
      //  continue;
      if (id == -1)
        break;
      indices.push_back(id - 2); // Idx of solver-2 == Idx of cluster
    }

    res_file.close();

    for (int i = 0; i < indices.size(); i++)
    {
      visits.push_back(selected_goals[indices[i]]);
    }
  }

  geometry_msgs::Point DualStateGraph::getBestLocalVertexPositionInMaximumCluster()
  {
    int maxClusterPointNum = 0, maxClusterId = -1;
    for (int c = 0; c < graph_gain_point_cloud_clusters_.size(); c++)
    {
      int clusterPointNum = graph_gain_point_cloud_clusters_[c].size();
      if (clusterPointNum > maxClusterPointNum)
      {
        maxClusterPointNum = clusterPointNum;
        maxClusterId = c;
      }
    }

    StateVec centroid;
    centroid[0] = 0, centroid[1] = 0, centroid[2] = 0;
    int clusterPointNum = graph_gain_point_cloud_clusters_[maxClusterId].size();
    double sumGain = 0;
    for (int k = 0; k < clusterPointNum; k++)
    {
      Eigen::Vector4d refPoint = graph_gain_point_cloud_clusters_[maxClusterId][k];
      sumGain += refPoint[3];
      centroid[0] += refPoint[0] * refPoint[3];
      centroid[1] += refPoint[1] * refPoint[3];
      centroid[2] += refPoint[2] * refPoint[3];
    }
    centroid[0] /= sumGain;
    centroid[1] /= sumGain;
    centroid[2] /= sumGain;

    geometry_msgs::Point best_vertex_location;
    best_vertex_location.x = centroid[0];
    best_vertex_location.y = centroid[1];
    best_vertex_location.z = centroid[2];
    return best_vertex_location;
  }

  //发布全局图/global_graph
  void DualStateGraph::publishGlobalGraph()
  {
    // Publish the current global graph
    global_graph_.header.stamp = ros::Time::now();
    global_graph_.header.frame_id = world_frame_id_;
    global_graph_pub_.publish(global_graph_);
  }

  //和addNewLocalVertex作用几乎一样，功能是加入一个节点
  //多了一个“相似性检查”，如果离得太近了就不加入图中
  void DualStateGraph::addNewLocalVertexWithoutDuplicates(geometry_msgs::Pose &vertex_msg,
                                                          graph_utils::TopologicalGraph &graph)
  {
    // Same as addNewLocalVertex but only adds vertex if a similar vertex doesn't
    // already exist

    // Extract the point
    geometry_msgs::Point new_vertex_location;
    new_vertex_location = vertex_msg.position;

    // Check if a similar vertex already exists
    bool already_exists = false;
    bool too_long = false;
    double distance = -1;
    int closest_vertex_idx = -1;
    if (!graph.vertices.empty())
    {
      closest_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(graph, new_vertex_location);
      auto &closest_vertex_location = graph.vertices[closest_vertex_idx].location;
      distance = misc_utils_ns::PointXYZDist(new_vertex_location, closest_vertex_location);
      if (distance < kMinVertexDist)
      {
        already_exists = true;
      }
      if (distance > kMaxLongEdgeDist || fabs(new_vertex_location.z - closest_vertex_location.z) > kMaxVertexDiffAlongZ)
      {
        too_long = true;
      }
    }

    // If not, add a new one
    if (!already_exists && !too_long)
    {
      prev_track_vertex_idx_ = closest_vertex_idx;
      addNewLocalVertex(vertex_msg, graph);
    }
  }

  //功能：加入一个新的节点，还要检查节点之间的欧式距离，不能超过阈值或低于阈值，还要保证不会发生碰撞
  void DualStateGraph::addNewPrunedVertex(geometry_msgs::Pose &vertex_msg, graph_utils::TopologicalGraph &graph)
  {
    // Extract the point
    geometry_msgs::Point new_vertex_location;
    new_vertex_location = vertex_msg.position;

    bool already_exists = false;
    bool too_long = false;
    double distance = -1;
    int closest_vertex_idx = -1;
    geometry_msgs::Point closest_vertex_location;
    if (!graph.vertices.empty())
    {
      closest_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(graph, new_vertex_location);
      closest_vertex_location = graph.vertices[closest_vertex_idx].location;
      distance = misc_utils_ns::PointXYZDist(new_vertex_location, closest_vertex_location);
      //如果新节点和图中的最近节点的欧式距离小于kMinVertexDist，不加入图中
      if (distance < kMinVertexDist)
      {
        already_exists = true;
      }
      //距离太远也不加入图中
      if (distance > kMaxLongEdgeDist || fabs(new_vertex_location.z - closest_vertex_location.z) > kMaxVertexDiffAlongZ)
      {
        too_long = true;
      }
    }

    // Check again if there is collision between the new node and its closest
    // node. Although this has been done for local
    // graph, but the octomap might be updated
    if (!already_exists && !too_long)
    {
      Eigen::Vector3d origin;
      Eigen::Vector3d end;
      origin.x() = new_vertex_location.x;
      origin.y() = new_vertex_location.y;
      origin.z() = new_vertex_location.z;
      end.x() = closest_vertex_location.x;
      end.y() = closest_vertex_location.y;
      end.z() = closest_vertex_location.z;
      //确认连线没有碰撞
      if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
          manager_->getLineStatusBoundingBox(origin, end, robot_bounding))
      {
        prev_track_vertex_idx_ = closest_vertex_idx;
        addNewLocalVertex(vertex_msg, graph);
      }
    }
  }

  //功能：给定一个pose，把它加入到图中
  //直接连接最近的点的边，其他靠近的边进行公式(6)的约束检查，再进行连边
  void DualStateGraph::addNewLocalVertex(geometry_msgs::Pose &vertex_msg, graph_utils::TopologicalGraph &graph)
  {
    // Add a new vertex to the graph associated with the input keypose
    // Return the index of the vertex

    // Extract the point
    geometry_msgs::Point new_vertex_location;
    new_vertex_location = vertex_msg.position;

    // Create a new vertex
    graph_utils::Vertex vertex;
    vertex.location = new_vertex_location;
    vertex.vertex_id = (int)graph.vertices.size();
    vertex.information_gain = vertex_msg.orientation.y;

    // Add this vertex to the graph
    graph.vertices.push_back(vertex);
    track_localvertex_idx_ = (int)graph.vertices.size() - 1;
    auto &vertex_new = graph.vertices[track_localvertex_idx_];
    // If there are already other vertices
    if (graph.vertices.size() > 1)
    {
      // Add a parent backpointer to previous vertex
      vertex_new.parent_idx = prev_track_vertex_idx_;

      // Add an edge to previous vertex
      addEdgeWithoutCheck(vertex_new.parent_idx, track_localvertex_idx_, graph);

      // Also add edges to nearby vertices
      for (auto &graph_vertex : graph.vertices)
      {
        // If within a distance
        float dist_diff = misc_utils_ns::PointXYZDist(graph_vertex.location, vertex_new.location);
        if (dist_diff < kConnectVertexDistMax)
        {
          // Add edge from this nearby vertex to current vertex
          addEdge(graph_vertex.vertex_id, vertex_new.vertex_id, graph);
        }
      }
    }
    else
    {
      // This is the first vertex -- no edges
      vertex.parent_idx = -1;
    }
  }

  //直接加入一个图节点，不加入边
  void DualStateGraph::addNewLocalVertexWithoutEdge(geometry_msgs::Pose &vertex_msg, graph_utils::TopologicalGraph &graph)
  {
    // Add a new vertex to the graph associated with the input keypose
    // Return the index of the vertex

    // Extract the point
    geometry_msgs::Point new_vertex_location;
    new_vertex_location = vertex_msg.position;

    // Create a new vertex
    graph_utils::Vertex vertex;
    vertex.location = new_vertex_location;
    vertex.vertex_id = (int)graph.vertices.size();
    vertex.information_gain = vertex_msg.orientation.y;

    // Add this vertex to the graph
    graph.vertices.push_back(vertex);
    track_localvertex_idx_ = (int)graph.vertices.size() - 1;
    auto &vertex_new = graph.vertices[track_localvertex_idx_];
  }

  //对全局图操作，加入一个节点，多了一个距离和相似性检查，没问题才调用addNewGlobalVertex
  //这个和addNewGlobalVertexWithKeypose几乎一样，
  //有微小的区别在于，相似性检查上，addNewGlobalVertexWithKeypose要欧式距离小于kMinVertexDist/2
  //本函数要求欧式距离小于于kMinVertexDist
  void DualStateGraph::addNewGlobalVertexWithoutDuplicates(geometry_msgs::Pose &vertex_msg)
  {
    // Same as addNewLocalVertex but only adds vertex if a similar vertex doesn't
    // already exist

    // Extract the point
    geometry_msgs::Point new_vertex_location;
    new_vertex_location = vertex_msg.position;

    // Check if a similar vertex already exists
    bool already_exists = false;
    bool too_long = false;
    double distance = -1;
    int closest_vertex_idx = -1;
    if (!global_graph_.vertices.empty())
    {
      closest_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(global_graph_, new_vertex_location);
      auto &closest_vertex_location = global_graph_.vertices[closest_vertex_idx].location;
      distance = misc_utils_ns::PointXYZDist(new_vertex_location, closest_vertex_location);
      if (distance < kMinVertexDist)
      {
        already_exists = true;
      }
      if (distance > kMaxLongEdgeDist || fabs(new_vertex_location.z - closest_vertex_location.z) > kMaxVertexDiffAlongZ)
      {
        too_long = true;
      }
    }

    // If not, add a new one
    if (!already_exists && !too_long)
    {
      prev_track_vertex_idx_ = closest_vertex_idx;
      addNewGlobalVertex(vertex_msg);
    }
  }

  //提取全局图中距离当前位置最近的节点，判断其与当前位置的距离，如果太大或太小，则忽略当前位置点，否则将当前位置作为新的节点添加到全局图中并添加连接边。
  void DualStateGraph::addNewGlobalVertexWithKeypose(geometry_msgs::Pose &vertex_msg)
  {
    // Same as addNewLocalVertex but only adds vertex if a similar vertex doesn't
    // already exist

    // Extract the point
    geometry_msgs::Point new_vertex_location;
    new_vertex_location = vertex_msg.position;

    // Check if a similar vertex already exists
    bool already_exists = false;
    bool too_long = false;
    double distance = -1;
    int closest_vertex_idx = -1;
    if (!global_graph_.vertices.empty())
    {
      closest_vertex_idx = graph_utils_ns::GetClosestVertexIdxToPoint(global_graph_, new_vertex_location);
      auto &closest_vertex_location = global_graph_.vertices[closest_vertex_idx].location;
      distance = misc_utils_ns::PointXYZDist(new_vertex_location, closest_vertex_location);
      if (distance < kMinVertexDist / 2)
      {
        already_exists = true;
      }
      if (distance > kMaxLongEdgeDist || fabs(new_vertex_location.z - closest_vertex_location.z) > kMaxVertexDiffAlongZ)
      {
        too_long = true;
      }
    }

    // If not, add a new one
    if (!already_exists && !too_long)
    {
      prev_track_vertex_idx_ = closest_vertex_idx;
      addNewGlobalVertex(vertex_msg);
      addGlobalEdgeWithoutCheck(prev_track_keypose_vertex_idx_, track_globalvertex_idx_, true);
      prev_track_keypose_vertex_idx_ = track_globalvertex_idx_;
    }
  }

  //功能：给定一个pose，把它加入到全局图中
  //直接连接最近的点的边，其他靠近的边进行公式(6)的约束检查，再进行连边
  void DualStateGraph::addNewGlobalVertex(geometry_msgs::Pose &vertex_msg)
  {
    // Extract the point
    geometry_msgs::Point new_vertex_location;
    new_vertex_location = vertex_msg.position;

    // Create a new vertex
    graph_utils::Vertex vertex;
    vertex.location = new_vertex_location;
    vertex.vertex_id = (int)global_graph_.vertices.size();
    vertex.information_gain = vertex_msg.orientation.y;

    // Add this vertex to the graph
    global_graph_.vertices.push_back(vertex);
    track_globalvertex_idx_ = (int)global_graph_.vertices.size() - 1;
    auto &vertex_new = global_graph_.vertices[track_globalvertex_idx_];
    // If there are already other vertices
    if (global_graph_.vertices.size() > 1)
    {
      // Add a parent backpointer to previous vertex
      vertex_new.parent_idx = prev_track_vertex_idx_;

      // Add an edge to previous vertex
      addGlobalEdgeWithoutCheck(prev_track_vertex_idx_, track_globalvertex_idx_, false);

      // Also add edges to nearby vertices
      for (auto &graph_vertex : global_graph_.vertices)
      {
        // If within a distance
        float dist_diff = misc_utils_ns::PointXYZDist(graph_vertex.location, vertex_new.location);
        if (dist_diff < kConnectVertexDistMax)
        {
          // Add edge from this nearby vertex to current vertex
          addGlobalEdge(graph_vertex.vertex_id, vertex_new.vertex_id);
        }
      }
    }
    else
    {
      // This is the first vertex -- no edges
      vertex.parent_idx = -1;
    }
  }

  //功能：不进行任何碰撞检查，直接对两个节点之间加入边（边为欧式距离）
  void DualStateGraph::addEdgeWithoutCheck(int start_vertex_idx, int end_vertex_idx, graph_utils::TopologicalGraph &graph)
  {
    // Add an edge in the graph from vertex start_vertex_idx to vertex
    // end_vertex_idx

    // Get the two vertices
    auto &start_vertex = graph.vertices[start_vertex_idx];
    auto &end_vertex = graph.vertices[end_vertex_idx];

    // Check if edge already exists -- don't duplicate
    for (auto &edge : start_vertex.edges)
    {
      if (edge.vertex_id_end == end_vertex_idx)
      {
        // don't add duplicate edge
        return;
      }
    }

    // Compute the distance between the two points
    float dist_diff = misc_utils_ns::PointXYZDist(start_vertex.location, end_vertex.location);

    // Add an edge connecting the current node and the existing node on the graph

    // Create two edge objects
    graph_utils::Edge edge_to_start;
    graph_utils::Edge edge_to_end;

    // Join the two edges
    edge_to_start.vertex_id_end = start_vertex_idx;
    edge_to_end.vertex_id_end = end_vertex_idx;

    // Compute the traversal cost
    // For now, this is just Euclidean distance
    edge_to_start.traversal_costs = dist_diff;
    edge_to_end.traversal_costs = dist_diff;

    // Add these two edges to the vertices
    start_vertex.edges.push_back(edge_to_end);
    end_vertex.edges.push_back(edge_to_start);

    localEdgeSize_++;
  }

  //功能：进行碰撞检查和公式(6)定义的约束检查，对两个节点之间进行连线
  void DualStateGraph::addEdge(int start_vertex_idx, int end_vertex_idx, graph_utils::TopologicalGraph &graph)
  {
    if (start_vertex_idx == end_vertex_idx)
    {
      // don't add edge to itself
      return;
    }

    // Get the edge distance
    float dist_edge =
        misc_utils_ns::PointXYZDist(graph.vertices[start_vertex_idx].location, graph.vertices[end_vertex_idx].location);

    if (dist_edge > kMaxLongEdgeDist)
    {
    }
    else
    {
      std::vector<int> path;
      graph_utils_ns::ShortestPathBtwVertex(path, graph, start_vertex_idx, end_vertex_idx);
      bool path_exists = true;
      float dist_path = 0;
      if (path.empty())
      {
        path_exists = false;
      }
      else
      {
        dist_path = graph_utils_ns::PathLength(path, graph);
      }

      //即论文中的（6）式，如果欧式距离小，节点距离/欧式距离大于kExistingPathRatioThreshold，并且不发生碰撞，就进行一个连线
      if ((!path_exists || (path_exists && ((dist_path / dist_edge) >= kExistingPathRatioThreshold))) &&
          (!zCollisionCheck(start_vertex_idx, end_vertex_idx, graph)) &&
          (!grid_->collisionCheckByTerrain(graph.vertices[start_vertex_idx].location,
                                           graph.vertices[end_vertex_idx].location)))
      {
        Eigen::Vector3d origin;
        Eigen::Vector3d end;
        origin.x() = graph.vertices[start_vertex_idx].location.x;
        origin.y() = graph.vertices[start_vertex_idx].location.y;
        origin.z() = graph.vertices[start_vertex_idx].location.z;
        end.x() = graph.vertices[end_vertex_idx].location.x;
        end.y() = graph.vertices[end_vertex_idx].location.y;
        end.z() = graph.vertices[end_vertex_idx].location.z;
        if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
            manager_->getLineStatusBoundingBox(origin, end, robot_bounding))
        {
          addEdgeWithoutCheck(start_vertex_idx, end_vertex_idx, graph);
        }
      }
    }
  }

  //功能：不进行任何碰撞检查，直接对两个全局节点之间加入边（边为欧式距离）
  //函数addEdgeWithoutCheck相当于可以传入图，而这个函数则是写死了对全局图进行操作。
  void DualStateGraph::addGlobalEdgeWithoutCheck(int start_vertex_idx, int end_vertex_idx, bool trajectory_edge)
  {
    // Add an edge in the graph from vertex start_vertex_idx to vertex
    // end_vertex_idx

    // Get the two vertices
    auto &start_vertex = global_graph_.vertices[start_vertex_idx];
    auto &end_vertex = global_graph_.vertices[end_vertex_idx];

    // Check if edge already exists -- don't duplicate
    for (auto &edge : start_vertex.edges)
    {
      if (edge.vertex_id_end == end_vertex_idx)
      {
        // don't add duplicate edge
        edge.keypose_edge = trajectory_edge;
        for (auto &edge : end_vertex.edges)
        {
          if (edge.vertex_id_end == start_vertex_idx)
          {
            // don't add duplicate edge
            edge.keypose_edge = trajectory_edge;
            break;
          }
        }
        return;
      }
    }

    // Compute the distance between the two points
    float dist_diff = misc_utils_ns::PointXYZDist(start_vertex.location, end_vertex.location);

    // Create two edge objects
    graph_utils::Edge edge_to_start;
    graph_utils::Edge edge_to_end;

    // Join the two edges
    edge_to_start.vertex_id_end = start_vertex_idx;
    edge_to_end.vertex_id_end = end_vertex_idx;

    // Compute the traversal cost
    // For now, this is just Euclidean distance
    edge_to_start.traversal_costs = dist_diff;
    edge_to_end.traversal_costs = dist_diff;

    edge_to_start.keypose_edge = trajectory_edge;
    edge_to_end.keypose_edge = trajectory_edge;

    // Add these two edges to the vertices
    start_vertex.edges.push_back(edge_to_end);
    end_vertex.edges.push_back(edge_to_start);

    globalEdgeSize_++;
  }

  //功能：进行碰撞检查和公式(6)定义的约束检查，对对两个全局节点之间进行连线
  //函数addEdge相当于可以传入图，而这个函数则是写死了对全局图进行操作。
  void DualStateGraph::addGlobalEdge(int start_vertex_idx, int end_vertex_idx)
  {
    if (start_vertex_idx == end_vertex_idx)
    {
      return;
    }

    // Get the edge distance
    float dist_edge = misc_utils_ns::PointXYZDist(global_graph_.vertices[start_vertex_idx].location,
                                                  global_graph_.vertices[end_vertex_idx].location);

    if (dist_edge > kMaxLongEdgeDist)
    {
      // VERY long edge. Don't add it
    }
    else
    {
      // Get the path distance
      std::vector<int> path;
      graph_utils_ns::ShortestPathBtwVertex(path, global_graph_, start_vertex_idx, end_vertex_idx);
      bool path_exists = true;
      float dist_path = 0;

      // Check if there is an existing path
      if (path.empty())
      {
        // No path exists
        path_exists = false;
      }
      else
      {
        // Compute path length
        dist_path = graph_utils_ns::PathLength(path, global_graph_);
        // ROS_WARN("path exists of edge of length %f, where path exists of length
        // %f", dist_edge, dist_path);
      }

      // Check if ratio is beyond threshold
      // bool collision = false;
      // 论文中公式（6）定义的约束检查
      if ((!path_exists || (path_exists && ((dist_path / dist_edge) >= kExistingPathRatioThresholdGlobal))) &&
          (!zCollisionCheck(start_vertex_idx, end_vertex_idx, global_graph_)))
      {
        Eigen::Vector3d origin;
        Eigen::Vector3d end;
        origin.x() = global_graph_.vertices[start_vertex_idx].location.x;
        origin.y() = global_graph_.vertices[start_vertex_idx].location.y;
        origin.z() = global_graph_.vertices[start_vertex_idx].location.z;
        end.x() = global_graph_.vertices[end_vertex_idx].location.x;
        end.y() = global_graph_.vertices[end_vertex_idx].location.y;
        end.z() = global_graph_.vertices[end_vertex_idx].location.z;

        //确保连线没有碰撞（光线追踪进行碰撞检测）
        if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
            manager_->getLineStatusBoundingBox(origin, end, robot_bounding))
        {
          addGlobalEdgeWithoutCheck(start_vertex_idx, end_vertex_idx, false);
        }
      }
    }
  }

  //两个节点之间，俯仰角小于38度，并且z轴方向距离超过0.5m（默认）则认为两节点之间在z轴方向碰撞会发生
  bool DualStateGraph::zCollisionCheck(int start_vertex_idx, int end_vertex_idx, graph_utils::TopologicalGraph graph)
  {
    auto &start_vertex_location = graph.vertices[start_vertex_idx].location;
    auto &end_vertex_location = graph.vertices[end_vertex_idx].location;
    float x_diff = start_vertex_location.x - end_vertex_location.x;
    float y_diff = start_vertex_location.y - end_vertex_location.y;
    float z_diff = start_vertex_location.z - end_vertex_location.z;
    float ang_diff = atan(fabs(z_diff) / sqrt(x_diff * x_diff + y_diff * y_diff)) * 180 / M_PI;
    if (fabs(ang_diff) > kMaxVertexAngleAlongZ || fabs(z_diff) > kMaxVertexDiffAlongZ)
    {
      return true;
    }
    return false;
  }

  int DualStateGraph::getLocalVertexSize()
  {
    return (local_graph_.vertices.size());
  }

  int DualStateGraph::getLocalEdgeSize()
  {
    return localEdgeSize_;
  }

  int DualStateGraph::getGlobalVertexSize()
  {
    return (global_graph_.vertices.size());
  }

  int DualStateGraph::getGlobalEdgeSize()
  {
    return globalEdgeSize_;
  }

  //根据传入的机器人当前位置,在现有的局部视点图上做精减,得到修剪后的局部图
  void DualStateGraph::pruneGraph(geometry_msgs::Point root)
  {
    int closest_vertex_idx_to_root;
    if (!local_graph_.vertices.empty())
    {
      //寻找离root位置最近的局部视点图节点
      closest_vertex_idx_to_root = graph_utils_ns::GetClosestVertexIdxToPoint(local_graph_, root);
      auto &closest_vertex_location = local_graph_.vertices[closest_vertex_idx_to_root].location;
      double distance = misc_utils_ns::PointXYZDist(root, closest_vertex_location);
      Eigen::Vector3d origin(root.x, root.y, root.z);
      Eigen::Vector3d end(closest_vertex_location.x, closest_vertex_location.y, closest_vertex_location.z);
      if (distance > kMaxDistToPrunedRoot)
      {
        return;
      }
      else if (volumetric_mapping::OctomapManager::CellStatus::kFree !=
               manager_->getLineStatusBoundingBox(origin, end, robot_bounding))
      {
        return;
      }
    }

    std::vector<int> path;
    for (int path_id = 0; path_id < local_graph_.vertices.size(); path_id++)
    {
      //遍历局部视点图节点,寻找与上述点的最短路径
      path.clear();
      graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, closest_vertex_idx_to_root, path_id);
      if (path.size() == 0)
      {
        //如果路径不存在,则忽略该局部点
        continue;
      }
      //只保留离root位置一定范围内(kMaxPrunedNodeDist,在配置文件中设置,default 15m)的局部点
      if (misc_utils_ns::PointXYZDist(root, local_graph_.vertices[path_id].location) > kMaxPrunedNodeDist)
        continue;
      //将最短路径上的点加入pruned图
      for (int j = 0; j < path.size(); j++)
      {
        tempvertex_.position = local_graph_.vertices[path[j]].location;
        tempvertex_.position.z = MIN(tempvertex_.position.z, root.z);
        tempvertex_.orientation.y = local_graph_.vertices[path[j]].information_gain;
        addNewPrunedVertex(tempvertex_, pruned_graph_);
      }
    }
  }

  //对全局图进行修剪操作，凡是处于占据状态的，把连接的边给删除掉
  void DualStateGraph::pruneGlobalGraph()
  {
    if (global_graph_.vertices.size() > 0)
    {
      for (int i = 0; i < global_graph_.vertices.size(); i++)
      {
        Eigen::Vector3d vertex_position(global_graph_.vertices[i].location.x, global_graph_.vertices[i].location.y,
                                        global_graph_.vertices[i].location.z);
        if (volumetric_mapping::OctomapManager::CellStatus::kFree != manager_->getCellStatusPoint(vertex_position))
        {
          for (int j = 0; j < global_graph_.vertices[i].edges.size(); j++)
          {
            int end_vertex_id = global_graph_.vertices[i].edges[j].vertex_id_end;
            for (int m = 0; m < global_graph_.vertices[end_vertex_id].edges.size(); m++)
            {
              if (global_graph_.vertices[end_vertex_id].edges[m].vertex_id_end == i)
              {
                global_graph_.vertices[end_vertex_id].edges.erase(global_graph_.vertices[end_vertex_id].edges.begin() +
                                                                  m);
                break;
              }
            }
          }
        }
      }
    }
  }

  void DualStateGraph::setCurrentPlannerStatus(bool status)
  {
    planner_status_ = status;
  }

  void DualStateGraph::clearLocalGraph()
  {
    local_graph_.vertices.clear();
    track_localvertex_idx_ = 0;
    robot_yaw_ = 0.0;
    localEdgeSize_ = 0;
    best_gain_ = 0;
    best_vertex_id_ = 0;
    gainID_.clear();
  }

  //这块内容实现的是公式4和5，得到当前区域朝着探索方向的最大的gain，以及节点号
  double DualStateGraph::getGain(geometry_msgs::Point robot_position)
  {
    for (auto &graph_vertex : local_graph_.vertices)
    {
      if (graph_vertex.vertex_id > 0)
      {
        // Get the path distance
        std::vector<int> path;
        graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, 0, graph_vertex.vertex_id);
        bool path_exists = true;
        float dist_path = 0;

        // Check if there is an existing path
        if (path.empty())
        {
          // No path exists
          path_exists = false;
          graph_vertex.information_gain = 0;
          continue;
        }
        else
        {
          // Compute path length
          dist_path = graph_utils_ns::PathLength(path, local_graph_);
        }
        int NodeCountArround = 1;
        for (auto &vertex : local_graph_.vertices)
        {
          float dist_edge = misc_utils_ns::PointXYZDist(graph_vertex.location, vertex.location);
          if (dist_edge < kSurroundRange)
          {
            NodeCountArround++;
          }
        }
        if (misc_utils_ns::PointXYZDist(graph_vertex.location, robot_position) < kMinGainRange)
          graph_vertex.information_gain = 0;

        if (graph_vertex.information_gain > 0)
        {
          if (std::isnan(explore_direction_.x()) || std::isnan(explore_direction_.y()))
            DTWValue_ = exp(1);
          else
          {
            DTW(path, robot_position);
            graph_vertex.information_gain = graph_vertex.information_gain / log(DTWValue_ * kDirectionCoeff);
          }
        }
        graph_vertex.information_gain =
            graph_vertex.information_gain * kDegressiveCoeff / dist_path * exp(0.1 * NodeCountArround);
      }
    }
    for (auto &graph_vertex : local_graph_.vertices)
    {
      if (graph_vertex.vertex_id > 0)
      {
        double path_information_gain = 0;
        std::vector<int> path;
        graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, 0, graph_vertex.vertex_id);
        if (path.empty())
        {
          // No path exists
          continue;
        }
        for (int i = 0; i < path.size(); i++)
        {
          path_information_gain += local_graph_.vertices[path[i]].information_gain;
        }
        if (path_information_gain > 0)
        {
          gainID_.push_back(graph_vertex.vertex_id);
        }

        if (path_information_gain > best_gain_)
        {
          best_vertex_id_ = graph_vertex.vertex_id;
          best_gain_ = path_information_gain;
        }
      }
    }
    // ROS_INFO("Best gain is %f.\n Best vertex id is %d", best_gain_,
    // best_vertex_id_);
    return best_gain_;
  }

  void DualStateGraph::DTW(std::vector<int> path, geometry_msgs::Point robot_position)
  {
    float dist_path = 0;
    int node_num = path.size();
    dist_path = graph_utils_ns::PathLength(path, local_graph_) / node_num;
    std::vector<geometry_msgs::Point> exp_node;
    geometry_msgs::Point node_position;
    for (int i = 0; i < node_num; i++)
    {
      node_position.x = robot_position.x + explore_direction_.x() * dist_path * (i + 1);
      node_position.y = robot_position.y + explore_direction_.y() * dist_path * (i + 1);
      exp_node.push_back(node_position);
    }

    std::vector<std::vector<double>> DTWValue;
    std::vector<double> sub_DTWValue;
    for (int i = 0; i < node_num + 1; i++)
    {
      for (int j = 0; j < node_num + 1; j++)
      {
        sub_DTWValue.push_back(1000000);
      }
      DTWValue.push_back(sub_DTWValue);
      sub_DTWValue.clear();
    }
    DTWValue[0][0] = 0;

    double dist = 0;
    for (int i = 1; i < node_num + 1; i++)
    {
      for (int j = 1; j < node_num + 1; j++)
      {
        dist = misc_utils_ns::PointXYDist(local_graph_.vertices[path[i - 1]].location, exp_node[j - 1]);
        DTWValue[i][j] = dist + std::fmin(std::fmin(DTWValue[i - 1][j], DTWValue[i][j - 1]), DTWValue[i - 1][j - 1]);
      }
    }
    DTWValue_ = DTWValue[node_num][node_num];
  }

  //根据局部图，将局部图0号节点至局部规划最佳点以及增益为正的点的最短路径上的点，加入全局图
  void DualStateGraph::updateGlobalGraph()
  {
    //计算从0号点出发，到最好顶点的最短路径，存入path中
    std::vector<int> path;
    graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, 0, best_vertex_id_);
    for (int i = 0; i < path.size(); i++)
    {
      //遍历这条路径，将路径点存入到全局图中
      tempvertex_.position = local_graph_.vertices[path[i]].location;
      tempvertex_.orientation.y = local_graph_.vertices[path[i]].information_gain;
      addNewGlobalVertexWithoutDuplicates(tempvertex_);
    }
    // gainID_是在函数getGain中得到的，具体表现为gain值为正的顶点，在这里根据论文内容，要加入到全局图中
    if (gainID_.size() > 0)
    {
      for (int i = 0; i < gainID_.size(); i++)
      {
        //遍历gain值为正的顶点，计算从0号点出发，到该点的最短路径，存入path中
        path.clear();
        graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, 0, gainID_[i]);
        //遍历这条路径，将路径点存入到全局图中
        for (int j = 0; j < path.size(); j++)
        {
          tempvertex_.position = local_graph_.vertices[path[j]].location;
          tempvertex_.orientation.y = local_graph_.vertices[path[j]].information_gain;
          addNewGlobalVertexWithoutDuplicates(tempvertex_);
        }
      }
    }
    publishGlobalGraph();
  }

  // jtbao
  void DualStateGraph::updateGlobalGraphNew()
  {
    //计算从局部图0号点出发，到最好顶点的最短路径，存入path中
    std::vector<int> path;
    graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, 0, best_vertex_id_);
    for (int i = 0; i < path.size(); i++)
    {
      //遍历这条路径，将路径点存入到全局图中
      tempvertex_.position = local_graph_.vertices[path[i]].location;
      tempvertex_.orientation.y = local_graph_.vertices[path[i]].information_gain;
      addNewGlobalVertexWithoutDuplicates(tempvertex_);
    }

    for (auto &graph_vertex : local_graph_.vertices)
    {
      if (graph_vertex.vertex_id > 0)
      {
        double path_information_gain = 0;
        std::vector<int> path;
        graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, 0, graph_vertex.vertex_id);
        if (path.empty())
        {
          // No path exists
          continue;
        }
        for (int i = 0; i < path.size(); i++)
        {
          path_information_gain += local_graph_.vertices[path[i]].information_gain;
        }
        if (path_information_gain > 0)
        {
          gainID_.push_back(graph_vertex.vertex_id);
        }
      }
    }

    if (gainID_.size() > 0)
    {
      for (int i = 0; i < gainID_.size(); i++)
      {
        //遍历gain值为正的顶点，计算从0号点出发，到该点的最短路径，存入path中
        path.clear();
        graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, 0, gainID_[i]);
        //遍历这条路径，将路径点存入到全局图中
        for (int j = 0; j < path.size(); j++)
        {
          tempvertex_.position = local_graph_.vertices[path[j]].location;
          tempvertex_.orientation.y = local_graph_.vertices[path[j]].information_gain;
          addNewGlobalVertexWithoutDuplicates(tempvertex_);
        }
      }
    }
    publishGlobalGraph();
  }

  //根据计算得到的best_vertex_id_，得到从局部图0节点到best_vertex_id_节点的探索方向的单位向量
  void DualStateGraph::updateExploreDirection()
  {
    std::vector<int> path;
    graph_utils_ns::ShortestPathBtwVertex(path, local_graph_, 0, best_vertex_id_);
    if (path.empty())
    {
      return;
    }
    if (path.size() > 5)
    {
      explore_direction_[0] =
          local_graph_.vertices[path[path.size() - 1]].location.x - local_graph_.vertices[path[0]].location.x;
      explore_direction_[1] =
          local_graph_.vertices[path[path.size() - 1]].location.y - local_graph_.vertices[path[0]].location.y;
    }
    else
    {
      explore_direction_[0] =
          local_graph_.vertices[path[path.size() - 1]].location.x - local_graph_.vertices[path[0]].location.x;
      explore_direction_[1] =
          local_graph_.vertices[path[path.size() - 1]].location.y - local_graph_.vertices[path[0]].location.y;
    }
    double length = sqrt(explore_direction_[0] * explore_direction_[0] + explore_direction_[1] * explore_direction_[1]);
    explore_direction_[0] = explore_direction_[0] / length;
    explore_direction_[1] = explore_direction_[1] / length;
  }

  Eigen::Vector3d DualStateGraph::getExploreDirection()
  {
    Eigen::Vector3d exploreDirection(explore_direction_[0], explore_direction_[1], explore_direction_[2]);
    return exploreDirection;
  }

  geometry_msgs::Point DualStateGraph::getBestLocalVertexPosition()
  {
    geometry_msgs::Point best_vertex_location = local_graph_.vertices[best_vertex_id_].location;
    return best_vertex_location;
  }

  geometry_msgs::Point DualStateGraph::getBestGlobalVertexPosition()
  {
    geometry_msgs::Point best_vertex_location = local_graph_.vertices[best_vertex_id_].location;
    return best_vertex_location;
  }

  //订阅机器人位置并增加全局图节点
  void DualStateGraph::keyposeCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    geometry_msgs::Pose keypose;
    keypose.position.x = msg->pose.pose.position.x;
    keypose.position.y = msg->pose.pose.position.y;
    keypose.position.z = msg->pose.pose.position.z;
    keypose.orientation.y = 0;
    addNewGlobalVertexWithKeypose(keypose);

    #ifdef NEW_METHOD
      //record robot trajectory
      StateVec curPos;
      curPos[0] = keypose.position.x;
      curPos[1] = keypose.position.y;
      curPos[2] = keypose.position.z;
      if(robot_trajectory_.size()==0){
        robot_trajectory_.push_back(curPos);
      }
      else{
        StateVec lastPos = robot_trajectory_.back();
        double dist = sqrt(SQ(lastPos[0] - curPos[0]) + SQ(lastPos[1] - curPos[1]) + SQ(lastPos[2] - curPos[2]));
        if(dist>1.0){
          robot_trajectory_.push_back(curPos);
        }
      }
      // jtbao: update global frontier cluster
      removeCandidateGoalPoint(StateVec(keypose.position.x, keypose.position.y, keypose.position.z));
    #endif
  }

  //监听/graph_planner_path,在局部图中找到离路径点最近的顶点，进行碰撞检查，擦除掉不合适的顶点之间的边
  void DualStateGraph::pathCallback(const nav_msgs::Path::ConstPtr &graph_path)
  {
    if (graph_path->poses.size() > 2)
    {
      for (int i = 1; i < graph_path->poses.size() - 1; i++)
      {
        int origin_vertex_id;
        int end_vertex_id;
        geometry_msgs::Point origin_position;
        geometry_msgs::Point end_position;
        origin_vertex_id = graph_utils_ns::GetClosestVertexIdxToPoint(local_graph_, graph_path->poses[i].pose.position);
        end_vertex_id = graph_utils_ns::GetClosestVertexIdxToPoint(local_graph_, graph_path->poses[i + 1].pose.position);
        origin_position = local_graph_.vertices[origin_vertex_id].location;
        end_position = local_graph_.vertices[end_vertex_id].location;
        Eigen::Vector3d origin(origin_position.x, origin_position.y, origin_position.z);
        Eigen::Vector3d end(end_position.x, end_position.y, end_position.z);
        double distance = misc_utils_ns::PointXYZDist(origin_position, robot_pos_);
        if (volumetric_mapping::OctomapManager::CellStatus::kFree != manager_->getLineStatus(origin, end) ||
            (kCropPathWithTerrain && distance < kMinDistanceToRobotToCheck &&
             grid_->collisionCheckByTerrain(origin_position, end_position)))
        {
          for (int j = 0; j < local_graph_.vertices[origin_vertex_id].edges.size(); j++)
          {
            if (local_graph_.vertices[origin_vertex_id].edges[j].vertex_id_end == end_vertex_id &&
                local_graph_.vertices[origin_vertex_id].edges[j].keypose_edge == false)
            {
              local_graph_.vertices[origin_vertex_id].edges.erase(local_graph_.vertices[origin_vertex_id].edges.begin() +
                                                                  j);
              if (planner_status_ == true && global_graph_.vertices[origin_vertex_id].edges[j].keypose_edge == false)
              {
                global_graph_.vertices[origin_vertex_id].edges.erase(
                    global_graph_.vertices[origin_vertex_id].edges.begin() + j);
              }
              break;
            }
          }
          for (int j = 0; j < local_graph_.vertices[end_vertex_id].edges.size(); j++)
          {
            if (local_graph_.vertices[end_vertex_id].edges[j].vertex_id_end == origin_vertex_id &&
                local_graph_.vertices[origin_vertex_id].edges[j].keypose_edge == false)
            {
              local_graph_.vertices[end_vertex_id].edges.erase(local_graph_.vertices[end_vertex_id].edges.begin() + j);
              if (planner_status_ == true && global_graph_.vertices[origin_vertex_id].edges[j].keypose_edge == false)
              {
                global_graph_.vertices[end_vertex_id].edges.erase(global_graph_.vertices[end_vertex_id].edges.begin() +
                                                                  j);
              }
              break;
            }
          }
          execute();
          break;
        }
      }
    }
    execute();
  }

  //订阅/graph_planner_status（由graph_planner节点发布），
  //当全局规划状态处于false的时候，对全局图进行剪枝优化和优化，以及发布
  void DualStateGraph::graphPlannerStatusCallback(const graph_planner::GraphPlannerStatusConstPtr &status)
  {
    graph_planner::GraphPlannerStatus prev_status = graph_planner_status_;
    graph_planner_status_ = *status;
    if (prev_status.status != graph_planner_status_.status &&
        graph_planner_status_.status == graph_planner::GraphPlannerStatus::STATUS_IN_PROGRESS)
    {
      if (planner_status_ == false)
      {
        pruneGlobalGraph();
        publishGlobalGraph();
      }
    }
  }

  DualStateGraph::DualStateGraph(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private,
                                 volumetric_mapping::OctomapManager *manager, OccupancyGrid *grid)
      : nh_(nh), nh_private_(nh_private)
  {
    manager_ = manager;
    grid_ = grid;
    initialize();
  }

  DualStateGraph::~DualStateGraph()
  {
  }

  bool DualStateGraph::initialize()
  {
    best_gain_ = 0;
    best_vertex_id_ = 0;
    explore_direction_[0] = 0;
    explore_direction_[1] = 0;
    explore_direction_[2] = 0;
    globalEdgeSize_ = 0;
    localEdgeSize_ = 0;
    robot_yaw_ = 0.0;
    track_localvertex_idx_ = 0;
    track_globalvertex_idx_ = 0;
    prev_track_keypose_vertex_idx_ = 0;
    DTWValue_ = 0;

    // Read in parameters
    if (!readParameters())
      return false;

    // Initialize subscriber
    key_pose_sub_ = nh_.subscribe<nav_msgs::Odometry>(sub_keypose_topic_, 5, &DualStateGraph::keyposeCallback, this);
    graph_planner_path_sub_ = nh_.subscribe<nav_msgs::Path>(sub_path_topic_, 1, &DualStateGraph::pathCallback, this);
    graph_planner_status_sub_ = nh_.subscribe<graph_planner::GraphPlannerStatus>(
        sub_graph_planner_status_topic_, 1, &DualStateGraph::graphPlannerStatusCallback, this);

    // Initialize publishers
    local_graph_pub_ = nh_.advertise<graph_utils::TopologicalGraph>(pub_local_graph_topic_, 2);
    global_graph_pub_ = nh_.advertise<graph_utils::TopologicalGraph>(pub_global_graph_topic_, 2);
    graph_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_global_points_topic_, 2);

    // jtbao
    local_frontier_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_local_frontier_cluster_topic_, 2);
    local_candidate_goal_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_local_candidate_goal_topic_, 2);
    candidate_goal_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_candidate_goal_points_topic_, 2);

    ROS_INFO("Successfully launched DualStateGraph node");

    return true;
  }

  bool DualStateGraph::execute()
  {
    // Update the graph
    publishLocalGraph();

    return true;
  }

}
// namespace dsvplanner_ns
