/**************************************************************************
dual_state_graph.h
Header of the dual_state_graph class

Hongbiao Zhu(hongbiaz@andrew.cmu.edu)
5/25/2020
**************************************************************************/
#ifndef DUAL_STATE_GRAPH_H
#define DUAL_STATE_GRAPH_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include "dsvplanner/grid.h"
#include "graph_planner/GraphPlannerStatus.h"
#include "graph_utils/TopologicalGraph.h"
#include "octomap_world/octomap_manager.h"
#include "def.h"

namespace dsvplanner_ns
{
class DualStateGraph
{
public:
  typedef std::shared_ptr<DualStateGraph> Ptr;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS subscribers
  ros::Subscriber key_pose_sub_;
  ros::Subscriber graph_planner_path_sub_;
  ros::Subscriber graph_planner_status_sub_;

  // ROS publishers
  ros::Publisher local_graph_pub_;
  ros::Publisher global_graph_pub_;
  ros::Publisher graph_points_pub_;

  // String constants
  std::string world_frame_id_;
  std::string pub_local_graph_topic_;
  std::string pub_global_graph_topic_;
  std::string pub_global_points_topic_;
  std::string sub_keypose_topic_;
  std::string sub_path_topic_;
  std::string sub_graph_planner_status_topic_;

  // Constants
  bool kCropPathWithTerrain;
  double kConnectVertexDistMax;
  double kDistBadEdge;
  double kDegressiveCoeff;
  double kDirectionCoeff;
  double kExistingPathRatioThreshold;
  double kExistingPathRatioThresholdGlobal;
  double kLongEdgePenaltyMultiplier;
  double kMinVertexDist;
  double kMaxLongEdgeDist;
  double kMaxVertexAngleAlongZ;
  double kMaxVertexDiffAlongZ;
  double kMaxDistToPrunedRoot;
  double kMaxPrunedNodeDist;
  double kSurroundRange;
  double kMinGainRange;
  double kMinDistanceToRobotToCheck;
  Eigen::Vector3d robot_bounding;

  // Variables
  Eigen::Vector3d explore_direction_;
  geometry_msgs::Point robot_pos_;
  geometry_msgs::Pose tempvertex_;
  graph_utils::TopologicalGraph global_graph_;
  graph_utils::TopologicalGraph local_graph_;
  graph_utils::TopologicalGraph pruned_graph_;
  graph_planner::GraphPlannerStatus graph_planner_status_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr graph_point_cloud_ =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<int> gainID_;
  volumetric_mapping::OctomapManager* manager_;
  OccupancyGrid* grid_;

  bool planner_status_;  // false means local plan and true means global plan
  int track_localvertex_idx_;
  int track_globalvertex_idx_;
  int prev_track_vertex_idx_;
  int prev_track_keypose_vertex_idx_;

  int localEdgeSize_;
  int globalEdgeSize_;
  int best_vertex_id_;
  double best_gain_;
  double DTWValue_;
  double robot_yaw_;

  // Feedback Functions
  double getGain(geometry_msgs::Point robot_position);
  int getLocalVertexSize();
  int getLocalEdgeSize();
  int getGlobalVertexSize();
  int getGlobalEdgeSize();
  geometry_msgs::Point getBestLocalVertexPosition();
  geometry_msgs::Point getBestGlobalVertexPosition();
  Eigen::Vector3d getExploreDirection();

  // General Functions
  void addEdgeWithoutCheck(int start_vertex_idx, int end_vertex_idx, graph_utils::TopologicalGraph& graph);
  void addEdge(int start_vertex_idx, int end_vertex_idx, graph_utils::TopologicalGraph& graph);
  void addNewLocalVertex(geometry_msgs::Pose& vertex_msg, graph_utils::TopologicalGraph& graph);
  void addNewLocalVertexWithoutEdge(geometry_msgs::Pose& vertex_msg, graph_utils::TopologicalGraph& graph);
  void addNewLocalVertexWithoutDuplicates(geometry_msgs::Pose& vertex_msg, graph_utils::TopologicalGraph& graph);
  void addNewPrunedVertex(geometry_msgs::Pose& vertex_msg, graph_utils::TopologicalGraph& graph);
  void addGlobalEdgeWithoutCheck(int start_vertex_idx, int end_vertex_idx, bool trajectory_edge);
  void addGlobalEdge(int start_vertex_idx, int end_vertex_idx);
  void addNewGlobalVertex(geometry_msgs::Pose& vertex_msg);
  void addNewGlobalVertexWithoutDuplicates(geometry_msgs::Pose& vertex_msg);
  void addNewGlobalVertexWithKeypose(geometry_msgs::Pose& vertex_msg);
  void clearLocalGraph();
  void DTW(std::vector<int> path, geometry_msgs::Point robot_position);
  void pruneGraph(geometry_msgs::Point root);
  void pruneGlobalGraph();
  void publishLocalGraph();
  void publishGlobalGraph();
  void setCurrentPlannerStatus(bool status);
  void updateGlobalGraph();
  void updateExploreDirection();

  bool zCollisionCheck(int start_vertex_idx, int end_vertex_idx, graph_utils::TopologicalGraph graph);

  // Callback Functions
  void keyposeCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void pathCallback(const nav_msgs::Path::ConstPtr& graph_path);
  void graphPlannerStatusCallback(const graph_planner::GraphPlannerStatusConstPtr& status);

  // jtbao added variables and functions
  Params params_;
  std::vector<StateVec> robot_trajectory_;
  StateVec last_local_goal_;
  std::vector<StateVec> cleaned_goals_;
  std::string pub_local_frontier_cluster_topic_;
  std::string pub_local_candidate_goal_topic_;
  std::string pub_candidate_goal_points_topic_;
  std::string tsp_dir_;
  ros::Publisher local_frontier_cluster_pub_;
  ros::Publisher local_candidate_goal_pub_;
  ros::Publisher candidate_goal_points_pub_;
  std::vector<std::vector<Eigen::Vector4d>> graph_gain_point_cloud_clusters_;
  Eigen::MatrixXd local_cost_mat_;
  std::vector<StateVec> cluster_centroids_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_frontier_cluster_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_candidate_goal_pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<StateVec> candidate_goals_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_goal_point_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  void setParams(Params params);
  void publishLocalGraphVertexHasGain();
  void publishCandidateGoals();
  int clusteringLocalGraphVertexHasGain();
  void calcLocalCandidateGoals();
  void addGlobalClusterIntoLocalClusters();
  void getFullCostMatrix(const Vector3d& cur_pos, Eigen::MatrixXd& mat);
  void findLocalTour(const Vector3d& cur_pos, std::vector<StateVec>& visits);
  void selectCandidateGoalsForGlobalPlanning(std::vector<StateVec> &res);
  void getGlobalFullCostMatrix(const Vector3d& cur_pos, const std::vector<StateVec> selected_goals, Eigen::MatrixXd& mat);
  void findGlobalTour(const Vector3d& cur_pos, std::vector<StateVec>& visits);
  void addCandidateGoalPoint(const StateVec &point);
  void updateGlobalGraphNew();
  void removeCandidateGoalPoint(const StateVec &robotPos);
  bool isOnTrajectory(const StateVec &point);
  void clearCandidateGoalsInLocalBoundary();
  geometry_msgs::Point getBestLocalVertexPositionInMaximumCluster();
  int getSafetyVertexIdxToPoint(const StateVec& pnt);
  void setLastLocalGoal(StateVec goal);
  StateVec getLastLocalGoal();
  double getDistFromNearestLocalGraphPointTo(const StateVec &goal);
  void addAsCleanedGoal(StateVec goal);
  bool isCleanedGoal(StateVec goal);
  void cleanLastLocalGoal();
  bool isClusterCentroid(StateVec pt);
  bool checkLocalGraphHasGainInLargerRange(StateVec &goal);
  StateVec calcMean(std::vector<StateVec> cluster);
  StateVec findMaxGainPoint(std::vector<Eigen::Vector4d> cluster);
  StateVec adjustCentroid(std::vector<StateVec> cluster, StateVec robotPos);
public:
  DualStateGraph(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                 volumetric_mapping::OctomapManager* manager, OccupancyGrid* grid);
  bool readParameters();
  bool initialize();
  bool execute();
  ~DualStateGraph();
};
}
#endif  // DUAL_STATE_GRAPH_H
