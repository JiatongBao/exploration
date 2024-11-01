/*
drrtp.cpp
Implementation of drrt_planner class

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <dsvplanner/drrtp.h>
#include <graph_utils.h>

using namespace Eigen;

dsvplanner_ns::drrtPlanner::drrtPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private)
{
  //八叉树体素化稀疏点云建图
  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);

  //订阅/state_estimation和/terrain_map_ext，建立局部栅格地图，每个栅格有四种状态：unknown, free, occupied, near_occupied
  grid_ = new OccupancyGrid(nh_, nh_private_);

  //图管理器
  dual_state_graph_ = new DualStateGraph(nh_, nh_private_, manager_, grid_);

  //边界管理器
  dual_state_frontier_ = new DualStateFrontier(nh_, nh_private_, manager_, grid_);

  drrt_ = new Drrt(manager_, dual_state_graph_, dual_state_frontier_, grid_);

  init();
  drrt_->setParams(params_);
  drrt_->init();

  #ifdef NEW_METHOD
    dual_state_graph_->setParams(params_);
  #endif
  last_exploration_stage_ = true;

  ROS_INFO("Successfully launched DSVP node");
}

dsvplanner_ns::drrtPlanner::~drrtPlanner()
{
  if (manager_)
  {
    delete manager_;
  }
  if (dual_state_graph_)
  {
    delete dual_state_graph_;
  }
  if (dual_state_frontier_)
  {
    delete dual_state_frontier_;
  }
  if (grid_)
  {
    delete grid_;
  }
  if (drrt_)
  {
    delete drrt_;
  }
}

void dsvplanner_ns::drrtPlanner::odomCallback(const nav_msgs::Odometry &pose)
{
  drrt_->setRootWithOdom(pose);
  // Planner is now ready to plan.
  drrt_->plannerReady_ = true;
}

void dsvplanner_ns::drrtPlanner::boundaryCallback(const geometry_msgs::PolygonStamped &boundary)
{
  drrt_->setBoundary(boundary);
  #ifdef NEW_METHOD
  #else
    dual_state_frontier_->setBoundary(boundary);
  #endif
}

//服务器/drrtPlannerSrv，被exploration.cpp所调用
bool dsvplanner_ns::drrtPlanner::plannerServiceCallbackNew(dsvplanner::dsvplanner_srv::Request &req,
                                                        dsvplanner::dsvplanner_srv::Response &res)
{
  plan_start_ = std::chrono::steady_clock::now();
  // drrt_->gotoxy(0, 10);  // Go to the specific line on the screen
  // Check if the planner is ready.
  if (!drrt_->plannerReady_)
  {
    std::cout << "No odometry. Planner is not ready!" << std::endl;
    return true;
  }
  if (manager_ == NULL)
  {
    std::cout << "No octomap. Planner is not ready!" << std::endl;
    return true;
  }
  if (manager_->getMapSize().norm() <= 0.0)
  {
    std::cout << "Octomap is empty. Planner is not set up!" << std::endl;
    return true;
  }

  // set terrain points and terrain voxel elevation
  drrt_->setTerrainVoxelElev();

  // Clear old tree and the last global frontier.
  // cleanLastSelectedGlobalFrontier(); //jtbao: do not need in our method
  drrt_->clear();

  // Reinitialize.
  drrt_->plannerInitNew();

  // Iterate the tree construction method.
  int loopCount = 0;
  //如果正常迭代情况下，新采样点数量大于params_.kVertexSize (默认120)且有增益，则停止采样
  //否则一直采样，直到新采样点数量大于params_.kCuttoffIterations (默认200)或者总次数超过特定数量时，才停止采样
  while (ros::ok() && drrt_->getNodeCounter() < params_.kCuttoffIterations &&
         !(drrt_->normal_local_iteration_ && (drrt_->getNodeCounter() >= params_.kVertexSize && drrt_->gainFound())))
  {
    if (loopCount > drrt_->loopCount_ * (drrt_->getNodeCounter() + 1))
    {
      break;
    }
    //采样新的点，加入到局部图中
    drrt_->plannerIterateNew();
    loopCount++;
  }

  // std::cout<<"flood filling ..." <<std::endl;
  // //Using flood fill to generate the tree
  // drrt_->floodFill(drrt_->root_[0],drrt_->root_[1],drrt_->root_[2]);
  // dual_state_graph_->execute();


  // Publish rrt
  drrt_->publishNode();
  std::cout << "     New node number is " << drrt_->getNodeCounter() << "\n"
            << "     Current local RRT size is " << dual_state_graph_->getLocalVertexSize() << "\n"
            << "     Current global graph size is " << dual_state_graph_->getGlobalVertexSize() << std::endl;
  RRT_generate_over_ = std::chrono::steady_clock::now();
  time_span = RRT_generate_over_ - plan_start_;
  double rrtGenerateTime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
  // Reset planner state
  // 指示本次迭代应为全局规划还是局部探索阶段(由上一次迭代输出)
  drrt_->global_plan_pre_ = drrt_->global_plan_;
  // 重置本次迭代输出的状态
  drrt_->global_plan_ = false;
  drrt_->local_plan_ = false;

  // update planner state of last iteration
  // jtbao: do not need
  // dual_state_frontier_->setPlannerStatus(drrt_->global_plan_pre_);

  // update local frontier clusters
  int clusterNum = 0;
  if (!drrt_->global_plan_pre_) //当前是局部探索阶段,对采样后的点进行聚类
  {
    clusterNum = dual_state_graph_->clusteringLocalGraphVertexHasGain();
    std::cout << "     Local clusters: " << clusterNum << std::endl;
  }

  // Update planner state of next iteration
  geometry_msgs::Point robot_position;
  robot_position.x = drrt_->root_[0];
  robot_position.y = drrt_->root_[1];
  robot_position.z = drrt_->root_[2];
  // drrt求解器没有发现下一个要去的全局规划路径点，当前处于全局规划状态，并且全局图中也没有一个点还能有新的信息增益, 认为探索已经完成，可以返航了
  if (!drrt_->nextNodeFound_ && drrt_->global_plan_pre_ && drrt_->gainFound() <= 0)
  {
    drrt_->return_home_ = true;
    geometry_msgs::Point home_position;
    home_position.x = 0;
    home_position.y = 0;
    home_position.z = 0;
    res.goal.push_back(home_position);
    res.mode.data = 2; // mode 2 means returning home

    // jtbao: do not need
    // dual_state_frontier_->cleanAllUselessFrontiers();
    return true;
  }
  // drrt求解器没有发现下一个要去的全局规划路径点，现处于局部探索阶段，并且没有边界点簇
  else if (!drrt_->nextNodeFound_ && !drrt_->global_plan_pre_ && clusterNum == 0)
  {
    // if(last_exploration_stage_){
    //   StateVec lastLocalGoal = dual_state_graph_->getLastLocalGoal();
    //   double dist = dual_state_graph_->getDistFromNearestLocalGraphPointTo(lastLocalGoal);
    //   if(dist > 1.0){
    //     std::cout << "     Warning: the RRT didn't cover the last local goal. Try exploration stage again." << std::endl;
    //     drrt_->local_plan_ = true;
    //     node_zero_cnt_++;
    //     if(node_zero_cnt_<=3)
    //       return true;
    //   }
    // }
    // node_zero_cnt_ = 0;

    // retry exploration
    #ifdef RETRY_EXPLORATION
    drrt_->local_plan_ = true;
    retry_cnt_++;
    if(retry_cnt_<=1){
      std::cout << "     Try exploration stage again !!!!!!!!!!!!!!!!!" << std::endl;
      return true;
    }
    retry_cnt_ = 0;
    #endif

    //认为当前位置的边界已经探索完成，接下来进入relocation阶段，准备出发去下一个全局候选目标点
    drrt_->global_plan_ = true;
    std::cout << "     No Remaining local frontiers  "
              << "\n"
              << "     Switch to relocation stage "
              << "\n"
              << "     Total plan lasted " << 0 << std::endl;
    return true;
  }
  else
  {
    //下一次迭代为局部探索阶段
    drrt_->local_plan_ = true;
  }
  gain_computation_over_ = std::chrono::steady_clock::now();
  time_span = gain_computation_over_ - RRT_generate_over_;
  double getGainTime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;

  // Extract next goal.
  geometry_msgs::Point next_goal_position;
  // nextNodeFound_表示发现了下一时刻应该去的全局节点位置
  if (drrt_->nextNodeFound_)
  {
    dual_state_graph_->best_vertex_id_ = drrt_->NextBestNodeIdx_;
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestGlobalVertexPosition();
    last_exploration_stage_ = false;
  }
  //未发现下一个节点，但现处于全局规划状态，并且有新的信息增益, 这种情况下，下一个目标点是局部点
  else if (drrt_->global_plan_pre_ == true && drrt_->gainFound())
  {
    dual_state_graph_->best_vertex_id_ = drrt_->bestNodeId_;
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
    last_exploration_stage_ = false;
  }
  else
  {
    StateVec cur_pos = drrt_->root_;
    dual_state_graph_->findLocalTour(cur_pos, local_tsp_visits_);
    if(local_tsp_visits_.size()==0)
      std::cout << "This should not happen." <<std::endl;
    next_goal_position.x = local_tsp_visits_[0][0];
    next_goal_position.y = local_tsp_visits_[0][1];
    next_goal_position.z = local_tsp_visits_[0][2];
    // manage the candidate goals
    dual_state_graph_->removeCandidateGoalPoint(local_tsp_visits_[0]);
    for (int i = 1; i < local_tsp_visits_.size(); i++)
    {
      dual_state_graph_->addCandidateGoalPoint(local_tsp_visits_[i]);
    }
    dual_state_graph_->publishCandidateGoals();
    dual_state_graph_->best_vertex_id_ = graph_utils_ns::GetClosestVertexIdxToPoint(dual_state_graph_->local_graph_, next_goal_position);
    //dual_state_graph_->best_vertex_id_ = dual_state_graph_->getSafetyVertexIdxToPoint(StateVec(next_goal_position.x, next_goal_position.y, next_goal_position.z));

    //未发现下一个该去的节点，之前处于探索阶段，就应该更新全局图, 下一个目标点是局部点
    dual_state_graph_->updateGlobalGraphNew();
    dual_state_graph_->updateExploreDirection();
    // dual_state_graph_->getGain(robot_position)会计算出best_gain_和best_vertex_id_，根据best_vertex_id_，dual_state_graph_->getBestLocalVertexPosition()直接获取对应的节点位置
    next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
    dual_state_graph_->setLastLocalGoal(StateVec(next_goal_position.x,next_goal_position.y,next_goal_position.z));
    last_exploration_stage_ = true;
  }

  retry_cnt_ = 0;

  dual_state_graph_->setCurrentPlannerStatus(drrt_->global_plan_pre_);
  //将下一个目标点装入服务器应答，进行返回
  //但是此处目测，只装了一次答复，
  //因此我们可以得知，在exploration.cpp中，请求到的服务，只返回一个路径点(虽然里面写成了循环的形式)
  res.goal.push_back(next_goal_position);
  res.mode.data = 1; // mode 1 means exploration

  geometry_msgs::PointStamped next_goal_point;
  next_goal_point.header.frame_id = "map";
  next_goal_point.point = next_goal_position;
  params_.nextGoalPub_.publish(next_goal_point);

  plan_over_ = std::chrono::steady_clock::now();
  time_span = plan_over_ - plan_start_;
  double plantime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
  std::cout << "     RRT generation lasted  " << rrtGenerateTime << "\n"
            << "     Computiong gain lasted " << getGainTime << "\n"
            << "     Total plan lasted " << plantime << std::endl;
  return true;
}

bool dsvplanner_ns::drrtPlanner::plannerServiceCallback(dsvplanner::dsvplanner_srv::Request &req,
                                                        dsvplanner::dsvplanner_srv::Response &res)
{
  plan_start_ = std::chrono::steady_clock::now();
  // drrt_->gotoxy(0, 10);  // Go to the specific line on the screen
  // Check if the planner is ready.
  if (!drrt_->plannerReady_)
  {
    std::cout << "No odometry. Planner is not ready!" << std::endl;
    return true;
  }
  if (manager_ == NULL)
  {
    std::cout << "No octomap. Planner is not ready!" << std::endl;
    return true;
  }
  if (manager_->getMapSize().norm() <= 0.0)
  {
    std::cout << "Octomap is empty. Planner is not set up!" << std::endl;
    return true;
  }

  // set terrain points and terrain voxel elevation
  drrt_->setTerrainVoxelElev();

  // Clear old tree and the last global frontier.
  cleanLastSelectedGlobalFrontier();
  drrt_->clear();

  // Reinitialize.
  drrt_->plannerInit();

  // Iterate the tree construction method.
  int loopCount = 0;
  while (ros::ok() && drrt_->remainingFrontier_ && drrt_->getNodeCounter() < params_.kCuttoffIterations &&
         !(drrt_->normal_local_iteration_ && (drrt_->getNodeCounter() >= params_.kVertexSize && drrt_->gainFound())))
  {
    if (loopCount > drrt_->loopCount_ * (drrt_->getNodeCounter() + 1))
    {
      break;
    }
    //采样新的点（靠近边界或者机器人附近）,加入到局部图中
    drrt_->plannerIterate();
    loopCount++;
  }

  // Publish rrt
  drrt_->publishNode();
  std::cout << "     New node number is " << drrt_->getNodeCounter() << "\n"
            << "     Current local RRT size is " << dual_state_graph_->getLocalVertexSize() << "\n"
            << "     Current global graph size is " << dual_state_graph_->getGlobalVertexSize() << std::endl;
  RRT_generate_over_ = std::chrono::steady_clock::now();
  time_span = RRT_generate_over_ - plan_start_;
  double rrtGenerateTime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
  // Reset planner state
  drrt_->global_plan_pre_ = drrt_->global_plan_;
  drrt_->global_plan_ = false;
  drrt_->local_plan_ = false;

  // update planner state of last iteration
  dual_state_frontier_->setPlannerStatus(drrt_->global_plan_pre_);

  // Update planner state of next iteration
  geometry_msgs::Point robot_position;
  robot_position.x = drrt_->root_[0];
  robot_position.y = drrt_->root_[1];
  robot_position.z = drrt_->root_[2];
  // drrt求解器没有发现下一个要去的距离全局边界最近的点，并且上一时刻处于全局规划状态，并且全局图中也没有一个点还能有新的信息增益, 认为探索已经完成，可以返航了
  if (!drrt_->nextNodeFound_ && drrt_->global_plan_pre_ && drrt_->gainFound() <= 0)
  {
    drrt_->return_home_ = true;
    geometry_msgs::Point home_position;
    home_position.x = 0;
    home_position.y = 0;
    home_position.z = 0;
    res.goal.push_back(home_position);
    res.mode.data = 2; // mode 2 means returning home

    dual_state_frontier_->cleanAllUselessFrontiers();
    return true;
  }
  // drrt求解器没有发现下一个节点，并且上一时刻并不是处于全局规划阶段，当前位置没有信息增益（得到当前区域朝着探索方向的最大的gain，以及节点号dual_state_graph_->best_vertex_id_）
  else if (!drrt_->nextNodeFound_ && !drrt_->global_plan_pre_ && dual_state_graph_->getGain(robot_position) <= 0)
  {
    //认为当前位置的边界已经探索完成，接下来进入relocation阶段，准备出发去下一个全局边界处
    drrt_->global_plan_ = true;
    std::cout << "     No Remaining local frontiers  "
              << "\n"
              << "     Switch to relocation stage "
              << "\n"
              << "     Total plan lasted " << 0 << std::endl;
    return true;
  }
  else
  {
    //当前局部窗口内还存在边界点，当前处于局部窗口探索规划阶段
    drrt_->local_plan_ = true;
  }
  gain_computation_over_ = std::chrono::steady_clock::now();
  time_span = gain_computation_over_ - RRT_generate_over_;
  double getGainTime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;

  // Extract next goal.
  geometry_msgs::Point next_goal_position;
  // nextNodeFound_表示发现了下一时刻应该去的全局节点位置
  if (drrt_->nextNodeFound_)
  {
    dual_state_graph_->best_vertex_id_ = drrt_->NextBestNodeIdx_;
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestGlobalVertexPosition();
  }
  //未发现下一个节点，但全局规划之前处于true状态，并且有新的信息增益, 这种情况下，下一个目标点是局部点
  else if (drrt_->global_plan_pre_ == true && drrt_->gainFound())
  {
    dual_state_graph_->best_vertex_id_ = drrt_->bestNodeId_;
    dual_state_graph_->updateExploreDirection();
    next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
  }
  else
  {
    //未发现下一个该去的节点，之前处于探索阶段，就应该更新全局图, 下一个目标点是局部点
    dual_state_graph_->updateGlobalGraph();
    dual_state_graph_->updateExploreDirection();
    // dual_state_graph_->getGain(robot_position)会计算出best_gain_和best_vertex_id_，根据best_vertex_id_，dual_state_graph_->getBestLocalVertexPosition()直接获取对应的节点位置
    next_goal_position = dual_state_graph_->getBestLocalVertexPosition();
  }
  dual_state_graph_->setCurrentPlannerStatus(drrt_->global_plan_pre_);
  //将下一个目标点装入服务器应答，进行返回
  //但是此处目测，只装了一次答复，
  //因此我们可以得知，在exploration.cpp中，请求到的服务，只返回一个路径点(虽然里面写成了循环的形式)
  res.goal.push_back(next_goal_position);
  res.mode.data = 1; // mode 1 means exploration

  geometry_msgs::PointStamped next_goal_point;
  next_goal_point.header.frame_id = "map";
  next_goal_point.point = next_goal_position;
  params_.nextGoalPub_.publish(next_goal_point);

  plan_over_ = std::chrono::steady_clock::now();
  time_span = plan_over_ - plan_start_;
  double plantime =
      double(time_span.count()) * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den;
  std::cout << "     RRT generation lasted  " << rrtGenerateTime << "\n"
            << "     Computiong gain lasted " << getGainTime << "\n"
            << "     Total plan lasted " << plantime << std::endl;
  return true;
}

bool dsvplanner_ns::drrtPlanner::cleanFrontierServiceCallback(dsvplanner::clean_frontier_srv::Request &req,
                                                              dsvplanner::clean_frontier_srv::Response &res)
{
  if (drrt_->nextNodeFound_)
  {
    #ifdef NEW_METHOD
    #else
      dual_state_frontier_->updateToCleanFrontier(drrt_->selectedGlobalFrontier_);
      dual_state_frontier_->globalFrontierUpdate();
    #endif
  }
  else
  {
    #ifdef NEW_METHOD
      dual_state_graph_->cleanLastLocalGoal();
    #endif
    dual_state_graph_->clearLocalGraph();
  }
  res.success = true;

  return true;
}

void dsvplanner_ns::drrtPlanner::cleanLastSelectedGlobalFrontier()
{
  // only when last plan is global plan, this function will be executed to clear
  // last selected global
  // frontier.
  if (drrt_->nextNodeFound_)
  {
    #ifdef NEW_METHOD
    #else
      dual_state_frontier_->updateToCleanFrontier(drrt_->selectedGlobalFrontier_);
      dual_state_frontier_->globalFrontierUpdate();
    #endif
  }
}

bool dsvplanner_ns::drrtPlanner::setParams()
{
  nh_private_.getParam("/rm/kSensorPitch", params_.sensorPitch);
  nh_private_.getParam("/rm/kSensorHorizontal", params_.sensorHorizontalView);
  nh_private_.getParam("/rm/kSensorVertical", params_.sensorVerticalView);
  nh_private_.getParam("/rm/kVehicleHeight", params_.kVehicleHeight);
  nh_private_.getParam("/rm/kBoundX", params_.boundingBox[0]);
  nh_private_.getParam("/rm/kBoundY", params_.boundingBox[1]);
  nh_private_.getParam("/rm/kBoundZ", params_.boundingBox[2]);
  nh_private_.getParam("/drrt/gain/kFree", params_.kGainFree);
  nh_private_.getParam("/drrt/gain/kOccupied", params_.kGainOccupied);
  nh_private_.getParam("/drrt/gain/kUnknown", params_.kGainUnknown);
  nh_private_.getParam("/drrt/gain/kMinEffectiveGain", params_.kMinEffectiveGain);
  nh_private_.getParam("/drrt/gain/kRange", params_.kGainRange);
  nh_private_.getParam("/drrt/gain/kRangeZMinus", params_.kGainRangeZMinus);
  nh_private_.getParam("/drrt/gain/kRangeZPlus", params_.kGainRangeZPlus);
  nh_private_.getParam("/drrt/gain/kZero", params_.kZeroGain);
  nh_private_.getParam("/drrt/tree/kExtensionRange", params_.kExtensionRange);
  nh_private_.getParam("/drrt/tree/kMinExtensionRange", params_.kMinextensionRange);
  nh_private_.getParam("/drrt/tree/kMaxExtensionAlongZ", params_.kMaxExtensionAlongZ);
  nh_private_.getParam("/rrt/tree/kExactRoot", params_.kExactRoot);
  nh_private_.getParam("/drrt/tree/kCuttoffIterations", params_.kCuttoffIterations);
  nh_private_.getParam("/drrt/tree/kGlobalExtraIterations", params_.kGlobalExtraIterations);
  nh_private_.getParam("/drrt/tree/kRemainingNodeScaleSize", params_.kRemainingNodeScaleSize);
  nh_private_.getParam("/drrt/tree/kRemainingBranchScaleSize", params_.kRemainingBranchScaleSize);
  nh_private_.getParam("/drrt/tree/kNewNodeScaleSize", params_.kNewNodeScaleSize);
  nh_private_.getParam("/drrt/tree/kNewBranchScaleSize", params_.kNewBranchScaleSize);
  nh_private_.getParam("/drrt/tfFrame", params_.explorationFrame);
  nh_private_.getParam("/drrt/vertexSize", params_.kVertexSize);
  nh_private_.getParam("/drrt/keepTryingNum", params_.kKeepTryingNum);
  nh_private_.getParam("/drrt/kLoopCountThres", params_.kLoopCountThres);
  nh_private_.getParam("/lb/kMinXLocal", params_.kMinXLocalBound);
  nh_private_.getParam("/lb/kMinYLocal", params_.kMinYLocalBound);
  nh_private_.getParam("/lb/kMinZLocal", params_.kMinZLocalBound);
  nh_private_.getParam("/lb/kMaxXLocal", params_.kMaxXLocalBound);
  nh_private_.getParam("/lb/kMaxYLocal", params_.kMaxYLocalBound);
  nh_private_.getParam("/lb/kMaxZLocal", params_.kMaxZLocalBound);
  nh_private_.getParam("/gb/kMinXGlobal", params_.kMinXGlobalBound);
  nh_private_.getParam("/gb/kMinYGlobal", params_.kMinYGlobalBound);
  nh_private_.getParam("/gb/kMinZGlobal", params_.kMinZGlobalBound);
  nh_private_.getParam("/gb/kMaxXGlobal", params_.kMaxXGlobalBound);
  nh_private_.getParam("/gb/kMaxYGlobal", params_.kMaxYGlobalBound);
  nh_private_.getParam("/gb/kMaxZGlobal", params_.kMaxZGlobalBound);
  nh_private_.getParam("/elevation/kTerrainVoxelSize", params_.kTerrainVoxelSize);
  nh_private_.getParam("/elevation/kTerrainVoxelWidth", params_.kTerrainVoxelWidth);
  nh_private_.getParam("/elevation/kTerrainVoxelHalfWidth", params_.kTerrainVoxelHalfWidth);
  nh_private_.getParam("/planner/odomSubTopic", odomSubTopic);
  nh_private_.getParam("/planner/boundarySubTopic", boundarySubTopic);
  nh_private_.getParam("/planner/newTreePathPubTopic", newTreePathPubTopic);
  nh_private_.getParam("/planner/remainingTreePathPubTopic", remainingTreePathPubTopic);
  nh_private_.getParam("/planner/boundaryPubTopic", boundaryPubTopic);
  nh_private_.getParam("/planner/globalSelectedFrontierPubTopic", globalSelectedFrontierPubTopic);
  nh_private_.getParam("/planner/localSelectedFrontierPubTopic", localSelectedFrontierPubTopic);
  nh_private_.getParam("/planner/plantimePubTopic", plantimePubTopic);
  nh_private_.getParam("/planner/nextGoalPubTopic", nextGoalPubTopic);
  nh_private_.getParam("/planner/randomSampledPointsPubTopic", randomSampledPointsPubTopic);
  nh_private_.getParam("/planner/shutDownTopic", shutDownTopic);
  nh_private_.getParam("/planner/plannerServiceName", plannerServiceName);
  nh_private_.getParam("/planner/cleanFrontierServiceName", cleanFrontierServiceName);

  return true;
}

bool dsvplanner_ns::drrtPlanner::init()
{
  if (!setParams())
  {
    ROS_ERROR("Set parameters fail. Cannot start planning!");
  }

  odomSub_ = nh_.subscribe(odomSubTopic, 10, &dsvplanner_ns::drrtPlanner::odomCallback, this);
  boundarySub_ = nh_.subscribe(boundarySubTopic, 10, &dsvplanner_ns::drrtPlanner::boundaryCallback, this);

  params_.newTreePathPub_ = nh_.advertise<visualization_msgs::Marker>(newTreePathPubTopic, 1000);
  params_.remainingTreePathPub_ = nh_.advertise<visualization_msgs::Marker>(remainingTreePathPubTopic, 1000);
  params_.boundaryPub_ = nh_.advertise<visualization_msgs::Marker>(boundaryPubTopic, 1000);
  params_.globalSelectedFrontierPub_ = nh_.advertise<sensor_msgs::PointCloud2>(globalSelectedFrontierPubTopic, 1000);
  params_.localSelectedFrontierPub_ = nh_.advertise<sensor_msgs::PointCloud2>(localSelectedFrontierPubTopic, 1000);
  params_.randomSampledPointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(randomSampledPointsPubTopic, 1000);
  params_.plantimePub_ = nh_.advertise<std_msgs::Float32>(plantimePubTopic, 1000);
  params_.nextGoalPub_ = nh_.advertise<geometry_msgs::PointStamped>(nextGoalPubTopic, 1000);
  params_.shutdownSignalPub = nh_.advertise<std_msgs::Bool>(shutDownTopic, 1000);

  #ifdef NEW_METHOD
    plannerService_ = nh_.advertiseService(plannerServiceName, &dsvplanner_ns::drrtPlanner::plannerServiceCallbackNew, this);
  #else
    plannerService_ = nh_.advertiseService(plannerServiceName, &dsvplanner_ns::drrtPlanner::plannerServiceCallback, this);
  #endif
  cleanFrontierService_ =
      nh_.advertiseService(cleanFrontierServiceName, &dsvplanner_ns::drrtPlanner::cleanFrontierServiceCallback, this);

  return true;
}
