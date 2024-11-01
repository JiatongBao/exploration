/*
drrt.cpp
Implementation of Drrt class. Dynamic tree is used to get random viewpoints in
local area. New rrt
is generated based on the pruned tree of last time.

Created by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
*/

#ifndef RRTTREE_HPP_
#define RRTTREE_HPP_

#include <cstdlib>
#include <dsvplanner/drrt.h>
#include <misc_utils/misc_utils.h>
#include <graph_utils.h>

dsvplanner_ns::Drrt::Drrt(volumetric_mapping::OctomapManager* manager, DualStateGraph* graph,
                          DualStateFrontier* frontier, OccupancyGrid* grid)
{
  manager_ = manager;
  grid_ = grid;
  dual_state_graph_ = graph;
  dual_state_frontier_ = frontier;

  ROS_INFO("Successfully launched Drrt node");
}

dsvplanner_ns::Drrt::~Drrt()
{
  delete rootNode_;
  kd_free(kdTree_);
}

void dsvplanner_ns::Drrt::init()
{
  kdTree_ = kd_create(3);
  iterationCount_ = 0;
  bestGain_ = params_.kZeroGain;
  bestNode_ = NULL;
  rootNode_ = NULL;
  nodeCounter_ = 0;
  plannerReady_ = false;
  boundaryLoaded_ = false;

  global_plan_ = false;
  global_plan_pre_ = true;
  local_plan_ = true;
  nextNodeFound_ = false;
  remainingFrontier_ = true;
  return_home_ = false;
  global_vertex_size_ = 0;
  NextBestNodeIdx_ = 0;
  for (int i = 0; i < params_.kTerrainVoxelWidth * params_.kTerrainVoxelWidth; i++)
  {
    terrain_voxle_elev_.push_back(params_.kVehicleHeight);
  }

  first_call_ = true; //jtbao add

  srand((unsigned)time(NULL));
}

void dsvplanner_ns::Drrt::setParams(Params params)
{
  params_ = params;
}

void dsvplanner_ns::Drrt::setRootWithOdom(const nav_msgs::Odometry& pose)
{
  root_[0] = pose.pose.pose.position.x;
  root_[1] = pose.pose.pose.position.y;
  root_[2] = pose.pose.pose.position.z;
}

void dsvplanner_ns::Drrt::setBoundary(const geometry_msgs::PolygonStamped& boundary)
{
  boundary_polygon_ = boundary.polygon;
  boundaryLoaded_ = true;
}

void dsvplanner_ns::Drrt::setTerrainVoxelElev()
{
  if (dual_state_frontier_->getTerrainVoxelElev().size() > 0)
  {
    terrain_voxle_elev_.clear();
    terrain_voxle_elev_ = dual_state_frontier_->getTerrainVoxelElev();
  }
}

int dsvplanner_ns::Drrt::getNodeCounter()
{
  return nodeCounter_;
}

int dsvplanner_ns::Drrt::getRemainingNodeCounter()
{
  return remainingNodeCount_;
}

bool dsvplanner_ns::Drrt::gainFound()
{
  return bestGain_ > params_.kZeroGain;
}

double dsvplanner_ns::Drrt::angleDiff(StateVec direction1, StateVec direction2)
{
  double degree;
  degree = acos((direction1[0] * direction2[0] + direction1[1] * direction2[1]) /
                (sqrt(direction1[0] * direction1[0] + direction1[1] * direction1[1]) *
                 sqrt(direction2[0] * direction2[0] + direction2[1] * direction2[1]))) *
           180 / M_PI;
  return degree;
}

double dsvplanner_ns::Drrt::getZvalue(double x_position, double y_position)
{
  int indX =
      int((x_position + params_.kTerrainVoxelSize / 2) / params_.kTerrainVoxelSize) + params_.kTerrainVoxelHalfWidth;
  int indY =
      int((y_position + params_.kTerrainVoxelSize / 2) / params_.kTerrainVoxelSize) + params_.kTerrainVoxelHalfWidth;
  if (x_position + params_.kTerrainVoxelSize / 2 < 0)
    indX--;
  if (y_position + params_.kTerrainVoxelSize / 2 < 0)
    indY--;
  if (indX > params_.kTerrainVoxelWidth - 1)
    indX = params_.kTerrainVoxelWidth - 1;
  if (indX < 0)
    indX = 0;
  if (indY > params_.kTerrainVoxelWidth - 1)
    indY = params_.kTerrainVoxelWidth - 1;
  if (indY < 0)
    indY = 0;
  double z_position = terrain_voxle_elev_[params_.kTerrainVoxelWidth * indX + indY] + params_.kVehicleHeight;
  return z_position;
}

//功能：传入一个方向，该方向和边界求和后，判断延伸后的点是否能很好的观察到边界
//注意：这个函数会修改形参，传入一个方向，在边界附近查找它，然后会被更新为边界附近的点
bool dsvplanner_ns::Drrt::inSensorRange(StateVec& node)
{
  StateVec root_node(rootNode_->state_[0], rootNode_->state_[1], rootNode_->state_[2]);
  StateVec init_node = node;
  StateVec dir;
  bool insideFieldOfView = false;
  for (int i = 0; i < localThreeFrontier_->points.size(); i++)
  {
    StateVec frontier_point(localThreeFrontier_->points[i].x, localThreeFrontier_->points[i].y,
                            localThreeFrontier_->points[i].z);
    node[0] = init_node[0] + frontier_point[0];
    node[1] = init_node[1] + frontier_point[1];
    double x_position = node[0] - root_node[0];
    double y_position = node[1] - root_node[1];
    node[2] = getZvalue(x_position, y_position);
    if (!inPlanningBoundary(node))
      continue;

    dir = frontier_point - node;
    // Skip if distance to sensor is too large
    double rangeSq = pow(params_.kGainRange, 2.0);
    if (dir.transpose().dot(dir) > rangeSq)
    {
      continue;
    }

    if (fabs(dir[2] < sqrt(dir[0] * dir[0] + dir[1] * dir[1]) * tan(M_PI * params_.sensorVerticalView / 360)))
    {
      insideFieldOfView = true;
    }
    if (!insideFieldOfView)
    {
      continue;
    }

    if (manager_->getCellStatusPoint(node) == volumetric_mapping::OctomapManager::CellStatus::kFree)
    {
      if (volumetric_mapping::OctomapManager::CellStatus::kOccupied !=
          this->manager_->getVisibility(node, frontier_point, false))
      {
        return true;
      }
    }
  }
  return false;
}

//是否在以机器人为中心的一个局部搜索区域内
bool dsvplanner_ns::Drrt::inPlanningBoundary(StateVec node)
{
  //minX_等根据机器人当前位置，结合配置的local exploration boundary参数来动态确定
  //params_.boundingBox为机器人自身尺寸参数
  if (node.x() < minX_ + 0.5 * params_.boundingBox.x())
  {
    return false;
  }
  else if (node.y() < minY_ + 0.5 * params_.boundingBox.y())
  {
    return false;
  }
  else if (node.z() < minZ_ + 0.5 * params_.boundingBox.z())
  {
    return false;
  }
  else if (node.x() > maxX_ - 0.5 * params_.boundingBox.x())
  {
    return false;
  }
  else if (node.y() > maxY_ - 0.5 * params_.boundingBox.y())
  {
    return false;
  }
  else if (node.z() > maxZ_ - 0.5 * params_.boundingBox.z())
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool dsvplanner_ns::Drrt::inGlobalBoundary(StateVec node)
{
  if (boundaryLoaded_)
  {
    geometry_msgs::Point node_point;
    node_point.x = node.x();
    node_point.y = node.y();
    node_point.z = node.z();
    if (!misc_utils_ns::PointInPolygon(node_point, boundary_polygon_))
    {
      return false;
    }
  }
  else
  {
    if (node.x() < params_.kMinXGlobalBound + 0.5 * params_.boundingBox.x())
    {
      return false;
    }
    else if (node.y() < params_.kMinYGlobalBound + 0.5 * params_.boundingBox.y())
    {
      return false;
    }
    else if (node.x() > params_.kMaxXGlobalBound - 0.5 * params_.boundingBox.x())
    {
      return false;
    }
    else if (node.y() > params_.kMaxYGlobalBound - 0.5 * params_.boundingBox.y())
    {
      return false;
    }
  }
  if (node.z() > params_.kMaxZGlobalBound - 0.5 * params_.boundingBox.z())
  {
    return false;
  }
  else if (node.z() < params_.kMinZGlobalBound + 0.5 * params_.boundingBox.z())
  {
    return false;
  }
  else
  {
    return true;
  }
}

// 功能：在边界附近生成一个比较好观测的点，送入到形参newNode中去
// 做法：随机延伸方向（x和y两个方向），判断在边界附近基于这个方向延伸的点，能不能观察到边界，主要在inSensorRange函数中实现
bool dsvplanner_ns::Drrt::generateRrtNodeToLocalFrontier(StateVec& newNode)
{
  StateVec potentialNode;
  bool nodeFound = false;
  int count = 0;
  double radius = sqrt(SQ(params_.kGainRange) + SQ(params_.kGainRange));
  while (!nodeFound)
  {
    count++;
    if (count >= 300)
    {
      return false;
    }
    potentialNode[0] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
    potentialNode[1] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
    potentialNode[2] = 0;
    if ((SQ(potentialNode[0]) + SQ(potentialNode[1])) > pow(radius, 2.0))
      continue;

    if (!inSensorRange(potentialNode))
    {
      continue;
    }

    if (!inPlanningBoundary(potentialNode) || !inGlobalBoundary(potentialNode))
    {
      continue;
    }
    nodeFound = true;
    newNode[0] = potentialNode[0];
    newNode[1] = potentialNode[1];
    newNode[2] = potentialNode[2];
    return true;
  }
  return false;
}

//通过两次循环，找到距离全局边界最近的局部点
void dsvplanner_ns::Drrt::getNextNodeToClosestGlobalFrontier()
{
  StateVec p1, p2;
  pcl::PointXYZ p3;
  double length1, length2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr globalSelectedFrontier =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  nextNodeFound_ = false;
  //从大值开始倒着搜，获取顶点位置，即寻找距离当前位置最近的全局边界点
  for (int i = dual_state_graph_->local_graph_.vertices.size() - 1; i >= 0; i--)  // Search from end to the begining.
                                                                                  // Nodes with large index means they
                                                                                  // are closer to the current position
                                                                                  // of the robot
  {
    //离当前位置最近的图节点
    p1.x() = dual_state_graph_->local_graph_.vertices[i].location.x;
    p1.y() = dual_state_graph_->local_graph_.vertices[i].location.y;
    p1.z() = dual_state_graph_->local_graph_.vertices[i].location.z;
    //遍历全局边界
    for (int j = 0; j < dual_state_frontier_->global_frontier_->size(); j++)
    {
      p3 = dual_state_frontier_->global_frontier_->points[j];
      p2.x() = dual_state_frontier_->global_frontier_->points[j].x;
      p2.y() = dual_state_frontier_->global_frontier_->points[j].y;
      p2.z() = dual_state_frontier_->global_frontier_->points[j].z;
      length1 = sqrt(SQ(p1.x() - p2.x()) + SQ(p1.y() - p2.y()));
      //比较局部图的点距离边界的距离
      if (length1 > (this->manager_->getSensorMaxRange() + params_.kGainRange) ||
          fabs(p1.z() - p2.z()) > params_.kMaxExtensionAlongZ)  // Not only
                                                                // consider the
      // sensor range,
      // also take the
      // node's view range into consideration
      {
        continue;  // When the node is too far away from the frontier or the
                   // difference between the z
      }            // of this node and frontier is too large, then skip to next frontier.
      // No need to use FOV here.
      if (volumetric_mapping::OctomapManager::CellStatus::kOccupied == manager_->getVisibility(p1, p2, false))
      {
        continue;  // Only when there is no occupied voxels between the node
                   // and
                   // frontier, we consider
      }            // the node can potentially see this frontier
      else
      {
        //nextBesetNodeIdx_代表下一个全局边界点附近的局部点
        NextBestNodeIdx_ = i;
        //nextnodefound_代表找到了下一个全局边界点附近的局部点
        nextNodeFound_ = true;
        //将得到的全局边界点记录下来
        globalSelectedFrontier->points.push_back(p3);
        //选择得到的全局边界点
        selectedGlobalFrontier_ = p3;
        break;
      }
    }
    if (nextNodeFound_)
      break;
  }
  //这个比较迷惑，反正上面的循环是先找到距离当前位置最近的边界点，但是辅助寻找该边界点对应的局部点并未得到采用，而通过下方代码继续确定局部点
  //然后根据这个边界点，再找到距离这个边界点最近的local节点
  //即论文图3中的A、B点
  if (nextNodeFound_)
  {
    for (int i = dual_state_graph_->local_graph_.vertices.size() - 1; i >= 0; i--)
    {
      p1.x() = dual_state_graph_->local_graph_.vertices[i].location.x;
      p1.y() = dual_state_graph_->local_graph_.vertices[i].location.y;
      p1.z() = dual_state_graph_->local_graph_.vertices[i].location.z;
      length2 = sqrt(SQ(p1.x() - p2.x()) + SQ(p1.y() - p2.y()));
      if (length2 > length1 || fabs(p1.z() - p2.z()) > params_.kMaxExtensionAlongZ)
      {
        continue;
      }
      if (volumetric_mapping::OctomapManager::CellStatus::kOccupied ==
          manager_->getLineStatusBoundingBox(p1, p2, params_.boundingBox))
      {
        continue;
      }
      length1 = length2;
      NextBestNodeIdx_ = i;
      nextNodeFound_ = true;
      p3.x = p1.x();
      p3.y = p1.y();
      p3.z = p1.z();
    }
    globalSelectedFrontier->points.push_back(p3);
  }
  sensor_msgs::PointCloud2 globalFrontier;
  pcl::toROSMsg(*globalSelectedFrontier, globalFrontier);
  globalFrontier.header.frame_id = params_.explorationFrame;
  params_.globalSelectedFrontierPub_.publish(globalFrontier);  // publish the next goal node and corresponing frontier
}

//获取当前探索方向最近的三个边界点
void dsvplanner_ns::Drrt::getThreeLocalFrontierPoint()  // Three local frontiers
                                                        // that are most close to
                                                        // the last
                                                        // exploration direciton
{                                                       // will be selected.
  StateVec exploreDirection, frontierDirection;
  double firstDirection = 180, secondDirection = 180, thirdDirection = 180;
  pcl::PointXYZ p1, p2, p3;  // three points to save frontiers.
  exploreDirection = dual_state_graph_->getExploreDirection();
  int localFrontierSize = dual_state_frontier_->local_frontier_->size();
  for (int i = 0; i < localFrontierSize; i++)
  {
    frontierDirection[0] =
        dual_state_frontier_->local_frontier_->points[i].x - root_[0];  // For ground robot, we only consider the
                                                                        // direction along x-y plane
    frontierDirection[1] = dual_state_frontier_->local_frontier_->points[i].y - root_[1];
    double theta = angleDiff(frontierDirection, exploreDirection);
    if (theta < firstDirection)
    {
      thirdDirection = secondDirection;
      secondDirection = firstDirection;
      firstDirection = theta;
      frontier3_direction_ = frontier2_direction_;
      frontier2_direction_ = frontier1_direction_;
      frontier1_direction_ = frontierDirection;
      p3 = p2;
      p2 = p1;
      p1 = dual_state_frontier_->local_frontier_->points[i];
    }
    else if (theta < secondDirection)
    {
      thirdDirection = secondDirection;
      secondDirection = theta;
      frontier3_direction_ = frontier2_direction_;
      frontier2_direction_ = frontierDirection;
      p3 = p2;
      p2 = dual_state_frontier_->local_frontier_->points[i];
    }
    else if (theta < thirdDirection)
    {
      thirdDirection = theta;
      frontier3_direction_ = frontierDirection;
      p3 = dual_state_frontier_->local_frontier_->points[i];
    }
  }

  localThreeFrontier_->clear();
  localThreeFrontier_->points.push_back(p1);
  localThreeFrontier_->points.push_back(p2);
  localThreeFrontier_->points.push_back(p3);
  sensor_msgs::PointCloud2 localThreeFrontier;
  pcl::toROSMsg(*localThreeFrontier_, localThreeFrontier);
  localThreeFrontier.header.frame_id = params_.explorationFrame;
  params_.localSelectedFrontierPub_.publish(localThreeFrontier);
}

//当前是否存在局部边界点
bool dsvplanner_ns::Drrt::remainingLocalFrontier() 
{
  int localFrontierSize = dual_state_frontier_->local_frontier_->points.size();
  if (localFrontierSize > 0)
    return true;
  return false;
}



void dsvplanner_ns::Drrt::floodFill(double posX, double posY, double posZ)
{
  double dx[4] = {0.6, 0, -0.6, 0};
  double dy[4] = {0, 0.6, 0, -0.6};
  StateVec pos;
  pos[0] = posX;
  pos[1] = posY;
  pos[2] = posZ;
  for(int i=0;i<4;i++)
  {
    pcl::PointXYZI sample;
    StateVec nearPos;
    nearPos[0] = posX + dx[i];
    nearPos[1] = posY + dy[i];
    nearPos[2] = getZvalue(nearPos[0], nearPos[1]);
    //std::cout<<"sample " << nearPos[0] << ", " << nearPos[1] << ", " << nearPos[2] <<std::endl;
    //是否是不可通过点
    if(nearPos[2]>=1000){
     //std::cout<<"z>1000." << std::endl;
     continue;
    }

    //是否在局部区域内
    if(!inPlanningBoundary(nearPos)){
      //std::cout<<"not in planning boundary" << std::endl;
      continue;
    }

    //是否是已采样点
    kdres* nearest = kd_nearest3(kdTree_, nearPos.x(), nearPos.y(), nearPos.z());
    if (kd_res_size(nearest) <= 0)
    {
      kd_res_free(nearest);
      continue;
    }
    dsvplanner_ns::Node* nearNode = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
    kd_res_free(nearest);
    StateVec direction(nearNode->state_[0] - nearPos[0], nearNode->state_[1] - nearPos[1], 0);
    if (direction.norm() < 0.2)
    {
      continue;
    }
    
    if (volumetric_mapping::OctomapManager::CellStatus::kFree == manager_->getLineStatusBoundingBox(pos, nearPos, params_.boundingBox) 
        ) //&& (!grid_->collisionCheckByTerrainWithVector(pos, nearPos))
    {
      //std::cout<<"use sample " << std::endl;
      //把新选好的采样点加入到sampledPoint_中
      sample.x = nearPos[0];
      sample.y = nearPos[1];
      sample.z = nearPos[2];
      sampledPoint_->points.push_back(sample);

      //kd树中找到距离采样点最近的图节点
      kdres* nearest = kd_nearest3(kdTree_, nearPos.x(), nearPos.y(), nearPos.z());
      if (kd_res_size(nearest) <= 0)
      {
        kd_res_free(nearest);
        continue;
      }
      dsvplanner_ns::Node* newParent = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
      kd_res_free(nearest);

      //把新的采样点插入到kd树中
      dsvplanner_ns::Node* newNode = new dsvplanner_ns::Node;
      newNode->state_ = nearPos;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + params_.kExtensionRange;
      newParent->children_.push_back(newNode);
      newNode->gain_ = gain(newNode->state_); //计算采样点的增益
      kd_insert3(kdTree_, nearPos.x(), nearPos.y(), nearPos.z(), newNode);

      //把新的采样点加入局部视点图中
      geometry_msgs::Pose p1;
      p1.position.x = nearPos.x();
      p1.position.y = nearPos.y();
      p1.position.z = nearPos.z();
      p1.orientation.y = newNode->gain_;
      dual_state_graph_->addNewLocalVertexWithoutDuplicates(p1, dual_state_graph_->local_graph_);
      dual_state_graph_->execute();

      // Display new node
      //把新的采样点加入node_array中，用于可视化
      node_array.push_back(newNode);
      // Update best IG and node if applicable
      if (newNode->gain_ > bestGain_)
      {
        bestGain_ = newNode->gain_;
      }
      nodeCounter_++;
      floodFill(nearPos[0],nearPos[1],nearPos[2]);
    }
  }
}

void dsvplanner_ns::Drrt::plannerIterateNew()
{
  // In this function a new configuration is sampled and added to the tree.
  StateVec newState;
  bool generateNodeArroundFrontierSuccess = false;

  double radius = 0.5 * sqrt(SQ(minX_ - maxX_) + SQ(minY_ - maxY_));
  bool candidateFound = false;
  int count = 0;
  while (!candidateFound)
  {
    count++;
    if (count > 1000)
      return;  // Plan fail if cannot find a required node in 1000 iterations

    newState[0] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);  //-radius ~ radius
    newState[1] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);  //-radius ~ radius
    newState[2] = 0;
    if (SQ(newState[0]) + SQ(newState[1]) > pow(radius, 2.0))
      continue;
    newState[0] += root_[0];
    newState[1] += root_[1];
    newState[2] += root_[2];
    if ((!inPlanningBoundary(newState)) || (!inGlobalBoundary(newState)))
    {   
      continue;
    }

    candidateFound = true;
  }

  //把新选好的采样点加入到sampledPoint_中
  pcl::PointXYZI sampledPoint;
  sampledPoint.x = newState[0];
  sampledPoint.y = newState[1];
  sampledPoint.z = newState[2];
  sampledPoint_->points.push_back(sampledPoint);

  //kd树中找到距离采样点最近的图节点
  // Find nearest neighbour
  kdres* nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return;
  }
  dsvplanner_ns::Node* newParent = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
  kd_res_free(nearest);

  // Check for collision of new connection.
  StateVec origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
  StateVec direction(newState[0] - origin[0], newState[1] - origin[1], 0);
  if (direction.norm() < params_.kMinextensionRange)
  {
    return;
  }
  else if (direction.norm() > params_.kExtensionRange)
  {
    //沿着kd树中最近点->采样点的这个方向，得到一个受限距离为kExtensionRange的向量
    direction = params_.kExtensionRange * direction.normalized();
  }
  //得到一个修正延伸后的新点endpoint
  StateVec endPoint = origin + direction;
  newState[0] = endPoint[0];
  newState[1] = endPoint[1];

  double x_position = newState[0] - root_[0];
  double y_position = newState[1] - root_[1];
  newState[2] = getZvalue(x_position, y_position);

  if (newState[2] >= 1000)  // the sampled position is above the untraversed area
  {
    return;
  }
  // Check if the new node is too close to any existing nodes after extension
  kdres* nearest_node = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest_node) <= 0)
  {
    kd_res_free(nearest_node);
    return;
  }
  dsvplanner_ns::Node* nearestNode = (dsvplanner_ns::Node*)kd_res_item_data(nearest_node);
  kd_res_free(nearest_node);

  origin[0] = newParent->state_[0];
  origin[1] = newParent->state_[1];
  origin[2] = newParent->state_[2];
  direction[0] = newState[0] - newParent->state_[0];
  direction[1] = newState[1] - newParent->state_[1];
  direction[2] = newState[2] - newParent->state_[2];
  if (direction.norm() < params_.kMinextensionRange || direction[2] > params_.kMaxExtensionAlongZ)
  {
    return;
  }
  // check collision if the new node is in the planning boundary
  if (!inPlanningBoundary(newState))
  {
    return;
  }
  else
  {
    if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
            manager_->getLineStatusBoundingBox(origin, newState, params_.boundingBox) &&
        (!grid_->collisionCheckByTerrainWithVector(origin, newState)))
    {  // connection is free
      // Create new node and insert into tree

      //把新的采样点插入到kd树中
      dsvplanner_ns::Node* newNode = new dsvplanner_ns::Node;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      newNode->gain_ = gain(newNode->state_); //计算采样点的增益

      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      //把新的采样点加入局部视点图中
      geometry_msgs::Pose p1;
      p1.position.x = newState.x();
      p1.position.y = newState.y();
      p1.position.z = newState.z();
      p1.orientation.y = newNode->gain_;
      dual_state_graph_->addNewLocalVertexWithoutDuplicates(p1, dual_state_graph_->local_graph_);
      dual_state_graph_->execute();

      // Display new node
      //把新的采样点加入node_array中，用于可视化
      node_array.push_back(newNode);
      // Update best IG and node if applicable
      if (newNode->gain_ > bestGain_)
      {
        bestGain_ = newNode->gain_;
      }
      nodeCounter_++;
    }
  }  
}

//采样新的点（靠近边界，或者机器人附近）,加入到RRT树中以及局部视点图中
void dsvplanner_ns::Drrt::plannerIterate()
{
  // In this function a new configuration is sampled and added to the tree.
  StateVec newState;
  bool generateNodeArroundFrontierSuccess = false;

  double radius = 0.5 * sqrt(SQ(minX_ - maxX_) + SQ(minY_ - maxY_));
  bool candidateFound = false;
  int count = 0;
  while (!candidateFound)
  {
    count++;
    if (count > 1000)
      return;  // Plan fail if cannot find a required node in 1000 iterations

    //采样一个随机数，0.25的概率
    if (((double)rand()) / ((double)RAND_MAX) > 0.75 && localThreeFrontier_->size() > 0)
    {
      if (local_plan_ == true)
      {
        //局部规划阶段，在边界附近生成一个比较好观测的点，存入形参newState中去
        generateNodeArroundFrontierSuccess = generateRrtNodeToLocalFrontier(newState);
      }
      if (!generateNodeArroundFrontierSuccess)
      {  // Generate node near local
         // frontier fail
        newState[0] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
        newState[1] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
        newState[2] = 0;  // Do not consider z value because ground robot cannot
                          // move along z
        if (SQ(newState[0]) + SQ(newState[1]) > pow(radius, 2.0))
          continue;
        //既然当前窗口没有合适观测边界的点，那就胡乱走吧，把方向加到机器人自身位置处
        newState[0] += root_[0];
        newState[1] += root_[1];
        newState[2] += root_[2];
        if ((!inPlanningBoundary(newState)) || (!inGlobalBoundary(newState)))
        {
          continue;
        }
      }
    }
    else //有0.75的概率在机器人周围随便采样一个点
    {
      newState[0] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
      newState[1] = 2.0 * radius * (((double)rand()) / ((double)RAND_MAX) - 0.5);
      newState[2] = 0;
      if (SQ(newState[0]) + SQ(newState[1]) > pow(radius, 2.0))
        continue;
      newState[0] += root_[0];
      newState[1] += root_[1];
      newState[2] += root_[2];
      if ((!inPlanningBoundary(newState)) || (!inGlobalBoundary(newState)))
      {
        continue;
      }
    }
    candidateFound = true;
  }

  //把新选好的采样点加入到sampledPoint_中
  pcl::PointXYZI sampledPoint;
  sampledPoint.x = newState[0];
  sampledPoint.y = newState[1];
  sampledPoint.z = newState[2];
  sampledPoint_->points.push_back(sampledPoint);

  //kd树中找到距离采样点最近的图节点
  // Find nearest neighbour
  kdres* nearest = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest) <= 0)
  {
    kd_res_free(nearest);
    return;
  }
  dsvplanner_ns::Node* newParent = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
  kd_res_free(nearest);

  // Check for collision of new connection.
  StateVec origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
  StateVec direction(newState[0] - origin[0], newState[1] - origin[1], 0);
  if (direction.norm() < params_.kMinextensionRange)
  {
    return;
  }
  else if (direction.norm() > params_.kExtensionRange)
  {
    //沿着kd树中最近点->采样点的这个方向，得到一个受限距离为kExtensionRange的向量
    direction = params_.kExtensionRange * direction.normalized();
  }
  //得到一个修正延伸后的新点endpoint
  StateVec endPoint = origin + direction;
  newState[0] = endPoint[0];
  newState[1] = endPoint[1];

  double x_position = newState[0] - root_[0];
  double y_position = newState[1] - root_[1];
  newState[2] = getZvalue(x_position, y_position);

  if (newState[2] >= 1000)  // the sampled position is above the untraversed area
  {
    return;
  }
  // Check if the new node is too close to any existing nodes after extension
  kdres* nearest_node = kd_nearest3(kdTree_, newState.x(), newState.y(), newState.z());
  if (kd_res_size(nearest_node) <= 0)
  {
    kd_res_free(nearest_node);
    return;
  }
  dsvplanner_ns::Node* nearestNode = (dsvplanner_ns::Node*)kd_res_item_data(nearest_node);
  kd_res_free(nearest_node);

  origin[0] = newParent->state_[0];
  origin[1] = newParent->state_[1];
  origin[2] = newParent->state_[2];
  direction[0] = newState[0] - newParent->state_[0];
  direction[1] = newState[1] - newParent->state_[1];
  direction[2] = newState[2] - newParent->state_[2];
  if (direction.norm() < params_.kMinextensionRange || direction[2] > params_.kMaxExtensionAlongZ)
  {
    return;
  }
  // check collision if the new node is in the planning boundary
  if (!inPlanningBoundary(newState))
  {
    return;
  }
  else
  {
    if (volumetric_mapping::OctomapManager::CellStatus::kFree ==
            manager_->getLineStatusBoundingBox(origin, newState, params_.boundingBox) &&
        (!grid_->collisionCheckByTerrainWithVector(origin, newState)))
    {  // connection is free
      // Create new node and insert into tree

      //把新的采样点插入到kd树中
      dsvplanner_ns::Node* newNode = new dsvplanner_ns::Node;
      newNode->state_ = newState;
      newNode->parent_ = newParent;
      newNode->distance_ = newParent->distance_ + direction.norm();
      newParent->children_.push_back(newNode);
      newNode->gain_ = gain(newNode->state_); //计算采样点的增益

      kd_insert3(kdTree_, newState.x(), newState.y(), newState.z(), newNode);

      //把新的采样点加入局部视点图中
      geometry_msgs::Pose p1;
      p1.position.x = newState.x();
      p1.position.y = newState.y();
      p1.position.z = newState.z();
      p1.orientation.y = newNode->gain_;
      dual_state_graph_->addNewLocalVertexWithoutDuplicates(p1, dual_state_graph_->local_graph_);
      dual_state_graph_->execute();

      // Display new node
      //把新的采样点加入node_array中，用于可视化
      node_array.push_back(newNode);
      // Update best IG and node if applicable
      if (newNode->gain_ > bestGain_)
      {
        bestGain_ = newNode->gain_;
      }
      nodeCounter_++;
    }
  }
}

//在重定位阶段，被调用
void dsvplanner_ns::Drrt::updateCandidateGoals()
{
  //判断原全局边界点，是否仍是边界点
  const double disc = manager_->getResolution();
  const double searchRadius = params_.kGainRange * 2;
  std::vector<StateVec> remove_list;
  for(int i=0;i<dual_state_graph_->candidate_goals_.size();i++)
  {
    StateVec state = dual_state_graph_->candidate_goals_[i];
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
    if(gain < params_.kMinEffectiveGain)
    {
      remove_list.push_back(state);
    }
  }
  for(int i=0;i<remove_list.size();i++)
  {
    dual_state_graph_->removeCandidateGoalPoint(remove_list[i]);
  }
}

void dsvplanner_ns::Drrt::plannerInitNew()
{
  // This function is to initialize the tree
  //创建kdTree
  kdTree_ = kd_create(3);

  node_array.clear();
  rootNode_ = new Node;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.kZeroGain;
  rootNode_->parent_ = NULL;

  global_vertex_size_ = 0;
  geometry_msgs::Pose p1;
  if (global_plan_ == false) //如果是局部探索
  {
    std::cout << "Exploration Stage" << std::endl;
    //根结点被设置为机器人的初始位置
    rootNode_->state_ = root_;
    kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);
    iterationCount_++;

    //如果目前还存在local frontier clusters
    if (first_call_ || dual_state_graph_->cluster_centroids_.size()>0)
    {
      if(first_call_)
        first_call_ = false;
      //std::cout << "    Still has local frontier clusters." << std::endl;

      localPlanOnceMore_ = true;
      loopCount_ = params_.kLoopCountThres;
      
      //这个normal_local_iteration_出现三次
      //如果当前处于探索阶段，还有边界点，它就是true
      //如果处于探索阶段并且没有边界点，系统也不打算等地图更新了，它也是true
      //只有没有边界，但是系统想等待一下地图边界更新的时候，它才是false
      normal_local_iteration_ = true;
      keepTryingNum_ = params_.kKeepTryingNum;  // Try 1 or 2 more times even if there
                                                // is no local frontier
      remainingFrontier_ = true;
      
      //we do not need this
      //getThreeLocalFrontierPoint();

      //传入机器人当前位置root_，得到一个pruned图dual_state_graph_->pruned_graph_，并恢复得到kdTree_
      pruneTree(root_);
      //将pruned图赋值给local图
      dual_state_graph_->clearLocalGraph();
      dual_state_graph_->local_graph_ = dual_state_graph_->pruned_graph_;
      
      //发布局部图
      dual_state_graph_->execute();
    }
    else //局部探索时,当聚类数为0时,会执行下面的代码。
    {
      //暂时没边界点，不打算再尝试了
      if (!localPlanOnceMore_)
      {
        //std::cout << "    Has no local frontier clusters, stop trying." << std::endl;
        dual_state_graph_->clearLocalGraph();
        dual_state_graph_->pruned_graph_.vertices.clear();
        remainingFrontier_ = false;
        localPlanOnceMore_ = true;
        normal_local_iteration_ = true;
      }
      //暂时没边界点，再尝试一下
      else
      {
        //std::cout << "    Has no local frontier clusters, keep trying." << std::endl;
        remainingFrontier_ = true;
        loopCount_ = params_.kLoopCountThres * 3;
        normal_local_iteration_ = false;
        //localThreeFrontier_->clear(); //do not need this
        //        pruneTree(root_);
        dual_state_graph_->clearLocalGraph();
        dual_state_graph_->pruned_graph_.vertices.clear();
        //        dual_state_graph_->local_graph_ =
        //        dual_state_graph_->pruned_graph_;
        dual_state_graph_->execute();
        keepTryingNum_--;
        if (keepTryingNum_ <= 0)
        {
          localPlanOnceMore_ = false;
          keepTryingNum_ = params_.kKeepTryingNum + 1;
        }
      }
    }

    //更新局部探索边界框
    maxX_ = rootNode_->state_.x() + params_.kMaxXLocalBound;
    maxY_ = rootNode_->state_.y() + params_.kMaxYLocalBound;
    maxZ_ = rootNode_->state_.z() + params_.kMaxZLocalBound;
    minX_ = rootNode_->state_.x() + params_.kMinXLocalBound;
    minY_ = rootNode_->state_.y() + params_.kMinYLocalBound;
    minZ_ = rootNode_->state_.z() + params_.kMinZLocalBound;
  }
  else //重定位阶段
  {
    std::cout << "Relocation Stage" << std::endl;
    localPlanOnceMore_ = true;
    StateVec node1;
    double gain1;
    if (dual_state_graph_->global_graph_.vertices.size() > 0)
    {
      //将全局图的0号节点加入到kd树中
      node1[0] = dual_state_graph_->global_graph_.vertices[0].location.x;
      node1[1] = dual_state_graph_->global_graph_.vertices[0].location.y;
      node1[2] = dual_state_graph_->global_graph_.vertices[0].location.z;
      rootNode_->state_ = node1;
      kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);

      //清除局部图
      dual_state_graph_->clearLocalGraph();
      //！！！这里比较重要，把全局图赋值给了局部图
      dual_state_graph_->local_graph_ = dual_state_graph_->global_graph_;
      dual_state_graph_->local_graph_.vertices[0].information_gain = rootNode_->gain_;
      dual_state_graph_->execute();

      //找到距离全局边界最近的局部点(其实这时候的局部图是全局图)
      //getNextNodeToClosestGlobalFrontier();

      //首先对所有的全局候选点进行更新，通过计算其周围增益点的个数，如果小于一定数量，则将该候选点忽略
      updateCandidateGoals();

      //改为:基于全局边界点簇来进行TSP规划得到下一个目标节点
      nextNodeFound_=false;
      Vector3d cur_pos;
      cur_pos[0] = root_[0];
      cur_pos[1] = root_[1];
      cur_pos[2] = root_[2];
      std::vector<StateVec> visits;
      dual_state_graph_->findGlobalTour(cur_pos, visits);
      if(visits.size()>0){
        geometry_msgs::Point next_goal_position;
        next_goal_position.x = visits[0][0];
        next_goal_position.y = visits[0][1];
        next_goal_position.z = visits[0][2];
        NextBestNodeIdx_ = graph_utils_ns::GetClosestVertexIdxToPoint(dual_state_graph_->local_graph_, next_goal_position);
        nextNodeFound_=true;
        //remove the selected goal from the candidate list
        dual_state_graph_->removeCandidateGoalPoint(StateVec(next_goal_position.x, next_goal_position.y, next_goal_position.z));
        dual_state_graph_->publishCandidateGoals();
      }
      visits.clear();

      if (nextNodeFound_)
      {
        dual_state_graph_->local_graph_.vertices[NextBestNodeIdx_].information_gain =
            300000;  // set a large enough value as the best gain
        bestGain_ = 300000;
        nodeCounter_ = dual_state_graph_->global_graph_.vertices.size();
        global_vertex_size_ = nodeCounter_;
        dual_state_graph_->publishGlobalGraph();
      }
      else //没有找到全局边界，根据全局图重建rrt（kdTree)
      {  // Rebuild the rrt accordingt to current graph and then extend
         // in
         // plannerIterate. This only happens when no
         // global frontiers can be seen. Mostly used at the end of the
         // exploration in case that there are some narrow
         // areas are ignored.
        for (int i = 1; i < dual_state_graph_->global_graph_.vertices.size(); i++)
        {
          p1.position = dual_state_graph_->global_graph_.vertices[i].location;
          node1[0] = p1.position.x;
          node1[1] = p1.position.y;
          node1[2] = p1.position.z;

          kdres* nearest = kd_nearest3(kdTree_, node1.x(), node1.y(), node1.z());
          if (kd_res_size(nearest) <= 0)
          {
            kd_res_free(nearest);
            continue;
          }
          dsvplanner_ns::Node* newParent = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
          kd_res_free(nearest);

          StateVec origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
          StateVec direction(node1[0] - origin[0], node1[1] - origin[1], node1[2] - origin[2]);
          if (direction.norm() > params_.kExtensionRange)
          {
            direction = params_.kExtensionRange * direction.normalized();
          }
          node1[0] = origin[0] + direction[0];
          node1[1] = origin[1] + direction[1];
          node1[2] = origin[2] + direction[2];
          global_vertex_size_++;
          // Create new node and insert into tree
          dsvplanner_ns::Node* newNode = new dsvplanner_ns::Node;
          newNode->state_ = node1;
          newNode->parent_ = newParent;
          newNode->distance_ = newParent->distance_ + direction.norm();
          newParent->children_.push_back(newNode);
          newNode->gain_ = gain(newNode->state_);

          kd_insert3(kdTree_, node1.x(), node1.y(), node1.z(), newNode);

          // save new node to node_array
          dual_state_graph_->local_graph_.vertices[i].information_gain = newNode->gain_;
          node_array.push_back(newNode);

          if (newNode->gain_ > bestGain_)
          {
            if (std::find(executedBestNodeList_.begin(), executedBestNodeList_.end(), i) != executedBestNodeList_.end())
            {
              bestGain_ = newNode->gain_;
              bestNodeId_ = i;
            }
          }
          // nodeCounter_++;
        }
        executedBestNodeList_.push_back(bestNodeId_);
        nodeCounter_ = dual_state_graph_->global_graph_.vertices.size();
      }
    }
    else //暂时没有全局图节点，把当前位置加入到kd树中
    {
      rootNode_->state_ = root_;
      kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);
      iterationCount_++;
    }
    //全局阶段，边界框使用的是整个地图的最大范围
    maxX_ = params_.kMaxXGlobalBound;
    maxY_ = params_.kMaxYGlobalBound;
    maxZ_ = params_.kMaxZGlobalBound;
    minX_ = params_.kMinXGlobalBound;
    minY_ = params_.kMinYGlobalBound;
    minZ_ = params_.kMinZGlobalBound;
  }
  //发布边界框
  publishPlanningHorizon();
}

//每次处理/drrtPlannerSrv请求时, 都进行初始化
void dsvplanner_ns::Drrt::plannerInit()
{
  // This function is to initialize the tree
  //创建kdTree
  kdTree_ = kd_create(3);

  node_array.clear();
  rootNode_ = new Node;
  rootNode_->distance_ = 0.0;
  rootNode_->gain_ = params_.kZeroGain;
  rootNode_->parent_ = NULL;

  global_vertex_size_ = 0;
  geometry_msgs::Pose p1;
  if (global_plan_ == false) //全局规划状态处于false时，那么当前是在局部探索阶段
  {
    std::cout << "Exploration Stage" << std::endl;
    //根结点被设置为机器人的初始位置
    rootNode_->state_ = root_;
    kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);
    iterationCount_++;

    //如果目前还存在局部边界点
    if (remainingLocalFrontier())
    {
      localPlanOnceMore_ = true;
      loopCount_ = params_.kLoopCountThres;
      
      //这个normal_local_iteration_出现三次
      //如果当前处于探索阶段，还有边界点，它就是true
      //如果处于探索阶段并且没有边界点，系统也不打算等地图更新了，它也是true
      //只有没有边界，但是系统想等待一下地图边界更新的时候，它才是false
      normal_local_iteration_ = true;
      keepTryingNum_ = params_.kKeepTryingNum;  // Try 1 or 2 more times even if there
                                                // is no local frontier
      remainingFrontier_ = true;
      //获取当前探索方向最近的三个边界点
      getThreeLocalFrontierPoint();
      //传入机器人当前位置root_，得到一个pruned图dual_state_graph_->pruned_graph_，并恢复得到kdTree_
      pruneTree(root_);
      //将pruned图赋值给local图
      dual_state_graph_->clearLocalGraph();
      dual_state_graph_->local_graph_ = dual_state_graph_->pruned_graph_;
      //发布局部图
      dual_state_graph_->execute();
    }
    else //目前不存在边界点的情况
    {
      //暂时没边界点，不打算再尝试了
      if (!localPlanOnceMore_)
      {
        dual_state_graph_->clearLocalGraph();
        dual_state_graph_->pruned_graph_.vertices.clear();
        remainingFrontier_ = false;
        localPlanOnceMore_ = true;
        normal_local_iteration_ = true;
      }
      //暂时没边界点，再尝试一下
      else
      {
        remainingFrontier_ = true;
        loopCount_ = params_.kLoopCountThres * 3;
        normal_local_iteration_ = false;
        localThreeFrontier_->clear();
        //        pruneTree(root_);
        dual_state_graph_->clearLocalGraph();
        dual_state_graph_->pruned_graph_.vertices.clear();
        //        dual_state_graph_->local_graph_ =
        //        dual_state_graph_->pruned_graph_;
        dual_state_graph_->execute();
        keepTryingNum_--;
        if (keepTryingNum_ <= 0)
        {
          localPlanOnceMore_ = false;
          keepTryingNum_ = params_.kKeepTryingNum + 1;  // After switching to relocation stage, give
          // another more chance in case that some frontiers
          // are not updated
        }
      }
    }

    //更新局部探索边界框
    maxX_ = rootNode_->state_.x() + params_.kMaxXLocalBound;
    maxY_ = rootNode_->state_.y() + params_.kMaxYLocalBound;
    maxZ_ = rootNode_->state_.z() + params_.kMaxZLocalBound;
    minX_ = rootNode_->state_.x() + params_.kMinXLocalBound;
    minY_ = rootNode_->state_.y() + params_.kMinYLocalBound;
    minZ_ = rootNode_->state_.z() + params_.kMinZLocalBound;
  }
  else //重定位阶段
  {
    std::cout << "Relocation Stage" << std::endl;
    localPlanOnceMore_ = true;
    StateVec node1;
    double gain1;
    if (dual_state_graph_->global_graph_.vertices.size() > 0)
    {
      //将全局图的0号节点加入到kd树中
      node1[0] = dual_state_graph_->global_graph_.vertices[0].location.x;
      node1[1] = dual_state_graph_->global_graph_.vertices[0].location.y;
      node1[2] = dual_state_graph_->global_graph_.vertices[0].location.z;
      rootNode_->state_ = node1;
      kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);

      //清除局部图
      dual_state_graph_->clearLocalGraph();
      //！！！这里比较重要，把全局图赋值给了局部图
      dual_state_graph_->local_graph_ = dual_state_graph_->global_graph_;
      dual_state_graph_->local_graph_.vertices[0].information_gain = rootNode_->gain_;
      dual_state_graph_->execute();
      //找到距离全局边界最近的局部点(其实这时候的局部图是全局图)
      getNextNodeToClosestGlobalFrontier();
      if (nextNodeFound_)
      {
        dual_state_graph_->local_graph_.vertices[NextBestNodeIdx_].information_gain =
            300000;  // set a large enough value as the best gain
        bestGain_ = 300000;
        nodeCounter_ = dual_state_graph_->global_graph_.vertices.size();
        global_vertex_size_ = nodeCounter_;
        dual_state_graph_->publishGlobalGraph();
      }
      else //没有找到全局边界，根据全局图重建rrt（kdTree)
      {  // Rebuild the rrt accordingt to current graph and then extend
         // in
         // plannerIterate. This only happens when no
         // global frontiers can be seen. Mostly used at the end of the
         // exploration in case that there are some narrow
         // areas are ignored.
        for (int i = 1; i < dual_state_graph_->global_graph_.vertices.size(); i++)
        {
          p1.position = dual_state_graph_->global_graph_.vertices[i].location;
          node1[0] = p1.position.x;
          node1[1] = p1.position.y;
          node1[2] = p1.position.z;

          kdres* nearest = kd_nearest3(kdTree_, node1.x(), node1.y(), node1.z());
          if (kd_res_size(nearest) <= 0)
          {
            kd_res_free(nearest);
            continue;
          }
          dsvplanner_ns::Node* newParent = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
          kd_res_free(nearest);

          StateVec origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
          StateVec direction(node1[0] - origin[0], node1[1] - origin[1], node1[2] - origin[2]);
          if (direction.norm() > params_.kExtensionRange)
          {
            direction = params_.kExtensionRange * direction.normalized();
          }
          node1[0] = origin[0] + direction[0];
          node1[1] = origin[1] + direction[1];
          node1[2] = origin[2] + direction[2];
          global_vertex_size_++;
          // Create new node and insert into tree
          dsvplanner_ns::Node* newNode = new dsvplanner_ns::Node;
          newNode->state_ = node1;
          newNode->parent_ = newParent;
          newNode->distance_ = newParent->distance_ + direction.norm();
          newParent->children_.push_back(newNode);
          newNode->gain_ = gain(newNode->state_);

          kd_insert3(kdTree_, node1.x(), node1.y(), node1.z(), newNode);

          // save new node to node_array
          dual_state_graph_->local_graph_.vertices[i].information_gain = newNode->gain_;
          node_array.push_back(newNode);

          if (newNode->gain_ > bestGain_)
          {
            if (std::find(executedBestNodeList_.begin(), executedBestNodeList_.end(), i) != executedBestNodeList_.end())
            {
              bestGain_ = newNode->gain_;
              bestNodeId_ = i;
            }
          }
          // nodeCounter_++;
        }
        executedBestNodeList_.push_back(bestNodeId_);
        nodeCounter_ = dual_state_graph_->global_graph_.vertices.size();
      }
    }
    else //暂时没有全局图节点，把当前位置加入到kd树中
    {
      rootNode_->state_ = root_;
      kd_insert3(kdTree_, rootNode_->state_.x(), rootNode_->state_.y(), rootNode_->state_.z(), rootNode_);
      iterationCount_++;
    }
    //全局阶段，边界框使用的是整个地图的最大范围
    maxX_ = params_.kMaxXGlobalBound;
    maxY_ = params_.kMaxYGlobalBound;
    maxZ_ = params_.kMaxZGlobalBound;
    minX_ = params_.kMinXGlobalBound;
    minY_ = params_.kMinYGlobalBound;
    minZ_ = params_.kMinZGlobalBound;
  }
  //发布边界框
  publishPlanningHorizon();
}

void dsvplanner_ns::Drrt::publishPlanningHorizon()
{  // Publish visualization of
   // current planning horizon
  visualization_msgs::Marker p;
  p.header.stamp = ros::Time::now();
  p.header.frame_id = params_.explorationFrame;
  p.id = 0;
  p.ns = "boundary";
  p.type = visualization_msgs::Marker::CUBE;
  p.action = visualization_msgs::Marker::ADD;
  p.pose.position.x = 0.5 * (minX_ + maxX_);
  p.pose.position.y = 0.5 * (minY_ + maxY_);
  p.pose.position.z = 0.5 * (minZ_ + maxZ_);
  tf::Quaternion quat;
  quat.setEuler(0.0, 0.0, 0.0);
  p.pose.orientation.x = quat.x();
  p.pose.orientation.y = quat.y();
  p.pose.orientation.z = quat.z();
  p.pose.orientation.w = quat.w();
  p.scale.x = maxX_ - minX_;
  p.scale.y = maxY_ - minY_;
  p.scale.z = maxZ_ - minZ_;
  p.color.r = 252.0 / 255.0;
  p.color.g = 145.0 / 255.0;
  p.color.b = 37.0 / 255.0;
  p.color.a = 0.3;
  p.lifetime = ros::Duration(0.0);
  p.frame_locked = false;
  params_.boundaryPub_.publish(p);
}

//传入一个位置root，得到一个pruned图，并恢复得到kdTree
void dsvplanner_ns::Drrt::pruneTree(StateVec root)
{
  //清空pruned图的节点
  dual_state_graph_->pruned_graph_.vertices.clear();
  geometry_msgs::Pose p1;
  p1.position.x = root[0];
  p1.position.y = root[1];
  p1.position.z = root[2];
  p1.orientation.y = params_.kZeroGain;
  //把节点root加入到图pruned_graph_中
  dual_state_graph_->addNewLocalVertexWithoutDuplicates(p1, dual_state_graph_->pruned_graph_);

  geometry_msgs::Point root_point;
  root_point.x = root[0];
  root_point.y = root[1];
  root_point.z = root[2];
  //根据机器人当前位置,在现有的局部视点图上做精减,得到修剪后的局部图
  dual_state_graph_->pruneGraph(root_point);

  //根据修剪后的局部图恢复kdTree
  StateVec node;
  for (int i = 1; i < dual_state_graph_->pruned_graph_.vertices.size(); i++)
  {
    node[0] = dual_state_graph_->pruned_graph_.vertices[i].location.x;
    node[1] = dual_state_graph_->pruned_graph_.vertices[i].location.y;
    node[2] = dual_state_graph_->pruned_graph_.vertices[i].location.z;

    //遍历prune图得到各个node,从kd树中找到距离node最近的点
    kdres* nearest = kd_nearest3(kdTree_, node.x(), node.y(), node.z());
    if (kd_res_size(nearest) <= 0)
    {
      //没找到则忽略该node
      kd_res_free(nearest);
      continue;
    }
    //取出这个节点,nearest中只装一个值
    dsvplanner_ns::Node* newParent = (dsvplanner_ns::Node*)kd_res_item_data(nearest);
    kd_res_free(nearest);

    // Check for collision
    //pruned图里每一个点，到它在kd树中查找得到的最近点的方向向量
    StateVec origin(newParent->state_[0], newParent->state_[1], newParent->state_[2]);
    StateVec direction(node[0] - origin[0], node[1] - origin[1], node[2] - origin[2]);

    //提取相关信息，打包成一个新的点
    dsvplanner_ns::Node* newNode = new dsvplanner_ns::Node;
    newNode->state_ = node;
    newNode->parent_ = newParent;
    newNode->distance_ = newParent->distance_ + direction.norm();
    newParent->children_.push_back(newNode);
    if (dual_state_graph_->pruned_graph_.vertices[i].information_gain > 0)
      newNode->gain_ = gain(newNode->state_); //局部图中该节点有增益时,则要更新增益,因为该节点可能从未知区域变为已探索区域
    else
    {
      newNode->gain_ = 0;
    }
    //把这个提取到的点插入到kd树中
    kd_insert3(kdTree_, node.x(), node.y(), node.z(), newNode);
    node_array.push_back(newNode);

    if (newNode->gain_ > bestGain_)
    {
      bestGain_ = newNode->gain_;
    }
    dual_state_graph_->pruned_graph_.vertices[i].information_gain = newNode->gain_;
  }
  remainingNodeCount_ = node_array.size();
}

double dsvplanner_ns::Drrt::gain(StateVec state)
{
  // This function computes the gain
  double gain = 0.0;
  const double disc = manager_->getResolution();
  StateVec origin(state[0], state[1], state[2]);
  StateVec vec;
  double rangeSq = pow(params_.kGainRange, 2.0);

  // Iterate over all nodes within the allowed distance
  for (vec[0] = std::max(state[0] - params_.kGainRange, minX_); vec[0] < std::min(state[0] + params_.kGainRange, maxX_);
       vec[0] += disc)
  {
    for (vec[1] = std::max(state[1] - params_.kGainRange, minY_);
         vec[1] < std::min(state[1] + params_.kGainRange, maxY_); vec[1] += disc)
    {
      for (vec[2] = std::max(state[2] - params_.kGainRangeZMinus, minZ_);
           vec[2] < std::min(state[2] + params_.kGainRangeZPlus, maxZ_); vec[2] += disc)
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

  // Scale with volume
  if (gain < params_.kMinEffectiveGain)
    gain = 0;
  gain *= pow(disc, 3.0);

  return gain;
}

void dsvplanner_ns::Drrt::clear()
{
  delete rootNode_;
  rootNode_ = NULL;

  nodeCounter_ = 0;
  bestGain_ = params_.kZeroGain;
  bestNode_ = NULL;
  if (nextNodeFound_)
  {
    dual_state_graph_->clearLocalGraph();
  }
  nextNodeFound_ = false;
  remainingFrontier_ = false;
  remainingNodeCount_ = 0;

  sampledPoint_->points.clear();

  kd_free(kdTree_);
}


void dsvplanner_ns::Drrt::publishNode()
{
  sensor_msgs::PointCloud2 random_sampled_points_pc;
  pcl::toROSMsg(*sampledPoint_, random_sampled_points_pc);
  random_sampled_points_pc.header.frame_id = params_.explorationFrame;
  params_.randomSampledPointsPub_.publish(random_sampled_points_pc);

  visualization_msgs::Marker node;
  visualization_msgs::Marker branch;
  node.header.stamp = ros::Time::now();
  node.header.frame_id = params_.explorationFrame;
  node.ns = "drrt_node";
  node.type = visualization_msgs::Marker::POINTS;
  node.action = visualization_msgs::Marker::ADD;
  node.scale.x = params_.kRemainingNodeScaleSize;
  node.color.r = 167.0 / 255.0;
  node.color.g = 167.0 / 255.0;
  node.color.b = 0.0;
  node.color.a = 1.0;
  node.frame_locked = false;

  branch.ns = "drrt_branches";
  branch.header.stamp = ros::Time::now();
  branch.header.frame_id = params_.explorationFrame;
  branch.type = visualization_msgs::Marker::LINE_LIST;
  branch.action = visualization_msgs::Marker::ADD;
  branch.scale.x = params_.kRemainingBranchScaleSize;
  branch.color.r = 167.0 / 255.0;
  branch.color.g = 167.0 / 255.0;
  branch.color.b = 0.0;
  branch.color.a = 1.0;
  branch.frame_locked = false;

  geometry_msgs::Point node_position;
  geometry_msgs::Point parent_position;
  //这里需要关注一下remainingNodeCount_和node_array的区别在哪里？
  //每次往kd树里面加入值，同时就会加入到这个node_array数据结构中
  //而remainingNodeCount_只是在函数pruneTree中被按node_array的大小赋值了一次
  if (remainingNodeCount_ > 0 && remainingNodeCount_ <= node_array.size())
  {
    for (int i = 0; i < remainingNodeCount_; i++)
    {
      node_position.x = node_array[i]->state_[0];
      node_position.y = node_array[i]->state_[1];
      node_position.z = node_array[i]->state_[2];
      node.points.push_back(node_position);

      if (node_array[i]->parent_)
      {
        parent_position.x = node_array[i]->parent_->state_[0];
        parent_position.y = node_array[i]->parent_->state_[1];
        parent_position.z = node_array[i]->parent_->state_[2];

        branch.points.push_back(parent_position);
        branch.points.push_back(node_position);
      }
    }
    params_.remainingTreePathPub_.publish(node);
    params_.remainingTreePathPub_.publish(branch);
    node.points.clear();
    branch.points.clear();
    node.scale.x = params_.kNewNodeScaleSize;
    node.color.r = 167.0 / 255.0;
    node.color.g = 0.0 / 255.0;
    node.color.b = 167.0 / 255.0;
    node.color.a = 1.0;
    branch.scale.x = params_.kNewBranchScaleSize;
    branch.color.r = 167.0 / 255.0;
    branch.color.g = 0.0 / 255.0;
    branch.color.b = 167.0 / 255.0;
    branch.color.a = 1.0;
    for (int i = remainingNodeCount_; i < node_array.size(); i++)
    {
      node_position.x = node_array[i]->state_[0];
      node_position.y = node_array[i]->state_[1];
      node_position.z = node_array[i]->state_[2];
      node.points.push_back(node_position);

      if (node_array[i]->parent_)
      {
        parent_position.x = node_array[i]->parent_->state_[0];
        parent_position.y = node_array[i]->parent_->state_[1];
        parent_position.z = node_array[i]->parent_->state_[2];

        branch.points.push_back(parent_position);
        branch.points.push_back(node_position);
      }
    }
    params_.newTreePathPub_.publish(node);
    params_.newTreePathPub_.publish(branch);
  }
  else
  {
    for (int i = 0; i < node_array.size(); i++)
    {
      node_position.x = node_array[i]->state_[0];
      node_position.y = node_array[i]->state_[1];
      node_position.z = node_array[i]->state_[2];
      node.points.push_back(node_position);

      if (node_array[i]->parent_)
      {
        parent_position.x = node_array[i]->parent_->state_[0];
        parent_position.y = node_array[i]->parent_->state_[1];
        parent_position.z = node_array[i]->parent_->state_[2];

        branch.points.push_back(parent_position);
        branch.points.push_back(node_position);
      }
    }
    params_.newTreePathPub_.publish(node);
    params_.newTreePathPub_.publish(branch);

    // When there is no remaining node, publish an empty one
    node.points.clear();
    branch.points.clear();
    params_.remainingTreePathPub_.publish(node);
    params_.remainingTreePathPub_.publish(branch);
  }
}

void dsvplanner_ns::Drrt::gotoxy(int x, int y)
{
  printf("%c[%d;%df", 0x1B, y, x);
}

#endif
