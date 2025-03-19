/**
 * *********************************************************
 *
 * @file: jump_point_search.cpp
 * @brief: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @date: 2023-12-14
 * @version: 1.1
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <path_searching/jump_point_search.h>
using Eigen::Vector2i;
using Eigen::Vector2d;
template<typename T> 
Eigen::Matrix<T, 2, 1> Vec3ToVec2(const Eigen::Matrix<T, 3, 1>& vec3) 
{
  return vec3.head(2); 
}

template<typename T> 
Eigen::Matrix<T, 3, 1> Vec2ToVec3(const Eigen::Matrix<T, 2, 1>& vec2) 
{
  return Eigen::Matrix<T, 3, 1>(vec2(0), vec2(1), 0); 
}

// 这些全都应该写到jps类里面去
// Bresenham算法实现
std::vector<Eigen::Vector2i> bresenham(const Eigen::Vector2i& start, const Eigen::Vector2i& end) {
    std::vector<Eigen::Vector2i> points;
    int x0 = start.x(), y0 = start.y();
    int x1 = end.x(), y1 = end.y();
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    while (true) {
        points.emplace_back(x0, y0); // 将当前点加入结果

        if (x0 == x1 && y0 == y1) break; // 终止条件

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
    return points;
}

// 处理一组点，返回连接所有点的连线上的所有点，避免重复点
std::vector<Eigen::Vector2i> connectPoints(const std::vector<Eigen::Vector2i>& points) {
    std::vector<Eigen::Vector2i> result;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        std::vector<Eigen::Vector2i> linePoints = bresenham(points[i], points[i + 1]);

        if (i > 0 && !result.empty()) {
            // 跳过 linePoints 的第一个点，避免重复
            result.insert(result.end(), linePoints.begin() + 1, linePoints.end());
        } else {
            // 对于第一段线，包含所有点
            result.insert(result.end(), linePoints.begin(), linePoints.end());
        }
    }
    return result;
}

std::vector<double> computePathDistances(const std::vector<Eigen::Vector3d>& path) {
    std::vector<double> distances(path.size(), 0.0);
    // 从倒数第二个点开始，往前累加路径距离
    for (int i = path.size() - 2; i >= 0; --i) {
        // 计算当前点和下一个点之间的欧氏距离
        double segmentDistance = (path[i + 1] - path[i]).norm();
        
        // 当前点到终点的距离 = 下一个点到终点的距离 + 当前段的距离
        distances[i] = distances[i + 1] + segmentDistance;
    }
    return distances;
}

namespace fast_planner
{

int JumpPointSearch::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic, double time_start)
{
  NodePtr cur_node = path_node_pool_[0];
  z_ = start_pt.z();
  cur_node->parent = NULL;
  cur_node->position = start_pt;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0; 
  cur_node->f_score = lambda_heu_ * getEuclHeu(cur_node->position, end_pt);
  cur_node->node_state = IN_OPEN_SET;

  end_pt(2) = start_pt(2);
  end_index_ = posToIndex(end_pt);
  open_set_.push(cur_node);
  use_node_num_ += 1;
  expanded_nodes_.insert(cur_node->index, cur_node);
  NodePtr neighbor = NULL;
  NodePtr terminate_node = NULL;


  // main loop
  while (!open_set_.empty())
  {
    // pop current node from open list
    cur_node = open_set_.top();
    /* ---------- determine termination ---------- */
    bool reach_end = abs(cur_node->index(0) - end_index_(0)) <= 1 &&
        abs(cur_node->index(1) - end_index_(1)) <= 1;
    if (reach_end) {
      // cout << "[Astar]:---------------------- " << use_node_num_ << endl;
      // cout << "use node num: " << use_node_num_ << endl;
      // cout << "iter num: " << iter_num_ << endl;
      terminate_node = cur_node;
      retrievePath(terminate_node);
      return REACH_END;
    }
    /* ---------- pop node and add to close set ---------- */
    open_set_.pop();
    cur_node->node_state = IN_CLOSE_SET;
    iter_num_ += 1;

    /* ---------- init neighbor expansion ---------- */

    Eigen::Vector2d cur_pos = cur_node->position.head(2);
    Eigen::Vector2d pro_pos;


    // explore neighbor of current node
    for (const auto& motion : expand_motions_)
    {
      pro_pos = jump(cur_pos, motion);
      double tmp_cost = (pro_pos - cur_pos).norm();
      if((pro_pos - fail_sign_.head(2)).norm() < 1e-5)
      {
        continue;
      }
      // std::cout << "cur_pos: " << cur_pos.transpose() << ", find a jump point " << pro_pos.transpose() << ", cost: " << tmp_cost << std::endl;
      /* not in close set */
      Eigen::Vector3i pro_id = posToIndex(Eigen::Vector3d(pro_pos(0), pro_pos(1), z_));

      NodePtr pro_node = expanded_nodes_.find(pro_id);

      if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET) {
        // cout << "in closeset" << endl;
        continue;
      }
      /* ---------- compute cost ---------- */
      double tmp_g_score, tmp_f_score;
      tmp_g_score = tmp_cost + cur_node->g_score;
      tmp_f_score = tmp_g_score + lambda_heu_ * getEuclHeu(pro_pos, end_pt.head(2));

      //是一个新的节点
      if (pro_node == NULL) {
        pro_node = path_node_pool_[use_node_num_];
        pro_node->index = pro_id;
        pro_node->position = Eigen::Vector3d(pro_pos(0), pro_pos(1), z_);
        pro_node->f_score = tmp_f_score;
        pro_node->g_score = tmp_g_score;
        pro_node->parent = cur_node;
        pro_node->node_state = IN_OPEN_SET;

        open_set_.push(pro_node);
        expanded_nodes_.insert(pro_id, pro_node);

        use_node_num_ += 1;
        if (use_node_num_ == allocate_num_) {
          cout << "[JPS]: run out of memory." << endl;
          return NO_PATH;
        }
      } else if (pro_node->node_state == IN_OPEN_SET) {
        if (tmp_g_score < pro_node->g_score) {
          // pro_node->index = pro_id;
          pro_node->position = Eigen::Vector3d(pro_pos(0), pro_pos(1), z_);
          pro_node->f_score = tmp_f_score;
          pro_node->g_score = tmp_g_score;
          pro_node->parent = cur_node;
        }
      } else {
        cout << "[JPS]: error type in searching: " << pro_node->node_state << endl;
      }
    }
  }
  cout << "[JPS]: Open set empty!" << std::endl;

  return NO_PATH;
}

/**
 * @brief Calculate jump node recursively
 * @param point  current node
 * @param motion the motion that current node executes
 * @return jump node
 */
 // 这里面需要记录cost。这里的cost需要回退吗？
 // 返回值，要么是碰撞or出了边界，要么是到了终点，要么是找到了force点。否则，不会停下来
Vector2d JumpPointSearch::jump(const Vector2d& cur_pos, const Vector2d& motion)
{
  //扩展后的位置
  Vector2d pro_pos = cur_pos + motion;
  // std::cout << "--------" << pro_pos.transpose() << std::endl;
  // next node hit the boundary or obstacle
  if (pro_pos(0) <= origin_(0) || pro_pos(0) >= map_size_3d_(0) || pro_pos(1) <= origin_(1) ||
      pro_pos(1) >= map_size_3d_(1)) {
    // cout << "[JPS]: outside map at " << pro_pos.transpose() << endl;
    return Vector2d(fail_sign_(0), fail_sign_(1));
  }
  if (edt_environment_->sdf_map_->getDistance2D(pro_pos) < 0.2)
  {
    // cout << "[JPS]: occlusion at" << pro_pos.transpose() << endl;
    return Vector2d(fail_sign_(0), fail_sign_(1));
  }


  // goal found
  if (posToIndex(Vec2ToVec3(pro_pos)).head(2) == end_index_.head(2))
  {
    // cout << "[JPS]: reach end" << endl;
    return pro_pos;
  }

  // diagonal
  if (motion.x() && motion.y())
  {
    // if exists jump point at horizontal or vertical
    Vector2d x_dir = Vector2d(motion.x(), 0);
    Vector2d y_dir = Vector2d(0, motion.y());
    // -1其实是代表jump失败了，超出边界or碰到障碍物了
    // 这里要给返回值搞一个什么标志符用来判断
    if((jump(pro_pos, x_dir) - fail_sign_.head(2)).norm() > 1e-5 || 
       (jump(pro_pos, y_dir) - fail_sign_.head(2)).norm() > 1e-5)
      return pro_pos;
  }

  // exists forced neighbor
  if (detectForceNeighbor(pro_pos, motion))
  {
    // cout << "[JPS]: find force point at " << pro_pos.transpose() << endl;
    return pro_pos;
  }
  else
    return jump(pro_pos, motion);
}

/**
 * @brief Detect whether current node has forced neighbors
 * @param point  current node
 * @param motion the motion that current node executes
 * @return true if current node has forced neighbor else false
 */
bool JumpPointSearch::detectForceNeighbor(const Vector2d& cur_pos, const Vector2d& motion)
{
  double x = cur_pos.x();
  double y = cur_pos.y();
  double x_dir = motion.x();
  double y_dir = motion.y();

  // horizontal
  if (x_dir && !y_dir)
  {
    if(edt_environment_->sdf_map_->getDistance2D(Vector2d(x, y + resolution_)) < 0.2 && 
       !edt_environment_->sdf_map_->getDistance2D(Vector2d(x + x_dir, y + resolution_)) < 0.2)
       return true;

    if(edt_environment_->sdf_map_->getDistance2D(Vector2d(x, y - resolution_)) < 0.2 && 
       !edt_environment_->sdf_map_->getDistance2D(Vector2d(x + x_dir, y - resolution_)) < 0.2)
       return true;
  }

  // vertical
  if (!x_dir && y_dir)
  {
    if(edt_environment_->sdf_map_->getDistance2D(Vector2d(x + resolution_, y)) < 0.2 && 
       !edt_environment_->sdf_map_->getDistance2D(Vector2d(x + resolution_, y + y_dir)) < 0.2)
       return true;

    if(edt_environment_->sdf_map_->getDistance2D(Vector2d(x - resolution_, y)) < 0.2 && 
       !edt_environment_->sdf_map_->getDistance2D(Vector2d(x - resolution_, y + y_dir)) < 0.2)
       return true;
  }

  // diagonal
  if (x_dir && y_dir)
  {
    if(edt_environment_->sdf_map_->getDistance2D(Vector2d(x - x_dir, y)) < 0.2 && 
       !edt_environment_->sdf_map_->getDistance2D(Vector2d(x - x_dir, y + y_dir)) < 0.2)
       return true;

    if(edt_environment_->sdf_map_->getDistance2D(Vector2d(x, y - y_dir)) < 0.2 && 
       !edt_environment_->sdf_map_->getDistance2D(Vector2d(x + x_dir, y - y_dir)) < 0.2)
       return true;
  }

  return false;
}

void JumpPointSearch::arrangePath()
{
  jps_path_ = this->getPath();
  jps_path_points_.clear();
  vector<Eigen::Vector2i> jps_points_int_2d, jps_path_int_2d;
  for(const auto & jps_path_point : jps_path_)
  {
    jps_path_int_2d.push_back(stateToIndex2D(jps_path_point.head(2)));
  }
  jps_points_int_2d = connectPoints(jps_path_int_2d);
  for(const auto & jps_point_int_2d : jps_points_int_2d)
  {
    auto kk = indexToState2D(jps_point_int_2d);
    jps_path_points_.push_back(Eigen::Vector3d(kk(0), kk(1), z_));
  }
  jps_path_distance_ = computePathDistances(jps_path_points_);
}
void JumpPointSearch::arrangePath(const vector<Eigen::Vector3d>& guide_path)
{
  jps_path_ = guide_path;
  jps_path_points_.clear();
  vector<Eigen::Vector2i> jps_points_int_2d, jps_path_int_2d;
  for(const auto & jps_path_point : jps_path_)
  {
    jps_path_int_2d.push_back(stateToIndex2D(jps_path_point.head(2)));
  }
  jps_points_int_2d = connectPoints(jps_path_int_2d);
  for(const auto & jps_point_int_2d : jps_points_int_2d)
  {
    auto kk = indexToState2D(jps_point_int_2d);
    jps_path_points_.push_back(Eigen::Vector3d(kk(0), kk(1), z_));
  }
  jps_path_distance_ = computePathDistances(jps_path_points_);
}

Eigen::Vector2i JumpPointSearch::stateToIndex2D(Eigen::Vector2d state)
{
  Vector2i idx;
  idx[0] = ((state[0] - origin_(0)) * inv_resolution_);
  idx[1] = ((state[1] - origin_(1)) * inv_resolution_);
  return idx;
}

Eigen::Vector2d JumpPointSearch::indexToState2D(Eigen::Vector2i index)
{
  Vector2d state;
  state[0] = index[0] * resolution_ + origin_(0);
  state[1] = index[1] * resolution_ + origin_(1);
  return state;
}

}  // namespace global_planner