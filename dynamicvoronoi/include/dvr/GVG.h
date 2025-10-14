#pragma once
#include <vector>
#include <unordered_map>
#include <queue>
#include <stack>
#include <memory>
#include "point.h"
#include "dynamicvoronoi.h"
#include <iostream>
#include <algorithm>
#include <boost/functional/hash.hpp>
#include <map>
#include <cassert> 
#include <opencv4/opencv2/core.hpp>
#include <omp.h>
#include "DBSCAN2D.h"
// #define VERBOSE
// # define DEBUG_IMG
// int stage_ = 0; // 全局变量，用于调试
// cv::Mat voronoi_img(800, 800, CV_8UC3, cv::Scalar(255, 255, 255)); // 创建白色背景的图像
// cv::Mat voronoi_img2(800, 800, CV_8UC3, cv::Scalar(255, 255, 255)); // 创建白色背景的图像
// int total_iter_ = 0; // DFS的iter，用于调试

// 定义哈希函数，用于 unordered_map
namespace std {
template <>
struct hash<IntPoint> {
    size_t operator()(const IntPoint& p) const {
        return hash<int>()(p.x) ^ hash<int>()(p.y);
    }
};
}
class SimpleTimer {
  public:
      SimpleTimer(const std::string& name = "") : name_(name) {
          start_ = std::chrono::steady_clock::now();
      }
      void reset() {
          start_ = std::chrono::steady_clock::now();
      }
      double elapsedMs() const {
          auto end = std::chrono::steady_clock::now();
          return std::chrono::duration<double, std::milli>(end - start_).count();
      }
      double elapsedSec() const {
          auto end = std::chrono::steady_clock::now();
          return std::chrono::duration<double>(end - start_).count();
      }
      void print(const std::string& msg = " ") const {
          std::cout << " [TimeCost]: " << (name_.empty() ? " " : name_ + " ") << msg
                    << " " << elapsedMs() << " ms." << std::endl;
      }
  private:
      std::chrono::steady_clock::time_point start_;
      std::string name_;
  };
namespace gvg {
// 定义邻接方向
enum Direction {
    UP = 0b0001,    // 上
    DOWN = 0b0010,  // 下
    RIGHT = 0b0100, // 右
    LEFT = 0b1000,  // 左
    NONE = 0b0000   // 无效
};
// 设置某个方向为邻接
static void setDirection(uint8_t& adjacency, Direction dir) {
    adjacency |= dir;  // 按位或，设置对应位为 1
}

// 检查某个方向是否邻接
static bool isDirectionSet(uint8_t adjacency, Direction dir) {
    return adjacency & dir;  // 按位与，检查对应位是否为 1
}
// 清除某个方向的邻接
static void clearDirection(uint8_t& adjacency, Direction dir) {
    adjacency &= ~dir;  // 按位与和按位取反，清除对应位
}

// 获取方向的逆方向
static Direction getOppositeDirection(Direction dir) {
    switch (dir) {
        case UP:    return DOWN;
        case DOWN:  return UP;
        case RIGHT: return LEFT;
        case LEFT:  return RIGHT;
        default:    return NONE;  // 如果传入无效方向，返回 NONE
    }
}

// 解析方向编码，返回邻接方向列表
static std::vector<Direction> parseDirection(uint8_t direction_code) {
    std::vector<Direction> directions;

    if (direction_code & UP) {
        directions.emplace_back(UP);
    }
    if (direction_code & DOWN) {
        directions.emplace_back(DOWN);
    }
    if (direction_code & RIGHT) {
        directions.emplace_back(RIGHT);
    }
    if (direction_code & LEFT) {
        directions.emplace_back(LEFT);
    }
    return directions;
}

static IntPoint getDirection(Direction dir)
{
    IntPoint direction(0, 0);
    if (dir & UP) {
        direction.y = 1;
    }
    if (dir & DOWN) {
        direction.y = -1;
    }
    if (dir & RIGHT) {
        direction.x = 1;
    }
    if (dir & LEFT) {
        direction.x = -1;
    }
    return direction;
}


// 定义路径
struct Path {
    std::vector<IntPoint> path;  // 路径上的点，不包含起点和终点
    float path_length;           // 路径长度，从起点到终点的实际距离 - 1
    float path_length_x;
    float path_length_y;
};

// 定义图节点
struct GraphNode {
    using Ptr = std::shared_ptr<GraphNode>;
    using WeakPtr = std::weak_ptr<GraphNode>;
    enum NODE_TYPE { None = 0, Strong = 1, Weak = 2 };

    GraphNode(int x, int y, NODE_TYPE type_) : pos(x, y), type(type_), parent() {
        node_state = NOT_EXPAND_GVG;
    }

    // 使用WeakPtr来打破循环引用
    void addNeighbor(WeakPtr neighbor, const std::vector<IntPoint>& path, bool sample_check = true) {
        if (this->type != Strong && !neighbor.lock() && neighbor.lock()->type != Strong) {
            std::cerr << "Error: Only strong nodes can have neighbors." << std::endl;
            return;
        }
        if (sample_check)
        {
            for (const auto& nb : this->neighbors) {
                if (nb.lock() == neighbor.lock()) {
                    // std::cout << "Error: Neighbor already exists." << std::endl;
                    return;
                }
            }        
        }
        else 
        {
            for (size_t i = 0; i < this->neighbors.size(); ++i) {
                auto nb_ptr = this->neighbors[i].lock();
                auto neighbor_ptr = neighbor.lock();
                if (nb_ptr && neighbor_ptr && nb_ptr->pos == neighbor_ptr->pos) {
                    // 判断路径是否也完全相同
                    if (i < neighbor_paths.size() && neighbor_paths[i].path == path) {
                        // std::cout << "Error: Neighbor with same pos and path already exists." << std::endl;
                        return;
                    }
                }
            }
        }
        Path neighbor_path;
        neighbor_path.path = path; // 复制路径
        neighbor_path.path_length_x = 0.0f;
        neighbor_path.path_length_y = 0.0f;

        // 计算累计增量
        for (size_t j = 1; j < path.size(); ++j) {
            neighbor_path.path_length_x += std::abs(path[j].x - path[j - 1].x);
            neighbor_path.path_length_y += std::abs(path[j].y - path[j - 1].y);
        }
        // 计算路径长度为欧式距离
        neighbor_path.path_length = std::sqrt(neighbor_path.path_length_x * neighbor_path.path_length_x +
                                            neighbor_path.path_length_y * neighbor_path.path_length_y);
        neighbor_paths.emplace_back(neighbor_path);
        this->neighbors.emplace_back(neighbor);
    }
    IntPoint pos;                              // 节点坐标
    NODE_TYPE type = None;                     // 节点类型
    bool IsVisitedStage3 = false;              // 用于Stage3的访问标记
    bool RemovedByPVS     = false;              // 用于标记是否被某个阶段删除
    std::vector<WeakPtr> neighbors;            // 邻居节点（使用WeakPtr）
    std::vector<Path> neighbor_paths;          // 到邻居的路径

    enum NODE_STATE { IN_CLOSE_SET_GVG = 0, IN_OPEN_SET_GVG = 1, NOT_EXPAND_GVG = 2 };
    double g_score = 0.0, f_score = 0.0;                   // 用于A*算法
    GraphNode::WeakPtr parent;
    char node_state; 
    GraphNode() : parent(), node_state(NOT_EXPAND_GVG) {} 
};




class NodeComparator0 {
public:
    bool operator()(GraphNode::Ptr node1, GraphNode::Ptr node2) {
    return node1->f_score > node2->f_score;
    }
};

class NodeHashTable0 {
public:
    /* data */
    std::unordered_map<IntPoint, GraphNode::Ptr, std::hash<IntPoint>> data_2d_;

public:
    NodeHashTable0(/* args */) {
    }
    ~NodeHashTable0() {
    }
    void insert(IntPoint idx, GraphNode::Ptr node) {
        // data_2d_.insert(make_pair(idx, node)); // 这种方法不会更新覆盖
        data_2d_[idx] = node;                     
    }
    GraphNode::Ptr find(const IntPoint& idx) const {
        auto iter = data_2d_.find(idx);
        return iter == data_2d_.end() ? nullptr : iter->second;
    }
    bool contains(const IntPoint& idx) const {
        return data_2d_.find(idx) != data_2d_.end();
    }
    void clear() {
        data_2d_.clear();
    }
};

// 定义扩展结果
struct ExpansionResult {
    Path path;               // 扩展生成的路径
    IntPoint end_point;      // 扩展结束的点
    ExpansionResult() : end_point(-1, -1) {}
};

// GVG 类
class GVG {
public:
    // 创建图
    void createGraph(const DynamicVoronoi& voronoi) {
        SimpleTimer timer;
        // Simple
        // DynaVoro::SimpleTimer timer;
        timer.reset();
        for(auto & graph : graphs_) {
            for(auto & node_pair : graph) {
                for(auto & neighbor : node_pair.second->neighbors) {
                    neighbor.reset(); // 清空邻居指针
                }
                node_pair.second.reset(); // 清空节点指针
            }
        }
        graphs_.clear();
        if (grid_types_.size() != voronoi.getSizeX() * voronoi.getSizeY()) {
            grid_types_.resize(voronoi.getSizeX() * voronoi.getSizeY(), GRID_TYPE::None);
            grid_adjs_. resize(voronoi.getSizeX() * voronoi.getSizeY(), (uint8_t)0);
            voronoi_new.resize(voronoi.getSizeX() * voronoi.getSizeY(), false);
            sizeX_ = voronoi.getSizeX();
            sizeY_ = voronoi.getSizeY();
            std::cout << "[GVG] Init!" << std::endl;
        }
        // std::fill(grid_types_.begin(), grid_types_.end(), GRID_TYPE::None);
        // std::fill(grid_adjs_.begin(), grid_adjs_.end(), (uint8_t)0);
        // std::fill(voronoi_new.begin(), voronoi_new.end(), false);
        truncated_points_.clear();
        completeCoonction_points_.clear();
        #ifdef VERBOSE
        timer.print("Step 0: Init");
        timer.reset();
        #endif
        // 整体的流程变化了
        // Step 1: 遍历所有点，从原始的voronoi里判断是否为Voronoi点；
        // Step 2: 对于每个Voronoi点，判断其邻接关系，分类为强节点、弱节点、边，然后进行一次GVG化；
        // Step 3：单独处理所有弱节点和对应的强节点，然后删除“寄生”边；
        // Step 4：将剩下的图结构，画到voronoi_new上；
        // Step 5：将等高线加入到voronoi_new上，并补全连接；
        // Step 6：将voronoi_new上所有的Voronoi点，进行一次四方格删除和六方格删除；
        // Step 7：在新的voronoi_new上，再进行一次GVG化，得到最终的图结构。

        // Step 1: 遍历所有点，从原始的voronoi里判断是否为Voronoi点；
        std::vector<IntPoint> strong_grid_vec;
        for (int x = 0; x < sizeX_; ++x) {
            for (int y = 0; y < sizeY_; ++y) {        
                if (voronoi.isVoronoiWithDisThr(x, y, cle_thr_sq_low_)) {
                    voronoi_new[y * sizeX_ + x] = true;  // 标记为已voronoi点
                }
                else
                {
                    voronoi_new[y * sizeX_ + x] = false; // 标记为非voronoi点
                }
            }
        }      
        #ifdef VERBOSE
        timer.print("Step 1: Voronoi点判断");
        timer.reset();
        #endif
        // Step 2: 对于每个Voronoi点，判断其邻接关系，分类为强节点、弱节点、边，然后进行一次GVG化；
        for (int x = 0; x < sizeX_; ++x) {
            for (int y = 0; y < sizeY_; ++y) {        
                if (!voronoi_new[y * sizeX_ + x]) {
                    continue;  // 跳过非维诺图点
                }
                uint8_t dir_code = 0;
                int neighbors_num = getNumVoronoiNeighbors(x, y, dir_code);
                grid_adjs_[y * sizeX_ + x] = dir_code; 
                if (neighbors_num == 1) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Weak;  // 弱节点
                } else if (neighbors_num == 2) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Edge;  // 边
                } else if (neighbors_num >= 3) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Strong;  // 强节点
                    strong_grid_vec.emplace_back(x, y);
                } else { // 这是由于设置了阈值，导致某些地方的连接会断掉
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::None;  // none
                    voronoi_new[y * sizeX_ + x] = false;  // 标记为非voronoi点
                }
            }
        }

        #ifdef DEBUG_IMG
        cv::Mat voronoi_img0(sizeY_, sizeX_, CV_8UC3, cv::Scalar(255, 255, 255)); // 创建白色背景的图像
        drawVoronoiByGrid(voronoi_img0);
        #endif
        // // std::cout << "--------在修剪后的voronoi图上进行搜索---------" << std::endl;
        // stage_ = 1;
        for (int i = 0; i < strong_grid_vec.size(); ++i) {
            IntPoint start = strong_grid_vec[i];
            if ( parseDirection(grid_adjs_[start.y * sizeX_ + start.x]).empty()){
                continue;  // 已访问，跳过
            }
            // DFS 搜索
            graphs_.emplace_back(DFSSearch(start));
        }
        #ifdef VERBOSE
        timer.print("Step 2: DFS stage-1");
        timer.reset();
        #endif
        // return;
        // Step 3：单独处理所有弱节点和对应的强节点，然后删除“寄生”边；
        // 这里测试将弱节点全部都删除，以删除寄生边，从而让图变得更简洁
        // 首先遍历所有的week_node，将其邻接的strong_node加入到队列里面去
        // 然后遍历到strong_node的时候，检查其邻接的状态：
        // **如果是4邻接，先不处理；
        // **如果是3邻接，检查其邻接的weak_node数量：
            // ***如果是1个，则不处理；
            // ***如果是2个，且当前strong_node的dis小于阈值，则将其标记被week_node，并将其邻接strong_node加入队列
            // ***如果是2个，且当前strong_node的dis大于阈值，则其肯定还有一个strong_node邻接
            // 判断当前节点和那个stong_node的角度关系，如果角度小于阈值，则将该节点标记为week_node，并将其邻接strong_node加入队列
            // ***如果是3个，则直接标记为weak_node
        int skip_border = 10; // 在四个角上的弱节点不处理
        std::queue<GraphNode::Ptr> strong_node_queue;
        for (const auto& graph : graphs_) {
            for (const auto& node : graph) {
                if (node.second->type == GraphNode::Weak) {
                    auto pos = node.second->pos;
                    if ((pos.x < skip_border && (pos.y < skip_border || pos.y >= sizeY_ - skip_border)) || // 左上/左下
                        (pos.x >= sizeX_ - skip_border && (pos.y < skip_border || pos.y >= sizeY_ - skip_border)) // 右上/右下
                    ) {
                        node.second->type = GraphNode::Strong;
                        continue;
                    }
                    // 将邻接的强节点加入队列
                    for (const auto& neighbor : node.second->neighbors) {
                        if (auto locked_neighbor = neighbor.lock()) {
                            if (locked_neighbor->type == GraphNode::Strong) {
                                strong_node_queue.push(locked_neighbor);
                            }
                        }
                    }
                }
            }
        }
        // 遍历强节点队列
        int neighbor_count = 0;
        int neighbor_week_count = 0;
        std::vector<int> neighbor_strong_idx;
        while (!strong_node_queue.empty()) {
            auto strong_node_pt = strong_node_queue.front();
            strong_node_queue.pop();
            if (strong_node_pt->type != GraphNode::Strong) continue;  // 只处理强节点
            neighbor_strong_idx.clear();
            neighbor_count = strong_node_pt->neighbors.size();
            neighbor_week_count = neighbor_count;
            for (int i = 0; i < neighbor_count; ++i) {
                auto neighbor_ptr = strong_node_pt->neighbors[i].lock();
                if (neighbor_ptr && neighbor_ptr->type == GraphNode::Strong) {
                    neighbor_strong_idx.emplace_back(i);
                    --neighbor_week_count;
                }
            }
            if (neighbor_count == 4) 
            {
                if(neighbor_week_count == 4) strong_node_pt->type = GraphNode::Weak;
                else continue;
            }
            else if (neighbor_count == 3) {
                if (neighbor_week_count == 3) continue;
                if (neighbor_week_count == 1) continue;
                // if (strong_node_pt->pos.x == 1002 && strong_node_pt->pos.y == 760) {
                //     int debug = 0; // debug
                // }
                if (neighbor_week_count == 2) {
                    if (voronoi.getDistance(strong_node_pt->pos.x, strong_node_pt->pos.y) < 2.0 * std::sqrt(cle_thr_sq_low_)) {
                        // 将其标记为弱节点
                        strong_node_pt->type = GraphNode::Weak;
                        // 将邻接的强节点加入队列
                        for (int i = 0; i < neighbor_strong_idx.size(); ++i) {
                            auto neighbor_strong_lock = strong_node_pt->neighbors[neighbor_strong_idx[i]].lock();
                            if(neighbor_strong_lock) strong_node_queue.push(neighbor_strong_lock);
                        }
                    } else {
                        // 使用theta_SMA方法来判断
                        for (int i = 0; i < neighbor_strong_idx.size(); ++i) {
                            auto neighbor_strong_pt = strong_node_pt->neighbors[neighbor_strong_idx[i]].lock();
                            IntPoint dir1 = neighbor_strong_pt->pos - strong_node_pt->pos;
                            IntPoint neighbor_parent = IntPoint(voronoi.getObstacleX(neighbor_strong_pt->pos.x, neighbor_strong_pt->pos.y),
                                                        voronoi.getObstacleY(neighbor_strong_pt->pos.x, neighbor_strong_pt->pos.y));
                            IntPoint dir2 = neighbor_parent - strong_node_pt->pos;                            
                            // 计算两个dir的夹角
                            float angle_cos = (dir1.x * dir2.x + dir1.y * dir2.y) / 
                                (sqrt(dir1.x * dir1.x + dir1.y * dir1.y) * sqrt(dir2.x * dir2.x + dir2.y * dir2.y));
                            if (angle_cos < -0.5) {  // 夹角大于120度
                                // 将其标记为弱节点
                                strong_node_pt->type = GraphNode::Weak;
                                // 将邻接的强节点加入队列
                                for (int j = 0; j < neighbor_strong_idx.size(); ++j) {
                                    strong_node_queue.push(strong_node_pt->neighbors[neighbor_strong_idx[j]].lock());
                                }
                            }
                        }
                    }
                }
            }
        }
        #ifdef VERBOSE
        timer.print("Step 3: 删除寄生边");
        timer.reset();
        #endif
        // Step 4：将剩下的图结构，画到voronoi_new上；
        // std::fill(grid_types_.begin(), grid_types_.end(), GRID_TYPE::None);
        // std::fill(grid_adjs_.begin(), grid_adjs_.end(), (uint8_t)0);
        // TODO：这里是不是可以通过判断哪些node被遍历过了，从而跳过，来减少遍历的计算量。
        std::fill(voronoi_new.begin(), voronoi_new.end(), false);
        for (size_t graph_idx = 0; graph_idx < graphs_.size(); ++graph_idx)
        {
            const auto& graph = graphs_[graph_idx];
            for (const auto& node_pair : graph)
            {
                const auto& node_ptr = node_pair.second;
                if (node_ptr->type == GraphNode::Weak) continue;                 
                const IntPoint& node_pos = node_ptr->pos;
                std::vector<Path> paths = node_ptr->neighbor_paths;
                voronoi_new[node_ptr->pos.x + node_ptr->pos.y * sizeX_] = true;
                for (int i = 0; i < paths.size(); ++i)
                {
                  const auto& path = paths[i];
                  if(node_ptr->neighbors[i].lock()->type == GraphNode::Weak) continue; 
                  for (size_t j = 0; j < path.path.size(); ++j)
                  {
                    voronoi_new[path.path[j].y * sizeX_ + path.path[j].x] = true;  // 标记为Voronoi起点
                  }
                }
            }
        }
        #ifdef VERBOSE
        timer.print("Step 4: 画出之前的voronoi_new");
        timer.reset();
        #endif
        graphs_.clear();

        #ifdef DEBUG_IMG
        cv::Mat voronoi_img1(sizeY_, sizeX_, CV_8UC3, cv::Scalar(255, 255, 255)); // 创建白色背景的图像
        drawVoronoiByGrid(voronoi_img1);
        #endif
        
        std::vector<IntPoint> strong_grid_vec2;
        // Step 5：将等高线加入到voronoi_new上，并补全连接；// 把下面的注释，就是不添加等高线。
    if (use_EGVG_){
        std::queue<IntPoint> voronoi_pt_queue;
        for (int x = 0; x < sizeX_; ++x) {
            for (int y = 0; y < sizeY_; ++y) {        
                float dis_sq = voronoi.getDistanceSq(x, y);
                if (voronoi_new[y * sizeX_ + x] == true) {
                    if (x >= 2 && x < sizeX_ - 2 && y >= 2 && y < sizeY_ - 2) {
                        voronoi_pt_queue.emplace(x, y);   //
                    }
                }                
                // 等高线点
                if (dis_sq >= cle_thr_sq_high_ - 1e-3 && dis_sq <= cle_thr_sq_high2_ - 1e-3) {
                    truncated_points_.emplace_back(x, y);  // 记录被截断的点
                    voronoi_new[y * sizeX_ + x] = true;  // 标记为已voronoi点
                    if (x >= 2 && x < sizeX_ - 2 && y >= 2 && y < sizeY_ - 2) {
                        voronoi_pt_queue.emplace(x, y);   //
                    }
                }
            }
        }
        #ifdef DEBUG_IMG
        drawVoronoiByGrid(voronoi_img0);
        #endif
        for (const auto& point : truncated_points_)
        {
            completeConnection(point.x, point.y, voronoi);
        }  
        for (const auto& point : completeCoonction_points_)
        {
            voronoi_pt_queue.emplace(point);  // 将补全连接的点加入队列
        }
        #ifdef VERBOSE
        timer.print("Step 5: 添加等高线");
        timer.reset();
        #endif
        // 这里我要把voronoi_new里面被判断为维诺图点的点都记录下来，然后遍历一次来去除四方格
        while (!voronoi_pt_queue.empty())
        {
            auto voronoi_pt = voronoi_pt_queue.front();
            voronoi_pt_queue.pop();
            bool retry = deleteblock(voronoi_pt.x, voronoi_pt.y);
            if (0){
                voronoi_pt_queue.push(voronoi_pt);  // 如果删除失败，重新入队
            }
        }
    }
        // 添加等高线的注释到这里
        #ifdef VERBOSE
        timer.print("Step 6: 删除四邻域和六邻域");
        timer.reset();
        #endif

        for (int x = 0; x < sizeX_; ++x) {
          for (int y = 0; y < sizeY_; ++y) {        
                if (!voronoi_new[y * sizeX_ + x]) {
                    continue;  // 跳过非维诺图点
                }
                uint8_t dir_code = 0;
                int neighbors_num = getNumVoronoiNeighbors(x, y, dir_code);
                grid_adjs_[y * sizeX_ + x] = dir_code; 
                if (neighbors_num == 1) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Weak;  // 弱节点
                } else if (neighbors_num == 2) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Edge;  // 边
                } else if (neighbors_num >= 3) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Strong;  // 强节点
                    strong_grid_vec2.emplace_back(x, y);
                } else {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::None;  // none
                    voronoi_new[y * sizeX_ + x] = false;  // 标记为非voronoi点
                }
            }
        }

        #ifdef DEBUG_IMG
        cv::Mat voronoi_img2(sizeY_, sizeX_, CV_8UC3, cv::Scalar(255, 255, 255)); // 创建白色背景的图像
        drawVoronoiByGrid(voronoi_img2);
        #endif

        // std::cout << "--------在加入等高线后的voronoi图上进行搜索---------" << std::endl;
        // stage_ = 2;
        for (int i = 0; i < strong_grid_vec2.size(); ++i) {
            IntPoint start = strong_grid_vec2[i];
            if ( parseDirection(grid_adjs_[start.y * sizeX_ + start.x]).empty()){
                continue;  // 已访问，跳过
            }
            // DFS 搜索
            graphs_.emplace_back(DFSSearch(start));
        }

        #ifdef VERBOSE
        timer.print("Step 7: DFS Stage-2");
        timer.reset();
        #endif

        DBSCAN2D dbscan(strong_grid_vec2, 4.3, 2); // 最大邻域判断点为(2, 1)
        std::vector<std::vector<int>> clusters;
        int num_clusters = dbscan.run(clusters);

        #ifdef VERBOSE
        timer.print("Step 8.1: DBSCAN");
        timer.reset();
        #endif

        #ifdef DEBUG_IMG
        drawVoronoiByGrid(voronoi_img2);
        #endif
        // // 把被聚类的点画到图上
        // for(int i = 0; i < num_clusters; ++i) {
        //     for (int j = 0; j < clusters[i].size(); ++j) {
        //         int idx = clusters[i][j];
        //         IntPoint pt = strong_grid_vec2[idx];
        //         voronoi_img.at<cv::Vec3b>(sizeY_ - pt.y - 1, pt.x) = cv::Vec3b(255, 255, 0); // 标记为色
        //     }
        // }
        
        for(int i = 0; i < num_clusters; ++i) {
            std::vector<IntPoint> cluster_points;
            for (int j = 0; j < clusters[i].size(); ++j) {
                int idx = clusters[i][j];
                IntPoint pt = strong_grid_vec2[idx];
                cluster_points.emplace_back(pt);
            }
            // 这里进行一系列检查
            // cluster_points: 当前聚类所有点
            std::vector<IntPoint> pre_outside_strong_neighbors;
            for (const auto& pt : cluster_points) {
                auto node = findNodeInGraphs(pt);
                if (!node) continue;
                for (const auto& nb : node->neighbors) {
                    auto nb_ptr = nb.lock();
                    if (!nb_ptr) continue;
                    // 不在聚类内，且是强节点
                    if (std::find(cluster_points.begin(), cluster_points.end(), nb_ptr->pos) == cluster_points.end()
                        && nb_ptr->type == gvg::GraphNode::Strong) {
                        pre_outside_strong_neighbors.push_back(nb_ptr->pos);
                    }
                }
            }
            // 去重
            std::sort(pre_outside_strong_neighbors.begin(), pre_outside_strong_neighbors.end());
            pre_outside_strong_neighbors.erase(
                std::unique(pre_outside_strong_neighbors.begin(), pre_outside_strong_neighbors.end()),
                pre_outside_strong_neighbors.end());

            // 把第一个点作为中心点
            auto center_node = findNodeInGraphs(cluster_points[0]);

            if (!center_node) continue;
            // std::cout << "Center nodeXXXX: " << center_node->pos.x << " ," << center_node->pos.y << std::endl;
            // if (center_node->pos == IntPoint(1447, 480) 
            //     || center_node->pos == IntPoint(1449, 480) 
            //     // || center_node->pos == IntPoint(358, 441)
            //     // || center_node->pos == IntPoint(359, 441)
            //     ){
            //     std::cout << "Center node: " << center_node->pos.x << " ," << center_node->pos.y << std::endl;
            // }
            DFSSearch2(center_node, cluster_points);
            // 简化后
            std::vector<IntPoint> center_neighbors;
            for (const auto& nb : center_node->neighbors) {
                auto nb_ptr = nb.lock();
                if (!nb_ptr) continue;
                if (nb_ptr->type != gvg::GraphNode::Strong) continue;
                if ((nb_ptr->pos == center_node->pos)) continue; // 这里可能碰到中心节点自成环的情况。
                center_neighbors.push_back(nb_ptr->pos);
                // 检查是否有聚类内邻居
                if (std::find(cluster_points.begin(), cluster_points.end(), nb_ptr->pos) != cluster_points.end()
                    // && !(nb_ptr->pos == center_node->pos)
                    ) {
                    // TODO: 这个地方还是有点错误。
                    std::cout << "Error: Center node still has cluster-internal neighbor: " << nb_ptr->pos.x << "," << nb_ptr->pos.y << std::endl;
                }
            }
            // 去重
            std::sort(center_neighbors.begin(), center_neighbors.end());
            center_neighbors.erase(
                std::unique(center_neighbors.begin(), center_neighbors.end()),
                center_neighbors.end());

            // 检查是否完全覆盖
            if (center_neighbors == pre_outside_strong_neighbors) {
                // std::cout << "Check passed: Center node neighbors match pre-simplification outside strong neighbors." << std::endl;
            } else {
                std::cout << "Check failed: Center node neighbors do not match!" << std::endl;
            }
            for (const auto& pt : cluster_points) {
                if (pt == center_node->pos) continue;
                auto node = findNodeInGraphs(pt);
                if (!node) continue;
                if (!node->neighbors.empty()) {
                    std::cout << "Error: Non-center cluster node " << pt.x << "," << pt.y << " still has neighbors!" << std::endl;
                }
            }
        }

        // 重新生成voronoi_new
        std::fill(voronoi_new.begin(), voronoi_new.end(), false);
        for(int i = 0; i < graphs_.size(); ++i)
        {
            auto graph = graphs_[i];
            for(auto& node : graph)
            {
                auto node_ptr = node.second;
                int x = node_ptr->pos.x, y = node_ptr->pos.y;
                if (node_ptr->type == GRID_TYPE::Strong)
                    voronoi_new[y * sizeX_ + x] = true;
                if (node_ptr->type == GRID_TYPE::Weak && node_ptr->neighbor_paths[0].path_length > 50)
                {
                    // 如果是弱节点，但是路径长度大于200，则认为是强节点
                    node_ptr->type = GraphNode::Strong;  // 将其标记为强节点
                    voronoi_new[y * sizeX_ + x] = true;  // 标记为Voronoi起点
                }
                else if (node_ptr->type == GraphNode::Weak) {
                    continue;  // 弱节点不处理  
                }
                if (node_ptr->type == GraphNode::None)
                {
                    this->removeNodeFromGraph(node_ptr->pos);
                }

                const std::vector<Path>& paths = node_ptr->neighbor_paths;
                for (const auto& path : paths)
                {
                    if (path.path.empty()) continue;
                    for (size_t j = 0; j < path.path.size(); ++j)
                    {
                        voronoi_new[sizeX_ * path.path[j].y + path.path[j].x] = true;
                    }
                }
            }
        }

        #ifdef DEBUG_IMG
        cv::Mat voronoi_img3(sizeY_, sizeX_, CV_8UC3, cv::Scalar(255, 255, 255));;
        drawVoronoiByGraphs(voronoi_img3);
        #endif

        #ifdef VERBOSE
        timer.print("Step 8.2: 删除冗余强节点");
        timer.reset();
        #endif
        // // debug用
        // grid_adjs_origin_ = grid_adjs_;  // 备份原始邻接方向
        // for (const auto& graph : graphs_) {
        //     for (const auto& node : graph) {
        //         int adj_dir_origin = parseDirection(grid_adjs_origin_[node.first.y * sizeX_ + node.first.x]).size();
        //         int adj_dir_now = parseDirection(grid_adjs_[node.first.y * sizeX_ + node.first.x]).size();
        //         if (adj_dir_now != 0 || adj_dir_origin != node.second->neighbors.size() 
        //             || node.second->neighbors.size() != node.second->neighbor_paths.size()) {
        //             std::cout << "ERROR! 这个节点的邻接方向不一致" << std::endl;
        //         }
        //         else
        //         {
        //             std::cout << "这个节点的邻接方向一致" << std::endl;
        //         }
        //     }
        // }

        // 这里，对truncated_points_进行四邻域检查，补全四邻域连接

        int debug = 0;

    }

    // 获取图
    std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>>& getGraphs() {
        return graphs_;
    }

    int getGraphsSize() {
        return static_cast<int>(graphs_.size());
    }

    void set_use_EGVG(bool use_egvg)
    {
        use_EGVG_ = use_egvg;
    }

    void getStrongNodes(std::vector<IntPoint>& strong_nodes) const {
        strong_nodes.clear();
        for (const auto& graph : graphs_)
        {
            for (const auto& node_pair : graph)
            {
                const auto& node_ptr = node_pair.second;
                IntPoint node_pos = node_ptr->pos;
                if (node_ptr->type == GraphNode::Strong) 
                {
                    strong_nodes.emplace_back(node_pos);
                }
                // else if (node_ptr->type == GraphNode::Weak) 
                // {
                //     if (!node_ptr->neighbor_paths.empty() && node_ptr->neighbor_paths[0].path_length > 20)
                //     {
                //         // 如果是弱节点，但是路径长度大于20，则认为是强节点
                //         strong_nodes.emplace_back(node_pos);
                //         node_ptr->type = GraphNode::Strong;  // 将其标记为强节点
                //     }
                // }
            }
        }
    }


    bool isVoronoi(int x, int y)
    {
        return voronoi_new[y * sizeX_ + x];
    }
    bool isVoronoi(IntPoint pt)
    {
        return voronoi_new[pt.y * sizeX_ + pt.x];
    }
    // 设置clearance_threshold_sq_
    void setClearanceThresholdSq(float threshold_low, float threshold_high) {
        cle_thr_sq_low_ = threshold_low;
        cle_thr_sq_high_ = threshold_high;
        cle_thr_sq_high2_ = (sqrt(threshold_high) + 1.0f) * (sqrt(threshold_high) + 1.0f);
    }
    const std::vector<IntPoint>& getTruncatedPoints() const {
        return truncated_points_;
    }
    const std::vector<IntPoint>& getCompleteConnectionPoints() const {
        return completeCoonction_points_;
    }

    GraphNode::Ptr findNodeInGraphs(const IntPoint& pt, int& graph_id) {
        if (graph_id == -1) {
            // 在所有图中查找
            for (size_t i = 0; i < graphs_.size(); ++i) {
                auto& graph = graphs_[i];
                auto it = graph.find(pt);
                if (it != graph.end()) {
                    graph_id = static_cast<int>(i); // 传出实际命中的id
                    return it->second;
                }
            }
            graph_id = -1; // 未找到
        } else if (graph_id >= 0 && graph_id < static_cast<int>(graphs_.size())) {
            // 只在指定图查找
            auto& graph = graphs_[graph_id];
            auto it = graph.find(pt);
            if (it != graph.end()) {
                return it->second;
            }
            // graph_id = -1; // 未找到
        } else {
            graph_id = -1; // 非法id
        }
        return nullptr;
    }
    GraphNode::Ptr findNodeInGraphs(const IntPoint& pt) {
        // 在所有图中查找
        for (auto& graph : graphs_) {
            auto it = graph.find(pt);
            if (it != graph.end()) {
                return it->second;
            }
        }
        return nullptr;
    }

    void insertNodeToGraph(const GraphNode::Ptr& node, int graph_id) {
        if (!node) return;
        if (graph_id < 0 || graph_id >= static_cast<int>(graphs_.size())) {
            std::cerr << "insertNodeToGraph: invalid graph_id " << graph_id << std::endl;
            return;
        }
        if (findNodeInGraphs(node->pos, graph_id)) {
            std::cerr << "insertNodeToGraph: node already exists in graph " << graph_id << std::endl;
            return;
        }
        graphs_[graph_id][node->pos] = node;
    }

    void removeNodeFromGraph(const IntPoint& pos, int graph_id) {
        if (graph_id < 0 || graph_id >= static_cast<int>(graphs_.size())) {
            std::cerr << "removeNodeFromGraph: invalid graph_id " << graph_id << std::endl;
            return;
        }
        graphs_[graph_id].erase(pos);
    }
    void removeNodeFromGraph(const IntPoint& pos) {
        for (size_t i = 0; i < graphs_.size(); ++i) {
            auto& graph = graphs_[i];
            auto it = graph.find(pos);
            if (it != graph.end()) {
                graph.erase(it);
                return;
            }
        }
    }

    

private:
    // 扩展函数。要通过result.path的size()来判断是不是有扩展
    // 根据result.end_point来判断，如果end_point是无效值，则说明扩展失败；然后判断end_point是强or弱grid
    ExpansionResult expandGrid(const IntPoint& start, Direction dir) {
        ExpansionResult result;
        // result.path.path.emplace_back(start); // start和end都不加入到path里面，这样是方便结果路径的生成
        result.path.path_length = 0.0f; result.path.path_length_x = 0.0f; result.path.path_length_y = 0.0f;

        IntPoint current = start;
        while (true) {
            // 计算下一个点
            IntPoint direction = getDirection(dir);
            IntPoint next(current.x + direction.x, current.y + direction.y);
            // if (next.x == 250 && next.y == 217 && stage_ == 2) {
            //     int debug = 0;
            // }
            // 检查边界 // TODO，或许边界检测可以省略
            // if (next.x < 0 || next.x >= sizeX_ || next.y < 0 || next.y >= sizeY_) {
            //     break;
            // }

            int next_idx = next.y * sizeX_ + next.x;
            int next_type = grid_types_[next_idx];
            uint8_t& next_adjs = grid_adjs_[next_idx];
            int next_visited = parseDirection(next_adjs).size();

            // 如果是空节点，跳过
            // 如果是成环的情况，这里会出现特殊情况，直接忽略就可以了
            if (next_type == 0) 
            {
                // std::cout << "ERROR! 应该不会返回维诺图以外的节点. Stage: " << stage_ << std::endl;
                break;
            }
            if (next_visited == 0) 
            {
                // std::cout << "ERROR! 应该不会出现next_visited = 0的情况. Stage: " << stage_ << std::endl;
                break;
            }

            // 如果排除了上述的情况，那么下面一定是有一个有效的节点
            // 那就直接先减去现在的方向
            clearDirection(next_adjs, getOppositeDirection(dir));

            if (next_type == 1 || next_type == 2) {
                // 如果是强节点或弱节点，直接返回
                result.end_point = next;
                break;
            }

            if (next_type == 3)
            {
                result.path.path.emplace_back(next);
                result.path.path_length_x += fabs(direction.x);
                result.path.path_length_y += fabs(direction.y);
                current = next;
                // direction需要更新
                std::vector<Direction> next_dir = parseDirection(next_adjs);
                if (next_dir.size() == 1)
                {
                    dir = next_dir[0];
                }
                else 
                {
                    // 这里出现这种情况，是因为这里成环了。
                    // 如果是一条普通边，就是过来的时候访问一次，过去访问一次
                    // 但如果是一个环的边，那么就会过来和过去都会发生两次
                    // 是不是这个直接忽略就可以了？也不用报错
                    // std::cout << "ERROR! 一个edge grid的dir是两个, 删除一个一定剩下一个. Stage: " << stage_ << std::endl;
                    // std::cout << "next_dir.size() = " << next_dir.size() << std::endl;
                }
            }
        }
        result.path.path_length = std::sqrt(result.path.path_length_x * result.path.path_length_x + 
                                        result.path.path_length_y * result.path.path_length_y);
        return result;
    }


    // DFS 搜索
    // 每次从这个节点开始搜索、或者由别的节点搜索到该节点，都要从grid_adjs_中删除掉这个节点的邻接方向
    std::unordered_map<IntPoint, GraphNode::Ptr> DFSSearch(const IntPoint& start) {
        std::unordered_map<IntPoint, GraphNode::Ptr> local_graph;  // 局部图结构
        std::stack<IntPoint> stack;  // 使用栈代替队列
        stack.push(start);

        while (!stack.empty()) {
            IntPoint current = stack.top();  // 获取栈顶元素
            stack.pop();  // 弹出栈顶元素
            // if (current.x == 250 && current.y == 217 && stage_ == 2) {
            //     int debug = 0;
            // }

            int current_idx = current.y * sizeX_ + current.x;
            int current_type = grid_types_[current_idx];            
            
            if (current_type != 1 && current_type != 2) 
            {
                std::cout << "ERROR! 只有强or弱 grid可以作为graph node." << std::endl;
            }

            // 创建当前节点
            GraphNode::Ptr current_node;
            if (local_graph.find(current) != local_graph.end()) {
                current_node = local_graph[current];  // 如果已经存在，直接使用
            } else {
                // 创建新的节点
                current_node = std::make_shared<GraphNode>(current.x, current.y,
                                current_type == 1 ? GraphNode::Strong : GraphNode::Weak);
                local_graph[current] = current_node;
            }

            // 遍历当前节点的邻接方向
            uint8_t& adj_dir = grid_adjs_[current_idx];
            std::vector<Direction> directions = parseDirection(adj_dir);

            for (Direction dir : directions) {
                clearDirection(adj_dir, dir);
                // 扩展到邻接节点
                ExpansionResult result = expandGrid(current, dir);

                // 如果扩展失败，跳过
                if (result.end_point.x == -1 && result.end_point.y == -1) {
                    continue;
                }

                int next_idx = result.end_point.y * sizeX_ + result.end_point.x;
                int next_type = grid_types_[next_idx];
                if (next_type != 1 && next_type != 2) 
                {
                    std::cout << "ERROR! 只有强or弱 grid可以作为end_point." << std::endl;
                }

                // 创建终点节点
                GraphNode::Ptr end_node;
                if (local_graph.find(result.end_point) != local_graph.end()) {
                    end_node = local_graph[result.end_point];  // 如果已经存在，直接使用
                } else {
                    // 创建新的节点
                    end_node = std::make_shared<GraphNode>(result.end_point.x, result.end_point.y,
                                    next_type == 1 ? GraphNode::Strong : GraphNode::Weak);
                    local_graph[result.end_point] = end_node;
                }
                if (end_node->pos == current_node->pos) {
                    // 如果终点和起点是同一个节点，跳过
                    continue;
                }
                // // 检查是否已经被添加过了 // TODO：这里如果被添加过了，则更新为更短的路径。
                // bool found = false;
                // for (const auto& nb : current_node->neighbors) {
                //     if (nb.lock() == end_node) {
                //         found = true;
                //         break;
                //     }
                // }
                bool found = false;
                for (size_t i = 0; i < current_node->neighbors.size(); ++i) {
                    auto nb_ptr = current_node->neighbors[i].lock();
                    if (nb_ptr == end_node) {
                        found = true;
                        // 如果新路径更短，则更新
                        if (result.path.path_length < current_node->neighbor_paths[i].path_length) {
                            current_node->neighbor_paths[i] = result.path;
                            // 反向也更新
                            for (size_t j = 0; j < end_node->neighbors.size(); ++j) {
                                auto back_ptr = end_node->neighbors[j].lock();
                                if (back_ptr == current_node) {
                                    Path reverse_path = result.path;
                                    std::reverse(reverse_path.path.begin(), reverse_path.path.end());
                                    end_node->neighbor_paths[j] = reverse_path;
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }

                if (!found) {
                    // 添加边到 current_node 的邻居列表
                    current_node->neighbors.emplace_back(end_node);
                    current_node->neighbor_paths.emplace_back(result.path);
                    // 添加边到 end_node 的邻居列表（双向边）
                    end_node->neighbors.emplace_back(current_node);
                    // 创建反向路径
                    Path reverse_path = result.path;
                    std::reverse(reverse_path.path.begin(), reverse_path.path.end());  // 反转路径
                    end_node->neighbor_paths.emplace_back(reverse_path);                    
                }


                // 如果终点是弱节点，不继续扩展
                if (next_type == GraphNode::Weak) {
                    continue;
                } else {
                    stack.push(result.end_point);
                }
            }
        }
        return local_graph;
    }

    void DFSSearch2(GraphNode::Ptr& center_node, const std::vector<IntPoint>& cluster_points) {

        // 不用标记所有聚类内节点未访问，因为初始化就是false
        using PathPair = std::pair<GraphNode::Ptr, std::vector<IntPoint>>;
        std::stack<PathPair> stack;
        stack.push({center_node, {}});

        while (!stack.empty()) {
            auto [cur_node, path] = stack.top(); stack.pop();
            cur_node->IsVisitedStage3 = true;

            for (size_t i = 0; i < cur_node->neighbors.size(); ++i) {
                auto nb_ptr = cur_node->neighbors[i].lock();
                if (!nb_ptr) continue;
                if (nb_ptr->type == GraphNode::Weak) continue;  // 跳过弱节点。这样可能会把弱节点的连接直接断开。应该没有什么关系吧

                std::vector<IntPoint> new_path = path;
                new_path.insert(new_path.end(), cur_node->neighbor_paths[i].path.begin(), 
                                cur_node->neighbor_paths[i].path.end());
                
                // 判断是否在聚类内
                if (std::find(cluster_points.begin(), cluster_points.end(), nb_ptr->pos) != cluster_points.end()) {
                    if (nb_ptr->IsVisitedStage3) continue;
                    new_path.push_back(nb_ptr->pos);
                    stack.push({nb_ptr, new_path});
                } else {
                    // 只用addNeighbor即可，内部已做重复判断
                    center_node->addNeighbor(nb_ptr, new_path, false);
                    // 反向也加
                    nb_ptr->addNeighbor(center_node, std::vector<IntPoint>(new_path.rbegin(), new_path.rend()), false);
                }
            }
        }
        // 清理聚类内非中心节点的邻接关系，并解除聚类外邻居对这些节点的连接
        for (const auto& pt : cluster_points) {
            if (pt == center_node->pos) continue;
            auto node = findNodeInGraphs(pt);
            if (!node) continue;

            // 1. 先清除所有聚类外邻居对该节点的连接
            for (auto& nb_weak : node->neighbors) {
                auto nb_ptr = nb_weak.lock();
                if (!nb_ptr) continue;
                // 如果邻居不在聚类内（即为聚类外邻居）
                if (std::find(cluster_points.begin(), cluster_points.end(), nb_ptr->pos) == cluster_points.end()) {
                    // 在邻居的neighbors中移除对node的连接
                    for (size_t i = 0; i < nb_ptr->neighbors.size(); ) {
                        auto back_ptr = nb_ptr->neighbors[i].lock();
                        if (back_ptr && back_ptr->pos == pt) {
                            nb_ptr->neighbors.erase(nb_ptr->neighbors.begin() + i);
                            nb_ptr->neighbor_paths.erase(nb_ptr->neighbor_paths.begin() + i);
                        } else {
                            ++i;
                        }
                    }
                }
            }

            // 2. 清除该节点自己的邻接关系
            node->neighbors.clear();
            node->neighbor_paths.clear();
            node->type = GraphNode::None;
        }

        // 解除中心节点到其他聚类内节点的邻居关系
        std::vector<size_t> remove_indices;
        for (size_t i = 0; i < center_node->neighbors.size(); ++i) {
            auto nb_ptr = center_node->neighbors[i].lock();
            if (!nb_ptr) continue;
            if (nb_ptr->pos == center_node->pos) continue;  // 跳过中心节点自身.这是由于有时候成环造成的
            if (std::find(cluster_points.begin(), cluster_points.end(), nb_ptr->pos) != cluster_points.end()
                && !(nb_ptr->pos == center_node->pos)) {
                remove_indices.push_back(i);
            }
        }
        for (auto it = remove_indices.rbegin(); it != remove_indices.rend(); ++it) {
            center_node->neighbors.erase(center_node->neighbors.begin() + *it);
            center_node->neighbor_paths.erase(center_node->neighbor_paths.begin() + *it);
        }
    }

    int getNumVoronoiNeighbors(int x, int y, uint8_t& dir_code)
    {
        int count = 0;
        dir_code = 0;
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
            // 这是四邻域，八邻域的直接跳过
            if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0)) continue;

            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || nx >= sizeX_ || ny < 0 || ny >= sizeY_) continue;

            if (voronoi_new[ny * sizeX_ + nx])  // 如果是voronoi点
            {
                ++count;
                // 设置邻接方向
                if (dx == -1 && dy == 0)      setDirection(dir_code, LEFT);
                else if (dx == 1 && dy == 0)  setDirection(dir_code, RIGHT);
                else if (dx == 0 && dy == -1) setDirection(dir_code, DOWN);   // 注意，Y轴朝上
                else if (dx == 0 && dy == 1)  setDirection(dir_code, UP);
            }
            }
        }
        return count;
    }

    void completeConnection(int x, int y, const DynamicVoronoi& voronoi)
    {
        if (x == 382 && y == 600 - 441)
        {
            int debug = 0;  // 调试用
        }
        // implementation of connectivity patterns
        bool f[8];
        
        int nx, ny;
        int dx, dy;
        
        int i = 0;
        int count = 0;
        //  int obstacleCount=0;
        int voroCount = 0;
        int voroCountFour = 0;
        
        // 只取前8个为8邻域
        for (int k = 0; k < 8; ++k) {
            int nx = x + ne_offset_[k].x;
            int ny = y + ne_offset_[k].y;
            bool isVoronoi = voronoi_new[ny * sizeX_ + nx];
            f[k] = isVoronoi;
            if (isVoronoi) {
                ++voroCount;
                // 四邻域：k==1,3,4,6
                if (k == 1 || k == 3 || k == 4 || k == 6)
                    ++voroCountFour;
            }
            if (isVoronoi && (k == 1 || k == 3 || k == 4 || k == 6))
                ++count;
        }
        /*
        * 5 6 7
        * 3   4
        * 0 1 2
        */
        if (f[0] && (!f[1] && !f[3]))
        {
            tryCompleteVoronoi(x, y, 1, 3, voronoi);
        }
        if (f[2] && (!f[1] && !f[4]))
        {
            tryCompleteVoronoi(x, y, 1, 4, voronoi);
        }
        if (f[5] && (!f[3] && !f[6]))
        {
            tryCompleteVoronoi(x, y, 3, 6, voronoi);
        }
        if (f[7] && (!f[6] && !f[4]))
        {
            tryCompleteVoronoi(x, y, 6, 4, voronoi);
        }
    }
    // 封装补全Voronoi点的函数
    void tryCompleteVoronoi(int x, int y, int idx1, int idx2, const DynamicVoronoi& voronoi) {
        int x1 = x + ne_offset_[idx1].x;
        int y1 = y + ne_offset_[idx1].y;
        int x2 = x + ne_offset_[idx2].x;
        int y2 = y + ne_offset_[idx2].y;
        if (voronoi.getDistanceSq(x1, y1) > cle_thr_sq_high_) {
            voronoi_new[y1 * sizeX_ + x1] = true;
            completeCoonction_points_.emplace_back(x1, y1);
        } else if (voronoi.getDistanceSq(x2, y2) > cle_thr_sq_high_) {
            voronoi_new[y2 * sizeX_ + x2] = true;
            completeCoonction_points_.emplace_back(x2, y2);
        }
    }

    bool deleteblock(int x, int y)
    {
        if (x == 461 && y == 472)
        {
            int debug = 0;  // 调试用
        }
        bool f[8];
        
        int nx, ny;
        int dx, dy;
        IntPoint pt_curr(x, y);
        int i = 0;
        int count = 0;
        //  int obstacleCount=0;
        int voroCount = 0;
        int voroCountFour = 0;
        
        // 只取前8个为8邻域
        for (int k = 0; k < 8; ++k) {
            int nx = x + ne_offset_[k].x;
            int ny = y + ne_offset_[k].y;
            bool isVoronoi = voronoi_new[ny * sizeX_ + nx];
            f[k] = isVoronoi;
            if (isVoronoi) {
                ++voroCount;
                // 四邻域：k==1,3,4,6
                if (k == 1 || k == 3 || k == 4 || k == 6)
                    ++voroCountFour;
            }
            // if (isVoronoi && (k == 1 || k == 3 || k == 4 || k == 6))
            //     ++count;
        }

        // 这里要先处理6方格的情况
        if (voroCount >= 5) {
            if ( (f[0] && f[1] && f[2] && f[3] && f[4] && !f[6]) || (f[3] && f[4] && f[5] && f[6] && f[7] && !f[1]) ||
             (f[0] && f[1] && f[3] && f[5] && f[6] && !f[4]) || (f[1] && f[2] && f[4] && f[6] && f[7] && !f[3]) )
            {
                // 6方格的情况，直接删除当前节点
                voronoi_new[(pt_curr.y) * sizeX_ + (pt_curr.x)] = false;  // 删除当前节点
                return false;  // 不需要重试
            }
        }

        /*
        * 19 20 21 22 23
        * 17 5  6  7  18
        * 15 3  X  4  16
        * 13 0  1  2  14
        * 8  9  10 11 12
        */
        bool retry = false; 
        if (voroCount < 3) {
            return retry;  // 不需要重试
        }
        if (f[0] && f[1] && f[3])
        {
            retry = true;
            // // 0
            // if (!(voronoiAt(pt_curr + ne_offset_[13]) || voronoiAt(pt_curr + ne_offset_[9])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[0].y) * sizeX_ + (pt_curr.x + ne_offset_[0].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // // 1
            // if (!(voronoiAt(pt_curr + ne_offset_[2]) || voronoiAt(pt_curr + ne_offset_[10])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[1].y) * sizeX_ + (pt_curr.x + ne_offset_[1].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // // 3
            // if (!(voronoiAt(pt_curr + ne_offset_[15]) || voronoiAt(pt_curr + ne_offset_[5])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[3].y) * sizeX_ + (pt_curr.x + ne_offset_[3].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // curr
            if (!(voronoiAt(pt_curr + ne_offset_[4]) || voronoiAt(pt_curr + ne_offset_[6])))
            {
                voronoi_new[(pt_curr.y) * sizeX_ + (pt_curr.x)] = false;  // 删除当前节点
                retry = false;
                return retry;
            }
        }
        if (f[1] && f[2] && f[4])
        {
            // retry = true;
            // // 1
            // if (!(voronoiAt(pt_curr + ne_offset_[0]) || voronoiAt(pt_curr + ne_offset_[10])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[1].y) * sizeX_ + (pt_curr.x + ne_offset_[1].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // // 2
            // if (!(voronoiAt(pt_curr + ne_offset_[11]) || voronoiAt(pt_curr + ne_offset_[14])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[2].y) * sizeX_ + (pt_curr.x + ne_offset_[2].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // // 4
            // if (!(voronoiAt(pt_curr + ne_offset_[7]) || voronoiAt(pt_curr + ne_offset_[16])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[4].y) * sizeX_ + (pt_curr.x + ne_offset_[4].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // curr
            if (!(voronoiAt(pt_curr + ne_offset_[3]) || voronoiAt(pt_curr + ne_offset_[6])))
            {
                voronoi_new[(pt_curr.y) * sizeX_ + (pt_curr.x)] = false;  // 删除当前节点
                retry = false;
                return retry;
            }
        }
        if (f[3] && f[5] && f[6])
        {
            retry = true;
            // // 3
            // if (!(voronoiAt(pt_curr + ne_offset_[0]) || voronoiAt(pt_curr + ne_offset_[15])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[3].y) * sizeX_ + (pt_curr.x + ne_offset_[3].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // // 5
            // if (!(voronoiAt(pt_curr + ne_offset_[17]) || voronoiAt(pt_curr + ne_offset_[20])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[5].y) * sizeX_ + (pt_curr.x + ne_offset_[5].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // // 6
            // if (!(voronoiAt(pt_curr + ne_offset_[7]) || voronoiAt(pt_curr + ne_offset_[21])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[6].y) * sizeX_ + (pt_curr.x + ne_offset_[6].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // curr
            if (!(voronoiAt(pt_curr + ne_offset_[4]) || voronoiAt(pt_curr + ne_offset_[1])))
            {
                voronoi_new[(pt_curr.y) * sizeX_ + (pt_curr.x)] = false;  // 删除当前节点
                retry = false;
                return retry;
            }
        }
        if (f[4] && f[6] && f[7])
        {
            retry = true;
            // // 4
            // if (!(voronoiAt(pt_curr + ne_offset_[2]) || voronoiAt(pt_curr + ne_offset_[16])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[4].y) * sizeX_ + (pt_curr.x + ne_offset_[4].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // // 6
            // if (!(voronoiAt(pt_curr + ne_offset_[5]) || voronoiAt(pt_curr + ne_offset_[21])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[6].y) * sizeX_ + (pt_curr.x + ne_offset_[6].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // // 7
            // if (!(voronoiAt(pt_curr + ne_offset_[22]) || voronoiAt(pt_curr + ne_offset_[18])))
            // {
            //     voronoi_new[(pt_curr.y + ne_offset_[7].y) * sizeX_ + (pt_curr.x + ne_offset_[7].x)] = false;  // 删除当前节点
            //     retry = false;
            // }
            // curr
            if (!(voronoiAt(pt_curr + ne_offset_[3]) || voronoiAt(pt_curr + ne_offset_[1])))
            {
                voronoi_new[(pt_curr.y) * sizeX_ + (pt_curr.x)] = false;  // 删除当前节点
                retry = false;
                return retry;
            }
        }
        return retry;
    }

    inline bool voronoiAt(int x, int y) const {
        return voronoi_new[y * sizeX_ + x];
    }
    inline bool voronoiAt(const IntPoint& pt) const {
        return voronoi_new[pt.y * sizeX_ + pt.x];
    }

    void drawVoronoiByGraphs(cv::Mat& img) const {
        img.setTo(cv::Scalar(255, 255, 255)); // 清空图像
        for (size_t i = 0; i < graphs_.size(); ++i) {
            const auto& graph = graphs_[i];
            for (const auto& node : graph) {
                auto node_ptr = node.second;
                IntPoint node_pos = node_ptr->pos;
                if (node_ptr->type == GraphNode::Strong) {
                    img.at<cv::Vec3b>(sizeY_ - node_pos.y - 1, node_pos.x) = cv::Vec3b(0, 0, 255);   // 红色
                } else if (node_ptr->type == GraphNode::Weak) {
                    img.at<cv::Vec3b>(sizeY_ - node_pos.y - 1, node_pos.x) = cv::Vec3b(255, 0, 0);   // 蓝色
                }
                const std::vector<Path>& paths = node_ptr->neighbor_paths;
                for (const auto& path : paths) {
                    if (path.path.empty()) continue;
                    for (size_t j = 0; j < path.path.size(); ++j) {
                        int px = path.path[j].x;
                        int py = path.path[j].y;
                        img.at<cv::Vec3b>(sizeY_ - py - 1, px)[0] = 0;   // 蓝色通道
                        img.at<cv::Vec3b>(sizeY_ - py - 1, px)[1] = 255; // 绿色通道
                        img.at<cv::Vec3b>(sizeY_ - py - 1, px)[2] = 0;   // 红色通道
                    }
                }
            }
        }
    }

    void drawVoronoiByGrid(cv::Mat& img) const {
        img.setTo(cv::Scalar(255, 255, 255)); // 清空图像
        for (int y = 0; y < sizeY_; ++y) {        
            for (int x = 0; x < sizeX_; ++x) {
                if (voronoiAt(x, y)) {
                    img.at<cv::Vec3b>(sizeY_ - y - 1, x) = cv::Vec3b(0, 0, 255); // 红色
                }
            }
        }
        for (int y = 0; y < sizeY_; ++y) {        
            for (int x = 0; x < sizeX_; ++x) {
                if (grid_types_[y * sizeX_ + x] == GRID_TYPE::Strong) {
                    img.at<cv::Vec3b>(sizeY_ - y - 1, x) = cv::Vec3b(255, 0, 255); // 紫色
                }
                else if (grid_types_[y * sizeX_ + x] == GRID_TYPE::Weak) {
                    img.at<cv::Vec3b>(sizeY_ - y - 1, x) = cv::Vec3b(0, 255, 0); // 绿色
                }
            }
        }
    }


    enum GRID_TYPE { None = 0, Strong = 1, Weak = 2, Edge = 3 };

    std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>> graphs_;  // 图结构
    // std::vector<bool> grid_visited_;        // 访问标记 // 现在不要这个了，全部用grid_adjs_中剩下的邻居方向来表示是否还可以扩展
    std::vector<GRID_TYPE>  grid_types_;          // 网格类型, 0表示none，1表示强节点，2表示弱节点，3表示edge
    std::vector<uint8_t> grid_adjs_;        // 邻接方向
    std::vector<uint8_t> grid_adjs_origin_;        // 邻接方向，保留最初数据
    std::vector<IntPoint> truncated_points_;       // 被cle_thr_sq_high_截断的点
    std::vector<IntPoint> completeCoonction_points_;
    std::vector<bool> voronoi_new;          //在gvg里面单独弄一个紧凑维诺图
    int sizeX_, sizeY_;  // 网格大小
    float cle_thr_sq_low_;          // 单位是网格数，平方，小于这个数的被维诺点不被考虑
    float cle_thr_sq_high_, cle_thr_sq_high2_;   // dis大于这个数的维诺点不被考虑，用来构建更加紧凑的维诺图结构
    bool use_EGVG_ = true;

    /*
    * 19 20 21 22 23
    * 17 5  6  7  18
    * 15 3  X  4  16
    * 13 0  1  2  14
    * 8  9  10 11 12
    */
    std::vector<IntPoint> ne_offset_ = {
        IntPoint(-1, -1), // 0
        IntPoint( 0, -1), // 1
        IntPoint( 1, -1), // 2
        IntPoint(-1,  0), // 3
        IntPoint( 1,  0), // 4
        IntPoint(-1,  1), // 5
        IntPoint( 0,  1), // 6
        IntPoint( 1,  1), // 7
        IntPoint(-2, -2), // 8
        IntPoint(-1, -2), // 9
        IntPoint( 0, -2), // 10
        IntPoint( 1, -2), // 11
        IntPoint( 2, -2), // 12
        IntPoint(-2, -1), // 13
        IntPoint( 2, -1), // 14
        IntPoint(-2,  0), // 15
        IntPoint( 2,  0), // 16
        IntPoint(-2,  1), // 17
        IntPoint( 2,  1), // 18
        IntPoint(-2,  2), // 19
        IntPoint(-1,  2), // 20
        IntPoint( 0,  2), // 21
        IntPoint( 1,  2), // 22
        IntPoint( 2,  2)  // 23
    };

};

struct GridNode {
    IntPoint pos;
    double g, f;
    GridNode* parent;
    GridNode(const IntPoint& p, double g_, double f_, GridNode* par) : pos(p), g(g_), f(f_), parent(par) {}
};

struct GridNodeCmp {
    bool operator()(const GridNode* a, const GridNode* b) const {
        return a->f > b->f;
    }
};


class Planner
{
public:
    Planner() {}
    ~Planner() {
        for(int i = 0; i < allocate_num_; ++i)
        {
            path_node_pool_[i].reset();
        }
    }
    void init(std::shared_ptr<GVG> gvg)
    {
        gvg_ = gvg;
        path_node_pool_.resize(allocate_num_);
        for (int i = 0; i < allocate_num_; ++i) {
          path_node_pool_[i] = std::make_shared<GraphNode>();
        }
      
        use_node_num_ = 0;
        iter_num_ = 0;
    }
    void reset() {
        expanded_nodes_.clear();
        path_nodes_.clear();
      
        std::priority_queue<GraphNode::Ptr, std::vector<GraphNode::Ptr>, NodeComparator0> empty_queue;
        open_set_.swap(empty_queue);
      
        for (int i = 0; i < use_node_num_; ++i) {
          GraphNode::Ptr node = path_node_pool_[i];
          node->parent.reset();
          node->node_state = gvg::GraphNode::NOT_EXPAND_GVG;
          node->g_score = 0.0;
          node->f_score = 0.0;
        }
        use_node_num_ = 0;
        iter_num_ = 0;
    }

    bool serachPath(GraphNode::Ptr start_node, GraphNode::Ptr goal_node)
    {
        // 1. 初始化
        reset();

        // 2. 创建起点和终点节点
        GraphNode::Ptr current_node = start_node;
        // path_node_pool_[use_node_num_++] = current_node;
        // current_node = start_node;
        current_node->parent.reset();
        current_node->g_score = 0.0;
        current_node->f_score = lambda_heu_ * getDiagHeu(current_node->pos, goal_node->pos);
        current_node->node_state = GraphNode::IN_OPEN_SET_GVG;
      
        open_set_.push(current_node);
        expanded_nodes_.insert(current_node->pos, current_node);
        GraphNode::Ptr terminate_node = nullptr;
        GraphNode::Ptr neighbor_node = nullptr;
        // 3. A*搜索
        while (!open_set_.empty()) {
            current_node = open_set_.top();
            // std::cout << "iter_num_: " << iter_num_ << ", current node: " 
            //           << current_node->pos.x << ", " << current_node->pos.y << std::endl;
            if (current_node->pos == goal_node->pos) {
                terminate_node = current_node;
                retrievePath(terminate_node);

                // for (const auto& node_pair : expanded_nodes_.data_2d_) {
                //     auto node = node_pair.second; // 获取节点指针
                //     // 输出节点信息
                //     std::cout << "Node Position: (" << 0.1 * node->pos.x - 50 << ", " << 0.1 * node->pos.y - 50 << "), "
                //             << "f_score: " << node->f_score << ", "
                //             << "g_score: " << node->g_score << std::endl;
                // }
                // std::cout << "----------------" << std::endl;
                // for (const auto& node : path_nodes_) {
                //     std::cout << "Path Node: (" << 0.1 * node->pos.x - 50 << ", " << 0.1 * node->pos.y - 50 << ")" 
                //             << "f_score: " << node->f_score << ", "
                //             << "g_score: " << node->g_score << std::endl;
                // }

                // IntPoint check_pt(224, 629);
                // const int range = 2; // 5x5 区域，范围为 ±2
                // for (int dx = -range; dx <= range; ++dx) {
                //     for (int dy = -range; dy <= range; ++dy) {
                //         IntPoint nearby_pt(check_pt.x + dx, check_pt.y + dy);
                //         auto nearby_node = gvg_->findNodeInGraphs(nearby_pt);
                //         if (nearby_node) {
                //             std::cout << "Found nearby node at: (" << nearby_node->pos.x << ", " << nearby_node->pos.y << ")" << std::endl;
                //             return 1; // 找到节点，直接返回
                //         }
                //     }
                // }

                return 1;
            }
            open_set_.pop();
            current_node->node_state = GraphNode::IN_CLOSE_SET_GVG;   
            iter_num_++;         

            // 扩展邻居
            for (int i = 0; i < current_node->neighbors.size(); ++i) {
                neighbor_node = current_node->neighbors[i].lock();
                if (!neighbor_node) 
                {
                    // std::cout << "current_node: " << current_node->pos.x << ", " << current_node->pos.y 
                    //           << ", i: " << i << ", iter: " << iter_num_ << std::endl;
                    std::cout << "------------Error! Invalid neighbor-----------" << std::endl;
                    continue;
                }
                // std::cout << ", iter: " << iter_num_ 
                //           << ", current_node: " << 0.1*current_node->pos.x-50 << ", " << 0.1*current_node->pos.y-50 
                //           << ", neighbor: " << 0.1*neighbor_node->pos.x-50 << ", " << 0.1*neighbor_node->pos.y-50 
                //           << ", edge length: " << current_node->neighbor_paths[i].path_length * 0.1
                //           << std::endl;

                IntPoint neighborPoint = neighbor_node->pos;
                if(neighbor_node->type != GraphNode::Strong || neighbor_node->RemovedByPVS) continue;                
                auto expanded_node = expanded_nodes_.find(neighborPoint);
                if (expanded_node && expanded_node->node_state == GraphNode::IN_CLOSE_SET_GVG)
                    continue;

                double tentative_g_score = current_node->g_score + current_node->neighbor_paths[i].path_length + 1.0; // 假设网格的距离为 1
                double tentative_f_score = tentative_g_score + lambda_heu_ * getEuclHeu(neighborPoint, goal_node->pos);
                if(!expanded_node)
                {
                    // path_node_pool_[use_node_num_++] = neighbor_node;
                    neighbor_node->g_score = tentative_g_score;
                    neighbor_node->f_score = tentative_f_score;
                    neighbor_node->parent = current_node;
                    neighbor_node->node_state = GraphNode::IN_OPEN_SET_GVG;
                    open_set_.push(neighbor_node);
                    expanded_nodes_.insert(neighborPoint, neighbor_node);
                    if (use_node_num_ == allocate_num_) {
                        std::cout << "A star on GVG run out of memory." << std::endl;
                        return 0;
                    }                    
                }else if(neighbor_node->node_state == GraphNode::IN_OPEN_SET_GVG)
                {
                    if(tentative_g_score < neighbor_node->g_score)
                    {
                        neighbor_node->f_score = tentative_f_score;
                        neighbor_node->g_score = tentative_g_score;
                        neighbor_node->parent = current_node;
                    }
                }
                else
                {
                    std::cout << "A star on GVG error type in searching: " << neighbor_node->node_state << std::endl;
                }
            }
        }
        // std::cout << "open set empty, no path!" << std::endl;
        // std::cout << "use node num: " << use_node_num_ << std::endl;
        // std::cout << "iter num: " << iter_num_ << std::endl;
        return 0;
    }

    void retrievePath(GraphNode::Ptr end_node) {
        GraphNode::Ptr cur_node = end_node;
        path_nodes_.emplace_back(cur_node);

        while (auto parent_ptr = cur_node->parent.lock()) {
            cur_node = parent_ptr;
            path_nodes_.emplace_back(cur_node);
        }
      
        reverse(path_nodes_.begin(), path_nodes_.end());
    }
    std::vector<IntPoint> getPath() {
        std::vector<IntPoint> path;
        for (int i = 0; i < path_nodes_.size(); ++i) {
          path.emplace_back(path_nodes_[i]->pos);
        }
        return path;
    }
    std::vector<IntPoint> getFullPath() {
        std::vector<IntPoint> path;
        if (path_nodes_.empty()) return path;
        path.emplace_back(path_nodes_.front()->pos);
        for (int i = 1; i < path_nodes_.size(); ++i) {
            GraphNode::Ptr prev = path_nodes_[i - 1];
            GraphNode::Ptr curr = path_nodes_[i];
            int neighbor_idx = -1;
            for (size_t j = 0; j < prev->neighbors.size(); ++j) {
                if (prev->neighbors[j].lock() == curr) {
                    neighbor_idx = j;
                    break;
                }
            }
            if (neighbor_idx == -1) {
                // 没找到，说明图结构有问题
                continue;
            }
            const auto& sub_path = prev->neighbor_paths[neighbor_idx].path;
            path.insert(path.end(), sub_path.begin(), sub_path.end());
    
            path.emplace_back(path_nodes_[i]->pos);
        }
        return path;
    }

    std::vector<IntPoint> getFullPathNodes() {
        std::vector<IntPoint> path_nodes;
        if (path_nodes_.empty()) return path_nodes;
        path_nodes.emplace_back(path_nodes_.front()->pos);
        for (int i = 1; i < path_nodes_.size(); ++i) {
            GraphNode::Ptr prev = path_nodes_[i - 1];
            GraphNode::Ptr curr = path_nodes_[i];
            int neighbor_idx = -1;
            for (size_t j = 0; j < prev->neighbors.size(); ++j) {
                if (prev->neighbors[j].lock() == curr) {
                    neighbor_idx = j;
                    break;
                }
            }
            if (neighbor_idx == -1) {
                // 没找到，说明图结构有问题
                continue;
            }
            path_nodes.emplace_back(path_nodes_[i]->pos);
        }
        return path_nodes;
    }    


    std::vector<IntPoint> getVisitedNodes() {
        std::vector<IntPoint> visited;
        for(int i = 0; i < use_node_num_ - 1; ++i)
        {
            visited.emplace_back(path_node_pool_[i]->pos);
        }
        return visited;
    }

    std::vector<IntPoint> AstarOnVoronoi(const IntPoint& pt1, const IntPoint& pt2, 
                                         int sizeX, int sizeY, int graph_id)
    {
        // 1. 检查起点和终点是否在骨架上
        if (!gvg_->isVoronoi(pt1) || !gvg_->isVoronoi(pt2)) {
            std::cout << "Start or goal not on skeleton!" << std::endl;
            return {};
        }
        if (graph_id < 0)
        {
            std::cout << "It is not a valid graph id!" << std::endl;
            return {};
        }
        const auto graph_current = gvg_->getGraphs()[graph_id];
        // 2. 清空A*相关成员
        reset();
        
        // 3. 初始化起点
        GraphNode::Ptr start_node = path_node_pool_[use_node_num_++];
        start_node->pos = pt1;
        start_node->type = GraphNode::Strong;
        start_node->g_score = 0.0;
        start_node->f_score = getDiagHeu(pt1, pt2);
        start_node->parent.reset();
        start_node->node_state = GraphNode::IN_OPEN_SET_GVG;
        open_set_.push(start_node);
        expanded_nodes_.insert(pt1, start_node);

        // 四邻域
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        while (!open_set_.empty()) {
            GraphNode::Ptr cur = open_set_.top();
            open_set_.pop();

            iter_num_++;

            auto it_curr = graph_current.find(cur->pos);
            if (cur->pos == pt2 
                || it_curr != graph_current.end()
                ) {
                // 回溯路径
                path_nodes_.clear();
                for (auto node = cur; node; node = node->parent.lock())
                    path_nodes_.push_back(node);
                std::reverse(path_nodes_.begin(), path_nodes_.end());
                std::vector<IntPoint> path;
                for (auto& n : path_nodes_) path.push_back(n->pos);
                return path;
            }

            cur->node_state = GraphNode::IN_CLOSE_SET_GVG;

            for (int i = 0; i < 4; ++i) {
                IntPoint nb(cur->pos.x + dx[i], cur->pos.y + dy[i]);
                // if (nb.x < 0 || nb.x >= sizeX || nb.y < 0 || nb.y >= sizeY)
                //     continue;
                if (!gvg_->isVoronoi(nb))
                    continue;

                auto nb_node = expanded_nodes_.find(nb);
                double g_new = cur->g_score + 1.0;
                double f_new = g_new + getDiagHeu(nb, pt2);

                if (!nb_node) {
                    GraphNode::Ptr new_node = path_node_pool_[use_node_num_++];
                    new_node->pos = nb;
                    new_node->type = GraphNode::Strong;
                    new_node->g_score = g_new;
                    new_node->f_score = f_new;
                    new_node->parent = cur;
                    new_node->node_state = GraphNode::IN_OPEN_SET_GVG;
                    open_set_.push(new_node);
                    expanded_nodes_.insert(nb, new_node);
                    if (use_node_num_ == allocate_num_) {
                        std::cout << "A star on GVG run out of memory." << std::endl;
                        return {};
                    }   
                } else if (nb_node->node_state == GraphNode::IN_OPEN_SET_GVG && g_new < nb_node->g_score) {
                    nb_node->g_score = g_new;
                    nb_node->f_score = f_new;
                    nb_node->parent = cur;
                }
            }
        }
        std::cout << "No path found on skeleton!" << std::endl;
        std::cout << "use node num: " << use_node_num_ << std::endl;
        std::cout << "iter num: " << iter_num_ << std::endl;
        return {};
    }

    std::vector<std::vector<IntPoint>> expand_voronoi_grid(const IntPoint& voronoi_grid, 
                                        const std::vector<IntPoint> strong_nodes,
                                        const int& sizeX, const int& sizeY)
    {
        // 1. 检查起点是否在骨架上isVoronoi,并且不在graph上；
        // 2. 这意味着这个起点一定是一个edge，有两个邻居的grid；
        //    沿着这两个邻居的方向分别进行扩展，直到碰到一个Strong节点，就停止；
        // 3. 返回扩展这一路的两条路径，包括那个强节点；
        // 4. 如果两次扩展碰到的强节点是同一个，则表示成环了，返回。
        if (!gvg_->isVoronoi(voronoi_grid)) {
            std::cout << "Start point is not on skeleton!" << std::endl;
            return {};
        }
        if (gvg_->findNodeInGraphs(voronoi_grid)) {
            std::cout << "Start point is already in graph!" << std::endl;
            return {};
        }
        // 四邻域
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};
        std::vector<IntPoint> expand_dirs;
        for (int i = 0; i < 4; ++i) {
            IntPoint nb(voronoi_grid.x + dx[i], voronoi_grid.y + dy[i]);
            if (!gvg_->isVoronoi(nb))
                continue;
            expand_dirs.push_back(IntPoint(dx[i], dy[i]));
        }
        if (expand_dirs.size() != 2) {
            std::cout << "Start point is not a valid edge point!" << std::endl;
            return {};
        }
        std::vector<std::vector<IntPoint>> result_paths;
        IntPoint curr_dir;
        // 沿着两个方向开始扩展
        for (int i = 0; i < expand_dirs.size(); ++i) {
            std::vector<IntPoint> path;
            curr_dir = expand_dirs[i];
            std::queue<std::pair<IntPoint, std::vector<IntPoint>>> queue;
            queue.push({voronoi_grid + curr_dir, {voronoi_grid + curr_dir}});
            while (!queue.empty()) {
                auto [current_point, path] = queue.front();
                queue.pop();

                if (std::find(strong_nodes.begin(), strong_nodes.end(), current_point) != strong_nodes.end()) {
                    result_paths.push_back(path);
                    break; // 找到强节点，结束扩展
                }
                if(path.size() >= 2) curr_dir = path.back() - path[path.size() - 2]; // 获取当前方向
                for (int i = 0; i < 4; ++i) {
                    IntPoint expand_dir(dx[i], dy[i]);
                    if (expand_dir.x == -curr_dir.x && expand_dir.y == -curr_dir.y) continue; // 跳过当前方向

                    IntPoint candidate = current_point + expand_dir;
                    if (candidate.x < 0 || candidate.x >= sizeX || candidate.y < 0 || candidate.y >= sizeY) continue;
                    if (!gvg_->isVoronoi(candidate)) continue;

                    
                    std::vector<IntPoint> new_path = path;
                    new_path.push_back(candidate);
                    queue.push({candidate, new_path});
                }
            }
        }
        // 检查两个路径是否有相同的强节点
        if (result_paths.size() == 2 && result_paths[0].back() == result_paths[1].back()) {
            // 两条路径的终点是同一个强节点，表示成环了
            std::cout << "Found a loop!" << std::endl;
            return { result_paths[0].size() < result_paths[1].size() ? result_paths[0] : result_paths[1]}; // 返回只包含一条路径的二维vector
        }
        return result_paths; // 返回两条路径
    }


    void DFSSearch(std::vector<GraphNode::Ptr>& vis, GraphNode::Ptr goal, int max_path_num) {
    GraphNode::Ptr cur = vis.back();

    for (int i = 0; i < cur->neighbors.size(); ++i) {
        // 到达终点
        if (cur->neighbors[i].lock() == goal) {
            raw_topo_paths_.push_back(vis); // 直接存GraphNode::Ptr序列
            raw_topo_paths_.back().push_back(goal); // 添加终点节点
            if (raw_topo_paths_.size() >= max_path_num) return;
            break;
        }
    }
    IntPoint dir_to_goal = goal->pos - cur->pos;
    double dx1 = static_cast<double>(dir_to_goal.x);
    double dy1 = static_cast<double>(dir_to_goal.y);

    // 2. 收集可扩展邻居及其cos值
    std::vector<std::pair<double, int>> neighbor_scores;
    for (int i = 0; i < cur->neighbors.size(); ++i) {
        GraphNode::Ptr neighbor = cur->neighbors[i].lock();
        // if (!neighbor) continue;
        // if (neighbor->type != GraphNode::Strong) continue;
        // if (neighbor == goal) continue;
        // // 跳过已访问节点
        // bool revisit = false;
        // for (const auto& node : vis) {
        //     if (neighbor == node) {
        //         revisit = true;
        //         break;
        //     }
        // }
        // if (revisit) continue;
        IntPoint dir_to_neighbor = neighbor->pos - cur->pos;
        IntPoint dir_neighbor_to_goal = goal->pos - neighbor->pos;
        double dx2 = static_cast<double>(dir_to_neighbor.x);
        double dy2 = static_cast<double>(dir_to_neighbor.y);
        double cos_theta = (dx1 * dx2 + dy1 * dy2) / (std::hypot(dx1, dy1) * std::hypot(dx2, dy2) + 1e-6);
        
        double dx3 = static_cast<double>(dir_neighbor_to_goal.x);
        double dy3 = static_cast<double>(dir_neighbor_to_goal.y);
        double dis = std::hypot(dx3, dy3);
        
        neighbor_scores.emplace_back(dis, i);
    }
    std::sort(neighbor_scores.begin(), neighbor_scores.end(),
        [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
            return a.first < b.first;
        });
    // if (cur->pos == IntPoint(359, 321))
    // {
    //     int debug = 0;
    //     }

    for (const auto& item : neighbor_scores) {
        int i = item.second;

    // for (int i = 0; i < cur->neighbors.size(); ++i) {
        GraphNode::Ptr neighbor = cur->neighbors[i].lock();
        if (neighbor->type != GraphNode::Strong) continue; // 只考虑强节点
        if (neighbor == goal) continue;
        // 跳过已访问节点，防止环路
        bool revisit = false;
        for (const auto& node : vis) {
            if (neighbor->pos == node->pos) {
                revisit = true;
                break;
            }
        }
        if (revisit) continue;

        vis.push_back(neighbor);
        DFSSearch(vis, goal, max_path_num);
        if (raw_topo_paths_.size() >= max_path_num) return;
        vis.pop_back();
      }
    }   

    std::vector<std::vector<IntPoint>> searchTopoPaths(GraphNode::Ptr start_node, GraphNode::Ptr goal_node, int max_path_num)
    {
        raw_topo_paths_.clear();
        std::vector<GraphNode::Ptr> vis;
        vis.push_back(start_node);
        DFSSearch(vis, goal_node, max_path_num);

        std::vector<std::vector<IntPoint>> topo_paths;
        for (const auto& path_nodes : raw_topo_paths_) {
            std::vector<IntPoint> path;
            if (path_nodes.empty()) {
                topo_paths.push_back(path);
                continue;
            }
            // 起点
            path.emplace_back(path_nodes.front()->pos);
            for (int i = 1; i < path_nodes.size(); ++i) {
                GraphNode::Ptr prev = path_nodes[i - 1];
                GraphNode::Ptr curr = path_nodes[i];
                int neighbor_idx = -1;
                for (size_t j = 0; j < prev->neighbors.size(); ++j) {
                    if (prev->neighbors[j].lock() == curr) {
                        neighbor_idx = j;
                        break;
                    }
                }
                if (neighbor_idx == -1) {
                    // 没找到，说明图结构有问题
                    continue;
                }
                // 插入 prev 到 curr 的中间点（不含起点和终点）
                const auto& sub_path = prev->neighbor_paths[neighbor_idx].path;
                path.insert(path.end(), sub_path.begin(), sub_path.end());
                // 插入当前节点
                path.emplace_back(curr->pos);
            }
            topo_paths.push_back(path);
        }
        std::cout << "Found " << raw_topo_paths_.size() << " topological paths." << std::endl;
        for(int i = 0; i < raw_topo_paths_.size(); ++i)
        {
            std::cout << "Path " << i << ": " << raw_topo_paths_[i].size() << " nodes, ";
            std::cout << topo_paths[i].size() << " points, ";
            std::cout << std::endl;
        }
        return topo_paths;
    }


private:
    // 和A*规划相关的
    std::vector<GraphNode::Ptr> path_node_pool_;
    int use_node_num_, iter_num_;
    NodeHashTable0 expanded_nodes_;
    std::priority_queue<GraphNode::Ptr, std::vector<GraphNode::Ptr>, NodeComparator0> open_set_;
    std::vector<GraphNode::Ptr> path_nodes_;
    double lambda_heu_ = 1.0;
    int allocate_num_ = 10000;
    double tie_breaker_ = 1.0 + 1.0 / 10000;
    // TODO，这里是不是不需要GVG的指针了？因为规划器只需要拿到起点和终点的指针就行了
    std::shared_ptr<GVG> gvg_;

    // 和topo规划相关的
    std::vector<std::vector<GraphNode::Ptr>> raw_topo_paths_;
    /* heuristic function */
    double getDiagHeu(const IntPoint& p1, const IntPoint& p2) {
        int dx = std::abs(p1.x - p2.x);
        int dy = std::abs(p1.y - p2.y);
        int diag = std::min(dx, dy);
        double h = std::sqrt(2.0) * diag + 1.0 * (std::max(dx, dy) - diag);
        return tie_breaker_ * h;
    }
    double getManhHeu(const IntPoint& p1, const IntPoint& p2)
    {
        int dx = std::abs(p1.x - p2.x);
        int dy = std::abs(p1.y - p2.y);
        return tie_breaker_ * (dx + dy);
    }
    double getEuclHeu(const IntPoint& p1, const IntPoint& p2)
    {
        return tie_breaker_ * std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }

};
}  // namespace gvg