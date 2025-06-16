#ifndef GVG_H
#define GVG_H

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

int stage_ = 0; // 全局变量，用于调试
// cv::Mat voronoi_img(800, 800, CV_8UC3, cv::Scalar(255, 255, 255)); // 创建白色背景的图像
// cv::Mat voronoi_img2(800, 800, CV_8UC3, cv::Scalar(255, 255, 255)); // 创建白色背景的图像

// 定义哈希函数，用于 unordered_map
namespace std {
template <>
struct hash<IntPoint> {
    size_t operator()(const IntPoint& p) const {
        return hash<int>()(p.x) ^ hash<int>()(p.y);
    }
};
}

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
void setDirection(uint8_t& adjacency, Direction dir) {
    adjacency |= dir;  // 按位或，设置对应位为 1
}

// 检查某个方向是否邻接
bool isDirectionSet(uint8_t adjacency, Direction dir) {
    return adjacency & dir;  // 按位与，检查对应位是否为 1
}
// 清除某个方向的邻接
void clearDirection(uint8_t& adjacency, Direction dir) {
    adjacency &= ~dir;  // 按位与和按位取反，清除对应位
}

// 获取方向的逆方向
Direction getOppositeDirection(Direction dir) {
    switch (dir) {
        case UP:    return DOWN;
        case DOWN:  return UP;
        case RIGHT: return LEFT;
        case LEFT:  return RIGHT;
        default:    return NONE;  // 如果传入无效方向，返回 NONE
    }
}

// 解析方向编码，返回邻接方向列表
std::vector<Direction> parseDirection(uint8_t direction_code) {
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

IntPoint getDirection(Direction dir)
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
};

// 定义图节点
struct GraphNode {
    using Ptr = std::shared_ptr<GraphNode>;
    enum NODE_TYPE { None = 0, Strong = 1, Weak = 2 };
    GraphNode(int x, int y, NODE_TYPE type_) : pos(x, y), type(type_) 
    {
        parent = nullptr; node_state = NOT_EXPAND;
    }  
    void addNeighbor(GraphNode::Ptr neighbor, const std::vector<IntPoint>& path) {
        if (this->type != Strong && neighbor->type != Strong) {
            std::cerr << "Error: Only strong nodes can have neighbors." << std::endl;
            return;
        }
        Path neighbor_path;
        neighbor_path.path_length = path.size();
        neighbor_path.path = path;
        this->neighbors.emplace_back(neighbor);
        neighbor_paths.emplace_back(neighbor_path);
    }
    IntPoint pos;                              // 节点坐标
    NODE_TYPE type = None;                     // 节点类型
    std::vector<GraphNode::Ptr> neighbors;     // 邻居节点
    std::vector<Path> neighbor_paths;          // 到邻居的路径

    enum NODE_STATE { IN_CLOSE_SET = 'a', IN_OPEN_SET = 'b', NOT_EXPAND = 'c' };
    double g_score = 0.0, f_score = 0.0;                   // 用于A*算法
    GraphNode::Ptr parent;
    char node_state; 
    GraphNode() {parent = nullptr; node_state = NOT_EXPAND;}    
};



class NodeComparator0 {
public:
    bool operator()(GraphNode::Ptr node1, GraphNode::Ptr node2) {
    return node1->f_score > node2->f_score;
    }
};

class NodeHashTable0 {
private:
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
        graphs_.clear();
        if (grid_types_.size() != voronoi.getSizeX() * voronoi.getSizeY()) {
            grid_types_.resize(voronoi.getSizeX() * voronoi.getSizeY(), GRID_TYPE::None);
            grid_adjs_. resize(voronoi.getSizeX() * voronoi.getSizeY(), (uint8_t)0);
            voronoi_new.resize(voronoi.getSizeX() * voronoi.getSizeY(), false);
            sizeX_ = voronoi.getSizeX();
            sizeY_ = voronoi.getSizeY();
            std::cout << "Init!" << std::endl;
        }
        // std::fill(grid_types_.begin(), grid_types_.end(), GRID_TYPE::None);
        // std::fill(grid_adjs_.begin(), grid_adjs_.end(), (uint8_t)0);
        // std::fill(voronoi_new.begin(), voronoi_new.end(), false);
        truncated_points_.clear();
        completeCoonction_points_.clear();

        // 整体的流程变化了
        // Step 1: 遍历所有点，从原始的voronoi里判断是否为Voronoi点；
        // Step 2: 对于每个Voronoi点，判断其邻接关系，分类为强节点、弱节点、边，然后进行一次GVG化；
        // Step 3：单独处理所有弱节点和对应的强节点，然后删除“寄生”边；
        // Step 4：将剩下的图结构，画到voronoi_new上；
        // Step 5：将等高线加入到voronoi_new上，并补全连接；
        // Step 6：将voronoi_new上所有的Voronoi点，进行一次四方格删除和六方格删除；
        // Step 7：在新的voronoi_new上，再进行一次GVG化，得到最终的图结构。

        // Step 1: 遍历所有点，从原始的voronoi里判断是否为Voronoi点；
        std::vector<IntPoint> week_grid_vec;
        // #ifdef USE_OPENMP
        #pragma omp parallel for num_threads(8)
        // #endif
        for (int y = 0; y < sizeY_; ++y) {        
            for (int x = 0; x < sizeX_; ++x) {
                if (voronoi.isVoronoiWithDisThr(x, y, cle_thr_sq_low_)) {
                    voronoi_new[y * sizeX_ + x] = true;  // 标记为已voronoi点
                }
                else
                {
                    voronoi_new[y * sizeX_ + x] = false; // 标记为非voronoi点
                }
            }
        }      
        // Step 2: 对于每个Voronoi点，判断其邻接关系，分类为强节点、弱节点、边，然后进行一次GVG化；
        for (int y = 0; y < sizeY_; ++y) {        
            for (int x = 0; x < sizeX_; ++x) {
                if (!voronoi_new[y * sizeX_ + x]) {
                    continue;  // 跳过非维诺图点
                }
                uint8_t dir_code = 0;
                int neighbors_num = getNumVoronoiNeighbors(x, y, dir_code);
                grid_adjs_[y * sizeX_ + x] = dir_code; 
                if (neighbors_num == 1) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Weak;  // 弱节点
                    week_grid_vec.emplace_back(x, y);
                } else if (neighbors_num == 2) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Edge;  // 边
                } else if (neighbors_num >= 3) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Strong;  // 强节点
                } else { // 这是由于设置了阈值，导致某些地方的连接会断掉
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::None;  // none
                    voronoi_new[y * sizeX_ + x] = false;  // 标记为非voronoi点
                }
            }
        }
        // voronoi_img.setTo(cv::Scalar(255, 255, 255)); // 清空图像
        // for (int y = 0; y < sizeY_; ++y) {        
        //     for (int x = 0; x < sizeX_; ++x) {
        //         if (voronoi_new[y * sizeX_ + x] == true) {
        //             voronoi_img.at<cv::Vec3b>(sizeY_ - y - 1, x) = cv::Vec3b(0, 0, 255); // 标记为红色
        //         }
        //     }
        // }  
        // for (int y = 0; y < sizeY_; ++y) {        
        //     for (int x = 0; x < sizeX_; ++x) {
        //         if (grid_types_[y * sizeX_ + x] == GRID_TYPE::Strong) {
        //             voronoi_img.at<cv::Vec3b>(sizeY_ - y - 1, x) = cv::Vec3b(255, 0, 255); 
        //         }
        //         else if (grid_types_[y * sizeX_ + x] == GRID_TYPE::Weak) {
        //             voronoi_img.at<cv::Vec3b>(sizeY_ - y - 1, x) = cv::Vec3b(0, 255, 0); 
        //         }
        //         else if (grid_types_[y * sizeX_ + x] == GRID_TYPE::Edge) {
        //             voronoi_img.at<cv::Vec3b>(sizeY_ - y - 1, x) = cv::Vec3b(0, 255, 255); 
        //         }
        //     }
        // }  
        // {
        //     int debug = 0;            
        // }
        // // std::cout << "--------在修剪后的voronoi图上进行搜索---------" << std::endl;
        // stage_ = 1;
        for (int i = 0; i < week_grid_vec.size(); ++i) {
            IntPoint start = week_grid_vec[i];
            if ( parseDirection(grid_adjs_[start.y * sizeX_ + start.x]).empty()){
                continue;  // 已访问，跳过
            }
            // DFS 搜索
            graphs_.emplace_back(DFSSearch(start));
        }
        // Step 3：单独处理所有弱节点和对应的强节点，然后删除“寄生”边；
        // 这里测试将弱节点全部都删除，以删除寄生边，从而让图变得更简洁
        // 首先遍历所有的week_node，将其邻接的strong_node加入到队列里面去
        // 然后遍历到strong_node的时候，检查其邻接的状态：
        // **如果是4邻接，先不处理；
        // **如果是3邻接，检查其邻接的weak_node数量：
                       // ***如果是1个，则不处理；
                       // ***如果是2个，且当前strong_node的dis小于阈值，则将其标记被week_node，并将其邻接strong_node加入队列
                       // ***如果是2个，且当前strong_node的dis大于阈值，则其肯定还有一个strong_node邻接
                       // 判断当前节点和那个stong_node的角度关系，如果角度小于XX，则将该节点标记为week_node，并将其邻接strong_node加入队列
                       // ***如果是3个，则直接标记为weak_node
        std::queue<GraphNode::Ptr> strong_node_queue;
        for (const auto& graph : graphs_) {
            for (const auto& node : graph) {
                if (node.second->type == GraphNode::Weak) {
                    // 将邻接的强节点加入队列
                    for (const auto& neighbor : node.second->neighbors) {
                        if (neighbor->type == GraphNode::Strong) {
                            strong_node_queue.push(neighbor);
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
                if (strong_node_pt->neighbors[i]->type == GraphNode::Strong) {
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
                if (neighbor_week_count == 2) {
                    if (voronoi.getDistance(strong_node_pt->pos.x, strong_node_pt->pos.y) < 5.0) {
                        // 将其标记为弱节点
                        strong_node_pt->type = GraphNode::Weak;
                        // 将邻接的强节点加入队列
                        for (int i = 0; i < neighbor_strong_idx.size(); ++i) {
                            strong_node_queue.push(strong_node_pt->neighbors[neighbor_strong_idx[i]]);
                        }
                    } else {
                        // 使用theta_SMA方法来判断
                        for (int i = 0; i < neighbor_strong_idx.size(); ++i) {
                            auto neighbor_strong_pt = strong_node_pt->neighbors[neighbor_strong_idx[i]];
                            IntPoint dir1 = neighbor_strong_pt->pos - strong_node_pt->pos;
                            IntPoint neighbor_parent = IntPoint(voronoi.getObstacleX(neighbor_strong_pt->pos.x, neighbor_strong_pt->pos.y),
                                                        voronoi.getObstacleY(neighbor_strong_pt->pos.x, neighbor_strong_pt->pos.y));
                            IntPoint dir2 = neighbor_parent - strong_node_pt->pos;                            
                            // 计算两个dir的夹角
                            float angle_cos = (dir1.x * dir2.x + dir1.y * dir2.y) / 
                                (sqrt(dir1.x * dir1.x + dir1.y * dir1.y) * sqrt(dir2.x * dir2.x + dir2.y * dir2.y));
                            if (angle_cos < -0.866) {  // 夹角大于145度
                                // 将其标记为弱节点
                                strong_node_pt->type = GraphNode::Weak;
                                // 将邻接的强节点加入队列
                                for (int j = 0; j < neighbor_strong_idx.size(); ++j) {
                                    strong_node_queue.push(strong_node_pt->neighbors[neighbor_strong_idx[j]]);
                                }
                            }
                        }
                    }
                }
            }
        }

        // 如果这里直接return的话，就相当于不添加等高线边了
        // 但是这样的话，对于A*搜索不会增加很多时间，但是在path shorten的时候会很耗时，而且如果后续搜多拓扑路径时是不好的
        // return;
        // Step 4：将剩下的图结构，画到voronoi_new上；
        // std::fill(grid_types_.begin(), grid_types_.end(), GRID_TYPE::None);
        // std::fill(grid_adjs_.begin(), grid_adjs_.end(), (uint8_t)0);
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
                  if(node_ptr->neighbors[i]->type == GraphNode::Weak) continue; 
                  for (size_t j = 0; j < path.path.size(); ++j)
                  {
                    voronoi_new[path.path[j].y * sizeX_ + path.path[j].x] = true;  // 标记为Voronoi起点
                  }
                }
            }
        }
        graphs_.clear();
        // cv::Mat voronoi_img(sizeY_, sizeX_, CV_8UC3, cv::Scalar(255, 255, 255)); // 创建白色背景的图像
        // for (int y = 0; y < sizeY_; ++y) {        
        //     for (int x = 0; x < sizeX_; ++x) {
        //         if (voronoi_new[y * sizeX_ + x] == true) {
        //             voronoi_img.at<cv::Vec3b>(sizeY_ - y - 1, x) = cv::Vec3b(0, 0, 255); // 标记为红色
        //         }
        //     }
        // }  
        // {
        //     int debug = 0;            
        // }
        
        // Step 5：将等高线加入到voronoi_new上，并补全连接；
        std::vector<IntPoint> week_grid_vec2;
        std::queue<IntPoint> voronoi_pt_queue;
        #ifdef USE_OPENMP
        #pragma omp parallel for
        #endif
        for (int y = 0; y < sizeY_; ++y) {        
            for (int x = 0; x < sizeX_; ++x) {
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
        for (const auto& point : truncated_points_)
        {
            completeConnection(point.x, point.y, voronoi);
        }  
        for (const auto& point : completeCoonction_points_)
        {
            voronoi_pt_queue.emplace(point);  // 将补全连接的点加入队列
        }
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
        // voronoi_img.setTo(cv::Scalar(255, 255, 255)); // 清空图像
        // for (int y = 0; y < sizeY_; ++y) {        
        //     for (int x = 0; x < sizeX_; ++x) {
        //         if (voronoi_new[y * sizeX_ + x] == true) {
        //             voronoi_img.at<cv::Vec3b>(sizeY_ - y - 1, x) = cv::Vec3b(0, 0, 255); // 标记为红色
        //         }
        //     }
        // }   
        // {
        //     int debug = 0;
        // }
        // #ifdef USE_OPENMP
        // #pragma omp parallel for num_threads(8)
        // #endif
        for (int y = 0; y < sizeY_; ++y) {        
            for (int x = 0; x < sizeX_; ++x) {
                if (!voronoi_new[y * sizeX_ + x]) {
                    continue;  // 跳过非维诺图点
                }
                uint8_t dir_code = 0;
                int neighbors_num = getNumVoronoiNeighbors(x, y, dir_code);
                grid_adjs_[y * sizeX_ + x] = dir_code; 
                if (neighbors_num == 1) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Weak;  // 弱节点
                    week_grid_vec2.emplace_back(x, y);
                } else if (neighbors_num == 2) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Edge;  // 边
                } else if (neighbors_num >= 3) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Strong;  // 强节点
                } else {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::None;  // none
                    voronoi_new[y * sizeX_ + x] = false;  // 标记为非voronoi点
                }
            }
        }
        
        // std::cout << "--------在加入等高线后的voronoi图上进行搜索---------" << std::endl;
        stage_ = 2;
        for (int i = 0; i < week_grid_vec2.size(); ++i) {
            IntPoint start = week_grid_vec2[i];
            if ( parseDirection(grid_adjs_[start.y * sizeX_ + start.x]).empty()){
                continue;  // 已访问，跳过
            }
            // DFS 搜索
            graphs_.emplace_back(DFSSearch(start));
        }


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

private:
    // 扩展函数。要通过result.path的size()来判断是不是有扩展
    // 根据result.end_point来判断，如果end_point是无效值，则说明扩展失败；然后判断end_point是强or弱grid
    ExpansionResult expandGrid(const IntPoint& start, Direction dir) {
        ExpansionResult result;
        // result.path.path.emplace_back(start); // start和end都不加入到path里面，这样是方便结果路径的生成
        result.path.path_length = 0.0f;

        IntPoint current = start;
        while (true) {
            // 计算下一个点
            IntPoint direction = getDirection(dir);
            IntPoint next(current.x + direction.x, current.y + direction.y);

            // 检查边界
            if (next.x < 0 || next.x >= sizeX_ || next.y < 0 || next.y >= sizeY_) {
                break;
            }

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
                result.path.path_length += 1.0f;  // 假设网格的距离为 1，2连通一定是1
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

            {
                IntPoint check_point(273, 304);
                if(current == check_point)
                {
                    int debug = 0;
                }
            }

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

                // 检查是否已经被添加过了
                if (std::find(current_node->neighbors.begin(), current_node->neighbors.end(), end_node) 
                    == current_node->neighbors.end()) {
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
                if (next_type == 2) {
                    continue;
                } else {
                    stack.push(result.end_point);
                }
            }
        }
        return local_graph;
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
          node->parent = nullptr;
          node->node_state = gvg::GraphNode::NOT_EXPAND;
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
        path_node_pool_[use_node_num_++] = current_node;
        // current_node = start_node;
        current_node->parent = nullptr;
        current_node->g_score = 0.0;
        current_node->f_score = lambda_heu_ * getDiagHeu(current_node->pos, goal_node->pos);
        current_node->node_state = GraphNode::IN_OPEN_SET;
      
        open_set_.push(current_node);
        expanded_nodes_.insert(current_node->pos, current_node);
        GraphNode::Ptr terminate_node = nullptr;
        GraphNode::Ptr neighbor_node = nullptr;
        // 3. A*搜索
        while (!open_set_.empty()) {
            current_node = open_set_.top();

            if (current_node->pos == goal_node->pos) {
                terminate_node = current_node;
                retrievePath(terminate_node);
                return 1;
            }
            open_set_.pop();
            current_node->node_state = GraphNode::IN_CLOSE_SET;   
            iter_num_++;         

            // 扩展邻居
            for (int i = 0; i < current_node->neighbors.size(); ++i) {
                neighbor_node = current_node->neighbors[i];
                IntPoint neighborPoint = neighbor_node->pos;
                if(neighbor_node->type == GraphNode::Weak) continue;                
                auto expanded_node = expanded_nodes_.find(neighborPoint);
                if (expanded_node && expanded_node->node_state == GraphNode::IN_CLOSE_SET)
                    continue;

                double tentative_g_score = current_node->g_score + current_node->neighbor_paths[i].path_length + 1.0; // 假设网格的距离为 1
                double tentative_f_score = tentative_g_score + lambda_heu_ * getDiagHeu(neighborPoint, goal_node->pos);
                if(!expanded_node)
                {
                    path_node_pool_[use_node_num_++] = neighbor_node;
                    neighbor_node->g_score = tentative_g_score;
                    neighbor_node->f_score = tentative_f_score;
                    neighbor_node->parent = current_node;
                    neighbor_node->node_state = GraphNode::IN_OPEN_SET;
                    open_set_.push(neighbor_node);
                    expanded_nodes_.insert(neighborPoint, neighbor_node);
                    if (use_node_num_ == allocate_num_) {
                        std::cout << "A star on GVG run out of memory." << std::endl;
                        return 0;
                    }                    
                }else if(neighbor_node->node_state == GraphNode::IN_OPEN_SET)
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
        std::cout << "open set empty, no path!" << std::endl;
        std::cout << "use node num: " << use_node_num_ << std::endl;
        std::cout << "iter num: " << iter_num_ << std::endl;
        return 0;
    }

    void retrievePath(GraphNode::Ptr end_node) {
        GraphNode::Ptr cur_node = end_node;
        path_nodes_.emplace_back(cur_node);
      
        while (cur_node->parent != NULL) {
          cur_node = cur_node->parent;
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
                if (prev->neighbors[j] == curr) {
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


    std::vector<IntPoint> getVisitedNodes() {
        std::vector<IntPoint> visited;
        for(int i = 0; i < use_node_num_ - 1; ++i)
        {
            visited.emplace_back(path_node_pool_[i]->pos);
        }
        return visited;
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
#endif  // GVG_H