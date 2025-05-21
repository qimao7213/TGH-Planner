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

// 定义哈希函数，用于 unordered_map
namespace std {
template <>
struct hash<IntPoint> {
    size_t operator()(const IntPoint& p) const {
        return hash<int>()(p.x) ^ hash<int>()(p.y);
    }
};
}

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
        directions.push_back(UP);
    }
    if (direction_code & DOWN) {
        directions.push_back(DOWN);
    }
    if (direction_code & RIGHT) {
        directions.push_back(RIGHT);
    }
    if (direction_code & LEFT) {
        directions.push_back(LEFT);
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
    std::vector<IntPoint> path;  // 路径上的点
    float path_length;           // 路径长度
};

// 定义图节点
struct GraphNode {
    enum NODE_TYPE { None = 0, Strong = 1, Weak = 2 };
    using Ptr = std::shared_ptr<GraphNode>;
    GraphNode(int x, int y, NODE_TYPE type_) : pos(x, y), type(type_) {}  
    
    
    IntPoint pos;                              // 节点坐标
    NODE_TYPE type = None;                     // 节点类型
    std::vector<GraphNode::Ptr> neighbors;     // 邻居节点
    std::vector<Path> neighbor_paths;          // 到邻居的路径
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
            sizeX_ = voronoi.getSizeX();
            sizeY_ = voronoi.getSizeY();
        }
        std::fill(grid_types_.begin(), grid_types_.end(), GRID_TYPE::None);
        std::fill(grid_adjs_.begin(), grid_adjs_.end(), (uint8_t)0);
        std::vector<IntPoint> week_grid_vec;

        for (int y = 0; y < sizeY_; ++y) {        
            for (int x = 0; x < sizeX_; ++x) {
                if (!voronoi.isVoronoi(x, y)) {
                    continue;  // 跳过非维诺图点
                }
                uint8_t dir_code = 0;
                int neighbors_num = getNumVoronoiNeighbors(voronoi, x, y, dir_code);
                grid_adjs_[y * sizeX_ + x] = dir_code; 
                if (neighbors_num == 1) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Weak;  // 弱节点
                    week_grid_vec.push_back(IntPoint(x, y));
                } else if (neighbors_num == 2) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Edge;  // 边
                } else if (neighbors_num >= 3) {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::Strong;  // 强节点
                } else {
                    grid_types_[y * sizeX_ + x] = GRID_TYPE::None;  // none
                }
            }
        }

        for (int i = 0; i < week_grid_vec.size(); ++i) {
            IntPoint start = week_grid_vec[i];
            if ( parseDirection(grid_adjs_[start.y * sizeX_ + start.x]).empty()){
                continue;  // 已访问，跳过
            }
            // DFS 搜索
            graphs_.push_back(DFSSearch(start));
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
        int debug = 0;

    }

    // 获取图
    const std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>>& getGraphs() const {
        return graphs_;
    }

private:
    // 扩展函数。要通过result.path的size()来判断是不是有扩展
    // 根据result.end_point来判断，如果end_point是无效值，则说明扩展失败；然后判断end_point是强or弱grid
    ExpansionResult expandGrid(const IntPoint& start, Direction dir) {
        ExpansionResult result;
        // result.path.path.push_back(start); // start和end都不加入到path里面，这样是方便结果路径的生成
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
            if (next_type == 0) 
            {
                std::cout << "ERROR! 应该不会返回维诺图以外的节点" << std::endl;
                break;
            }
            if (next_visited == 0) 
            {
                std::cout << "ERROR! 应该不会出现next_visited=0的情况" << std::endl;
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
                result.path.path.push_back(next);
                result.path.path_length += 1.0f;  // 假设网格的距离为 1，4连通一定是1
                current = next;
                // direction需要更新
                std::vector<Direction> next_dir = parseDirection(next_adjs);
                if (next_dir.size() == 1)
                {
                    dir = next_dir[0];
                }
                else 
                {
                    std::cout << "ERROR! 一个edge grid的dir是两个, 删除一个一定剩下一个" << std::endl;
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
                    current_node->neighbors.push_back(end_node);
                    current_node->neighbor_paths.push_back(result.path);
                    // 添加边到 end_node 的邻居列表（双向边）
                    end_node->neighbors.push_back(current_node);
                    // 创建反向路径
                    Path reverse_path = result.path;
                    std::reverse(reverse_path.path.begin(), reverse_path.path.end());  // 反转路径
                    end_node->neighbor_paths.push_back(reverse_path);                    
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

    int getNumVoronoiNeighbors(const DynamicVoronoi& voronoi, int x, int y, uint8_t& dir_code)
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

            if (voronoi.isVoronoi(nx, ny))
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
    enum GRID_TYPE { None = 0, Strong = 1, Weak = 2, Edge = 3 };

    std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>> graphs_;  // 图结构
    // std::vector<bool> grid_visited_;        // 访问标记 // 现在不要这个了，全部用grid_adjs_中剩下的邻居方向来表示是否还可以扩展
    std::vector<GRID_TYPE>  grid_types_;          // 网格类型, 0表示none，1表示强节点，2表示弱节点，3表示edge
    std::vector<uint8_t> grid_adjs_;        // 邻接方向
    std::vector<uint8_t> grid_adjs_origin_;        // 邻接方向，保留最初数据
    int sizeX_, sizeY_;  // 网格大小
};

#endif  // GVG_H