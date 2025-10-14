#ifndef DBSCAN2D_H
#define DBSCAN2D_H

#include <vector>
#include <cmath>
#include <queue>
#include "point.h"

enum DBSCANLabel {
    UNCLASSIFIED = -2,
    NOISE = -1
};

class DBSCAN2D {
public:
    DBSCAN2D(const std::vector<IntPoint>& points, float eps, int minPts)
        : points_(points), eps_(eps), minPts_(minPts), labels_(points.size(), UNCLASSIFIED) {}

    // 运行聚类，返回聚类总数
    int run(std::vector<std::vector<int>>& clusters) {
        int clusterId = 0;
        for (size_t i = 0; i < points_.size(); ++i) {
            if (labels_[i] == UNCLASSIFIED) {
                if (expandCluster(i, clusterId)) {
                    ++clusterId;
                }
            }
        }
        // 输出每个聚类包含的点编号
        clusters.clear();
        clusters.resize(clusterId);
        for (size_t i = 0; i < labels_.size(); ++i) {
            if (labels_[i] >= 0) {
                clusters[labels_[i]].push_back(i);
            }
        }
        return clusterId;
    }

    // 获取每个点的聚类标签（-1为噪声）
    const std::vector<int>& getLabels() const { return labels_; }

private:

    const std::vector<IntPoint>& points_;
    float eps_;
    int minPts_;
    std::vector<int> labels_;

    // 计算欧氏距离平方
    inline float dist2(const IntPoint& a, const IntPoint& b) const {
        int dx = a.x - b.x, dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    // 找到邻域内所有点的索引
    std::vector<int> regionQuery(int idx) const {
        std::vector<int> neighbors;
        for (size_t i = 0; i < points_.size(); ++i) {
            if (dist2(points_[idx], points_[i]) <= eps_ * eps_) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    }

    // 尝试扩展一个聚类
    bool expandCluster(int idx, int clusterId) {
        auto seeds = regionQuery(idx);
        if ((int)seeds.size() < minPts_) {
            labels_[idx] = NOISE;
            return false;
        }
        for (int seedIdx : seeds) {
            labels_[seedIdx] = clusterId;
        }
        std::queue<int> q;
        for (int seedIdx : seeds) {
            if (seedIdx != idx) q.push(seedIdx);
        }
        while (!q.empty()) {
            int curr = q.front(); q.pop();
            auto neighbors = regionQuery(curr);
            if ((int)neighbors.size() >= minPts_) {
                for (int nIdx : neighbors) {
                    if (labels_[nIdx] == UNCLASSIFIED || labels_[nIdx] == NOISE) {
                        if (labels_[nIdx] == UNCLASSIFIED)
                            q.push(nIdx);
                        labels_[nIdx] = clusterId;
                    }
                }
            }
        }
        return true;
    }
};

#endif // DBSCAN2D_H