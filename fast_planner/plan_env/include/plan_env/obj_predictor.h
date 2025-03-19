/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#ifndef _OBJ_PREDICTOR_H_
#define _OBJ_PREDICTOR_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <list>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using std::cout;
using std::endl;
using std::list;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class PolynomialPrediction;
typedef shared_ptr<vector<PolynomialPrediction>> ObjPrediction;
typedef shared_ptr<vector<Eigen::Vector3d>> ObjScale;

/* ========== prediction polynomial ========== */
// xyz三个方向上，都是5次多项式
class PolynomialPrediction {
public:
  vector<Eigen::Matrix<double, 6, 1>> polys;
  double t1, t2;  // start / end

public:
  PolynomialPrediction(/* args */) {
  }
  ~PolynomialPrediction() {
  }

  void setPolynomial(vector<Eigen::Matrix<double, 6, 1>>& pls) {
    polys = pls;
  }
  void setTime(double t1, double t2) {
    this->t1 = t1;
    this->t2 = t2;
  }

  bool valid() {
    return polys.size() == 3;
  }

  /* note that t should be in [t1, t2] */
  // 估计的位置？
  Eigen::Vector3d evaluate(double t) {
    Eigen::Matrix<double, 6, 1> tv;
    tv << 1.0, pow(t, 1), pow(t, 2), pow(t, 3), pow(t, 4), pow(t, 5);

    Eigen::Vector3d pt;
    pt(0) = tv.dot(polys[0]), pt(1) = tv.dot(polys[1]), pt(2) = tv.dot(polys[2]);

    return pt;
    
  }

  // 估计的速度？
  Eigen::Vector3d evaluateConstVel(double t) {
    Eigen::Matrix<double, 2, 1> tv;
    tv << 1.0, pow(t, 1);

    Eigen::Vector3d pt;
    pt(0) = tv.dot(polys[0].head(2)), pt(1) = tv.dot(polys[1].head(2)), pt(2) = tv.dot(polys[2].head(2));

    return pt;
  }
};

/* ========== subscribe and record object history ========== */
// 这是某一个动态物体的历史信息。记录的是x,y,z| t
class ObjHistory {
public:
  static int skip_num_;
  static int queue_size_;
  static ros::Time global_start_time_;

  ObjHistory() {
  }
  ~ObjHistory() {
  }

  void init(int id);

  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void clear() {
    history_.clear();
  }

  void getHistory(list<Eigen::Vector4d>& his) {
    his = history_;
  }

private:
  list<Eigen::Vector4d> history_;  // x,y,z;t
  int skip_;
  int obj_idx_;
  Eigen::Vector3d scale_;
};

/* ========== predict future trajectory using history ========== */
class ObjPredictor {
private:
  ros::NodeHandle node_handle_;

  int obj_num_;
  double lambda_;
  double predict_rate_;

  vector<ros::Subscriber> pose_subs_;
  ros::Subscriber marker_sub_;
  ros::Timer predict_timer_;
  vector<shared_ptr<ObjHistory>> obj_histories_;

  /* share data with planner */
  ObjPrediction predict_trajs_;//所有物体预测出来的轨迹多项式
  ObjScale obj_scale_;//所有物体的尺寸
  vector<bool> scale_init_;

  void markerCallback(const visualization_msgs::MarkerConstPtr& msg);

  void predictCallback(const ros::TimerEvent& e);
  //这是两个不同的预测方式吗？
  void predictPolyFit();//将物体的历史位置信息拟合为5次多项式
  void predictConstVel();//利用一个匀速模型来近似物体的运动信息。将物体上一时刻的速度当作当前的速度，然后继续积分一次得到预测的位置
  void visualizeObj(int id);
  Eigen::Vector3d getPositionAtTime(int idx, double t) {
    Eigen::Vector3d position;
    const std::vector<Eigen::Matrix<double, 6, 1>>& polys = (this->predict_trajs_)->at(idx).polys;
    t = (this->predict_trajs_)->at(idx).t2 + t;
    for (int dim = 0; dim < 3; ++dim) {
        // 计算多项式值
        position(dim) = polys[dim][0]  + polys[dim][1]  * t + polys[dim][2]  * t * t +
                        polys[dim][2]  * t * t * t + polys[dim][3]  * t * t * t * t +
                        polys[dim][4]  * t * t * t * t * t;
    }
    return position;
}


public:
  ObjPredictor(/* args */);
  ObjPredictor(ros::NodeHandle& node);
  ~ObjPredictor();

  void init();

  ObjPrediction getPredictionTraj();
  ObjScale getObjScale();

  typedef shared_ptr<ObjPredictor> Ptr;
};

}  // namespace fast_planner

#endif