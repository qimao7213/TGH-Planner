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




#include <plan_manage/kino_replan_fsm.h>

namespace fast_planner {

void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);      //
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);//

  nh.param("fsm/waypoint_num", waypoint_num_, -1);
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }


  /* initialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);
  odom_record_timer_ = nh.createTimer(ros::Duration(0.2), &KinoReplanFSM::odomRecordCallback, this);
  topo_update_timer_ = nh.createTimer(ros::Duration(0.1), &KinoReplanFSM::TopoContainerUpdate, this);

  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);
  reset_srv_ = nh.advertiseService("/planning/reset_env", &fast_planner::KinoReplanFSM::reset_env, this);
}

void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  // 将终点存起来
  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
    end_yaw_ << tf::getYaw(msg->poses[0].pose.orientation), 0.0, 0.0;

  } 
  //相当于这里，即使设置了一系列waypoints，但是终点的切换还是手动完成的
  else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }
  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  planner_manager_->resetTopoPathContainer();
  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");//TRIG是指指定了终点
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

void KinoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  odom_pos_(0) = msg->pose.pose.position.x;
  odom_pos_(1) = msg->pose.pose.position.y;
  odom_pos_(2) = msg->pose.pose.position.z;

  odom_vel_(0) = msg->twist.twist.linear.x;
  odom_vel_(1) = msg->twist.twist.linear.y;
  odom_vel_(2) = msg->twist.twist.linear.z;

  odom_orient_.w() = msg->pose.pose.orientation.w;
  odom_orient_.x() = msg->pose.pose.orientation.x;
  odom_orient_.y() = msg->pose.pose.orientation.y;
  odom_orient_.z() = msg->pose.pose.orientation.z;

  have_odom_ = true;
}

void KinoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call) {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };
  int    pre_s        = int(exec_state_);
  exec_state_         = new_state;
  cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

void KinoReplanFSM::printFSMExecState() {
  string state_str[5] = { "INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ" };

  cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void KinoReplanFSM::execFSMCallback(const ros::TimerEvent& e) {
  static int fsm_num = 0;
  fsm_num++;
  if (fsm_num == 100) {
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    fsm_num = 0;
  }

  switch (exec_state_) {
    case INIT: {
      if (!have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFSMExecState(WAIT_TARGET, "FSM");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();
      t_cur                   = min(info->duration_, t_cur);

      // 为什么要计算这个pos？这是当前时间的轨迹的位置，也就是期望的？为什么不向前计算一点？
      // 因为轨迹就是带时间的，只要每个周期都执行好当前的就可以了。
      // 注意看，这个pos是轨迹上随着时间取的，而不是真正的位置
      // 所以在仿真中，即使机器人没动，这个pos也是一直在往前走。触发重规划
      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        return;

      } else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // cout << "near end" << endl;
        return;

      } 
      else if((t_cur - last_plan_time_ > 0.8))
      {
        last_plan_time_ = t_cur;
        changeFSMExecState(REPLAN_TRAJ, "FSM");//周期性地在进行重规划
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        // cout << "----------near start------------" << endl;
        return;

      } else {
        // cout << "-----------:" << (info->start_pos_ - pos).norm() << ", check?-----------------" << std::endl; 
        last_plan_time_ = t_cur;
        changeFSMExecState(REPLAN_TRAJ, "FSM");//周期性地在进行重规划
      }
      break;
    }

    case REPLAN_TRAJ: {
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();
      double         t_cur    = (time_now - info->start_time_).toSec();

      if(planner_manager_->only2D())
      {
        start_pt_  = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

        Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
        start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
        start_yaw_(1) = start_yaw_(2) = 0.0;

      }
      else
      {
        start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
        start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
        start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

        start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
        start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
        start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];        
      }


      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFSMExecState(EXEC_TRAJ, "FSM");
      } else {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

void KinoReplanFSM::odomRecordCallback(const ros::TimerEvent& e)
{
  if(!have_odom_) return;
  // 这个只能是-0.5，和topo路径搜索时的z轴一致
  Eigen::Vector3d odom_pos(odom_pos_.x(), odom_pos_.y(), -0.5);
  if(start_change_.empty())
    start_change_.emplace_back(odom_pos);
  else if((start_change_.back() - odom_pos).squaredNorm() > 0.09)
    start_change_.emplace_back(odom_pos);
}

void KinoReplanFSM::TopoContainerUpdate(const ros::TimerEvent& e)
{
  // ROS_WARN("Update TopoPath Container!");
  planner_manager_->topoUpdate(start_change_);
  start_change_.resize(0);
}

//当前的轨迹和新观察到的障碍物碰撞了
void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;
    auto only2D = planner_manager_->only2D();
    auto obj_predictor = planner_manager_->obj_predictor_;
    // DYNAMIC
    if(planner_manager_->pp_.dynamic_)
    {
      edt_env->setObjPrediction(obj_predictor->getPredictionTraj());
      edt_env->setObjScale(obj_predictor->getObjScale());
    }
    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_, only2D) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0, only2D);

    //当前的end_pt_！发生了碰撞，那么就在终点附近重新找一个最好的无碰撞的end_pt_
    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_, only2D) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0, only2D);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        end_pt_      = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  // 如果是轨迹发生了碰撞，那么就立即重新规划一条轨迹
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

bool KinoReplanFSM::callKinodynamicReplan() {
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_, start_yaw_, end_yaw_, start_change_);
  start_change_.resize(0);
  if (plan_success) {

    planner_manager_->planYaw(start_yaw_, end_yaw_);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id    = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();
    // std::cout << "position crt_pt size: " << bspline.pos_pts.size() << ", yaw crt_pt size: " << yaw_pts.rows() << std::endl;
    // std::cout << "position knots: " << knots.transpose() << std::endl;
    bspline_pub_.publish(bspline);
    ROS_WARN("Pub New Traj!!");
    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.09, Eigen::Vector4d(1, 0.8, 0.2, 0.9));
    visualization_->drawBspline(info->position_traj_, 0.08, Eigen::Vector4d(1.0, 0, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));
    visualization_->drawGuidePath(plan_data->topo_guide_path_, 0.075, Eigen::Vector4d(0.5, 0.5, 0.0, 1.0));

    visualization_->drawYawTraj(info->position_traj_, info->yaw_traj_, plan_data->dt_yaw_);
    visualization_->drawVisitedNodes(plan_data->visited_nodes);
    visualization_->drawSearchTree(plan_data->search_tree, -0.5, Eigen::Vector4d(0.0, 0.0, 0.0, 1.0));
    // // //画出时间优化前的B样条曲线，看看
    visualization_->drawBspline(info->position_traj_tmp_, 0.08, Eigen::Vector4d(0.0, 0, 0.0, 0.7), true, 0.2,
                                Eigen::Vector4d(0, 0, 0, 0.7), 1, 1, 1);
    visualization_->drawTopoGraph(plan_data->topo_graph_, 0.2, 0.05, Eigen::Vector4d(1.0, 0, 0.0, 1.0), 
                                  Eigen::Vector4d(0.0, 1, 0.0, 1.0), Eigen::Vector4d(0.0, 1, 0.0, 1.0));
    visualization_->drawTopoPathsPhase1(plan_data->topo_filtered_paths_, 0.07);
    visualization_->drawTopoPathsPhase2(plan_data->topo_select_paths_, 0.15);
    // visualization_->drawTopoSampleArea(plan_data->topo_sample_area_, 0.05, Eigen::Vector4d(0.5, 0.5, 0.5, 0.5));
    visualization_->drawPerceptionInfo(plan_data->block_pts_, 0.3);

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

bool KinoReplanFSM::reset_env(common_srvs::reset_env::Request &req, common_srvs::reset_env::Response &res)
{
  ROS_WARN("Receive Service Call!");
  this->planner_manager_->edt_environment_->sdf_map_->resetBuffer();
  std_msgs::Empty emt;
  this->new_pub_.publish(emt);
  res.success = true;
  return true;
}


// KinoReplanFSM::
}  // namespace fast_planner
