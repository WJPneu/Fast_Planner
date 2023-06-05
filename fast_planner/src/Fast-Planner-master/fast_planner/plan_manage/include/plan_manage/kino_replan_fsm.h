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



#ifndef _KINO_REPLAN_FSM_H_
#define _KINO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace fast_planner {

//测试？
class Test {
private:
  /* data */
  int test_;
  std::vector<int> test_vec_;
  ros::NodeHandle nh_;

public:
  Test(const int& v) {
    test_ = v;
  }
  Test(ros::NodeHandle& node) {
    nh_ = node;
  }
  ~Test() {
  }
  void print() {
    std::cout << "test: " << test_ << std::endl;
  }
};

//大类(状态机函数)
/*
  FSM:有限状态机
*/
class KinoReplanFSM {

private:
  /* ---------- flag ---------- */
  //{初始状态，等待目标状态，生成新轨迹状态，重新规划轨迹状态，执行轨迹状态，重新规划新轨迹状态}
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  //{手动选择目标类型,预设目标类型,参考路径类型}
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  /* planning utils(规划工具) */
  //这里是独占所有权的智能指针
  FastPlannerManager::Ptr planner_manager_;//规划者
  PlanningVisualization::Ptr visualization_;//可视化

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_;

  /* planning data */
  bool trigger_, have_target_, have_odom_;
  FSM_EXEC_STATE exec_state_;//状态机

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;//四元数

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state(竟然有yaw角，6)
  Eigen::Vector3d end_pt_, end_vel_;                              // target state
  int current_wp_; //当前路径点的索引？

  /* ROS utils */
  // ROS工具
  ros::NodeHandle node_;//句柄
  ros::Timer exec_timer_, safety_timer_, vis_timer_, test_something_timer_;//时间计算
  ros::Subscriber waypoint_sub_, odom_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* helper functions */
  bool callKinodynamicReplan();        // front-end and back-end method
  bool callTopologicalTraj(int step);  // topo path guided gradient-based
                                       // optimization; 1: new, 2: replan
  //状态改变函数
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  //输出状态函数
  void printFSMExecState();

  /* ROS functions */
  void execFSMCallback(const ros::TimerEvent& e);
  void checkCollisionCallback(const ros::TimerEvent& e);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);

public:
  //构造和析构
  KinoReplanFSM(/* args */) {
  }
  ~KinoReplanFSM() {
  }


  //大头(初始化函数)
  void init(ros::NodeHandle& nh);

  /*
    是一个宏定义，用于为使用 Eigen 库的类定义自定义的 operator new 和 operator delete 函数，以实现内存对齐。
    用处：提高计算性能和避免潜在的对齐错误
  */
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif