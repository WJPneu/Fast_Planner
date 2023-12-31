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

//状态机
namespace fast_planner {

//初始化函数
void KinoReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_  = 0;
  exec_state_  = FSM_EXEC_STATE::INIT;
  have_target_ = false;
  have_odom_   = false;

  /*  fsm param  */
  nh.param("fsm/flight_type", target_type_, -1);  //1
  nh.param("fsm/thresh_replan", replan_thresh_, -1.0);  //1.5   重新规划阈值
  nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);  //2.0   不需要重新规划阈值

  nh.param("fsm/waypoint_num", waypoint_num_, -1); //2 这里不是太懂

  //waypoints_p[0]={19,0,1} waypoints_p[1]={-19,0,1}
  for (int i = 0; i < waypoint_num_; i++) {
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

  /* initialize main modules */
  /*
    reset() 函数的作用是释放先前指向的对象（如果存在），然后重新分配内存并将指针指向新的对象。
    可以认为是更新
  */
  planner_manager_.reset(new FastPlannerManager);
  //初始化规划模块(真是个多重嵌套的初始化🍀)
  planner_manager_->initPlanModules(nh);
  //可视化模块
  visualization_.reset(new PlanningVisualization(nh));

  /* callback */
  //状态机函数的回调(每10ms回调一次)
  exec_timer_   = nh.createTimer(ros::Duration(0.01), &KinoReplanFSM::execFSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &KinoReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &KinoReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &KinoReplanFSM::odometryCallback, this);

  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 10);

  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 10);//B样条信息发布
}

//由waypoint_generator.cpp接收终点并发出，由这个函数接收到终点(第一个数据就是终点)
void KinoReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses[0].pose.position.z < -0.1) return;

  cout << "Triggered!" << endl;
  trigger_ = true;

  //是手动的
  //这个z是1.0就很6
  if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

  } 
  //不是这个哦QAQ
  else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
    end_pt_(0)  = waypoints_[current_wp_][0];
    end_pt_(1)  = waypoints_[current_wp_][1];
    end_pt_(2)  = waypoints_[current_wp_][2];
    current_wp_ = (current_wp_ + 1) % waypoint_num_;
  }

  //画出目标？ yes QAQ
  visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  //目标速度设置为0
  end_vel_.setZero();
  //有了目标
  have_target_ = true;

  //有了目标立即转换状态为生成轨迹(这里就改变状态的好处就是及时改变状态，不需要等到FSM函数回调)
  if (exec_state_ == WAIT_TARGET)
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  //如果状态是再执行轨迹就改为重规划
  else if (exec_state_ == EXEC_TRAJ)
    changeFSMExecState(REPLAN_TRAJ, "TRIG");
}

//接收odom
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
    //1s打印一次状态
    printFSMExecState();
    if (!have_odom_) cout << "no odom." << endl;
    if (!trigger_) cout << "wait for goal." << endl;
    //重新置0
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
      //处于初始化状态，有odom和trigger后改变状态为WAIT_TARGET
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        //如果有了目标状态就变成GEN_NEW_TRAJ
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ: {
      start_pt_  = odom_pos_;
      //因为是开始 所及基本都是(0，0，0)
      start_vel_ = odom_vel_;
      start_acc_.setZero();

      //将四元数转换成矩阵再取第一列前三个数
      Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      //得到偏航角
      start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      //判断是否能完成整个的轨迹规划
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

    //执行轨迹
    case EXEC_TRAJ: {
      /* determine if need to replan */
      //决定是否学要重规划
      LocalTrajData* info     = &planner_manager_->local_data_;//取地址确实很省资源
      ros::Time      time_now = ros::Time::now();//记录此时
      double         t_cur    = (time_now - info->start_time_).toSec();//计算从混合A*开始到执行轨迹前的时间(这属于规划轨迹所消耗的时间)
      //info->duration_是轨迹规划出来的时间(也就是我规划出的这条轨迹要执行这么长时间)
      t_cur                   = min(info->duration_, t_cur);//一般来说info->duration_>t_cur ,因为程序规划时间也就不到1s,规划出来时间一般肯定不小于1s，除非路程很短。

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);//计算出t_cur的pos

      /* && (end_pt_ - pos).norm() < 0.5 */
      //如果当前时间都超过了执行时间，则将have_target置于false，将执行状态变为WAIT_TARGET
      if (t_cur > info->duration_ - 1e-2) {
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "FSM");
        ROS_ERROR("TIME OUT");
        return;

      } 
      //如果当前距离与终点距离小于不需要重规划的阈值，或者当前距离与起点距离小于阈值，那么直接返回
      //no_replan_thresh_=2
      else if ((end_pt_ - pos).norm() < no_replan_thresh_) {
        // cout << "near end" << endl;
        return;
      //replan_thresh_=1.5
      } else if ((info->start_pos_ - pos).norm() < replan_thresh_) {
        // cout << "near start" << endl;
        return;

      }
      //再不然就重规划
       else {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    //重规划
    case REPLAN_TRAJ: {
      LocalTrajData* info     = &planner_manager_->local_data_;
      ros::Time      time_now = ros::Time::now();//获得当前时间
      double         t_cur    = (time_now - info->start_time_).toSec();

      start_pt_  = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      //发送空消息
      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);
      //重新规划
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

/*
  这一执行器回调函数是用来判断目标点是否有障碍物或者是轨迹执行过程中是否有障碍物的。
  果目标点有障碍物，就在目标点周围通过离散的半径及角度循环来寻找新的安全的目标点。
  如果找到了，就直接改变状态进入REPLAN_TRAJ。
  如果目标点周围没有障碍物且目前状态是EXEX_TRAJ,则利用planner_manager的checkTrajCollision函数进行轨迹检查，
  如果轨迹不发生碰撞，则无事发生，如果轨迹碰撞，则状态变为REPLAN_TRAJ,进行重规划。
*/
void KinoReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  //开始的支撑在于有目标
  if (have_target_)
  {
    //获得edt环境
    auto edt_env = planner_manager_->edt_environment_;

    //dynamic_应该是false 所以获得是距离场地图的值
    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    //如果小于0.3
    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      //在周围找一个最大距离值的目标
      bool            new_goal = false;
      const double    dr = 0.5, dtheta = 30, dz = 0.3;
      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      //r=0.5, 1, 1.5, 2, 2.5
      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        //整个360 30度一划分
        for (double theta = -90; theta <= 270; theta += dtheta) {
          //0.3 0 0.3
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

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
      } 
      else {
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
  if (exec_state_ == FSM_EXEC_STATE::EXEC_TRAJ) {
    double dist;
    //不safe的距离值会存入dist 
    bool   safe = planner_manager_->checkTrajCollision(dist);
    
    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    }
  }
}

//混合A*搜索+
bool KinoReplanFSM::callKinodynamicReplan() {
  
  /*
    start_pt_=odom的
    start_vel_=(0，0，0)
    start_acc_=(0，0，0)
    end_pt_=自己点的(z默认为1)
    end_vel_=(0,0,0)
  */

  //这前段混合A*搜索，后端B样条优化都有
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  if (plan_success) {

    //yaw角规划
    planner_manager_->planYaw(start_yaw_);

    //获取到相关数据
    auto info = &planner_manager_->local_data_;

    /* publish traj */
    //自定义msg
    plan_manage::Bspline bspline;

    bspline.order      = 3;//3次
    bspline.start_time = info->start_time_;//开始混合A*规划的时间
    bspline.traj_id    = info->traj_id_;//每全部完成一次搜索 traj_id_+1

    //获得控制点
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      //存入pos_pts
      bspline.pos_pts.push_back(pt);
    }

    //获得时间节点
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    //获得yaw角的控制点
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getInterval();//yaw角的时间间隔

    //b样条信息发布出去
    bspline_pub_.publish(bspline);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;//取地址
    //kino_path_ 混合A*路径(0.01s为间隔)
    visualization_->drawGeometricPath(plan_data->kino_path_, 0.075, Eigen::Vector4d(0, 0, 1, 0.4));
    //优化后的B样条曲线
    visualization_->drawBspline(info->position_traj_, 0.1, Eigen::Vector4d(0.0, 1, 0.0, 1), true, 0.2,
                                Eigen::Vector4d(1, 0, 0, 1));

    return true;

  } else {
    cout << "generate new traj fail." << endl;
    return false;
  }
}

// KinoReplanFSM::
}  // namespace fast_planner
