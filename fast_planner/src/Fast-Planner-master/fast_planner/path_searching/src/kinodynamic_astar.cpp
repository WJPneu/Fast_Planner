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

#include <path_searching/kinodynamic_astar.h>
#include <sstream>
#include <plan_env/sdf_map.h>

using namespace std;
using namespace Eigen;

namespace fast_planner
{
KinodynamicAstar::~KinodynamicAstar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

//开始混合A*搜索 默认dynamic false ;  time_start -1
//init 初次传入为true
int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a,
                             Eigen::Vector3d end_pt, Eigen::Vector3d end_v, bool init, bool dynamic, double time_start)
{
  start_vel_ = start_v;
  start_acc_ = start_a;

  //用0来初始化起点我是没想到的 boyu zhou和 fei gao两位大佬的代码习惯确实有点不一样阿 
  PathNodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->state.head(3) = start_pt;
  cur_node->state.tail(3) = start_v;
  cur_node->index = posToIndex(start_pt);
  cur_node->g_score = 0.0;

  Eigen::VectorXd end_state(6);
  Eigen::Vector3i end_index;
  double time_to_goal;
  
  //初始化终点
  end_state.head(3) = end_pt;
  end_state.tail(3) = end_v;
  end_index = posToIndex(end_pt);
  //lambda_heu_=5 
  cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  //把起点->开集
  cur_node->node_state = IN_OPEN_SET;
  open_set_.push(cur_node);
  use_node_num_ += 1;

  //默认为false
  if (dynamic)
  {
    time_origin_ = time_start;//-1
    cur_node->time = time_start;
    cur_node->time_idx = timeToIndex(time_start);
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
    // cout << "time start: " << time_start << endl;
  }
  else
    //把点放入扩展集合
    expanded_nodes_.insert(cur_node->index, cur_node);

  //创建邻居节点和终点节点
  PathNodePtr neighbor = NULL;
  PathNodePtr terminate_node = NULL;
  //只有每次开始的时候是true
  bool init_search = init;
  //ceil向上取整
  const int tolerance = ceil(1 / resolution_);//10

  while (!open_set_.empty())
  { 
    //感觉写的就很      6
    cur_node = open_set_.top();

    // Terminate?
    //判断当前点是否距离起点已经7m
    bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
    // 判断当前点的珊格坐标是否与终点在10个格子以内,也就是在1m之内(到达终点)
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                    abs(cur_node->index(2) - end_index(2)) <= tolerance;
    
    //如果满足上述一个
    if (reach_horizon || near_end)
    {
      //把当前点->terminate_node
      terminate_node = cur_node;
      //获取路径
      retrievePath(terminate_node);
      //如果是接近终点
      if (near_end)
      {
        // Check whether shot traj exist
        estimateHeuristic(cur_node->state, end_state, time_to_goal);//再算一次h
        //这里会验证有没有一条曲线能直达终点，并且速度，加速度都不超过限制 ,如果没问题 那么is_shot_succ_会置true
        computeShotTraj(cur_node->state, end_state, time_to_goal);
        //这里是true
        if (init_search)
          ROS_ERROR("Shot in first search loop!");
      }
    }
    
    //就是一些输出     is_shot_succ_为真的前提就是要near_end 
    if (reach_horizon)
    {
      if (is_shot_succ_)
      { 
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else
      {
        std::cout << "reach horizon" << std::endl;
        return REACH_HORIZON;
      }
    }


    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      //接近了终点，但是没有shoot成功，并且当前点的父节点不是NULL也就是说这个点就是在终点附近的点(感觉还是有点问题)
      else if (cur_node->parent != NULL)
      {
        std::cout << "near end" << std::endl;
        return NEAR_END;
      }
      else
      {
        std::cout << "no path" << std::endl;
        return NO_PATH;
      }
    }
    //在开集中弹出
    open_set_.pop();
    //放入闭集
    cur_node->node_state = IN_CLOSE_SET;
    //计数用的吧？
    iter_num_ += 1;

    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;//初次搜索的时间分辨率,也就是把时间分成多少部分
    //记录cur_node->state
    Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
    Eigen::Matrix<double, 6, 1> pro_state;
    //扩展节点
    vector<PathNodePtr> tmp_expand_nodes;
    Eigen::Vector3d um;
    double pro_t;
    vector<Eigen::Vector3d> inputs;
    vector<double> durations;
    //init_search这里是true 但是不知道有什么用?好像表示初次搜索
    if (init_search)
    { 
      //开始的加速度传入
      inputs.push_back(start_acc_);
      //tau=1/20*0.8 把0.8分成20份   tau表示τ    🍀  所以max_tau_表示最大时间间隔？
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
        durations.push_back(tau);
      init_search = false;
    }
    else
    {
      //res=0.5  max_acc_=2 
      //ax= -max_acc_, -0.5*max_acc_, 0 ,0.5*max_acc_ ,  max_acc_ 
      //ay,az如上 那么就有125种搭配
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
          for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
          {
            um << ax, ay, az;
            inputs.push_back(um);
          }
        //time_res=1.0   max_tau_=0.6
      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
        //这不是只有一个？
        durations.push_back(tau);
    }

    // cout << "cur state:" << cur_state.head(3).transpose() << endl;
    //inputs.size()如果是初次搜寻，那么就是1   如果不是，那么就是125
    for (int i = 0; i < inputs.size(); ++i)
    //durations.size()如果是初次搜寻，那么就是20   如果不是，那么就是1
      for (int j = 0; j < durations.size(); ++j)
      {
        //得到输入(a)
        um = inputs[i];
        //得到时间
        double tau = durations[j];
        //算出一定加速度输入以及时间后的位置和速度
        stateTransit(cur_state, pro_state, um, tau);
        //这个pro_t是干嘛用的(如果不是dynamic就不用管他)
        pro_t = cur_node->time + tau;

        //这里的pro_pos就是前向积分后的点
        Eigen::Vector3d pro_pos = pro_state.head(3);

        // Check if in close set
        Eigen::Vector3i pro_id = posToIndex(pro_pos);
        int pro_t_id = timeToIndex(pro_t);
        //dynamic为false所以
        PathNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        //在扩展集合中，并且在闭集中
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          //在闭集中 就没必要操作了
          continue;
        }

        // Check maximal velocity
        //  得到速度
        Eigen::Vector3d pro_v = pro_state.tail(3);
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
        {
          if (init_search)
            std::cout << "vel" << std::endl;
          continue;
        }

        // Check not in the same voxel
        Eigen::Vector3i diff = pro_id - cur_node->index;
        int diff_time = pro_t_id - cur_node->time_idx;
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)
            std::cout << "same" << std::endl;
          continue;
        }

        // Check safety
        // 安全检测
        Eigen::Vector3d pos;
        Eigen::Matrix<double, 6, 1> xt;
        bool is_occ = false;
        //check_num_=5
        for (int k = 1; k <= check_num_; ++k)
        {
          //把tau分成5份
          double dt = tau * double(k) / double(check_num_);
          //再算出位置和速度
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(3);
          if (edt_environment_->sdf_map_->getInflateOccupancy(pos) == 1 )
          {
            is_occ = true;
            break;
          }
        }
        if (is_occ)
        {
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }

        double time_to_goal, tmp_g_score, tmp_f_score;
        //g_score=(um.squaredNorm() + w_time_)* tau,也就是(合加速度+10)*时间,因为这个宗旨就是能量损耗最小，
        //这个时间与加速度的乘积也正好可以代替能量的消耗

        //算出g和f
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // Compare nodes expanded from the same parent
        bool prune = false;
        for (int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          PathNodePtr expand_node = tmp_expand_nodes[j];
          //想不明白 为什么 (好像可能是由于节点更新的原因)
          if ((pro_id - expand_node->index).norm() == 0 && ((!dynamic) || pro_t_id == expand_node->time_idx))
          {
            prune = true;
            if (tmp_f_score < expand_node->f_score)
            {
              expand_node->f_score = tmp_f_score;
              expand_node->g_score = tmp_g_score;
              expand_node->state = pro_state;
              expand_node->input = um;
              expand_node->duration = tau;
              if (dynamic)
                expand_node->time = cur_node->time + tau;
            }
            break;
          }
        }

        // This node end up in a voxel different from others
        //如果不用剪枝
        if (!prune)
        {
          //如果节点不在拓展集合
          if (pro_node == NULL)
          {
            pro_node = path_node_pool_[use_node_num_];
            pro_node->index = pro_id;
            pro_node->state = pro_state;
            pro_node->f_score = tmp_f_score;
            pro_node->g_score = tmp_g_score;
            pro_node->input = um;
            pro_node->duration = tau;
            pro_node->parent = cur_node;
            pro_node->node_state = IN_OPEN_SET;
            //false 不用管
            if (dynamic)
            {
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            open_set_.push(pro_node);

            if (dynamic)
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
              //放入拓展集合
            else
              expanded_nodes_.insert(pro_id, pro_node);

            //放入临时拓展集合
            tmp_expand_nodes.push_back(pro_node);

            //标记+1
            use_node_num_ += 1;
            //超过100000也是厉害了
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          }
          //如果在开集中
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            //更新
            if (tmp_g_score < pro_node->g_score)
            {
              // pro_node->index = pro_id;
              pro_node->state = pro_state;
              pro_node->f_score = tmp_f_score;
              pro_node->g_score = tmp_g_score;
              pro_node->input = um;
              pro_node->duration = tau;
              pro_node->parent = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
    // init_search = false;
  }

  cout << "open set empty, no path!" << endl;
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

void KinodynamicAstar::setParam(ros::NodeHandle& nh)
{
  nh.param("search/max_tau", max_tau_, -1.0);//0.6
  nh.param("search/init_max_tau", init_max_tau_, -1.0);//0.8
  nh.param("search/max_vel", max_vel_, -1.0);//3
  nh.param("search/max_acc", max_acc_, -1.0);//2
  nh.param("search/w_time", w_time_, -1.0);//10
  nh.param("search/horizon", horizon_, -1.0);//7
  nh.param("search/resolution_astar", resolution_, -1.0);//0.1
  nh.param("search/time_resolution", time_resolution_, -1.0);//0.8
  nh.param("search/lambda_heu", lambda_heu_, -1.0);//5
  nh.param("search/allocate_num", allocate_num_, -1);//100000
  nh.param("search/check_num", check_num_, -1);//5
  nh.param("search/optimistic", optimistic_, true);//这个launch好像没有默认为true

  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);//为0
  max_vel_ += vel_margin;//3
}

//获取路径
void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
  PathNodePtr cur_node = end_node;
  path_nodes_.push_back(cur_node);

  while (cur_node->parent != NULL)
  {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node);
  }

  reverse(path_nodes_.begin(), path_nodes_.end());
}

//x1 x2 都是6行1列的向量
double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
  //x,y,z方向上的差值
  const Vector3d dp = x2.head(3) - x1.head(3);
  //segment(3, 3)表示从第三个元素开始取3个向量
  const Vector3d v0 = x1.segment(3, 3);
  const Vector3d v1 = x2.segment(3, 3);

  //点乘
  double c1 = -36 * dp.dot(dp);  
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
  double c4 = 0; 
  //10 时间权重
  double c5 = w_time_;

  //通过费拉里法求根  quartic代表4次
  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);
  
  // std::vector<double> ts_get_value = get_value(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;//3*0.5=1.5
  //这个t_bar是啥？我猜测:这里是OBVP求解能求出一个最优解t，使得能量最小，但是并没有考虑速度的约束，所以这里用极限时间t_bar来限制
  /*
    这里是找到x,y,z三维距离的最大值/v_max
  */  
  double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);
  // ts_get_value.push_back(t_bar);

  double cost = 100000000;
  // double cost_get_value=100000000;
  //把t_bar赋值给t_d
  double t_d = t_bar;
  // double t_d_get_value=t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    //这个是按照hw4.pdf中J化简之后的结果 没问题的 自己算一下就行
    double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }
  
  // for (auto s : ts_get_value)
  // {
  //   if (s < t_bar)
  //     continue;
  //   double c = -c1 / (3 * s * s * s) - c2 / (2 * s * s) - c3 / s + w_time_ * s;
  //   if (c < cost_get_value)
  //   {
  //     cost_get_value = c;
  //     t_d_get_value = s;
  //   }
  // }
  // ROS_ERROR("t_d is %f,  t_d_get_value is %f ",t_d,t_d_get_value);
  // ROS_ERROR("cost is %f,  cost_get_value is %f ",cost,cost_get_value);
  optimal_time = t_d;
  
  return 1.0 * (1 + tie_breaker_) * cost;
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  /* ---------- get coefficient ---------- */
  const Vector3d p0 = state1.head(3);
  const Vector3d dp = state2.head(3) - p0;
  const Vector3d v0 = state1.segment(3, 3);
  const Vector3d v1 = state2.segment(3, 3);
  const Vector3d dv = v1 - v0;
  //庞特里亚金最优时间
  double t_d = time_to_goal;
  //三行四列
  /*
    * * * *
    * * * *
    * * * *
  */
  MatrixXd coef(3, 4);
  //把末尾速度给end_vel_(0,0,0);
  end_vel_ = v1;


  Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);///1/6 * alpha
  Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);//1/2 * beta
  Vector3d c = v0;//v_uc
  Vector3d d = p0;//p_uc

  // / * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  Vector3d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector3i index;

  Eigen::MatrixXd Tm(4, 4);
  /*
    0 1 0 0 
    0 0 2 0
    0 0 0 3
    0 0 0 0
  */
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  /* ---------- forward checking of trajectory ---------- */
  //把时间分成10份
  double t_delta = t_d / 10;
  for (double time = t_delta; time <= t_d; time += t_delta)
  {
    /*
      [0 0 0 0]T
    */
    t = VectorXd::Zero(4);
    
    for (int j = 0; j < 4; j++)
      //T^0 T^1 T^2 T^3
      t(j) = pow(time, j);

    for (int dim = 0; dim < 3; dim++)
    {
      //0，1，2行代表 x，y，z
      poly1d = coef.row(dim);
      //计算出位置，速度，加速度
      coord(dim) = poly1d.dot(t);
      vel(dim) = (Tm * poly1d).dot(t);
      acc(dim) = (Tm * Tm * poly1d).dot(t);
      

      //怎么不限制了？
      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    //如果这个点超过地图界限 则pass
    if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
        coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
    {
      return false;
    }

    // if (edt_environment_->evaluateCoarseEDT(coord, -1.0) <= margin_) {
    //   return false;
    // }
    //如果这个点是障碍物 PASS
    if (edt_environment_->sdf_map_->getInflateOccupancy(coord) == 1)
    {
      return false;
    }
  }

  coef_shot_ = coef;
  t_shot_ = t_d;
  is_shot_succ_ = true;
  return true;
}
/*
  a,b,c,d就是三次函数的三次，二次，一次，零次的系数
*/
vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  //这里的D是判别式
  double D = Q * Q * Q + R * R;
  //如果D>0则有一个实根和两个共轭复根； 
  if (D > 0)
  {
    //cbrt()是立方根
    //这里-a2 / 3 + (S + T)对应的就是https://zh.wikipedia.org/wiki/%E4%B8%89%E6%AC%A1%E6%96%B9%E7%A8%8B 中求根公式x1；
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  //当判别式为0 是三个实根，细看x1,x2,x3
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    //对于判别式小于0，运用三角函数求解
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

//四次求根
vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  /*
    这里的a是10 为啥要除以10呢?(chatgpt说是时间权重)
    这里 a0 a1 a2 a3对应t的0，1，2，3次
    a3=0
  */
  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  //嵌套三次(这里得到是三次函数的所有实根)
  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

vector<double> KinodynamicAstar::get_value(double a, double b, double c, double d, double e)
{
    vector<double> s;
    double c3 = b / a;
    double c2 = c / a;
    double c1 = d / a;
    double c0 = e / a;
    Eigen::Matrix<double,4,4> m;

    m << 0,0,0,-c0,
         1,0,0,-c1,
         0,1,0,-c2,
         0,0,1,c3;

    Eigen::Matrix<complex<double>,Eigen::Dynamic,Eigen::Dynamic> eigenValules;
    eigenValules = m.eigenvalues();


    for(int i=0; i<4; ++i){
        double T = std::real(eigenValules(i));
        double img = std::imag(eigenValules(i));

        if(T <= 0 || std::abs(img) >= 1e-16){
            continue;
        }

        s.push_back(T);


    }
    return s;
}



void KinodynamicAstar::init()
{
  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;//10
  inv_time_resolution_ = 1.0 / time_resolution_;//1.25
  //从SDF地图中获得原点和地图大小给混合A*使用
  edt_environment_->sdf_map_->getRegion(origin_, map_size_3d_);

  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  /* ---------- pre-allocated node ---------- */
  //这应该是节点集合，初始化为100000？🍀 说实话，必然造成浪费QAQ
  path_node_pool_.resize(allocate_num_);
  //这样初始化？
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new PathNode;
  }
  //把状态转移矩阵变成单位矩阵
  phi_ = Eigen::MatrixXd::Identity(6, 6);
  use_node_num_ = 0;
  iter_num_ = 0;
}

void KinodynamicAstar::setEnvironment(const EDTEnvironment::Ptr& env)
{
  this->edt_environment_ = env;
}

//重置函数
void KinodynamicAstar::reset()
{ 
  //扩展点清理
  expanded_nodes_.clear();
  //路径点清理
  path_nodes_.clear();

  std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
  //swap是交换数据，其实就是把open_set_清空了
  open_set_.swap(empty_queue);

  //第一次use_node_num_是0
  for (int i = 0; i < use_node_num_; i++)
  {
    PathNodePtr node = path_node_pool_[i];
    node->parent = NULL;
    node->node_state = NOT_EXPAND;
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}

//delta_t=0.01
//以0.01s为间隔 得到离散的轨迹点(世界坐标)
std::vector<Eigen::Vector3d> KinodynamicAstar::getKinoTraj(double delta_t)
{
  vector<Vector3d> state_list;

  /* ---------- get traj of searching ---------- */
  //path_nodes_就是路径点，由于hoizen为7m，规划到离起点7m时就停止规划，因此这个点可能不是终点
  PathNodePtr node = path_nodes_.back();//这是容器最后一个点
  Matrix<double, 6, 1> x0, xt;

  while (node->parent != NULL)
  {
    Vector3d ut = node->input;
    double duration = node->duration;
    x0 = node->parent->state;

    //时间回溯 delta_t=0.01
    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      stateTransit(x0, xt, ut, t);
      //得到pos
      state_list.push_back(xt.head(3));
    }
    node = node->parent;
  }
  reverse(state_list.begin(), state_list.end());
  /* ---------- get traj of one shot ---------- */
  if (is_shot_succ_)
  {
    Vector3d coord;
    VectorXd poly1d, time(4);

    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  return state_list;
}

//获取样本  ts=0.5/3=0.1666666........(最小时间)
void KinodynamicAstar::getSamples(double& ts, vector<Eigen::Vector3d>& point_set,
                                  vector<Eigen::Vector3d>& start_end_derivatives)
{
  /* ---------- path duration ---------- */
  double T_sum = 0.0;
  //如果shot成功就是找到和终点的连线就加上这段时间t_shot_
  if (is_shot_succ_)
    T_sum += t_shot_;
  //找到这条路径的最后一个点(ps:不一定是终点)
  PathNodePtr node = path_nodes_.back();
  //把整条路径的时间加起来
  while (node->parent != NULL)
  {
    T_sum += node->duration;//这个时间就是离散输入后规定的时间，不是OBVP计算得到的时间 记得区分
    node = node->parent;
  }
  // cout << "duration:" << T_sum << endl;

  // Calculate boundary vel and acc(获得终点的速度和加速度)
  Eigen::Vector3d end_vel, end_acc;
  double t;
  if (is_shot_succ_)
  {
    t = t_shot_;
    //(0,0,0)
    end_vel = end_vel_;
    for (int dim = 0; dim < 3; ++dim)
    { 
      //多项式系数
      Vector4d coe = coef_shot_.row(dim);
      //得到终点加速度
      end_acc(dim) = 2 * coe(2) + 6 * coe(3) * t_shot_;
    }
  }
  else
  {
    t = path_nodes_.back()->duration;
    end_vel = node->state.tail(3);
    end_acc = path_nodes_.back()->input;
  }

  // Get point samples  T_sum/0.166666666.......
  int seg_num = floor(T_sum / ts);
  //最少八段
  seg_num = max(8, seg_num);
  ts = T_sum / double(seg_num);
  bool sample_shot_traj = is_shot_succ_;
  node = path_nodes_.back();

  
  for (double ti = T_sum; ti > -1e-5; ti -= ts)
  {
    //如果连接终点成功,就把
    if (sample_shot_traj)
    {
      // samples on shot traj
      Vector3d coord;
      Vector4d poly1d, time;

      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 3; dim++)
      {
        //获得点坐标
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }

      point_set.push_back(coord);
      t -= ts;

      /* end of segment */
      //也就是t<0
      if (t < -1e-5)
      {
        sample_shot_traj = false;
        if (node->parent != NULL)
          t += node->duration;
      }
    }
    else
    {
      // samples on searched traj
      Eigen::Matrix<double, 6, 1> x0 = node->parent->state;
      Eigen::Matrix<double, 6, 1> xt;
      Vector3d ut = node->input;

      stateTransit(x0, xt, ut, t);

      point_set.push_back(xt.head(3));
      t -= ts;

      // cout << "t: " << t << ", t acc: " << T_accumulate << endl;
      //这种操作就是把时间均分,由于每段时间与时间间隔不一定是倍数关系，这样就可以使得均分时间线的同时，也再每一段计算出点坐标
      if (t < -1e-5 && node->parent->parent != NULL)
      {
        node = node->parent;
        t += node->duration;
      }
    }
  }
  reverse(point_set.begin(), point_set.end());

  // calculate start acc
  Eigen::Vector3d start_acc;
  if (path_nodes_.back()->parent == NULL)
  {
    // no searched traj, calculate by shot traj
    //如果shoot成功
    start_acc = 2 * coef_shot_.col(2);
  }
  else
  {
    // input of searched traj
    start_acc = node->input;
  }

  start_end_derivatives.push_back(start_vel_);
  start_end_derivatives.push_back(end_vel);
  start_end_derivatives.push_back(start_acc);
  start_end_derivatives.push_back(end_acc);
}

std::vector<PathNodePtr> KinodynamicAstar::getVisitedNodes()
{
  vector<PathNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
  Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();

  // idx << floor((pt(0) - origin_(0)) * inv_resolution_), floor((pt(1) -
  // origin_(1)) * inv_resolution_),
  //     floor((pt(2) - origin_(2)) * inv_resolution_);

  return idx;
}

int KinodynamicAstar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
  //phi_状态转移矩阵 6x6
  //根据输入um(ax,ay,az)计算出一定时间tau之后的位置    ps:就是匀加速运动的公式
  for (int i = 0; i < 3; ++i)
    phi_(i, i + 3) = tau;

  Eigen::Matrix<double, 6, 1> integral;
  integral.head(3) = 0.5 * pow(tau, 2) * um;
  integral.tail(3) = tau * um;

  state1 = phi_ * state0 + integral;
}

}  // namespace fast_planner
