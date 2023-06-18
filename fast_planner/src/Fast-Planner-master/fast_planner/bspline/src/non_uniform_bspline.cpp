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



#include "bspline/non_uniform_bspline.h"
#include <ros/ros.h>

namespace fast_planner {

//初始化均匀b样条参数
NonUniformBspline::NonUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                     const double& interval) {
  setUniformBspline(points, order, interval);
}

NonUniformBspline::~NonUniformBspline() {}

void NonUniformBspline::setUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                          const double& interval) {
  //拟合的b样条控制点
  control_points_ = points;
  //次数
  p_              = order;
  //时间间隔
  interval_       = interval;

  n_ = points.rows() - 1;//n+1个控制点
  m_ = n_ + p_ + 1;//必须满足的公式

  //节点向量
  u_ = Eigen::VectorXd::Zero(m_ + 1);
  for (int i = 0; i <= m_; ++i) {
    
    //前p_+1个节点
    //例如p_为3， (-3+0)*interval_  (-3+1)*interval_ (-3+2)*interval_ (-3+3)*interval_
    if (i <= p_) 
    {
      u_(i) = double(-p_ + i) * interval_;
    } 
    //中间节点 正常
    else if (i > p_ && i <= m_ - p_) 
    {
      u_(i) = u_(i - 1) + interval_;
    } 
    //后面p_+1个节点
    else if (i > m_ - p_) 
    {
      u_(i) = u_(i - 1) + interval_;
    }
  }
}

void NonUniformBspline::setKnot(const Eigen::VectorXd& knot) { this->u_ = knot; }

Eigen::VectorXd NonUniformBspline::getKnot() { return this->u_; }

void NonUniformBspline::getTimeSpan(double& um, double& um_p) {
  um   = u_(p_);//起始时间
  um_p = u_(m_ - p_);//终止时间
}

Eigen::MatrixXd NonUniformBspline::getControlPoint() { return control_points_; }

pair<Eigen::VectorXd, Eigen::VectorXd> NonUniformBspline::getHeadTailPts() {
  Eigen::VectorXd head = evaluateDeBoor(u_(p_));
  Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_));
  return make_pair(head, tail);
}

//DeBoor算法
Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double& u) {

  //获得时间值
  /*
    先和起始时间比，取max，
  */
  double ub = min(max(u_(p_), u), u_(m_ - p_));

  // determine which [ui,ui+1] lay in
  int k = p_;//3

  /*
    代码通过循环确定参数u所处的区间，即确定区间的索引值k。
    循环条件是判断当前区间的上限节点值是否大于等于ub，如果是则跳出循环，此时k的值即为所在区间的索引。
  */
  while (true) 
  {
    if (u_(k + 1) >= ub) break;
    ++k;
  }

  /* deBoor's alg */
  vector<Eigen::VectorXd> d;
  for (int i = 0; i <= p_; ++i) {
    d.push_back(control_points_.row(k - p_ + i));
    // cout << d[i].transpose() << endl;
  }

  for (int r = 1; r <= p_; ++r) {
    for (int i = p_; i >= r; --i) {
      double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      // cout << "alpha: " << alpha << endl;
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  return d[p_];
}


//t=0
Eigen::VectorXd NonUniformBspline::evaluateDeBoorT(const double& t) {
  //t + u_(p_)就是整个时间
  return evaluateDeBoor(t + u_(p_));
}

//获得控制点导数
Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints() {
  // The derivative of a b-spline is also a b-spline, its order become p_-1
  // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());
  for (int i = 0; i < ctp.rows(); ++i) {
    ctp.row(i) =
        p_ * (control_points_.row(i + 1) - control_points_.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
  }
  return ctp;
}

//获得导数
NonUniformBspline NonUniformBspline::getDerivative() {
  Eigen::MatrixXd   ctp = getDerivativeControlPoints();
  NonUniformBspline derivative(ctp, p_ - 1, interval_);

  /* cut the first and last knot */
  Eigen::VectorXd knot(u_.rows() - 2);
  knot = u_.segment(1, u_.rows() - 2);
  derivative.setKnot(knot);

  return derivative;
}

double NonUniformBspline::getInterval() { return interval_; }

void NonUniformBspline::setPhysicalLimits(const double& vel, const double& acc) {
  limit_vel_   = vel;
  limit_acc_   = acc;
  limit_ratio_ = 1.1;//限制比列
}

//检查可行性(速度和加速度控制点)   show=false
bool NonUniformBspline::checkFeasibility(bool show) {

  bool fea = true;
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;

  Eigen::MatrixXd P         = control_points_;//获得控制点
  int             dimension = control_points_.cols();//获得列数也就是维数

  /* check vel feasibility and insert points */
  double max_vel = -1.0;
  //速度点=位置点-1
  for (int i = 0; i < P.rows() - 1; ++i) {
    //因为是非均匀B样条了，所以时间间隔变了 
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));//速度点公式
    
    //如果有超过最大值的 就置false
    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4) 
    {
      //测试以好久 怎么一次都没超过过 6
      if (show) 
      {
        ROS_ERROR("Infeasible vel");
        cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
      }
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }
    }
  }

  /* acc feasibility */
  //加速度类似
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {

    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {

      if (show) cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
      fea = false;

      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }
    }
  }

  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

  return fea;
}

double NonUniformBspline::checkRatio() {
  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  // find max vel
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    for (int j = 0; j < dimension; ++j) {
      max_vel = max(max_vel, fabs(vel(j)));
    }
  }
  // find max acc
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));
    for (int j = 0; j < dimension; ++j) {
      max_acc = max(max_acc, fabs(acc(j)));
    }
  }
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));
  ROS_ERROR_COND(ratio > 2.0, "max vel: %lf, max acc: %lf.", max_vel, max_acc);

  return ratio;
}


//可行性不行的话就进行时间重分配
bool NonUniformBspline::reallocateTime(bool show) {
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;
  // cout << "origin knots:\n" << u_.transpose() << endl;
  bool fea = true;

  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  double max_vel, max_acc;

  /* check vel feasibility and insert points */
  // Vi = p / (Ui+p+1 - Ui+1) * (Qi+1 - Qi)
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));

    //如果速度超过最大值
    if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
        fabs(vel(2)) > limit_vel_ + 1e-4)
    {

      fea = false;
      if (show) cout << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose() << endl;

      max_vel = -1.0;
      //找到速度最大值
      for (int j = 0; j < dimension; ++j) {
        max_vel = max(max_vel, fabs(vel(j)));
      }
      
      //获得超速比例   limit_ratio_=1.1
      double ratio = max_vel / limit_vel_ + 1e-4;
      //正解：防止μv和μa过大, 影响其它控制点, 这里引入limit_ratio_来做一个限制, 但这样不能保证一次调整后V,A一定满足要求, 因此需迭代调整.
      if (ratio > limit_ratio_) ratio = limit_ratio_;//这里最多设置为1.1可能是为了保证调整的过多(是不是？)

      //参考论文Knot Spans Adjustment这一部分公式
      double time_ori = u_(i + p_ + 1) - u_(i + 1);//原来的时间间隔
      double time_new = ratio * time_ori;//新的时间间隔
      double delta_t  = time_new - time_ori;//新旧时间差
      double t_inc    = delta_t / double(p_);// 时间调整共涉及p个节点区间, 这里将总的增量均匀地加到各个区间上

      for (int j = i + 2; j <= i + p_ + 1; ++j) {//将增量加到 Ui+2 ~ Ui+p+1 上
        u_(j) += double(j - i - 1) * t_inc;
        if (j <= 5 && j >= 1) {
          // cout << "vel j: " << j << endl;
        }
      }

      for (int j = i + p_ + 2; j < u_.rows(); ++j) {// 将后续的节点区间依次后移, 保证整条时间轴不冲突
        u_(j) += delta_t;
      }
    }
  }

  /* acc feasibility */
  // Ai = (p - 1) / (Ui+p+1 - Ui+2) * [ p / (Ui+p+2 - Ui+2) * (Qi+2 - Qi+1) - p / (Ui+p+1 - Ui+1) * (Qi+1 - Qi) ]
  for (int i = 0; i < P.rows() - 2; ++i) {

    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));

    if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
        fabs(acc(2)) > limit_acc_ + 1e-4) {

      fea = false;
      if (show) cout << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose() << endl;

      max_acc = -1.0;
      for (int j = 0; j < dimension; ++j) {
        max_acc = max(max_acc, fabs(acc(j)));
      }

      double ratio = sqrt(max_acc / limit_acc_) + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;
      // cout << "ratio: " << ratio << endl;

      double time_ori = u_(i + p_ + 1) - u_(i + 2);
      double time_new = ratio * time_ori;
      double delta_t  = time_new - time_ori;
      double t_inc    = delta_t / double(p_ - 1);

      // 对于起点附近的控制点, 强制延长时间的区间为U2 ~ U5(区间更长了), 为了让起始的速度, 加速度不那么激进?
      if (i == 1 || i == 2) {
        // cout << "acc i: " << i << endl;
        for (int j = 2; j <= 5; ++j) {
          u_(j) += double(j - 1) * t_inc;
        }

        for (int j = 6; j < u_.rows(); ++j) {
          u_(j) += 4.0 * t_inc;
        }

      } else {
        
        // 将增量加到 Ui+3 ~ Ui+p+1 上
        for (int j = i + 3; j <= i + p_ + 1; ++j) {
          u_(j) += double(j - i - 2) * t_inc;
          if (j <= 5 && j >= 1) {
            // cout << "acc j: " << j << endl;
          }
        }
        // 将后续的节点区间依次后移, 保证整条时间轴不冲突
        for (int j = i + p_ + 2; j < u_.rows(); ++j) {
          u_(j) += delta_t;
        }
      }
    }
  }

  return fea;
}

void NonUniformBspline::lengthenTime(const double& ratio) {
  int num1 = 5;
  int num2 = getKnot().rows() - 1 - 5;

  double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
  double t_inc   = delta_t / double(num2 - num1);
  for (int i = num1 + 1; i <= num2; ++i) u_(i) += double(i - num1) * t_inc;
  for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;
}

void NonUniformBspline::recomputeInit() {}

void NonUniformBspline::parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                              const vector<Eigen::Vector3d>& start_end_derivative,
                                              Eigen::MatrixXd&               ctrl_pts) {
  if (ts <= 0) {
    cout << "[B-spline]:time step error." << endl;
    return;
  }

  if (point_set.size() < 2) {
    cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    cout << "[B-spline]:derivatives error." << endl;
  }

  /*
    首先B样条曲线有这样的规定：有着N+1个控制点和节点向量(一般为时间节点)[t0,t1,t2......tM]并且为Pb次数的曲线满足：
    M=N+1+Pb，并且t属于[t(Pb),t(M-Pb)]；
    由于节点最大下标是M，所以节点数量为M+1=N+1+Pb+1
    也就是节点数量=控制点数量+阶数(次数+1)；
    N+1也就是控制点数量=M-Pb
  */
  /*
    这里我们已知Pb=3，离散点数量=K，也就是有K-1段轨迹，t属于[ts,tf],ts=t3，tf=t(M-Pb)下标就是控制点数量
    可以知道普通的均匀b样条是不会通过起点和终点，这时候就有一个特殊曲线称为clamped-B样条曲线
    把第一个点和最后一个点重复Pb+1次，也就值在头部，加3个一样的起点，在尾部加3个一样的终点，这样定义域也符合了，也能经过起点和终点了
    综上所述 tf=t(s+k-1)=t(k+2)=t(M-3)
    所以M=k+5 一共有K+6个节点， k+2个控制点
  */
  int K = point_set.size();

  // write A
  //默认都是列向量
  Eigen::Vector3d prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;
  vrow << -1, 0, 1;
  arow << 1, -2, 1;

  //k+2如上所述，k+4是k个离散点，加上起点和终点的速度以及加速度
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);


  /*
             1  4  1  0
    M4=1/6* -3  0  3  0
             3 -6  3  0
            -1  3 -3  1
  */

  //起始于i,i 大小为1行3列的矩阵

  /*
    以下推导经过本人手推检验，正确的，如果多天后回看不懂，那就滚回去看ipad上的笔记 🍀 真笨
  */
  for (int i = 0; i < K; ++i) A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

  //ts就是时间间隔
  A.block(K, 0, 1, 3)         = (1 / 2.0 / ts) * vrow.transpose();
  A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

  A.block(K + 2, 0, 1, 3)     = (1 / ts / ts) * arow.transpose();
  A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();
  // cout << "A:\n" << A << endl;

  // A.block(0, 0, K, K + 2) = (1 / 6.0) * A.block(0, 0, K, K + 2);
  // A.block(K, 0, 2, K + 2) = (1 / 2.0 / ts) * A.block(K, 0, 2, K + 2);
  // A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
  // A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

  // write b
  Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
  for (int i = 0; i < K; ++i) {
    bx(i) = point_set[i](0);
    by(i) = point_set[i](1);
    bz(i) = point_set[i](2);
  }

  for (int i = 0; i < 4; ++i) {
    bx(K + i) = start_end_derivative[i](0);
    by(K + i) = start_end_derivative[i](1);
    bz(K + i) = start_end_derivative[i](2);
  }

  // solve Ax = b
  //线性拟合
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // convert to control pts
  ctrl_pts.resize(K + 2, 3);
  ctrl_pts.col(0) = px;
  ctrl_pts.col(1) = py;
  ctrl_pts.col(2) = pz;

  // cout << "[B-spline]: parameterization ok." << endl;
}

//获得总时间
double NonUniformBspline::getTimeSum() {
  double tm, tmp;
  getTimeSpan(tm, tmp);
  return tmp - tm;
}

double NonUniformBspline::getLength(const double& res) {
  double          length = 0.0;
  double          dur    = getTimeSum();
  Eigen::VectorXd p_l    = evaluateDeBoorT(0.0), p_n;
  for (double t = res; t <= dur + 1e-4; t += res) {
    p_n = evaluateDeBoorT(t);
    length += (p_n - p_l).norm();
    p_l = p_n;
  }
  return length;
}

double NonUniformBspline::getJerk() {
  NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

  Eigen::VectorXd times     = jerk_traj.getKnot();
  Eigen::MatrixXd ctrl_pts  = jerk_traj.getControlPoint();
  int             dimension = ctrl_pts.cols();

  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    for (int j = 0; j < dimension; ++j) {
      jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
    }
  }

  return jerk;
}

void NonUniformBspline::getMeanAndMaxVel(double& mean_v, double& max_v) {
  NonUniformBspline vel = getDerivative();
  double            tm, tmp;
  vel.getTimeSpan(tm, tmp);

  double max_vel = -1.0, mean_vel = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
    double          vn  = vxd.norm();

    mean_vel += vn;
    ++num;
    if (vn > max_vel) {
      max_vel = vn;
    }
  }

  mean_vel = mean_vel / double(num);
  mean_v   = mean_vel;
  max_v    = max_vel;
}

void NonUniformBspline::getMeanAndMaxAcc(double& mean_a, double& max_a) {
  NonUniformBspline acc = getDerivative().getDerivative();
  double            tm, tmp;
  acc.getTimeSpan(tm, tmp);

  double max_acc = -1.0, mean_acc = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd axd = acc.evaluateDeBoor(t);
    double          an  = axd.norm();

    mean_acc += an;
    ++num;
    if (an > max_acc) {
      max_acc = an;
    }
  }

  mean_acc = mean_acc / double(num);
  mean_a   = mean_acc;
  max_a    = max_acc;
}
}  // namespace fast_planner
