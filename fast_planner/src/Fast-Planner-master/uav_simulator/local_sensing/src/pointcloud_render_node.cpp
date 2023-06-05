#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <pcl/search/impl/kdtree.hpp>
#include <vector>

using namespace std;
using namespace Eigen;

ros::Publisher pub_cloud;

//ros点云
sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub;

ros::Timer local_sensing_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

nav_msgs::Odometry _odom;

double sensing_horizon, sensing_rate, estimation_rate;
//地图三维方向上大小值
double _x_size, _y_size, _z_size;
//可以认为是偏移量(x,y都是地图一半大小 z=0s)
double _gl_xl, _gl_yl, _gl_zl;
//分辨率
double _resolution, _inv_resolution;
//最大最小珊格坐标
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;

ros::Time last_odom_stamp = ros::TIME_MAX;

//珊格坐标转世界坐标
inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;

  return pt;
};

//世界坐标转珊格坐标
inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0),
                    _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),
                    _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),
                    _GLZ_SIZE - 1);

  return idx;
};

//odom订阅
void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;
  _odom = odom;
}

pcl::PointCloud<pcl::PointXYZ> _cloud_all_map, _local_map;
/*
  体素化采样是一种常用的点云数据降采样方法，通过将点云划分为均匀的体素（三维像素）网格，
  然后在每个体素中选择一个代表性点，从而减少点云数据的数量。
  这种降采样方法可以在保留点云形状特征的同时，减少点的数量，从而提高点云处理的效率
*/
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
sensor_msgs::PointCloud2 _local_map_pcd;

pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
vector<int> _pointIdxRadiusSearch;
vector<float> _pointRadiusSquaredDistance;

//接收全局地图
void rcvGlobalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  if (has_global_map) return;

  ROS_WARN("Global Pointcloud received...");
  
  //pcl点云
  pcl::PointCloud<pcl::PointXYZ> cloud_input;
  //ros点云转换成pcl点云
  pcl::fromROSMsg(pointcloud_map, cloud_input);

  //网格大小
  _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
  //把转换好的pcl点云传入
  _voxel_sampler.setInputCloud(cloud_input.makeShared());
  //使用 _voxel_sampler.filter()方法对输入的点云数据进行体素化采样，并将结果存储在_cloud_all_map 中。
  _voxel_sampler.filter(_cloud_all_map);

  _kdtreeLocalMap.setInputCloud(_cloud_all_map.makeShared());

  has_global_map = true;
}

void renderSensedPoints(const ros::TimerEvent& event) {
  if (!has_global_map || !has_odom) return;

  Eigen::Quaterniond q;
  q.x() = _odom.pose.pose.orientation.x;
  q.y() = _odom.pose.pose.orientation.y;
  q.z() = _odom.pose.pose.orientation.z;
  q.w() = _odom.pose.pose.orientation.w;

  Eigen::Matrix3d rot;
  //四元数转换成矩阵
  rot = q;
  //把矩阵的第一列赋值给yaw_vec
  Eigen::Vector3d yaw_vec = rot.col(0);

  _local_map.points.clear();
  //设置中心点
  pcl::PointXYZ searchPoint(_odom.pose.pose.position.x,
                            _odom.pose.pose.position.y,
                            _odom.pose.pose.position.z);
  _pointIdxRadiusSearch.clear();
  _pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;
  //_pointIdxRadiusSearch:存储搜索半径内点的索引
  //_pointRadiusSquaredDistance：存储每个点与搜索点的距离的平方
  if (_kdtreeLocalMap.radiusSearch(searchPoint, sensing_horizon,
                                   _pointIdxRadiusSearch,
                                   _pointRadiusSquaredDistance) > 0) {
    //遍历每个点
    for (size_t i = 0; i < _pointIdxRadiusSearch.size(); ++i) {
      pt = _cloud_all_map.points[_pointIdxRadiusSearch[i]];

      if ((fabs(pt.z - _odom.pose.pose.position.z) / (sensing_horizon)) >
          tan(M_PI / 12.0))
        continue;

      //排除与航向相反的点(解释了地图只显示航向部分)
      //经过测试注释后确实显示的是前后都有了
      Vector3d pt_vec(pt.x - _odom.pose.pose.position.x,
                      pt.y - _odom.pose.pose.position.y,
                      pt.z - _odom.pose.pose.position.z);

      if (pt_vec.dot(yaw_vec) < 0) continue;

      _local_map.points.push_back(pt);
    }
  } else {
    return;
  }

  _local_map.width = _local_map.points.size();
  _local_map.height = 1;
  _local_map.is_dense = true;

  //得到了局部地图转换成ros点云
  pcl::toROSMsg(_local_map, _local_map_pcd);
  _local_map_pcd.header.frame_id = "map";

  //发布出去
  pub_cloud.publish(_local_map_pcd);
}


//🍀 啥也没干?
void rcvLocalPointCloudCallBack(
    const sensor_msgs::PointCloud2& pointcloud_map) {
  // do nothing, fix later
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", sensing_horizon);//5
  nh.getParam("sensing_rate", sensing_rate);//30
  nh.getParam("estimation_rate", estimation_rate);//30

  nh.getParam("map/x_size", _x_size);//40
  nh.getParam("map/y_size", _y_size);//20
  nh.getParam("map/z_size", _z_size);//5

  // subscribe point cloud
  //订阅全局地图
  global_map_sub = nh.subscribe("global_map", 1, rcvGlobalPointCloudCallBack);
  //订阅局部地图
  local_map_sub = nh.subscribe("local_map", 1, rcvLocalPointCloudCallBack);
  odom_sub = nh.subscribe("odometry", 50, rcvOdometryCallbck);

  // publisher depth image and color image
  pub_cloud =
      nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/cloud", 10);

  //1/30*2.5=0,083333.........
  double sensing_duration = 1.0 / sensing_rate * 2.5;

  //85ms左右执行一次
  local_sensing_timer =
      nh.createTimer(ros::Duration(sensing_duration), renderSensedPoints);

  _inv_resolution = 1.0 / _resolution;

  _gl_xl = -_x_size / 2.0;
  _gl_yl = -_y_size / 2.0;
  _gl_zl = 0.0;

  _GLX_SIZE = (int)(_x_size * _inv_resolution);
  _GLY_SIZE = (int)(_y_size * _inv_resolution);
  _GLZ_SIZE = (int)(_z_size * _inv_resolution);

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}
