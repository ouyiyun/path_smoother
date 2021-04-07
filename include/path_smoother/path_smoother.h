#ifndef PATH_SMOOTHER_PATH_SMOOTHER_H_
#define PATH_SMOOTHER_PATH_SMOOTHER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "path_smoother/vec2d.h"
#include "path_smoother/pose2d.h"
#include "path_smoother/dynamic_voronoi.h"

class PathSmoother
{
public:
  PathSmoother(std::string name, int size_x, int size_y);
  ~PathSmoother() = default;

  //! update grid map with occupancy grid map
  void updateMap(const nav_msgs::OccupancyGrid::Ptr& occMap);
  void updateMap(costmap_2d::Costmap2D* costmap);

  std::vector<geometry_msgs::Pose2D> getOriginalPath();
  void setOriginalPath(const std::vector<geometry_msgs::Pose2D>& path);

  std::vector<geometry_msgs::Pose2D> getSmoothedPath();

  void smoothPath();

private:
  //! method
  //! 障碍物项，用于约束路径远离障碍物
  Vec2d obstacleTerm(Vec2d xi);

  //! 曲率项，用于保证可转弯性及通行性
  Vec2d curvatureTerm(Vec2d xim1, Vec2d xi, Vec2d xip1);

  //! 平滑项，用于将节点等距分布并尽量保持同一个方向
  Vec2d smoothnessTerm(Vec2d xim2, Vec2d xim1, Vec2d xi, Vec2d xip1, Vec2d xip2);
  Vec2d smoothnessTerm(Vec2d xim1, Vec2d xi, Vec2d xip1);

  //! 尽量在路的中间
  Vec2d voronoiTerm(Vec2d xi);

  bool isOnGrid(Vec2d vec);

  //! 连续转离散
  bool coordToIndex(const Vec2d& coord, Vec2i& index);

  //! 离散转连续
  void indexToCoord(const Vec2i& index, Vec2d& coord);

  //! 平均曲率
  float pathAverageCurvature(const std::vector<Pose2d>& path);

private:
  /* data */
  //! 最大迭代次数
  int max_iterations_;
  //! maximum possible curvature of the non-holonomic vehicle
  float min_turning_radius_;
  float kappaMax_;
  //! maximum distance to obstacles that is penalized
  float obsDMax_;
  //！ maximum distance for obstacles to influence the voronoi field
  //! obsDMax的作用更显著，应该 vorObsDMax >= obsDMax。首先要不碰撞障碍物，在不碰的前提下，调整离障碍物的距离
  float vorObsDMax_;

  //! falloff rate for the voronoi field
  float alpha_ = 0.1;
  //! 权重
  float wObstacle_;
  float wVoronoi_;
  float wCurvature_;
  float wSmoothness_;

  //! occupancy grid map info
  int width_;   //! x
  int height_;  //! y
  float origin_x_;
  float origin_y_;
  float resolution_;

  //! Voronoi map
  DynamicVoronoi voronoi_;

  //! grid map
  bool** gridMap;

  //! path info
  std::vector<Pose2d> original_path_;
  std::vector<Pose2d> smoothed_path_;
};

#endif