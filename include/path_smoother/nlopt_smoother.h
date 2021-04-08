#ifndef PATH_SMOOTHER_NLOPT_SMOOTHER_H_
#define PATH_SMOOTHER_NLOPT_SMOOTHER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>

#include <nlopt.hpp>

#include "path_smoother/vec2d.h"
#include "path_smoother/pose2d.h"
#include "path_smoother/dynamic_voronoi.h"

class NLoptSmoother
{
public:
  NLoptSmoother(std::string name, int size_x, int size_y);
  ~NLoptSmoother() = default;

  //! update grid map with occupancy grid map
  void updateMap(const nav_msgs::OccupancyGrid::Ptr& occMap);
  void updateMap(costmap_2d::Costmap2D* costmap);

  std::vector<geometry_msgs::Pose2D> getOriginalPath();
  void setOriginalPath(const std::vector<geometry_msgs::Pose2D>& path);

  std::vector<geometry_msgs::Pose2D> getSmoothedPath();

  void smoothPath();

private:
  //! method
  //! NLopt cost
  static double costFunction(const std::vector<double>& points, std::vector<double>& grad, void* func_data);

  //! calculate each part of cost function with points
  void combineCost(const std::vector<double>& points, std::vector<double>& grad, double& f_commbin);

  //! 平滑
  void calcSmoothness(const std::vector<Vec2d>& points, std::vector<Vec2d>& gradient, double& cost);

  //! 到障碍物的距离
  void calcObsDistance(const std::vector<Vec2d>& points, std::vector<Vec2d>& gradient, double& cost);

  //! 维诺图
  void calcVornoiField(const std::vector<Vec2d>& points, std::vector<Vec2d>& gradient, double& cost);

  //! 离点最近的障碍物点及相应距离
  //! 若xi在地图外，最近点为其本身，距离为0
  void nearestObsDistance(const Vec2d& xi, Vec2d& oi, double& dist);

  void nearestEdgeDistance(const Vec2d& xi, Vec2d& vi, double& dist);

  bool isOnGrid(Vec2d vec);

  //! 连续转离散
  bool coordToIndex(const Vec2d& coord, Vec2i& index);

  //! 离散转连续
  void indexToCoord(const Vec2i& index, Vec2d& coord);

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

  //! nlopt 参数
  int iter_nums_;
  double min_cost_;
  int points_num_;

  //! for evaluation
  std::vector<double> vec_cost_;
  std::vector<double> best_variable_;
  //   std::vector<double> vec_time_;

  Vec2d first_point_;
  Vec2d last_point_;
};

#endif