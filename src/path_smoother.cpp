#include "path_smoother/path_smoother.h"
#include "path_smoother/math_utils.h"

PathSmoother::PathSmoother(std::string name, int size_x, int size_y)
{
  ros::NodeHandle private_nh("~/" + name);
  // ros::NodeHandle private_nh("~");

  private_nh.param<int>("max_iterations", max_iterations_, 50);
  private_nh.param<float>("min_turning_radius", min_turning_radius_, 5.0);
  kappaMax_ = 1.f / (min_turning_radius_ * 1.1);
  private_nh.param<float>("max_obs_distance", obsDMax_, 1.5);
  private_nh.param<float>("max_vor_obs_distance", vorObsDMax_, 2.0);
  private_nh.param<float>("weight_obstacle", wObstacle_, 0.2);
  private_nh.param<float>("weight_voronoi", wVoronoi_, 0.2);
  private_nh.param<float>("weight_curvature", wCurvature_, 0.0);
  private_nh.param<float>("weight_smoothness", wSmoothness_, 0.6);
  ROS_INFO("max_iterations: %d", max_iterations_);
  ROS_INFO("max_obs_distance: %lf", obsDMax_);
  ROS_INFO("max_vor_obs_distance: %lf", vorObsDMax_);
  ROS_INFO("weight_obstacle: %lf", wObstacle_);
  ROS_INFO("weight_voronoi: %lf", wVoronoi_);
  ROS_INFO("weight_curvature: %lf", wCurvature_);
  ROS_INFO("weight_smoothness: %lf", wSmoothness_);

  //! no change
  width_ = size_x;
  height_ = size_y;

  gridMap = new bool*[size_x];
  for (int x = 0; x < size_x; ++x)
  {
    gridMap[x] = new bool[size_y];
  }

  ROS_INFO("PathSmoothe initialized!");
}

void PathSmoother::updateMap(const nav_msgs::OccupancyGrid::Ptr& occMap)
{
  origin_x_ = occMap->info.origin.position.x;
  origin_y_ = occMap->info.origin.position.y;
  resolution_ = occMap->info.resolution;

  for (int x = 0; x < width_; ++x)
  {
    for (int y = 0; y < height_; ++y)
    {
      gridMap[x][y] = occMap->data[y * width_ + x] ? true : false;
    }
  }

  voronoi_.initializeMap(width_, height_, gridMap);
  voronoi_.update();
  voronoi_.prune();
  voronoi_.CollectVoronoiEdgePoints();
  // voronoi_.visualize("/home/chelizi/biyesheji_ws/src/path_smoother/voronoi.ppm");
}

void PathSmoother::updateMap(costmap_2d::Costmap2D* costmap)
{
  origin_x_ = costmap->getOriginX();
  origin_y_ = costmap->getOriginY();
  resolution_ = costmap->getResolution();

  for (int x = 0; x < width_; ++x)
  {
    for (int y = 0; y < height_; ++y)
    {
      gridMap[x][y] = costmap->getCost(x, y) == costmap_2d::FREE_SPACE ? false : true;
    }
  }

  voronoi_.initializeMap(width_, height_, gridMap);
  voronoi_.update();
  voronoi_.prune();
  voronoi_.CollectVoronoiEdgePoints();
  voronoi_.visualize("/home/chelizi/biyesheji_ws/src/path_smoother/voronoi.ppm");
}

std::vector<geometry_msgs::Pose2D> PathSmoother::getOriginalPath()
{
  std::vector<geometry_msgs::Pose2D> path;
  path.clear();

  for (const auto pt : original_path_)
  {
    geometry_msgs::Pose2D pose;
    pose.x = pt.x();
    pose.y = pt.y();
    pose.theta = pt.theta();
    path.push_back(pose);
  }

  return path;
}

void PathSmoother::setOriginalPath(const std::vector<geometry_msgs::Pose2D>& path)
{
  original_path_.clear();

  for (const auto pt : path)
  {
    original_path_.emplace_back(pt.x, pt.y, pt.theta);
  }
}

std::vector<geometry_msgs::Pose2D> PathSmoother::getSmoothedPath()
{
  std::vector<geometry_msgs::Pose2D> path;
  path.clear();

  for (const auto pt : smoothed_path_)
  {
    geometry_msgs::Pose2D pose;
    pose.x = pt.x();
    pose.y = pt.y();
    pose.theta = pt.theta();
    path.push_back(pose);
  }

  return path;
}

float PathSmoother::pathAverageCurvature(const std::vector<Pose2d>& path)
{
  float kappa_sum = .0f, kappa_i;
  // Vec2d xim1, xi, xip1;
  // Vec2d Dxi, Dxip1;

  int n = path.size() - 2;
  for (uint i = 1; i < path.size() - 2; ++i)
  {
    Vec2d xim1(path[i - 1].x(), path[i - 1].y());
    Vec2d xi(path[i - 0].x(), path[i - 0].y());
    Vec2d xip1(path[i + 1].x(), path[i + 1].y());

    Vec2d Dxi = xi - xim1;
    Vec2d Dxip1 = xip1 - xi;

    float absDxi = Dxi.Length();
    float absDxip1 = Dxip1.Length();
    if (absDxi > 0 && absDxip1 > 0)
    {
      float Dphi = std::acos(Clamp<float>(Dxi.InnerProd(Dxip1) / (absDxi * absDxip1), -1, 1));
      kappa_i = Dphi / absDxi;
      kappa_sum += kappa_i;
    }
  }

  return (kappa_sum / ((float)n));
}

void PathSmoother::smoothPath()
{
  int iterations = 0;
  std::vector<Pose2d> smooth_path;
  smooth_path = original_path_;
  smoothed_path_ = original_path_;
  float totalWeight = wSmoothness_ + wCurvature_ + wVoronoi_ + wObstacle_;

  // float kappai, kappami;
  // kappami = pathAverageCurvature(smooth_path);

  // Todo:make sure the cycle end condition
  while (iterations < max_iterations_)
  {
    for (int i = 1; i < smooth_path.size() - 1; ++i)
    {
      // Vec2d xim2(smooth_path[i - 2].x(), smooth_path[i - 2].y());
      Vec2d xim1(smooth_path[i - 1].x(), smooth_path[i - 1].y());
      Vec2d xi(smooth_path[i - 0].x(), smooth_path[i - 0].y());
      Vec2d xip1(smooth_path[i + 1].x(), smooth_path[i + 1].y());
      // Vec2d xip2(smooth_path[i + 2].x(), smooth_path[i + 2].y());
      Vec2d correction;

      correction = correction - obstacleTerm(xi);
      if (!isOnGrid(xi + correction))
      {
        continue;
      }

      // correction = correction - smoothnessTerm(xim2, xim1, xi, xip1, xip2);
      correction = correction - smoothnessTerm(xim1, xi, xip1);
      if (!isOnGrid(xi + correction))
      {
        continue;
      }

      correction = correction - curvatureTerm(xim1, xi, xip1);
      if (!isOnGrid(xi + correction))
      {
        continue;
      }

      correction = correction - voronoiTerm(xi);
      if (!isOnGrid(xi + correction))
      {
        continue;
      }

      //      Vec2d delta_change = alpha_ * correction/totalWeight;
      //      std::cout <<"iterations=" << iterations <<", point index=" << i <<
      //      ", delta_change=(" << delta_change.x() << ", " << delta_change.y()
      //      << ")" << std::endl;

      xi = xi + alpha_ * correction / totalWeight;
      smooth_path[i].set_x(xi.x());
      smooth_path[i].set_y(xi.y());
      Vec2d Dxi = xi - xim1;
      smooth_path[i - 1].set_theta(std::atan2(Dxi.y(), Dxi.x()));
    }

    // kappai = pathAverageCurvature(smooth_path);
    // if (kappai > kappami)
    // {
    //   break;
    // }
    smoothed_path_ = smooth_path;
    // kappami = kappai;
    iterations++;
  }
}

Vec2d PathSmoother::obstacleTerm(Vec2d xi)
{
  Vec2d gradient;
  Vec2i xi_index;

  //! xi是否在map中，不在返回0梯度
  if (!coordToIndex(xi, xi_index))
  {
    return gradient;
  }
  // the distance to the closest obstacle from the current node
  float obsDst = voronoi_.getDistance(xi_index.x(), xi_index.y()) * resolution_;

  // if the node is within the map
  Vec2i obs_index = voronoi_.GetClosetObstacleCoor(xi_index);
  Vec2d obs;
  indexToCoord(obs_index, obs);
  Vec2d obsVct(xi.x() - obs.x(), xi.y() - obs.y());

  // obsDst should be equal to the length of obsVct. However, their difference
  // may be larger than 1m.
  //    std::cout << "(==) dis to closest obs = " << obsDst << ", Vector Mod =
  //    " << obsVct.length() << std::endl;
  // the closest obstacle is closer than desired correct the path for that
  // obsDMax = 2m

  if (obsDst < obsDMax_ && obsDst > 1e-6)
  {
    gradient = wObstacle_ * 2 * (obsDst - obsDMax_) * obsVct / obsDst;
    return gradient;
  }

  return gradient;
}

Vec2d PathSmoother::curvatureTerm(Vec2d xim1, Vec2d xi, Vec2d xip1)
{
  Vec2d gradient;
  // the vectors between the nodes
  Vec2d Dxi = xi - xim1;
  Vec2d Dxip1 = xip1 - xi;
  // orthogonal complements vector
  Vec2d p1, p2;

  float absDxi = Dxi.Length();
  float absDxip1 = Dxip1.Length();

  // ensure that the absolute values are not null
  if (absDxi > 0 && absDxip1 > 0)
  {
    float Dphi = std::acos(Clamp<float>(Dxi.InnerProd(Dxip1) / (absDxi * absDxip1), -1, 1));
    float kappa = Dphi / absDxi;

    if (kappa <= kappaMax_)
    {
      Vec2d zeros;
      //      std::cout << "curvatureTerm is 0 because kappa(" << kappa << ") <
      //      kappamax(" << kappaMax << ")" << std::endl;
      return zeros;
    }
    else
    {
      //代入原文公式(2)与(3)之间的公式. 参考：
      // Dolgov D, Thrun S, Montemerlo M, et al. Practical search techniques in
      // path planning for
      //  autonomous driving[J]. Ann Arbor, 2008, 1001(48105): 18-80.
      float absDxi1Inv = 1 / absDxi;
      float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
      float u = -absDxi1Inv * PDphi_PcosDphi;
      p1 = xi.ort(-xip1) / (absDxi * absDxip1);  //公式(4)
      p2 = -xip1.ort(xi) / (absDxi * absDxip1);
      float s = Dphi / (absDxi * absDxi);
      Vec2d ones(1, 1);
      Vec2d ki = u * (-p1 - p2) - (s * ones);
      Vec2d kim1 = u * p2 - (s * ones);
      Vec2d kip1 = u * p1;
      gradient = wCurvature_ * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);
      // gradient = wCurvature_ * 2.0 * (kappa - kappaMax_) * ki;

      if (std::isnan(gradient.x()) || std::isnan(gradient.y()))
      {
        //        std::cout << "nan values in curvature term" << std::endl;
        Vec2d zeros;
        //        std::cout << "curvatureTerm is 0 because gradient is non" <<
        //        std::endl;
        return zeros;
      }
      else
      {
        //        std::cout << "curvatureTerm is (" << gradient.x() << ", " <<
        //        gradient.y() << ")" << std::endl;
        return gradient;
      }
    }
  }
  else
  {
    std::cout << "abs values not larger than 0" << std::endl;
    std::cout << absDxi << absDxip1 << std::endl;
    Vec2d zeros;
    std::cout << "curvatureTerm is 0 because abs values not larger than 0" << std::endl;
    return zeros;
  }
}

Vec2d PathSmoother::smoothnessTerm(Vec2d xim2, Vec2d xim1, Vec2d xi, Vec2d xip1, Vec2d xip2)
{
  return wSmoothness_ * (xim2 - 4.0 * xim1 + 6.0 * xi - 4.0 * xip1 + xip2);
}

Vec2d PathSmoother::smoothnessTerm(Vec2d xim1, Vec2d xi, Vec2d xip1)
{
  return wSmoothness_ * (-4.0) * (xip1 - 2 * xi + xim1);
}

Vec2d PathSmoother::voronoiTerm(Vec2d xi)
{
  Vec2d gradient, obs;
  Vec2i obs_index, xi_index;

  if (!coordToIndex(xi, xi_index))
  {
    return gradient;
  }

  //! 最近障碍物
  float obsDst = voronoi_.getDistance(xi_index.x(), xi_index.y()) * resolution_;
  obs_index = voronoi_.GetClosetObstacleCoor(xi_index);
  indexToCoord(obs_index, obs);
  Vec2d obsVct(xi.x() - obs.x(), xi.y() - obs.y());

  //! 最近边
  double edgDst = 0.0;
  Vec2i closest_edge_pt_index = voronoi_.GetClosestVoronoiEdgePoint(xi_index, edgDst);
  edgDst *= resolution_;
  Vec2d closest_edge_pt;
  indexToCoord(closest_edge_pt_index, closest_edge_pt);
  Vec2d edgVct(xi.x() - closest_edge_pt.x(), xi.y() - closest_edge_pt.y());

  if (obsDst < vorObsDMax_ && obsDst > 1e-6)
  {
    if (edgDst > 0)
    {
      Vec2d PobsDst_Pxi = obsVct / obsDst;
      Vec2d PedgDst_Pxi = edgVct / edgDst;
      //      float PvorPtn_PedgDst = alpha * obsDst * std::pow(obsDst -
      //      vorObsDMax, 2) /
      //                              (std::pow(vorObsDMax, 2) * (obsDst +
      //                              alpha) * std::pow(edgDst + obsDst, 2));
      float PvorPtn_PedgDst = (alpha_ / alpha_ + obsDst) * (pow(obsDst - vorObsDMax_, 2) / pow(vorObsDMax_, 2)) *
                              (obsDst / pow(obsDst + edgDst, 2));

      //      float PvorPtn_PobsDst = (alpha * edgDst * (obsDst - vorObsDMax) *
      //      ((edgDst + 2 * vorObsDMax + alpha)
      //                                                                         * obsDst + (vorObsDMax + 2 * alpha) *
      //                                                                         edgDst + alpha * vorObsDMax))
      //                              / (std::pow(vorObsDMax, 2) *
      //                              std::pow(obsDst + alpha, 2) *
      //                              std::pow(obsDst + edgDst, 2));
      float PvorPtn_PobsDst =
          (alpha_ / (alpha_ + obsDst)) * (edgDst / (edgDst + obsDst)) * ((obsDst - vorObsDMax_) / pow(vorObsDMax_, 2)) *
          (-(obsDst - vorObsDMax_) / (alpha_ + obsDst) - (obsDst - vorObsDMax_) / (obsDst + edgDst) + 2);
      gradient = wVoronoi_ * (PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi) * 100;
      return gradient;
    }
    return gradient;
  }
  return gradient;
}

bool PathSmoother::coordToIndex(const Vec2d& coord, Vec2i& index)
{
  if (coord.x() < origin_x_ || coord.y() < origin_y_)
  {
    return false;
  }

  int x = (int)((coord.x() - origin_x_) / resolution_);
  int y = (int)((coord.y() - origin_y_) / resolution_);

  index.set_x(x);
  index.set_y(y);

  if (x < width_ && y < height_)
    return true;

  return false;
}

void PathSmoother::indexToCoord(const Vec2i& index, Vec2d& coord)
{
  double x = origin_x_ + (index.x() + 0.5) * resolution_;
  double y = origin_y_ + (index.y() + 0.5) * resolution_;

  coord.set_x(x);
  coord.set_y(y);
}

bool PathSmoother::isOnGrid(Vec2d vec)
{
  Vec2i index;
  return coordToIndex(vec, index);
}