#include "path_smoother/nlopt_smoother.h"

NLoptSmoother::NLoptSmoother(std::string name, int size_x, int size_y)
{
  // ros::NodeHandle private_nh("~/" + name);
  ros::NodeHandle private_nh("~");

  private_nh.param<int>("max_iterations", max_iterations_, 200);
  private_nh.param<float>("min_turning_radius", min_turning_radius_, 5.0);
  kappaMax_ = 1.f / (min_turning_radius_ * 1.1);
  private_nh.param<float>("max_obs_distance", obsDMax_, 1.5);
  private_nh.param<float>("max_vor_obs_distance", vorObsDMax_, 2.0);
  private_nh.param<float>("weight_obstacle", wObstacle_, 0.2);
  private_nh.param<float>("weight_voronoi", wVoronoi_, 0.2);
  private_nh.param<float>("weight_curvature", wCurvature_, 0.2);
  private_nh.param<float>("weight_smoothness", wSmoothness_, 0.6);
  private_nh.param<int>("algorithm", algorithm_, 11);
  ROS_INFO("max_iterations: %d", max_iterations_);
  ROS_INFO("algorithm: %d", algorithm_);
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

void NLoptSmoother::updateMap(const nav_msgs::OccupancyGrid::Ptr& occMap)
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

void NLoptSmoother::updateMap(costmap_2d::Costmap2D* costmap)
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
  //   voronoi_.visualize("/home/chelizi/biyesheji_ws/src/path_smoother/voronoi.ppm");
}

std::vector<geometry_msgs::Pose2D> NLoptSmoother::getOriginalPath()
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

void NLoptSmoother::setOriginalPath(const std::vector<geometry_msgs::Pose2D>& path)
{
  original_path_.clear();

  for (const auto pt : path)
  {
    original_path_.emplace_back(pt.x, pt.y, pt.theta);
  }
  points_num_ = original_path_.size();
}

std::vector<geometry_msgs::Pose2D> NLoptSmoother::getSmoothedPath()
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

void NLoptSmoother::smoothPath()
{
  min_cost_ = std::numeric_limits<double>::max();
  iter_nums_ = 0;

  nlopt::opt opt(nlopt::algorithm(algorithm_), points_num_ * 2);
  opt.set_min_objective(NLoptSmoother::costFunction, this);
  opt.set_maxeval(max_iterations_);

  //! ????????????????????????
  smoothed_path_ = original_path_;

  first_point_.set_x(smoothed_path_[0].x());
  first_point_.set_y(smoothed_path_[0].y());
  last_point_.set_x(smoothed_path_.back().x());
  last_point_.set_y(smoothed_path_.back().y());

  std::vector<double> points(points_num_ * 2);
  double f_cost;
  for (uint i = 0; i < original_path_.size(); ++i)
  {
    points[2 * i] = original_path_[i].x();
    points[2 * i + 1] = original_path_[i].y();
  }

  try
  {
    /* ---------- optimization ---------- */
    std::cout << "[Optimization]: begin-------------\n";
    std::cout << std::fixed << std::setprecision(7);
    vec_cost_.clear();

    ros::Time t0 = ros::Time::now();
    nlopt::result result = opt.optimize(points, f_cost);
    double smooth_time = (ros::Time::now() - t0).toSec() * 1000;

    ROS_INFO("NLopt's result: %d, time: %lf ms", result, smooth_time);
    std::cout << "Min cost: " << min_cost_ << std::endl;

    //! ????????????????????????????????????
    for (int i = 1; i < points_num_ - 1; ++i)
    {
      Vec2d xim1(best_variable_[2 * (i - 1) + 0], best_variable_[2 * (i - 1) + 1]);
      Vec2d xi(best_variable_[2 * i + 0], best_variable_[2 * i + 1]);
      Vec2d Dxi = xi - xim1;
      smoothed_path_[i].set_x(xi.x());
      smoothed_path_[i].set_y(xi.y());
      smoothed_path_[i - 1].set_theta(std::atan2(Dxi.y(), Dxi.x()));
    }
    std::cout << "[Optimization]: end-------------\n";
  }
  catch (std::exception& e)
  {
    std::cout << "[Optimization]: nlopt exception: " << e.what() << std::endl;
  }
}

void NLoptSmoother::calcSmoothness(const std::vector<Vec2d>& points, std::vector<Vec2d>& gradient, double& cost)
{
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Vec2d(.0, .0));

  Vec2d fn;
  for (int i = 1; i < points_num_ - 1; ++i)
  {
    fn = points[i + 1] - 2.0 * points[i + 0] + points[i - 1];
    gradient[i - 1] += 2.0 * fn;
    gradient[i + 0] += -4.0 * fn;
    gradient[i + 1] += 2.0 * fn;

    cost += fn.LengthSquare();
  }
}

void NLoptSmoother::calcObsDistance(const std::vector<Vec2d>& points, std::vector<Vec2d>& gradient, double& cost)
{
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Vec2d(.0, .0));

  Vec2d xi, oi;
  Vec2d zero(.0, .0);
  double dist;

  for (int i = 0; i < points_num_; ++i)
  {
    xi = points[i];
    nearestObsDistance(xi, oi, dist);

    if (dist < obsDMax_ && dist > 1e-6)
    {
      gradient[i] += 2.0 * (dist - obsDMax_) * (xi - oi) / dist;
      cost += pow(dist - obsDMax_, 2);
    }
    else
    {
      gradient[i] += zero;
      cost += .0;
    }
  }
}

void NLoptSmoother::calcVornoiField(const std::vector<Vec2d>& points, std::vector<Vec2d>& gradient, double& cost)
{
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Vec2d(.0, .0));

  Vec2d xi, vi, oi;
  //! obs distance and edge distance
  double d_o, d_v;

  for (int i = 0; i < points_num_; ++i)
  {
    xi = points[i];
    nearestObsDistance(xi, oi, d_o);
    nearestEdgeDistance(xi, vi, d_v);
    if (d_o < obsDMax_ && d_o > 1e-6)
    {
      if (d_v > 1e-6)
      {
        Vec2d Pdo_Pxi = (xi - oi) / d_o;
        Vec2d Pdv_Pxi = (xi - vi) / d_v;

        double Pphov_Pdv = alpha_ / (alpha_ + d_o) * pow(d_o - obsDMax_ / obsDMax_, 2) * d_o / pow(d_o + d_v, 2);

        double Pphov_pdo = alpha_ / (alpha_ + d_o) * d_v / (d_o + d_v) * (d_o - obsDMax_) / pow(obsDMax_, 2) *
                           ((obsDMax_ - d_o) / (alpha_ + d_o) - (d_o - obsDMax_) / (d_o + d_v) + 2);

        gradient[i] += Pphov_pdo * Pdo_Pxi + Pphov_Pdv * Pdv_Pxi;

        cost += alpha_ / (alpha_ + d_o) * d_v / (d_v + d_o) * pow((d_o - obsDMax_) / obsDMax_, 2);
      }
    }
  }
}

void NLoptSmoother::calcCurvature(const std::vector<Vec2d>& points, std::vector<Vec2d>& gradient, double& cost)
{
  cost = 0.0;
  std::fill(gradient.begin(), gradient.end(), Vec2d(.0, .0));

  Vec2d xi, xim1, xip1;
  Vec2d Dxi, Dxip1;

  Vec2d v(1.0, 1.0);

  for (int i = 1; i < points_num_ - 1; ++i)
  {
    xim1 = points[i - 1];
    xi = points[i + 0];
    xip1 = points[i + 1];

    Dxi = xi - xim1;
    Dxip1 = xip1 - xi;

    double absDxi = Dxi.Length();
    double absDxip1 = Dxip1.Length();

    if (absDxi > 1e-6 && absDxip1 > 1e-6)
    {
      double Dphi = std::acos(std::min(std::max(Dxi.InnerProd(Dxip1) / (absDxi * absDxip1), -1.0), 1.0));
      double kappa = Dphi / absDxi;

      if (kappa > kappaMax_)
      {
        double absDxiInv = 1.0 / absDxi;
        double PDphi_PcosDphi = -1.0 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
        double u = Dphi / Dxi.LengthSquare();

        // ROS_INFO("absDxiInv: %lf, PDphi_PcosDphi: %lf, u: %lf", absDxiInv, PDphi_PcosDphi, u);

        if (xi.Length() > 1e-6 && xip1.Length() > 1e-6)
        {
          Vec2d p1 = xi.ort(-xip1) / (absDxi * absDxip1);
          Vec2d p2 = -xip1.ort(xi) / (absDxi * absDxip1);

          Vec2d PcosDphi_Pxi = -p1 - p2;
          Vec2d PcosDphi_Pxim1 = p2;
          Vec2d PcosDphi_Pxip1 = p1;

          Vec2d Pki_Pxi = (-1.0) * absDxiInv * PDphi_PcosDphi * PcosDphi_Pxi - u * v;
          Vec2d Pki_Pxim1 = (-1.0) * absDxiInv * PDphi_PcosDphi * PcosDphi_Pxim1 - u * (-v);
          Vec2d Pki_Pxip1 = (-1.0) * absDxiInv * PDphi_PcosDphi * PcosDphi_Pxip1;

          if (Vec2dIsnNan(Pki_Pxi) || Vec2dIsnNan(Pki_Pxim1) || Vec2dIsnNan(Pki_Pxip1))
            continue;

          double dot = 2.0 * (kappa - kappaMax_);

          gradient[i + 0] += dot * Pki_Pxi;
          gradient[i - 1] += dot * Pki_Pxim1;
          gradient[i + 1] += dot * Pki_Pxip1;
          cost += std::pow(kappa - kappaMax_, 2);
        }
      }
    }
  }
}

void NLoptSmoother::combineCost(const std::vector<double>& points, std::vector<double>& grad, double& f_commbin)
{
  //! points?????????vec2D
  std::vector<Vec2d> q;

  Vec2d p;

  //! ????????????
  q.push_back(first_point_);

  for (int i = 1; i < points_num_; ++i)
  {
    p.set_x(points[2 * i + 0]);
    p.set_y(points[2 * i + 1]);
    q.push_back(p);
  }

  //??? ???????????????
  // q.push_back(last_point_);

  /* ---------- evaluate cost and gradient ---------- */
  //! cost
  double f_smoothness, f_distance, f_vornoi, f_curvature;

  //! gradient
  std::vector<Vec2d> g_smoothness, g_distance, g_vornoi, g_curvature;

  g_smoothness.resize(points_num_);
  g_distance.resize(points_num_);
  g_vornoi.resize(points_num_);
  g_curvature.resize(points_num_);

  calcSmoothness(q, g_smoothness, f_smoothness);
  calcObsDistance(q, g_distance, f_distance);
  calcVornoiField(q, g_vornoi, f_vornoi);
  calcCurvature(q, g_curvature, f_curvature);

  /* ---------- convert to NLopt format...---------- */
  grad.resize(points_num_);

  f_commbin = wSmoothness_ * f_smoothness + wObstacle_ * f_distance + wVoronoi_ * f_vornoi + wCurvature_ * f_curvature;

  for (int i = 0; i < points_num_; ++i)
  {
    grad[2 * i + 0] = wSmoothness_ * g_smoothness[i].x() + wObstacle_ * g_distance[i].x() +
                      wVoronoi_ * g_vornoi[i].x() + wCurvature_ * g_curvature[i].x();
    grad[2 * i + 1] = wSmoothness_ * g_smoothness[i].y() + wObstacle_ * g_distance[i].y() +
                      wVoronoi_ * g_vornoi[i].y() + wCurvature_ * g_curvature[i].y();
  }
  iter_nums_ += 1;
}

double NLoptSmoother::costFunction(const std::vector<double>& points, std::vector<double>& grad, void* func_data)
{
  NLoptSmoother* opt = reinterpret_cast<NLoptSmoother*>(func_data);
  double cost;

  opt->combineCost(points, grad, cost);

  //! ???????????????????????????????????????
  if (cost < opt->min_cost_)
  {
    opt->min_cost_ = cost;
    opt->best_variable_ = points;
  }

  /* ---------- evaluation ---------- */
  ROS_INFO("%d : %lf", opt->iter_nums_, opt->min_cost_);

  return cost;
}

void NLoptSmoother::nearestObsDistance(const Vec2d& xi, Vec2d& oi, double& dist)
{
  Vec2i xi_index, oi_index;

  if (!coordToIndex(xi, xi_index))
  {
    oi = xi;
    dist = .0;
    return;
  }

  dist = voronoi_.getDistance(xi_index.x(), xi_index.y()) * resolution_;

  oi_index = voronoi_.GetClosetObstacleCoor(xi_index);
  indexToCoord(oi_index, oi);
}

void NLoptSmoother::nearestEdgeDistance(const Vec2d& xi, Vec2d& vi, double& dist)
{
  Vec2i xi_index, vi_index;
  if (!coordToIndex(xi, xi_index))
  {
    vi = xi;
    dist = .0;
    return;
  }
  vi_index = voronoi_.GetClosestVoronoiEdgePoint(xi_index, dist);
  dist *= resolution_;
  indexToCoord(vi_index, vi);
}

bool NLoptSmoother::coordToIndex(const Vec2d& coord, Vec2i& index)
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

void NLoptSmoother::indexToCoord(const Vec2i& index, Vec2d& coord)
{
  double x = origin_x_ + (index.x() + 0.5) * resolution_;
  double y = origin_y_ + (index.y() + 0.5) * resolution_;

  coord.set_x(x);
  coord.set_y(y);
}

bool NLoptSmoother::isOnGrid(Vec2d vec)
{
  Vec2i index;
  return coordToIndex(vec, index);
}

bool NLoptSmoother::Vec2dIsnNan(const Vec2d& vec)
{
  if (std::isnan(vec.x()) || std::isnan(vec.y()))
  {
    return true;
  }
  return false;
}