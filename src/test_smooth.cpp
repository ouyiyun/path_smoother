#include <path_smoother/path_smoother.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>

nav_msgs::OccupancyGrid::Ptr map;
int width, height;

void path_pub(const ros::Publisher& pub, const std::vector<geometry_msgs::Pose2D>& points)
{
  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();

  for (auto it = points.begin(); it != points.end(); ++it)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = (*it).x;
    pose.pose.position.y = (*it).y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw((*it).theta);

    path.poses.push_back(pose);
  }

  pub.publish(path);
}

void setmap(const nav_msgs::OccupancyGrid::Ptr& map_in)
{
  map = map_in;
  width = map->info.width;
  height = map->info.height;
  ROS_INFO("Received map!");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "smooth_node");
  ros::NodeHandle nh;

  ros::Subscriber map_sub = nh.subscribe("map", 1, setmap);
  ros::Publisher origin_path_pub = nh.advertise<nav_msgs::Path>("origin_path", 1);
  ros::Publisher smoothed_path_pub = nh.advertise<nav_msgs::Path>("smoothed_path", 1);

  std::vector<geometry_msgs::Pose2D> origin_path, smoothed_path;
  std::FILE* fp = fopen("/home/chelizi/biyesheji_ws/src/path_smoother/src/path.txt", "r");
  double x, y, t;
  while (fscanf(fp, "%lf, %lf, %lf\n", &x, &y, &t) != EOF)
  {
    geometry_msgs::Pose2D pt;
    pt.x = x;
    pt.y = y;
    pt.theta = t;
    origin_path.push_back(pt);
  }

  ros::Duration(2.0).sleep();
  path_pub(origin_path_pub, origin_path);
  ros::Duration(2.0).sleep();

  while (map == nullptr)
  {
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  // PathSmoother ps("test_smooth", width, height);
  boost::shared_ptr<PathSmoother> ps = boost::shared_ptr<PathSmoother>(new PathSmoother("smooth_node", width, height));

  ps->updateMap(map);
  ps->setOriginalPath(origin_path);
  ps->smoothPath();

  smoothed_path = ps->getSmoothedPath();
  path_pub(smoothed_path_pub, smoothed_path);

  ros::Duration(10.0).sleep();
  return 0;
}