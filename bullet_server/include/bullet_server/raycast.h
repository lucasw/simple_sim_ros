/**
  Copyright 2017 Lucas Walter
  GNU GPL 3.0

*/

#ifndef BULLET_SERVER_RAYCAST_H
#define BULLET_SERVER_RAYCAST_H

#include <bullet/btBulletDynamicsCommon.h>
#include <bullet_server/Line.h>
// #include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <vector>

class BulletServer;

class Raycast
{
public:
  Raycast(BulletServer* parent,
      const std::string name, const std::string frame_id,
      const std::vector<bullet_server::Line>& lines,
      const std::string topic_name,
      ros::NodeHandle& nh,
      btDiscreteDynamicsWorld* dynamics_world);

  // TODO(lucasw) this should probably be a subclass instead
  sensor_msgs::LaserScan laser_scan_;
  Raycast(BulletServer* parent,
      const std::string name,
      const sensor_msgs::LaserScan& laser_scan,
      const std::string topic_name,
      ros::NodeHandle& nh,
      btDiscreteDynamicsWorld* dynamics_world);

  bool update(tf2_ros::Buffer& tf_buffer);
private:
  BulletServer* parent_;
  ros::Publisher point_cloud_pub_;
  ros::Publisher laser_scan_pub_;

  const std::string name_;
  const std::string frame_id_;
  // all lines are in frame_id_ tf frame
  // const
  std::vector<bullet_server::Line> lines_;

  const btDiscreteDynamicsWorld* dynamics_world_;
};

#endif  // BULLET_SERVER_RAYCAST_H
