/**
  Copyright 2017 Lucas Walter

*/

#include <bullet_server/raycast.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

Raycast::Raycast(const std::string name, const std::string frame_id,
      const geometry_msgs::Point start, const geometry_msgs::Point end,
      const std::string topic_name,
      ros::NodeHandle& nh,
      btDiscreteDynamicsWorld* dynamics_world) :
    name_(name),
    frame_id_(frame_id),
    start_(start),
    end_(end),
    dynamics_world_(dynamics_world)
{
  point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>(topic_name, 3);
}

// TODO(lucasw) later this will return an entire ros message
// in PointCloud, PointCloud2, LaserScan, or Range formats as desired
bool Raycast::update(tf2_ros::Buffer& tf_buffer)
{
  geometry_msgs::TransformStamped transform_stamped;
  // TODO(lucasw) pass this in in constructor
  const std::string dest_frame = "map";
  try
  {
    transform_stamped = tf_buffer.lookupTransform(dest_frame, frame_id_, ros::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
  // ROS_INFO_STREAM(transform_stamped);
  geometry_msgs::PointStamped start_world;
  geometry_msgs::PointStamped end_world;

  geometry_msgs::PointStamped start_stamped, end_stamped;
  start_stamped.header.stamp = ros::Time(0);
  start_stamped.header.frame_id = frame_id_;
  start_stamped.point = start_;
  end_stamped.header = start_stamped.header;
  end_stamped.point = end_;
  tf2::doTransform(start_stamped, start_world, transform_stamped);
  tf2::doTransform(end_stamped, end_world, transform_stamped);
  // geometry_msgs::PointStamped start_world = tf_buffer.transform(start_stamped, dest_frame);
  // geometry_msgs::PointStamped end_world = tf_buffer.transform(end_stamped, dest_frame);

  // TODO(lucasw) do raycast, but for now just return end_world
  // TODO(lucasw) transform back into frame_id (after doing raycast)
  geometry_msgs::Point32 rv;

  sensor_msgs::PointCloud pc;
  pc.header = transform_stamped.header;

  rv.x = start_world.point.x;
  rv.y = start_world.point.y;
  rv.z = start_world.point.z;
  pc.points.push_back(rv);
  rv.x = end_world.point.x;
  rv.y = end_world.point.y;
  rv.z = end_world.point.z;
  pc.points.push_back(rv);

  point_cloud_pub_.publish(pc);
  return true;
}
