/**
  Copyright 2017 Lucas Walter

*/

#include <bullet_server/raycast.h>
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>
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
  geometry_msgs::TransformStamped frame_to_world;
  geometry_msgs::TransformStamped world_to_frame;
  // TODO(lucasw) pass this in in constructor
  const std::string world_frame = "map";
  try
  {
    frame_to_world = tf_buffer.lookupTransform(world_frame, frame_id_, ros::Time(0));
    world_to_frame = tf_buffer.lookupTransform(frame_id_, world_frame, ros::Time(0));
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
  tf2::doTransform(start_stamped, start_world, frame_to_world);
  tf2::doTransform(end_stamped, end_world, frame_to_world);

  btVector3 start_bt(start_world.point.x, start_world.point.y, start_world.point.z);
  btVector3 end_bt(end_world.point.x, end_world.point.y, end_world.point.z);
  btCollisionWorld::ClosestRayResultCallback closest_results(start_bt, end_bt);
  closest_results.m_flags |= btTriangleRaycastCallback::kF_FilterBackfaces;
  dynamics_world_->rayTest(start_bt, end_bt, closest_results);


  // put in start and end for debug
  #if 0
    sensor_msgs::PointCloud pc;
    pc.header = frame_to_world.header;
  geometry_msgs::Point32 start_pt;
  start_pt.x = start_world.point.x;
  start_pt.y = start_world.point.y;
  start_pt.z = start_world.point.z;
  pc.points.push_back(start_pt);

  geometry_msgs::Point32 end_pt;
  end_pt.x = end_world.point.x;
  end_pt.y = end_world.point.y;
  end_pt.z = end_world.point.z;
  pc.points.push_back(end_pt);
  #endif

  if (closest_results.hasHit())
  {
    sensor_msgs::PointCloud pc;
    pc.header = world_to_frame.header;
    const btVector3 hit = start_bt.lerp(end_bt, closest_results.m_closestHitFraction);
    geometry_msgs::PointStamped world_pt;
    world_pt.header = frame_to_world.header;
    world_pt.point.x = hit.x();
    world_pt.point.y = hit.y();
    world_pt.point.z = hit.z();
    // transform into frame_id to be from sensors point of view
    geometry_msgs::PointStamped frame_pt;
    tf2::doTransform(world_pt, frame_pt, world_to_frame);
    geometry_msgs::Point32 frame_pt32;
    frame_pt32.x = frame_pt.point.x;
    frame_pt32.y = frame_pt.point.y;
    frame_pt32.z = frame_pt.point.z;
    pc.points.push_back(frame_pt32);
    point_cloud_pub_.publish(pc);
  }
  return true;
}
