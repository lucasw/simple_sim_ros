/**
  Copyright (C) 2017  Lucas Walter

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.


*/

#include <bullet_server/bullet_server.h>
#include <bullet_server/raycast.h>
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

Raycast::Raycast(
      BulletServer* parent,
      const std::string name, const std::string frame_id,
      const std::vector<bullet_server::Line>& lines,
      const std::string topic_name,
      ros::NodeHandle& nh,
      btDiscreteDynamicsWorld* dynamics_world) :
    parent_(parent),
    name_(name),
    frame_id_(frame_id),
    lines_(lines),
    dynamics_world_(dynamics_world)
{
  point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>(topic_name, 3);
}

Raycast::Raycast(
      BulletServer* parent,
      const std::string name,
      const sensor_msgs::LaserScan& laser_scan,
      const std::string topic_name,
      ros::NodeHandle& nh,
      btDiscreteDynamicsWorld* dynamics_world) :
    // TODO(lucasw) reuse the other constructor
    parent_(parent),
    name_(name),
    frame_id_(laser_scan.header.frame_id),
    laser_scan_(laser_scan),
    dynamics_world_(dynamics_world)
{
  // TODO(lucasw) the point_cloud_pub may still be useful for debug
  point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>(topic_name + "_point_cloud_debug", 3);
  laser_scan_pub_ = nh.advertise<sensor_msgs::LaserScan>(topic_name, 3);

  for (float angle = laser_scan_.angle_min; angle < laser_scan_.angle_max;
      angle += laser_scan_.angle_increment)
  {
    bullet_server::Line line;
    line.start.x = laser_scan_.range_min * cos(angle);
    line.start.y = laser_scan_.range_min * sin(angle);
    line.end.x = laser_scan_.range_max * cos(angle);
    line.end.y = laser_scan_.range_max * sin(angle);
    lines_.push_back(line);
  }
}

// TODO(lucasw) later this will return an entire ros message
// in PointCloud, PointCloud2, LaserScan, or Range formats as desired
bool Raycast::update(tf2_ros::Buffer& tf_buffer)
{
  geometry_msgs::TransformStamped frame_to_world;
  geometry_msgs::TransformStamped world_to_frame;
  // TODO(lucasw) pass this in in constructor
  const std::string world_frame = parent_->config_.frame_id;
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
  sensor_msgs::PointCloud pc;
  pc.header = world_to_frame.header;
  sensor_msgs::LaserScan laser_scan = laser_scan_;
  laser_scan.header = pc.header;
  laser_scan.ranges.resize(0);
  laser_scan.intensities.resize(0);

  // TODO(lucasw) put all points into point cloud that can be transformed in one call
  for (auto line : lines_)
  {
    // ROS_INFO_STREAM(transform_stamped);
    geometry_msgs::PointStamped start_world;
    geometry_msgs::PointStamped end_world;

    geometry_msgs::PointStamped start_stamped, end_stamped;
    start_stamped.header.stamp = ros::Time(0);
    start_stamped.header.frame_id = frame_id_;
    start_stamped.point = line.start;
    end_stamped.header = start_stamped.header;
    end_stamped.point = line.end;
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

      if (laser_scan_pub_.getTopic() != "")
      {
        // avoid doing sqrt and other math on vector result
        const float extent = (laser_scan_.range_max - laser_scan_.range_min);
        const float range = laser_scan_.range_min + closest_results.m_closestHitFraction * extent;
        laser_scan.ranges.push_back(range);
      }
    }
    else
    {
      if (laser_scan_pub_.getTopic() != "")
      {
        // TODO(lucasw) not sure sure putting in range_max value
        // means that there is empty space there, or some other value
        // communicates that.
        laser_scan.ranges.push_back(laser_scan_.range_max);
      }
    }
  }  // loop through all rays
  point_cloud_pub_.publish(pc);
  if (laser_scan_pub_.getTopic() != "")
  {
    laser_scan_pub_.publish(laser_scan);
  }
  return true;
}
