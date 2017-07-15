/**
  Copyright 2016 Lucas Walter

  First step:

  Create a bullet simulation, default with an infinite plane.

  ros messages or service calls create body primitives in the simulation.
  Send the position and attitude of each body to tf.

  Maybe publish Markers for rviz, but could leave that to client- ideally 
  no visualization at all would be done here.

  No joints initially, or actuation, or sensors.

  Publish to clock if use_sim_time is set, otherwise try to keep up with
  wall clock time as much as possible (maybe issue a warning if the sim
  gets more than 5 or 10 seconds behind current time).
*/

#ifndef BULLET_SERVER_BODY_H
#define BULLET_SERVER_BODY_H

#include <boost/functional/hash.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <bullet_server/Constraint.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <map>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class BulletServer;
class Constraint;

class Body
{
  BulletServer* parent_;
  tf::TransformBroadcaster* br_;
  // ros::Publisher* marker_pub_;
  ros::Publisher* marker_array_pub_;
  visualization_msgs::MarkerArray marker_array_;

  // TODO(lucasw) put this in a child class
  // for triangle meshes
  btVector3* vertices_;
  int* indices_;
  btTriangleIndexVertexArray* index_vertex_arrays_;

  std::map<std::string, Constraint*> constraints_;
  btCollisionShape* shape_;
  btDefaultMotionState* motion_state_;

  btDiscreteDynamicsWorld* dynamics_world_;
  int state_;
public:
  Body(BulletServer* parent,
      const std::string name,
      unsigned int type,
      const float mass,
      geometry_msgs::Pose pose,
      geometry_msgs::Twist twist,
      geometry_msgs::Vector3 scale,
      btDiscreteDynamicsWorld* dynamics_world,
      tf::TransformBroadcaster* br,
      ros::Publisher* marker_array_pub_);
  // heightfield
  Body(BulletServer* parent,
    const std::string name,
    // unsigned int type,
    // geometry_msgs::Pose pose,
    // geometry_msgs::Vector3 scale,
    cv::Mat& image,
    const float resolution,
    const float height_scale,
    const bool flip_quad_edges,
    btDiscreteDynamicsWorld* dynamics_world,
    tf::TransformBroadcaster* br,
    ros::Publisher* marker_array_pub);
  ~Body();

  // keep track of constraints attached to this body
  void addConstraint(Constraint* constraint);
  void removeConstraint(const Constraint* constraint);
  const std::string name_;
  btRigidBody* rigid_body_;
  void update();
};

#endif  // BULLET_SERVER_BODY_H
