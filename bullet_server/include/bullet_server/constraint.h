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

#ifndef BULLET_SERVER_CONSTRAINT_H
#define BULLET_SERVER_CONSTRAINT_H

#include <boost/functional/hash.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h>
// #include <bullet_server/bullet_server.h>
#include <bullet_server/AddBody.h>
#include <bullet_server/AddCompound.h>
#include <bullet_server/AddConstraint.h>
#include <bullet_server/AddHeightfield.h>
#include <bullet_server/AddImpulse.h>
#include <bullet_server/Anchor.h>
#include <bullet_server/Body.h>
#include <bullet_server/Constraint.h>
#include <bullet_server/Heightfield.h>
#include <bullet_server/Impulse.h>
#include <bullet_server/SoftBody.h>
#include <bullet_server/SoftConfig.h>
#include <bullet_server/Material.h>
#include <bullet_server/Node.h>
#include <bullet_server/Link.h>
#include <bullet_server/Face.h>
#include <bullet_server/Tetra.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <map>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Body;

class Constraint
{
  ros::NodeHandle nh_;
  btTypedConstraint* constraint_;

  btDiscreteDynamicsWorld* dynamics_world_;
  ros::Publisher* marker_array_pub_;
  std::map<std::string, float> command_;
  const bool enable_pos_pub_;
  std::map<std::string, ros::Publisher> pubs_;
  const bool enable_motor_sub_;
  std::map<std::string, ros::Subscriber> subs_;
  visualization_msgs::MarkerArray marker_array_;
  float max_motor_impulse_;
  void commandCallback(const std_msgs::Float64::ConstPtr msg, const std::string motor_name);
public:
  Constraint(
      const std::string name,
      unsigned int type,
      Body* body_a,
      Body* body_b,
      geometry_msgs::Point pivot_in_a,
      geometry_msgs::Point pivot_in_b,
      geometry_msgs::Vector3 axis_in_a,
      geometry_msgs::Vector3 axis_in_b,
      const double lower_lin_lim,
      const double upper_lin_lim,
      const double lower_ang_lim,
      const double upper_ang_lim,
      const float max_motor_impulse,
      const bool enable_pos_pub,
      const bool enable_motor_sub,
      const bool disable_collisions_between_linked_bodies,
      btDiscreteDynamicsWorld* dynamics_world,
      ros::Publisher* marker_array_pub);
  ~Constraint();

  const std::string name_;
  Body* body_a_;
  Body* body_b_;
  void update();
};

#endif  // BULLET_SERVER_CONSTRAINT_H
