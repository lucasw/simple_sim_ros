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

#ifndef BULLET_SERVER_SOFT_BODY_H
#define BULLET_SERVER_SOFT_BODY_H

#include <boost/functional/hash.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <bullet_server/constraint.h>
#include <bullet_server/AddBody.h>
#include <bullet_server/AddCompound.h>
#include <bullet_server/AddConstraint.h>
#include <bullet_server/AddHeightfield.h>
#include <bullet_server/AddImpulse.h>
#include <bullet_server/Anchor.h>
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

// TODO(lucasw) make a common base class
class SoftBody
{
  BulletServer* parent_;
  btSoftRigidDynamicsWorld* dynamics_world_;
  tf::TransformBroadcaster* br_;
  ros::Publisher* marker_array_pub_;
  visualization_msgs::MarkerArray marker_array_;
  const std::string name_;
public:
  SoftBody(BulletServer* parent,
      const std::string name,
      btSoftBodyWorldInfo* soft_body_world_info,
      const std::vector<bullet_server::Node>& nodes,
      const std::vector<bullet_server::Link>& links,
      const std::vector<bullet_server::Face>& faces,
      const std::vector<bullet_server::Tetra>& tetras,
      const std::vector<bullet_server::Material>& materials,
      const std::vector<bullet_server::Anchor>& anchors,
      const bullet_server::SoftConfig& config,
      const float margin,
      const bool randomize_constraints,
      const uint8_t k_clusters,
      btSoftRigidDynamicsWorld* dynamics_world,
      tf::TransformBroadcaster* br,
      ros::Publisher* marker_array_pub);
  ~SoftBody();
  void update();
  btSoftBody* soft_body_;

  enum MarkerInd
  {
    NODES = 0,
    LINKS,
    ANCHORS,
    ANCHOR_PIVOTS,
    TETRAS,
    FACES,
    COUNT
  };
};

#endif  // BULLET_SERVER_SOFT_BODY_H
