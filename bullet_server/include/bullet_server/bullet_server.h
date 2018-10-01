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

#ifndef BULLET_SERVER_BULLET_SERVER_H
#define BULLET_SERVER_BULLET_SERVER_H

#include <boost/functional/hash.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <bullet_server/constraint.h>
#include <bullet_server/AddBody.h>
#include <bullet_server/AddCompound.h>
#include <bullet_server/AddConstraint.h>
#include <bullet_server/AddHeightfield.h>
#include <bullet_server/AddImpulse.h>
#include <bullet_server/AddLaserScan.h>
#include <bullet_server/AddRaycast.h>
#include <bullet_server/Anchor.h>
#include <bullet_server/Body.h>
#include <bullet_server/BulletServerConfig.h>
#include <bullet_server/Constraint.h>
#include <bullet_server/Heightfield.h>
#include <bullet_server/Impulse.h>
#include <bullet_server/SetMaterial.h>
#include <bullet_server/SoftBody.h>
#include <bullet_server/SoftConfig.h>
#include <bullet_server/Material.h>
#include <bullet_server/Node.h>
#include <bullet_server/Link.h>
#include <bullet_server/Face.h>
#include <bullet_server/SetTransform.h>
#include <bullet_server/Tetra.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <map>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Body;
class SoftBody;
struct Raycast;

// TODO(lucasw) replace this with something better, numbers aren't random enough
// http://stackoverflow.com/questions/2535284/how-can-i-hash-a-string-to-an-int-using-c
uint16_t hash(const char *str);

class BulletServer
{
  ros::NodeHandle nh_;
  ros::ServiceServer add_compound_;
  bool addCompound(bullet_server::AddCompound::Request& req,
                   bullet_server::AddCompound::Response& res);
  ros::ServiceServer add_raycast_;
  bool addRaycast(bullet_server::AddRaycast::Request& req,
                  bullet_server::AddRaycast::Response& res);
  ros::ServiceServer add_laser_scan_;
  bool addLaserScan(bullet_server::AddLaserScan::Request& req,
                    bullet_server::AddLaserScan::Response& res);

  ros::ServiceServer set_transform_;
  bool setTransform(bullet_server::SetTransform::Request& req,
                    bullet_server::SetTransform::Response& res);

  // TODO(lucasw) deprecate adding bodies and constraints and heightfields
  // through topics?
  ros::Subscriber body_sub_;
  void bodyCallback(const bullet_server::Body::ConstPtr& msg);
  bool softBodyCallback(const bullet_server::SoftBody::ConstPtr& msg);
  ros::Subscriber constraint_sub_;
  void constraintCallback(const bullet_server::Constraint::ConstPtr& msg);
  ros::Subscriber impulse_sub_;
  void impulseCallback(const bullet_server::Impulse::ConstPtr& msg);
  ros::Subscriber set_material_sub_;
  void setMaterialCallback(const bullet_server::SetMaterial::ConstPtr& msg);
  ros::Subscriber heightfield_sub_;
  void heightfieldCallback(const bullet_server::Heightfield::ConstPtr& msg);
  ros::Subscriber republish_markers_sub_;
  void republishMarkers(const std_msgs::Empty::ConstPtr&);

  std::string tf_prefix_;
  // TODO(lucasw) update to tf2
  tf::TransformBroadcaster br_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  float period_;
  // ros::Publisher marker_pub_;
  ros::Publisher marker_array_pub_;

  bool rigid_only_;
  btBroadphaseInterface* broadphase_;

  btDefaultCollisionConfiguration* collision_configuration_;
  btSoftBodyRigidBodyCollisionConfiguration* soft_rigid_collision_configuration_;

  btCollisionDispatcher* dispatcher_;
  // only used with opencl
  // btSoftBodySolver* soft_body_solver_;
  btSequentialImpulseConstraintSolver* solver_;

  // the dynamics world will operate in config_.frame_id
  // If this frame were to move relative to a higher non-simulated frame
  // there would be no effect on the sim.
  btDiscreteDynamicsWorld* dynamics_world_;

  // TODO(lucasw) make this configurable by addCompound
  // instead of hard coded
  btCollisionShape* ground_shape_;
  btDefaultMotionState* ground_motion_state_;
  btRigidBody* ground_rigid_body_;

  btSoftBodyWorldInfo soft_body_world_info_;
  std::map<std::string, SoftBody*> soft_bodies_;
  std::map<std::string, Constraint*> constraints_;
  std::map<std::string, Raycast*> raycasts_;

  boost::recursive_mutex dr_mutex_;
  typedef dynamic_reconfigure::Server<bullet_server::BulletServerConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  void reconfigureCallback(
      bullet_server::BulletServerConfig& config,
      uint32_t level);

  ros::Timer timer_;
  double internal_elapsed_time_;
  ros::Publisher tick_pub_;
  ros::Publisher internal_time_pub_;
  int init();
public:
  BulletServer();
  ~BulletServer();
  void reset();
  bullet_server::BulletServerConfig config_;
  void tickCallback(btScalar time_step);
  void update(const ros::TimerEvent& e);
  void removeConstraint(const Constraint* constraint,
      const bool remove_from_bodies = false);
  std::map<std::string, Body*> bodies_;
};

#endif  // BULLET_SERVER_BULLET_SERVER_H
