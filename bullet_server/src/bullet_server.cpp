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

#include <boost/functional/hash.hpp>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <bullet/BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <bullet/BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <bullet_server/AddBody.h>
#include <bullet_server/AddCompound.h>
#include <bullet_server/AddConstraint.h>
#include <bullet_server/AddHeightfield.h>
#include <bullet_server/AddImpulse.h>
#include <bullet_server/AddRaycast.h>
#include <bullet_server/Anchor.h>
#include <bullet_server/Body.h>
#include <bullet_server/Constraint.h>
#include <bullet_server/Face.h>
#include <bullet_server/Heightfield.h>
#include <bullet_server/Impulse.h>
#include <bullet_server/Link.h>
#include <bullet_server/Material.h>
#include <bullet_server/Node.h>
#include <bullet_server/SoftBody.h>
#include <bullet_server/SoftConfig.h>
#include <bullet_server/Tetra.h>
#include <bullet_server/bullet_server.h>
#include <bullet_server/body.h>
#include <bullet_server/constraint.h>
#include <bullet_server/raycast.h>
#include <bullet_server/soft_body.h>
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

// TODO(lucasw) replace this with something better, numbers aren't random enough
// http://stackoverflow.com/questions/2535284/how-can-i-hash-a-string-to-an-int-using-c
uint16_t hash(const char *str)
{
  uint16_t hash = 5381;
  int c;

  while (c = *str++)
  {
    hash = ((hash << 5) + hash) + c;  // hash * 33 + c
  }

  return hash;
}

BulletServer::BulletServer() :
  rigid_only_(true),
  broadphase_(NULL),
  collision_configuration_(NULL),
  soft_rigid_collision_configuration_(NULL),
  tf_listener_(tf_buffer_)
{
  init();
}

int BulletServer::init()
{
  ros::param::get("~rigid_only", rigid_only_);
  ros::param::get("~tf_prefix", tf_prefix_);

  // TODO(lucasw) can soft body only work with btAxisSweep3?
  if (rigid_only_)
  {
    broadphase_ = new btDbvtBroadphase();
    collision_configuration_ = new btDefaultCollisionConfiguration();
    dispatcher_ = new btCollisionDispatcher(collision_configuration_);
  }
  else
  {
    const int max_proxies = 32766;
    btVector3 world_aabb_min(-1000, -1000, -1000);
    btVector3 world_aabb_max(1000, 1000, 1000);
    broadphase_ = new btAxisSweep3(world_aabb_min, world_aabb_max, max_proxies);
    soft_body_world_info_.m_broadphase = broadphase_;
    soft_rigid_collision_configuration_ = new btSoftBodyRigidBodyCollisionConfiguration();
    dispatcher_ = new btCollisionDispatcher(soft_rigid_collision_configuration_);
    soft_body_world_info_.m_dispatcher = dispatcher_;
  }

  solver_ = new btSequentialImpulseConstraintSolver;

  if (rigid_only_)
  {
    dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_, broadphase_,
        solver_, collision_configuration_);
  }
  else
  {
    dynamics_world_ = new btSoftRigidDynamicsWorld(dispatcher_, broadphase_,
        solver_, soft_rigid_collision_configuration_);
  }
  // TODO(lucasw) provide this in ros param
  btVector3 gravity(0.0, 0.0, -10.0);
  dynamics_world_->setGravity(gravity);
  soft_body_world_info_.m_gravity = gravity;
  // sdf -> 'signed distance field'
  // sparse version of soft body to make collision detection easier
  soft_body_world_info_.air_density = (btScalar)1.2;
  soft_body_world_info_.water_density = 0;
  soft_body_world_info_.water_offset = 0;
  soft_body_world_info_.water_normal = btVector3(0, 0, 0);
  soft_body_world_info_.m_sparsesdf.Initialize();

  // TODO(lucasw) make a service set where the ground plane is, if any
  if (false)
  {
    ground_shape_ = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(0, 0, 0));
    ground_motion_state_ = new btDefaultMotionState(transform);
    // setting inertia to zero makes the body static
    const btVector3 inertia(0, 0, 0);
    btRigidBody::btRigidBodyConstructionInfo
        ground_rigid_body_ci(0, ground_motion_state_, ground_shape_, inertia);
    ground_rigid_body_ = new btRigidBody(ground_rigid_body_ci);
    ground_rigid_body_->setFriction(1.4);
    dynamics_world_->addRigidBody(ground_rigid_body_);
  }

  // TODO(lucasw) could have multiple threads, but would need proper protections
  timer_ = nh_.createTimer(ros::Duration(0.1),
      &BulletServer::update, this);

	// dynamic reconfigure init
	{
		reconfigure_server_.reset(
				new ReconfigureServer(dr_mutex_, nh_));
		dynamic_reconfigure::Server<bullet_server::BulletServerConfig>::CallbackType bsc =
			boost::bind(&BulletServer::reconfigureCallback, this, _1, _2);
		reconfigure_server_->setCallback(bsc);
	}

  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  // TODO(lucasw) make this a service
  // rostopic pub /add_body bullet_server/Body "{name: 'test6', pose:
  // {position: {x: 0.201, y: 0.001, z: 10}, orientation: {w: 1}}}" -1
  body_sub_ = nh_.subscribe("add_body", 10, &BulletServer::bodyCallback, this);
  heightfield_sub_ = nh_.subscribe("add_heightfield", 10, &BulletServer::heightfieldCallback, this);
  constraint_sub_ = nh_.subscribe("add_constraint", 10, &BulletServer::constraintCallback, this);
  impulse_sub_ = nh_.subscribe("add_impulse", 10, &BulletServer::impulseCallback, this);

  add_compound_ = nh_.advertiseService("add_compound", &BulletServer::addCompound, this);
  add_raycast_ = nh_.advertiseService("add_raycast", &BulletServer::addRaycast, this);
  add_laser_scan_ = nh_.advertiseService("add_laser_scan", &BulletServer::addLaserScan, this);

  republish_markers_sub_ = nh_.subscribe("republish_markers", 3,
      &BulletServer::republishMarkers, this);

  return 0;
}

void BulletServer::reconfigureCallback(
    bullet_server::BulletServerConfig& config,
    uint32_t level)
{
  config_ = config;
  // TODO(lucasw) adjust timer with new time step
  timer_.setPeriod(ros::Duration(config_.target_time_step), false);
}

void BulletServer::republishMarkers(const std_msgs::Empty::ConstPtr&)
{
  // loop through all bodies and have them republish their markers
  // std::map<std::string, Body*> bodies_;
  // std::map<std::string, SoftBody*> soft_bodies_;
  for (const auto& body : bodies_)
  {
    body.second->publishMarker();
    ros::Duration(0.05).sleep();
  }
}

bool BulletServer::addCompound(bullet_server::AddCompound::Request& req,
                               bullet_server::AddCompound::Response& res)
{
  res.success = true;
  for (size_t i = 0; i < req.body.size(); ++i)
  {
    if (req.remove)
    {
      // TODO(lucasw) if name doesn't exist, then return false?
      const std::string name = req.body[i].name;
      if (bodies_.count(name) > 0)
      {
        delete bodies_[name];
        bodies_.erase(name);
      }
    }
    else
    {
      // TODO(lucasw) need a return value for each of these
      bullet_server::Body::ConstPtr body_ptr(new bullet_server::Body(req.body[i]));
      bodyCallback(body_ptr);
    }
  }

  for (size_t i = 0; i < req.soft_body.size(); ++i)
  {
    if (req.remove)
    {
      // TODO(lucasw) if name doesn't exist, then return false?
      const std::string name = req.soft_body[i].name;
      if (soft_bodies_.count(name) > 0)
      {
        delete soft_bodies_[name];
        soft_bodies_.erase(name);
      }
    }
    else
    {
      // TODO(lucasw) need a return value for each of these
      bullet_server::SoftBody::ConstPtr body_ptr(new bullet_server::SoftBody(req.soft_body[i]));
      if (!softBodyCallback(body_ptr))
        res.success = false;
    }
  }

  for (size_t i = 0; i < req.constraint.size(); ++i)
  {
    if (req.remove)
    {
      const std::string name = req.constraint[i].name;
      if (constraints_.count(name) > 0)
      {
        Constraint* constraint = constraints_[name];
        removeConstraint(constraints_[name], true);
        delete constraint;
      }
    }
    else
    {
      bullet_server::Constraint::ConstPtr constraint_ptr(new bullet_server::Constraint(req.constraint[i]));
      constraintCallback(constraint_ptr);
    }
  }

  return res.success;
}

bool BulletServer::addRaycast(bullet_server::AddRaycast::Request& req,
                              bullet_server::AddRaycast::Response& res)
{
  if (raycasts_.count(req.name) > 0)
  {
    delete raycasts_[req.name];
  }

  raycasts_[req.name] = new Raycast(req.name, req.frame_id,
      req.lines,
      req.topic_name,
      nh_,
      dynamics_world_);

  return true;
}

// TODO(lucasw) this is where a plugin architecture starts to become necessary,
// the LaserScan is actually a specific kind of Raycast
bool BulletServer::addLaserScan(bullet_server::AddLaserScan::Request& req,
                                bullet_server::AddLaserScan::Response& res)
{
  if (raycasts_.count(req.name) > 0)
  {
    delete raycasts_[req.name];
  }

  raycasts_[req.name] = new Raycast(req.name,
      req.laser_scan,
      req.topic_name,
      nh_,
      dynamics_world_);

  return true;
}

void BulletServer::bodyCallback(const bullet_server::Body::ConstPtr& msg)
{
  if (bodies_.count(msg->name) > 0)
  {
    delete bodies_[msg->name];
  }

  bodies_[msg->name] = new Body(this, msg->name, msg->type, msg->mass,
      msg->pose, msg->twist, msg->scale,
      dynamics_world_, &br_, &marker_array_pub_, tf_prefix_);
}

bool BulletServer::softBodyCallback(const bullet_server::SoftBody::ConstPtr& msg)
{
  btSoftRigidDynamicsWorld* soft_rigid_dynamics_world =
      dynamic_cast<btSoftRigidDynamicsWorld*>(dynamics_world_);

  if (rigid_only_ || (soft_rigid_dynamics_world == NULL))
  {
    ROS_ERROR_STREAM("can't create soft body in rigid body mode " + msg->name);
    return false;
  }

  if (msg->name == "")
  {
    ROS_ERROR_STREAM("empty name");
    return false;
  }

  if (soft_bodies_.count(msg->name) > 0)
  {
    delete soft_bodies_[msg->name];
  }

  // std::vector
  soft_bodies_[msg->name] = new SoftBody(this, msg->name,
      &soft_body_world_info_,
      msg->node, msg->link, msg->face, msg->tetra,
      msg->material, msg->anchor,
      msg->config,
      soft_rigid_dynamics_world,
      &br_, &marker_array_pub_);

  soft_rigid_dynamics_world->addSoftBody(soft_bodies_[msg->name]->soft_body_);

  return true;
}


void BulletServer::constraintCallback(const bullet_server::Constraint::ConstPtr& msg)
{
  // Can't handle joints between bodies that don't exist yet
  if (msg->body_a == msg->body_b)
  {
    ROS_WARN_STREAM("can't have constraint on same body "
        << msg->body_a << " " << msg->body_b);
    return;
  }
  if (bodies_.count(msg->body_a) == 0)
  {
    ROS_WARN_STREAM("body does not exist " << msg->body_a);
    return;
  }
  if (bodies_.count(msg->body_b) == 0)
  {
    ROS_WARN_STREAM("body does not exist " << msg->body_b);
    return;
  }
  if (constraints_.count(msg->name) > 0)
  {
    ROS_INFO_STREAM("removing same named constraint: " << msg->name);
    Constraint* constraint = constraints_[msg->name];
    removeConstraint(constraints_[msg->name], true);
    delete constraint;
  }
  Constraint* constraint = new Constraint(
      msg->name,
      msg->type,
      bodies_[msg->body_a],
      bodies_[msg->body_b],
      msg->pivot_in_a,
      msg->pivot_in_b,
      msg->axis_in_a,
      msg->axis_in_b,
      msg->lower_lin_lim,
      msg->upper_lin_lim,
      msg->lower_ang_lim,
      msg->upper_ang_lim,
      msg->max_motor_impulse,
      msg->enable_pos_pub,
      msg->enable_motor_sub,
      dynamics_world_,
      &marker_array_pub_);

  constraints_[msg->name] = constraint;
}

void BulletServer::impulseCallback(const bullet_server::Impulse::ConstPtr& msg)
{
  if (bodies_.count(msg->body) == 0)
  {
    ROS_WARN_STREAM("body does not exist " << msg->body);
    return;
  }
  // ROS_INFO_STREAM("impulse " << msg->body << "\n" << msg->location << "\n" << msg->impulse);
  const btVector3 point_rel_body(msg->location.x, msg->location.y, msg->location.z);
  const btVector3 impulse(msg->impulse.x, msg->impulse.y, msg->impulse.z);
  bodies_[msg->body]->rigid_body_->activate();
  bodies_[msg->body]->rigid_body_->applyImpulse(impulse, point_rel_body);
}

void BulletServer::heightfieldCallback(const bullet_server::Heightfield::ConstPtr& msg)
{
  // convert to cv Mat - TODO(lucasw) do I need to?
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (bodies_.count(msg->name) > 0)
  {
    delete bodies_[msg->name];
  }

  bodies_[msg->name] = new Body(this, msg->name,
      cv_ptr->image, msg->resolution, msg->height_scale, msg->flip_quad_edges,
      dynamics_world_, &br_, &marker_array_pub_, tf_prefix_);
}


void BulletServer::update(const ros::TimerEvent& e)
{
  // Only do this when not generating clock
  dynamics_world_->stepSimulation((e.current_real - e.last_real).toSec());  // ,
  //    config_.max_sub_steps);

  for (std::map<std::string, Body*>::iterator it = bodies_.begin();
      it != bodies_.end(); ++it)
  {
    it->second->update();
  }
  for (std::map<std::string, SoftBody*>::iterator it = soft_bodies_.begin();
      it != soft_bodies_.end(); ++it)
  {
    it->second->update();
  }
  // TODO(lucasw) maybe Body and Constraint should inherit from same base
  for (std::map<std::string, Constraint*>::iterator it = constraints_.begin();
      it != constraints_.end(); ++it)
  {
    it->second->update();
  }

  for (auto it : raycasts_)
  {
    it.second->update(tf_buffer_);
  }
}

void BulletServer::removeConstraint(const Constraint* constraint,
    const bool remove_from_bodies)
{
  if (remove_from_bodies)
  {
    constraint->body_a_->removeConstraint(constraint);
    constraint->body_b_->removeConstraint(constraint);
  }

  // This doesn't delete the constraint just removes it (most likely
  // because it is about to be deleted from the a Body it is
  // attached to).
  ROS_DEBUG_STREAM("BulletServer: remove constraint " << constraint->name_);
  constraints_.erase(constraint->name_);
}

BulletServer::~BulletServer()
{
  for (std::map<std::string, Body*>::iterator it = bodies_.begin();
      it != bodies_.end(); ++it)
  {
    delete it->second;
  }

  if (false)
  {
  dynamics_world_->removeRigidBody(ground_rigid_body_);
  delete ground_rigid_body_->getMotionState();
  delete ground_rigid_body_;
  delete ground_shape_;
  }

  delete dynamics_world_;
  delete solver_;

  if (collision_configuration_)
    delete collision_configuration_;
  if (soft_rigid_collision_configuration_)
    delete soft_rigid_collision_configuration_;

  delete dispatcher_;
  delete broadphase_;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bullet_server");
  BulletServer bullet_server;
  ros::spin();
}

