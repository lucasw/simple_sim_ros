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
  soft_rigid_collision_configuration_(NULL)
{
  init();
}

int BulletServer::init()
{
  ros::param::get("~rigid_only", rigid_only_);

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

  period_ = 1.0 / 60.0;

  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  // TODO(lucasw) make this a service
  // rostopic pub /add_body bullet_server/Body "{name: 'test6', pose:
  // {position: {x: 0.201, y: 0.001, z: 10}, orientation: {w: 1}}}" -1
  body_sub_ = nh_.subscribe("add_body", 10, &BulletServer::bodyCallback, this);
  heightfield_sub_ = nh_.subscribe("add_heightfield", 10, &BulletServer::heightfieldCallback, this);
  constraint_sub_ = nh_.subscribe("add_constraint", 10, &BulletServer::constraintCallback, this);
  impulse_sub_ = nh_.subscribe("add_impulse", 10, &BulletServer::impulseCallback, this);

  add_compound_ = nh_.advertiseService("add_compound", &BulletServer::addCompound, this);

  return 0;
}

bool BulletServer::addCompound(bullet_server::AddCompound::Request& req,
                               bullet_server::AddCompound::Response& res)
{
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
      softBodyCallback(body_ptr);
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

  res.success = true;
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
      dynamics_world_, &br_, &marker_array_pub_);
}

void BulletServer::softBodyCallback(const bullet_server::SoftBody::ConstPtr& msg)
{
  btSoftRigidDynamicsWorld* soft_rigid_dynamics_world =
      dynamic_cast<btSoftRigidDynamicsWorld*>(dynamics_world_);

  if (rigid_only_ || (soft_rigid_dynamics_world == NULL))
  {
    ROS_ERROR_STREAM("can't create soft body in rigid body mode " + msg->name);
    return;
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
      dynamics_world_, &br_, &marker_array_pub_);
}


void BulletServer::update()
{
  // TODO(lucasw) make these parameters dynamically configurable
  dynamics_world_->stepSimulation(period_, 10);

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

  ros::spinOnce();
  ros::Duration(period_).sleep();
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

///////////////////////////////////////////////////////////////////////////////
SoftBody::SoftBody(BulletServer* parent,
    const std::string name,
    btSoftBodyWorldInfo* soft_body_world_info,
    const std::vector<bullet_server::Node>& nodes,
    const std::vector<bullet_server::Link>& links,
    const std::vector<bullet_server::Face>& faces,
    const std::vector<bullet_server::Tetra>& tetras,
    const std::vector<bullet_server::Material>& materials,
    const std::vector<bullet_server::Anchor>& anchors,
    const bullet_server::SoftConfig& config,
    btSoftRigidDynamicsWorld* dynamics_world,
    tf::TransformBroadcaster* br,
    ros::Publisher* marker_array_pub) :
  dynamics_world_(dynamics_world),
  name_(name),
  br_(br),
  marker_array_pub_(marker_array_pub)
{
  btVector3* points = new btVector3[nodes.size()];
  btScalar* masses = new btScalar[nodes.size()];
  for (size_t i = 0; i < nodes.size(); ++i)
  {
    btVector3 pos(nodes[i].position.x, nodes[i].position.y, nodes[i].position.z);
    btScalar mass(nodes[i].mass);
    // this was segfaulting
    // soft_body_->appendNode(pos, mass);
    points[i] = pos;
    masses[i] = mass;
    // ROS_INFO_STREAM(pos << " " << mass);
  }

  soft_body_ = new btSoftBody(soft_body_world_info, nodes.size(), points, masses);
  delete[] points;
  delete[] masses;

  soft_body_->m_cfg.kVCF = config.kVCF;
  soft_body_->m_cfg.kDP = config.kDP;
  soft_body_->m_cfg.kDG = config.kDG;
  soft_body_->m_cfg.kLF = config.kLF;
  soft_body_->m_cfg.kPR = config.kPR;
  soft_body_->m_cfg.kVC = config.kVC;
  soft_body_->m_cfg.kDF = config.kDF;
  soft_body_->m_cfg.kMT = config.kMT;
  soft_body_->m_cfg.kCHR = config.kCHR;
  soft_body_->m_cfg.kKHR = config.kKHR;
  soft_body_->m_cfg.kSHR = config.kSHR;
  soft_body_->m_cfg.kAHR = config.kAHR;
  // TODO(lucasw) cluster stuff
  soft_body_->m_cfg.maxvolume = config.maxvolume;
  soft_body_->m_cfg.timescale = config.timescale;

  btSoftBody::Material* pm = NULL;
  for (size_t i = 0; i < materials.size(); ++i)
  {
    // btSoftBody::Material*
    pm = soft_body_->appendMaterial();
    pm->m_kLST = materials[i].kLST;
    pm->m_kAST = materials[i].kAST;
    pm->m_kVST = materials[i].kVST;
    // const int distance = 1;
    // soft_body_->generateBendingConstraints(distance, pm);
    ROS_INFO_STREAM(name_ << " " << pm->m_kLST);
  }

  for (size_t i = 0; i < links.size(); ++i)
  {
    // TODO(lucasw) need to provide an optional material index
    // for each link
    // With bcheckexist set to true redundant links ought
    // to be filtered out.
    soft_body_->appendLink(links[i].node_indices[0],
      links[i].node_indices[1], pm, true);
  }
  for (size_t i = 0; i < faces.size(); ++i)
  {
    soft_body_->appendFace(faces[i].node_indices[0],
      faces[i].node_indices[1],
      faces[i].node_indices[2], pm);
  }
  for (size_t i = 0; i < tetras.size(); ++i)
  {
    soft_body_->appendTetra(tetras[i].node_indices[0],
      tetras[i].node_indices[1],
      tetras[i].node_indices[2],
      tetras[i].node_indices[3], pm);
  }

#if 0
  for (size_t i = 0; i < materials.size(); ++i)
  {
    btSoftBody::Material* pm = soft_body_->appendMaterial();
    pm->m_kLST = materials[i].kLST;
    pm->m_kAST = materials[i].kAST;
    pm->m_kVST = materials[i].kVST;
    const int distance = 1;
    soft_body_->generateBendingConstraints(distance, pm);
    ROS_INFO_STREAM(name_ << " " << pm->m_kLST);
  }
#endif
  for (size_t i = 0; i < anchors.size(); ++i)
  {
    if (parent->bodies_.count(anchors[i].rigid_body_name) == 0)
    {
      ROS_ERROR_STREAM("no rigid body " << anchors[i].rigid_body_name
        << " to append anchor to");
      continue;
    }
    btRigidBody* rigid_body = parent->bodies_[anchors[i].rigid_body_name]->rigid_body_;
    const btVector3 local_pivot(
      anchors[i].local_pivot.x,
      anchors[i].local_pivot.y,
      anchors[i].local_pivot.z);

    soft_body_->appendAnchor(anchors[i].node_index, rigid_body,
      local_pivot, anchors[i].disable_collision_between_linked_bodies,
      anchors[i].influence);
  }

  {
    visualization_msgs::Marker marker;
    // TODO(lucasw) also have a LINES and TRIANGLE_LIST marker
    marker.type = visualization_msgs::Marker::POINTS;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.ns = "nodes";
    // marker_.header.stamp = ros::Time::now();
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // TODO(lucasw) could turn this into function
    marker.id = hash(name_.c_str());
    marker.header.frame_id = "map";
    marker.color.r = 0.45;
    marker.color.g = 0.4;
    marker.color.b = 0.65;
    marker_array_.markers.push_back(marker);
  }

  // link markers
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.005;
    marker.scale.y = 0.005;
    marker.scale.z = 0.005;
    marker.ns = "links";
    // marker_.header.stamp = ros::Time::now();
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // TODO(lucasw) could turn this into function
    marker.id = hash((name_ + "lines").c_str());
    marker.header.frame_id = "map";
    marker.color.r = 0.3;
    marker.color.g = 0.67;
    marker.color.b = 0.65;
    marker_array_.markers.push_back(marker);
  }

  // anchor markers
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.ns = "anchors";
    // marker_.header.stamp = ros::Time::now();
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // TODO(lucasw) could turn this into function
    marker.id = hash((name_ + "anchors").c_str());
    marker.header.frame_id = "map";
    marker.color.r = 0.67;
    marker.color.g = 0.17;
    marker.color.b = 0.95;
    marker_array_.markers.push_back(marker);

    marker.ns = "anchor_pivots";
    marker.color.r = 0.67;
    marker.color.g = 0.87;
    marker.color.b = 0.45;
    marker_array_.markers.push_back(marker);
  }

  // tetra markers
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.ns = "tetras";
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    marker.id = hash((name_ + "tetras").c_str());
    marker.header.frame_id = "map";
    marker.color.r = 0.0;
    marker.color.g = 0.67;
    marker.color.b = 0.75;
    marker_array_.markers.push_back(marker);
  }
}

SoftBody::~SoftBody()
{
  if (soft_body_)
  {
    dynamics_world_->removeSoftBody(soft_body_);
    delete soft_body_;
  }
}

void SoftBody::update()
{
  // TODO(lucasw) getAabb

  // TODO(lucasw) instead of hardcoded markers indices, do something better

  btSoftBody::tNodeArray& nodes(soft_body_->m_nodes);
  marker_array_.markers[0].points.clear();
  for (size_t i = 0; i < nodes.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = nodes[i].m_x.getX();
    pt.y = nodes[i].m_x.getY();
    pt.z = nodes[i].m_x.getZ();
    marker_array_.markers[0].points.push_back(pt);
  }

  btSoftBody::tLinkArray& links(soft_body_->m_links);
  marker_array_.markers[1].points.clear();
  for (size_t i = 0; i < links.size(); ++i)
  {
    geometry_msgs::Point pt1;
    pt1.x = links[i].m_n[0]->m_x.getX();
    pt1.y = links[i].m_n[0]->m_x.getY();
    pt1.z = links[i].m_n[0]->m_x.getZ();
    marker_array_.markers[1].points.push_back(pt1);

    geometry_msgs::Point pt2;
    pt2.x = links[i].m_n[1]->m_x.getX();
    pt2.y = links[i].m_n[1]->m_x.getY();
    pt2.z = links[i].m_n[1]->m_x.getZ();
    marker_array_.markers[1].points.push_back(pt2);
  }

  btSoftBody::tTetraArray& tetras(soft_body_->m_tetras);
  marker_array_.markers[4].points.clear();
  for (size_t i = 0; i < tetras.size(); ++i)
  {
    // the indices ought to be ordered so that
    // the right hand rule will be an outward normal here.
    int tr[4][3] = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
    for (size_t j = 0; j < 4; ++j)
    {
      for (size_t k = 0; k < 3; ++k)
      {
        const btSoftBody::Node* node = tetras[i].m_n[tr[j][k]];
        geometry_msgs::Point pt;
        pt.x = node->m_x.getX();
        pt.y = node->m_x.getY();
        pt.z = node->m_x.getZ();
        marker_array_.markers[4].points.push_back(pt);
      }
    }
  }

  // TODO(lucasw) also do something with faces

  btSoftBody::tAnchorArray& anchors(soft_body_->m_anchors);
  marker_array_.markers[2].points.clear();
  marker_array_.markers[3].points.clear();
  for (size_t i = 0; i < anchors.size(); ++i)
  {
    geometry_msgs::Point pt1;
    pt1.x = anchors[i].m_node->m_x.getX();
    pt1.y = anchors[i].m_node->m_x.getY();
    pt1.z = anchors[i].m_node->m_x.getZ();
    marker_array_.markers[2].points.push_back(pt1);

    btTransform trans;
    anchors[i].m_body->getMotionState()->getWorldTransform(trans);

    btVector3 world_point = trans * anchors[i].m_local;
    geometry_msgs::Point pt2;
    pt2.x = world_point.getX();
    pt2.y = world_point.getY();
    pt2.z = world_point.getZ();
    marker_array_.markers[2].points.push_back(pt2);
    marker_array_.markers[3].points.push_back(pt2);

    geometry_msgs::Point pt3;
    pt3.x = trans.getOrigin().getX();
    pt3.y = trans.getOrigin().getY();
    pt3.z = trans.getOrigin().getZ();
    marker_array_.markers[3].points.push_back(pt3);
  }

  marker_array_pub_->publish(marker_array_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bullet_server");
  BulletServer bullet_server;

  while (ros::ok())
  {
    bullet_server.update();
  }
}

