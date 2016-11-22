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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Body;
class Constraint;
class SoftBody;

// TODO(lucasw) replace this with something better, numbers aren't random enough
// http://stackoverflow.com/questions/2535284/how-can-i-hash-a-string-to-an-int-using-c
unsigned short hash(const char *str) {
    unsigned short hash = 5381;
    int c;

    while (c = *str++) {
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
    }

    return hash;
}

class BulletServer
{
  ros::NodeHandle nh_;
  ros::ServiceServer add_compound_;
  bool addCompound(bullet_server::AddCompound::Request& req,
                   bullet_server::AddCompound::Response& res);
  ros::Subscriber body_sub_;
  void bodyCallback(const bullet_server::Body::ConstPtr& msg);
  void softBodyCallback(const bullet_server::SoftBody::ConstPtr& msg);
  ros::Subscriber constraint_sub_;
  void constraintCallback(const bullet_server::Constraint::ConstPtr& msg);
  ros::Subscriber impulse_sub_;
  void impulseCallback(const bullet_server::Impulse::ConstPtr& msg);
  ros::Subscriber heightfield_sub_;
  void heightfieldCallback(const bullet_server::Heightfield::ConstPtr& msg);

  tf::TransformBroadcaster br_;
  float period_;
  // ros::Publisher marker_pub_;
  ros::Publisher marker_array_pub_;

  btBroadphaseInterface* broadphase_;
  // btDefaultCollisionConfiguration* collision_configuration_;
  btSoftBodyRigidBodyCollisionConfiguration* collision_configuration_;
  btCollisionDispatcher* dispatcher_;
  // only used with opencl
  // btSoftBodySolver* soft_body_solver_;
  btSequentialImpulseConstraintSolver* solver_;
  // btDiscreteDynamicsWorld* dynamics_world_;
  btSoftRigidDynamicsWorld* dynamics_world_;

  // TODO(lucasw) make this configurable by addCompound
  // instead of hard coded
  btCollisionShape* ground_shape_;
  btDefaultMotionState* ground_motion_state_;
  btRigidBody* ground_rigid_body_;

  btSoftBodyWorldInfo soft_body_world_info_;
  std::map<std::string, SoftBody*> soft_bodies_;
  std::map<std::string, Constraint*> constraints_;

  int init();
public:
  BulletServer();
  ~BulletServer();
  void update();
  void removeConstraint(const Constraint* constraint, const bool remove_from_bodies);
  std::map<std::string, Body*> bodies_;
};

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
  // btDiscreteDynamicsWorld* dynamics_world_;
  btSoftRigidDynamicsWorld* dynamics_world_;
  int state_;
public:
  Body(BulletServer* parent,
      const std::string name,
      unsigned int type,
      const float mass,
      geometry_msgs::Pose pose,
      geometry_msgs::Vector3 scale,
      // btDiscreteDynamicsWorld* dynamics_world,
      btSoftRigidDynamicsWorld* dynamics_world,
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
    // btDiscreteDynamicsWorld* dynamics_world,
    btSoftRigidDynamicsWorld* dynamics_world,
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
      btSoftRigidDynamicsWorld* dynamics_world,
      tf::TransformBroadcaster* br,
      ros::Publisher* marker_array_pub);
  ~SoftBody();
  void update();
  btSoftBody* soft_body_;
};

class Constraint
{
  ros::NodeHandle nh_;
  btTypedConstraint* constraint_;

  // btDiscreteDynamicsWorld* dynamics_world_;
  btSoftRigidDynamicsWorld* dynamics_world_;
  ros::Publisher* marker_array_pub_;
  std::map<std::string, float> command_;
  std::map<std::string, ros::Publisher> pubs_;
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
      // btDiscreteDynamicsWorld* dynamics_world,
      btSoftRigidDynamicsWorld* dynamics_world,
      ros::Publisher* marker_array_pub);
  ~Constraint();

  const std::string name_;
  Body* body_a_;
  Body* body_b_;
  void update();
};

Constraint::Constraint(
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
      // btDiscreteDynamicsWorld* dynamics_world,
      btSoftRigidDynamicsWorld* dynamics_world,
      ros::Publisher* marker_array_pub) :
    nh_(name),
    name_(name),
    body_a_(body_a),
    body_b_(body_b),
    max_motor_impulse_(max_motor_impulse),
    dynamics_world_(dynamics_world),
    marker_array_pub_(marker_array_pub)
{
  ROS_DEBUG_STREAM("new Constraint " << name << " " << body_a->name_ << " " << body_b->name_);
  // TODO(lucasw) pivot -> translation or similar
  const btVector3 pivot_in_a_bt(pivot_in_a.x, pivot_in_a.y, pivot_in_a.z);
  const btVector3 pivot_in_b_bt(pivot_in_b.x, pivot_in_b.y, pivot_in_b.z);
  const btVector3 axis_in_a_bt(axis_in_a.x, axis_in_a.y, axis_in_a.z);
  const btVector3 axis_in_b_bt(axis_in_b.x, axis_in_b.y, axis_in_b.z);
  bool dont_collide = true;
  // TODO(lucasw) instead of this big switch here, need to have
  // subclasses
  if (type == bullet_server::Constraint::HINGE)
  {
    // TODO(lucasw) what about hinge2?
    btHingeConstraint* hinge = new btHingeConstraint(
        *body_a->rigid_body_,
        *body_b->rigid_body_,
        pivot_in_a_bt,
        pivot_in_b_bt,
        axis_in_a_bt,
        axis_in_b_bt);

    // need flag to set no limits?
    hinge->setLimit(lower_ang_lim, upper_ang_lim);
    pubs_["angular_pos"] = nh_.advertise<std_msgs::Float64>("angular_pos", 1);
    const std::string motor_name = "target_ang_motor_vel";
    command_[motor_name] = 0;
    subs_[motor_name] = nh_.subscribe<std_msgs::Float64>(motor_name, 1,
                                                         boost::bind(&Constraint::commandCallback,
                                                                     this, _1, motor_name));


    constraint_ = hinge;
  }
  else if (type == bullet_server::Constraint::SLIDER)
  {
    // the x-axis is where the slider joint will be along
    // This works for making the axis align with cylinder objects, may not be desired
    // for other uses.
    const btTransform frame_in_a = btTransform(btQuaternion(0.5, -0.5, 0.5, -0.5), pivot_in_a_bt);
    const btTransform frame_in_b = btTransform(btQuaternion(0.5, -0.5, 0.5, -0.5), pivot_in_b_bt);
    const bool use_linear_reference_frame_a = true;

    btSliderConstraint* slider = new btSliderConstraint(
        *body_a->rigid_body_,
        *body_b->rigid_body_,
        frame_in_a,
        frame_in_b,
        use_linear_reference_frame_a);

    pubs_["linear_pos"] = nh_.advertise<std_msgs::Float64>("linear_pos", 1);
    // body_sub_ = nh_.subscribe("add_body", 10, &BulletServer::bodyCallback, this);
    const std::string motor_name = "target_lin_motor_vel";
    command_[motor_name] = 0;
    subs_[motor_name] = nh_.subscribe<std_msgs::Float64>(motor_name, 1,
                                                         boost::bind(&Constraint::commandCallback,
                                                                     this, _1, motor_name));

    // TODO(lucasw) make these controllable via service (or topic?)
    slider->setLowerAngLimit(lower_ang_lim);
    slider->setUpperAngLimit(upper_ang_lim);
    slider->setPoweredAngMotor(true);
    slider->setMaxAngMotorForce(max_motor_impulse);
    slider->setTargetAngMotorVelocity(0);

    slider->setLowerLinLimit(lower_lin_lim);
    slider->setUpperLinLimit(upper_lin_lim);
    slider->setPoweredLinMotor(true);
    slider->setMaxLinMotorForce(max_motor_impulse);
    slider->setTargetLinMotorVelocity(0);

    constraint_ = slider;

    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::SPHERE;
      // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
      // KDL::Rotation(-M_PI_2, 0, 0)?
      // tf::Quaternion quat = tf::createQuaternionFromRPY();
      // tf::Matrix3x3(quat)
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.ns = "constraints";
      // marker_.header.stamp = ros::Time::now();
      marker.frame_locked = true;
      marker.action = visualization_msgs::Marker::ADD;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration();

      // TODO(lucasw) could turn this into function
      marker.id = hash(name.c_str());
      marker.header.frame_id = body_a->name_;
      marker.pose.position = pivot_in_a;
      marker.color.r = 0.5;
      marker.color.g = 0.7;
      marker.color.b = 0.3;
      marker_array_.markers.push_back(marker);

      marker.id = hash((name + "_b").c_str());
      marker.header.frame_id = body_b->name_;
      marker.pose.position = pivot_in_b;
      marker.color.r = 0.6;
      marker.color.g = 0.3;
      marker.color.b = 0.7;
      marker_array_.markers.push_back(marker);

      // draw lines from the origin to the pivot
      marker.scale.x = 0.05;

      marker.id = hash((name + "_line_a").c_str());
      marker.header.frame_id = body_a->name_;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.points.resize(2);
      marker.points[1] = pivot_in_a;
      marker_array_.markers.push_back(marker);

      marker.id = hash((name + "_line_b").c_str());
      marker.header.frame_id = body_b->name_;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.points.resize(2);
      marker.points[1] = pivot_in_b;
      marker_array_.markers.push_back(marker);
    }

    marker_array_pub_->publish(marker_array_);
  }
  else if (type == bullet_server::Constraint::FIXED)
  {
    #if BT_BULLET_VERSION >= 282
    // TODO(lucasw) need orientation
    const btTransform frame_in_a = btTransform(btQuaternion(0, 0, 0, 1), pivot_in_a_bt);
    const btTransform frame_in_b = btTransform(btQuaternion(0, 0, 0, 1), pivot_in_b_bt);

    constraint_ = new btFixedConstraint(
        *body_a->rigid_body_,
        *body_b->rigid_body_,
        frame_in_a,
        frame_in_b);
    #else
    ROS_ERROR_STREAM("fixed joint not supported in this bullet version " << BT_BULLET_VERSION);
    #endif
  }
  if (type == bullet_server::Constraint::POINT2POINT)
  {
    // TODO(lucasw) set this in Constraint?
    dont_collide = false;
    // ROS_INFO_STREAM(name << " " << body_a->name_ << " " << body_b->name_
    //     << pivot_in_a << " " << pivot_in_b);
    constraint_ = new btPoint2PointConstraint(
        *body_a->rigid_body_,
        *body_b->rigid_body_,
        pivot_in_a_bt,
        pivot_in_b_bt);

    // TODO(lucasw) publish a marker for both bodies- a line to the center of the body
    // to the pivot, and then a sphere at the ball joint
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::SPHERE;
      // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
      // KDL::Rotation(-M_PI_2, 0, 0)?
      // tf::Quaternion quat = tf::createQuaternionFromRPY();
      // tf::Matrix3x3(quat)
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.ns = "constraints";
      // marker_.header.stamp = ros::Time::now();
      marker.frame_locked = true;
      marker.action = visualization_msgs::Marker::ADD;
      marker.color.a = 1.0;
      marker.lifetime = ros::Duration();

      // TODO(lucasw) could turn this into function
      marker.id = hash(name.c_str());
      marker.header.frame_id = body_a->name_;
      marker.pose.position = pivot_in_a;
      marker.color.r = 0.5;
      marker.color.g = 0.7;
      marker.color.b = 0.3;
      marker_array_.markers.push_back(marker);

      marker.id = hash((name + "_b").c_str());
      marker.header.frame_id = body_b->name_;
      marker.pose.position = pivot_in_b;
      marker.color.r = 0.6;
      marker.color.g = 0.3;
      marker.color.b = 0.7;
      marker_array_.markers.push_back(marker);

      // draw lines from the origin to the pivot
      marker.scale.x = 0.1;

      marker.id = hash((name + "_line_a").c_str());
      marker.header.frame_id = body_a->name_;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.points.resize(2);
      marker.points[1] = pivot_in_a;
      marker_array_.markers.push_back(marker);

      marker.id = hash((name + "_line_b").c_str());
      marker.header.frame_id = body_b->name_;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.points.resize(2);
      marker.points[1] = pivot_in_b;
      marker_array_.markers.push_back(marker);
    }

    marker_array_pub_->publish(marker_array_);
  }
  dynamics_world_->addConstraint(constraint_, dont_collide);
  body_a->addConstraint(this);
  body_b->addConstraint(this);
  body_a->rigid_body_->activate();
  body_b->rigid_body_->activate();
}

Constraint::~Constraint()
{
  ROS_DEBUG_STREAM("Constraint: delete " << name_ << " "
    << dynamics_world_->getNumConstraints());

  for (size_t i = 0; i < marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
    marker_array_.markers[i].points.resize(0);
  }
  marker_array_pub_->publish(marker_array_);

  // TODO(lucasw) it needs to be removed from BulletServer->constraints_
  dynamics_world_->removeConstraint(constraint_);
  delete constraint_;
  ROS_INFO_STREAM("remaining constraints in world " << dynamics_world_->getNumConstraints());
}

void Constraint::update()
{
  // TODO(lucasw) get rid of this with subclasses
  btSliderConstraint* slider = dynamic_cast<btSliderConstraint*>(constraint_);
  if (slider)
  {
    std_msgs::Float64 msg;
    msg.data = slider->getLinearPos();
    pubs_["linear_pos"].publish(msg);

    // TODO(lucasw) need to be able to disable the motor
    const float vel = command_["target_lin_motor_vel"];
    slider->setTargetLinMotorVelocity(btScalar(vel));
    ROS_DEBUG_STREAM(name_ << " slider " << vel);
  }
  btHingeConstraint* hinge = dynamic_cast<btHingeConstraint*>(constraint_);
  if (hinge)
  {
    // TODO(lucasw) detect roll-over and make this continuous
    // rather than -pi to pi
    std_msgs::Float64 msg;
    msg.data = hinge->getHingeAngle();
    pubs_["angular_pos"].publish(msg);

    const float vel = command_["target_ang_motor_vel"];
    hinge->enableAngularMotor(true, vel, max_motor_impulse_);
  }
}

void Constraint::commandCallback(const std_msgs::Float64::ConstPtr msg,
                                 const std::string motor_name)
{
  ROS_DEBUG_STREAM(name_ << ": " << motor_name << " " << msg->data);
  if (command_.count(motor_name) == 0)
    ROS_WARN_STREAM(motor_name << " is not in command");
  command_[motor_name] = msg->data;
  body_a_->rigid_body_->activate();
  body_b_->rigid_body_->activate();
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
BulletServer::BulletServer()
{
  init();
}

int BulletServer::init()
{
  // TODO(lucasw) can soft body only work with btAxisSweep3?
  // broadphase_ = new btDbvtBroadphase();
  const int max_proxies = 32766;
  btVector3 world_aabb_min(-1000, -1000, -1000);
  btVector3 world_aabb_max(1000, 1000, 1000);
  broadphase_ = new btAxisSweep3(world_aabb_min, world_aabb_max, max_proxies);
  soft_body_world_info_.m_broadphase = broadphase_;

  // collision_configuration_ = new btDefaultCollisionConfiguration();
  collision_configuration_ = new btSoftBodyRigidBodyCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(collision_configuration_);
  soft_body_world_info_.m_dispatcher = dispatcher_;
  solver_ = new btSequentialImpulseConstraintSolver;
  // dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher, broadphase_,
  //    solver, collision_configuration_);
  dynamics_world_ = new btSoftRigidDynamicsWorld(dispatcher_, broadphase_,
      solver_, collision_configuration_);
 // TODO(lucasw) provide this in ros param
  btVector3 gravity(0.0, 0.0, -10.0);
  dynamics_world_->setGravity(gravity);
  soft_body_world_info_.m_gravity = gravity;
  // sdf -> 'signed distance field'
  // sparse version of soft body to make collision detection easier
  soft_body_world_info_.air_density   = (btScalar)1.2;
  soft_body_world_info_.water_density = 0;
  soft_body_world_info_.water_offset    = 0;
  soft_body_world_info_.water_normal    = btVector3(0,0,0);
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
      msg->pose, msg->scale,
      dynamics_world_, &br_, &marker_array_pub_);
}

void BulletServer::softBodyCallback(const bullet_server::SoftBody::ConstPtr& msg)
{
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
      dynamics_world_, &br_, &marker_array_pub_);

  dynamics_world_->addSoftBody(soft_bodies_[msg->name]->soft_body_);
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
    const bool remove_from_bodies=false)
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
  delete collision_configuration_;
  delete dispatcher_;
  delete broadphase_;
}

///////////////////////////////////////////////////////////////////////////////

Body::Body(BulletServer* parent,
    const std::string name,
    unsigned int type,
    const float mass,
    geometry_msgs::Pose pose,
    geometry_msgs::Vector3 scale,
    // btDiscreteDynamicsWorld* dynamics_world,
    btSoftRigidDynamicsWorld* dynamics_world,
    tf::TransformBroadcaster* br,
    ros::Publisher* marker_array_pub) :
  parent_(parent),
  vertices_(NULL),
  indices_(NULL),
  index_vertex_arrays_(NULL),
  shape_(NULL),
  rigid_body_(NULL),
  name_(name),
  dynamics_world_(dynamics_world),
  br_(br),
  marker_array_pub_(marker_array_pub),
  state_(-1)
{
  ROS_DEBUG_STREAM("new Body " << name << " " << type);  // << " " << pose);

  // TODO(lucasw) rename this Body to disambiguate?
  if (type == bullet_server::Body::SPHERE)
  {
    visualization_msgs::Marker marker;
    shape_ = new btSphereShape(scale.x);

    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = scale.x * 2;
    marker.scale.y = scale.x * 2;
    marker.scale.z = scale.x * 2;
    marker.pose.orientation.w = 1.0;
    marker_array_.markers.push_back(marker);
  }
  else if (type == bullet_server::Body::BOX)
  {
    shape_ = new btBoxShape(btVector3(scale.x, scale.y, scale.z));

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = scale.x * 2;
    marker.scale.y = scale.y * 2;
    marker.scale.z = scale.z * 2;
    marker_array_.markers.push_back(marker);
  }
  else if (type == bullet_server::Body::CYLINDER)
  {
    // cylinders in bullet have y central axis,
    // cylinder is rviz have a central z axis- so rotate the rviz cylinder
    shape_ = new btCylinderShape(btVector3(scale.x, scale.y, scale.z));

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CYLINDER;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.x = 0.70710678;
    marker.pose.orientation.w = 0.70710678;
    marker.scale.x = scale.x * 2;
    marker.scale.y = scale.x * 2;  // no support for flattened cylinder
    marker.scale.z = scale.y * 2;
    marker_array_.markers.push_back(marker);
  }
  else if (type == bullet_server::Body::CYLINDER)
  {
    // cylinders in bullet have y central axis,
    // cylinder is rviz have a central z axis- so rotate the rviz cylinder
    // TODO(lucasw) why does the cylinder take three parameters when it only uses two?
    shape_ = new btCylinderShape(btVector3(scale.x, scale.y, scale.x));

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CYLINDER;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.x = 0.70710678;
    marker.pose.orientation.w = 0.70710678;
    marker.scale.x = scale.x * 2;
    marker.scale.y = scale.x * 2;
    marker.scale.z = scale.y * 2;
    marker_array_.markers.push_back(marker);
  }
  else if (type == bullet_server::Body::CAPSULE)
  {
    shape_ = new btCapsuleShape(scale.x, scale.y);

    // build the capsule out of three shape because rviz doesn't have
    // a capsule marker
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::CYLINDER;
      // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
      // KDL::Rotation(-M_PI_2, 0, 0)?
      // tf::Quaternion quat = tf::createQuaternionFromRPY();
      // tf::Matrix3x3(quat)
      marker.pose.orientation.x = 0.70710678;
      marker.pose.orientation.w = 0.70710678;
      marker.scale.x = scale.x * 2;  // diameter
      marker.scale.y = scale.x * 2;
      marker.scale.z = scale.y * 2 - scale.x * 2;  // the axis of the cylinder
      marker_array_.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::SPHERE;
      // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
      // KDL::Rotation(-M_PI_2, 0, 0)?
      // tf::Quaternion quat = tf::createQuaternionFromRPY();
      // tf::Matrix3x3(quat)
      marker.pose.position.y = -scale.y + scale.x;
      // marker.pose.orientation.x = 0.70710678;
      // marker.pose.orientation.w = 0.70710678;
      marker.scale.x = scale.x * 2;
      marker.scale.y = scale.x * 2;
      marker.scale.z = scale.x * 2;
      marker_array_.markers.push_back(marker);
    }
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::SPHERE;
      // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
      // KDL::Rotation(-M_PI_2, 0, 0)?
      // tf::Quaternion quat = tf::createQuaternionFromRPY();
      // tf::Matrix3x3(quat)
      marker.pose.position.y = scale.y - scale.x;
      // marker.pose.orientation.x = 0.70710678;
      // marker.pose.orientation.w = 0.70710678;
      marker.scale.x = scale.x * 2;
      marker.scale.y = scale.x * 2;
      marker.scale.z = scale.x * 2;
      marker_array_.markers.push_back(marker);
    }
  }
  else if (type == bullet_server::Body::CONE)
  {
    // TODO(lucasw) construct a cone out of triangles, for now use cylinders
    shape_ = new btConeShape(scale.x, scale.y);

    const int num_segs = 8;
    const float full_height = scale.y;
    const float height = full_height / static_cast<float>(num_segs);
    for (size_t i = 0; i < num_segs; ++i)
    {
      const float diameter = 2.0 * scale.x * (num_segs - i) / static_cast<float>(num_segs);

      // TODO(lucasw) there is an extra gap between the cones I can't account for
      // is the bullet cone slightly taller than scale.y?
      // It is taller by 0.080 when the set height is 1.0
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.pose.position.y = height * i + height/2.0 - full_height / 2.0;
      // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
      // KDL::Rotation(-M_PI_2, 0, 0)?
      // tf::Quaternion quat = tf::createQuaternionFromRPY();
      // tf::Matrix3x3(quat)
      marker.pose.orientation.x = 0.70710678;
      marker.pose.orientation.w = 0.70710678;
      marker.scale.x = diameter;
      marker.scale.y = diameter;
      marker.scale.z = height;  // the length along axis of the cylinder
      marker_array_.markers.push_back(marker);
    }
  }
  else
    return;

  motion_state_ = new btDefaultMotionState(btTransform(
      btQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
      btVector3(pose.position.x, pose.position.y, pose.position.z)));
  // TODO(lucasw) provide in message
  btScalar scalar_mass = mass;
  btVector3 fallInertia(0, 0, 0);
  shape_->calculateLocalInertia(scalar_mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(scalar_mass, motion_state_,
      shape_, fallInertia);
  rigid_body_ = new btRigidBody(fallRigidBodyCI);
  dynamics_world_->addRigidBody(rigid_body_);

  // TODO(lucasw) is it more efficient for every marker to have the same ns,
  // and have id be a hash of the name?
  for (size_t i = 0; i < marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].ns = "bodies"; // name;
    marker_array_.markers[i].id = hash(name.c_str()) + i * 10000;
    marker_array_.markers[i].header.frame_id = name;
    // marker_.header.stamp = ros::Time::now();
    marker_array_.markers[i].frame_locked = true;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].color.r = 1.0;
    marker_array_.markers[i].color.g = 0.7;
    marker_array_.markers[i].color.a = 1.0;
    marker_array_.markers[i].lifetime = ros::Duration();
  }
  marker_array_pub_->publish(marker_array_);
}

// make a static height map
Body::Body(
    BulletServer* parent,
    const std::string name,
    // unsigned int type,
    // geometry_msgs::Pose pose,
    // geometry_msgs::Vector3 scale,
    cv::Mat& image,
    const float resolution,
    const float height_scale,
    const bool flip_quad_edges,
    // btDiscreteDynamicsWorld* dynamics_world,
    btSoftRigidDynamicsWorld* dynamics_world,
    tf::TransformBroadcaster* br,
    ros::Publisher* marker_array_pub) :
  parent_(parent),
  shape_(NULL),
  rigid_body_(NULL),
  name_(name),
  vertices_(NULL),
  indices_(NULL),
  index_vertex_arrays_(NULL),
  dynamics_world_(dynamics_world),
  br_(br),
  marker_array_pub_(marker_array_pub)
{
  // TODO(lucasw) some indications that 2.8x heightfield is screwy
  // also btBvhTriangleMeshShape looks like proper efficient way to do big terrains
  double min_val, max_val;
  cv::minMaxLoc(image, &min_val, &max_val);
  const float min_height = min_val * height_scale;
  const float max_height = max_val * height_scale;
  btTransform tf;
  tf.setIdentity();
#if 0
  // TODO(lucasw) Convert image BGR2GRAY if it isn't already?
  // Also make sure is uchar mono8- support floating types later

  // TODO(lucasw) 
  // This needs to be true: height_scale * max(image.data) = max_height
  // so enforce that here
  // TODO(lucasw) the terrain may get center automatically,
  // so offset the rviz marker to match
  const int up_axis = 2;
  btHeightfieldTerrainShape* heightfield_shape = new btHeightfieldTerrainShape(
      image.size().width,
      image.size().height,
      image.data,
      height_scale,
      min_height,
      max_height,
      up_axis,
      PHY_UCHAR,
      flip_quad_edges);

  // the height scale takes care of the vertical scaling
  btVector3 local_scaling(resolution, resolution, 1.0);
  heightfield_shape->setLocalScaling(local_scaling);

  shape_ = heightfield_shape;

  tf.setOrigin(btVector3(0, 0, 0));

  // make triangle list marker
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.header.frame_id = "map";
    marker.ns = "heightfield";
    marker.frame_locked = true;
    marker.id = hash(name.c_str());
    marker.color.r = 1.0;
    marker.color.g = 0.6;
    marker.color.a = 1.0;
    marker.pose.position.x = -image.size().width * resolution / 2.0;
    marker.pose.position.y = -image.size().height * resolution / 2.0;
    marker.pose.position.z = -(max_height - min_height)/2.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = height_scale;  // the length along axis of the cylinder

    ROS_INFO_STREAM("channels " << image.channels());
    // TODO(lucasw) need a height field rviz display type
    // or at least a triangle strip, this has tons of 
    // duplicate triangles
    for (size_t y = 0; y < image.size().height - 1; ++y)
    {
      for (size_t x = 0; x < image.size().width - 1; ++x)
      {
        geometry_msgs::Point p1;
        p1.x = x;
        p1.y = y;
        p1.z = image.at<uchar>(y, x);
        geometry_msgs::Point p2;
        p2.x = (x + 1);
        p2.y = y;
        p2.z = image.at<uchar>(y, x + 1);
        geometry_msgs::Point p3;
        p3.x = (x + 1);
        p3.y = (y + 1);
        p3.z = image.at<uchar>(y + 1, x + 1);
        geometry_msgs::Point p4;
        p4.x = x;
        p4.y = (y + 1);
        p4.z = image.at<uchar>(y + 1, x);

        // first triangle in quad
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);

        // second triangle in quad
        marker.points.push_back(p3);
        marker.points.push_back(p4);
        marker.points.push_back(p1);
      }
    }
    marker_array_.markers.push_back(marker);
    marker_array_pub_->publish(marker_array_);
  }
  #else

  // make triangle list marker and bvh triangle mesh
  {
    const size_t wd = image.size().width;
    const size_t ht = image.size().height;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.header.frame_id = "map";
    marker.ns = "heightfield";
    marker.frame_locked = true;
    marker.id = hash(name.c_str());
    marker.color.r = 1.0;
    marker.color.g = 0.6;
    marker.color.a = 1.0;
    marker.pose.position.x = -wd * resolution / 2.0;
    marker.pose.position.y = -ht * resolution / 2.0;
    marker.pose.position.z = -(max_height - min_height)/2.0;
    tf.setOrigin(btVector3(marker.pose.position.x,
        marker.pose.position.y,
        marker.pose.position.z));

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = height_scale;  // the length along axis of the cylinder

    // create vertices
    const int total_vertices = ht * wd;
    const int total_triangles = 2 * (ht - 1) * (wd - 1);
    vertices_ = new btVector3[total_vertices];
    indices_ = new int[total_triangles * 3];
    for (size_t y = 0; y < ht - 1; ++y)
    {
      for (size_t x = 0; x < wd - 1; ++x)
      {
        vertices_[x + y * wd].setValue(
            x * resolution,
            y * resolution,
            image.at<uchar>(y, x) * height_scale);
      }
    }

    // create indicies
    ROS_DEBUG_STREAM("channels " << image.channels());
    // TODO(lucasw) need a height field rviz display type
    // or at least a triangle strip, this has tons of 
    // duplicate triangles
    int index = 0;
    for (size_t y = 0; y < ht - 1; ++y)
    {
      for (size_t x = 0; x < wd - 1; ++x)
      {
        indices_[index++] = x + y * wd;
        indices_[index++] = (x + 1) + y * wd;
        indices_[index++] = (x + 1) + (y + 1) * wd;

        indices_[index++] = (x + 1) + (y + 1) * wd;
        indices_[index++] = x + (y + 1) * wd;
        indices_[index++] = x + y * wd;

        geometry_msgs::Point p1;
        p1.x = x;
        p1.y = y;
        p1.z = image.at<uchar>(y, x);
        geometry_msgs::Point p2;
        p2.x = (x + 1);
        p2.y = y;
        p2.z = image.at<uchar>(y, x + 1);
        geometry_msgs::Point p3;
        p3.x = (x + 1);
        p3.y = (y + 1);
        p3.z = image.at<uchar>(y + 1, x + 1);
        geometry_msgs::Point p4;
        p4.x = x;
        p4.y = (y + 1);
        p4.z = image.at<uchar>(y + 1, x);

        // first triangle in quad
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.points.push_back(p3);

        // second triangle in quad
        marker.points.push_back(p3);
        marker.points.push_back(p4);
        marker.points.push_back(p1);
      }
    }
    marker_array_.markers.push_back(marker);
    marker_array_pub_->publish(marker_array_);

    const int vert_stride = sizeof(btVector3);
    const int index_stride = 3 * sizeof(int);
    index_vertex_arrays_ = new btTriangleIndexVertexArray(total_triangles,
        indices_,
        index_stride,
        total_vertices,
        (btScalar*) &vertices_[0].x(),
        vert_stride);

    const bool use_quantized_aabb_compression = true;
    shape_ = new btBvhTriangleMeshShape(index_vertex_arrays_,
        use_quantized_aabb_compression);
  }
  #endif

  // TODO(lucasw) calculate btTransform from frame_id 
  motion_state_ = new btDefaultMotionState(tf);
  const btScalar mass = 0.0;
  btVector3 fall_inertia(0, 0, 0);
  // shape_->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, motion_state_,
      shape_, fall_inertia);
  rigid_body_ = new btRigidBody(fallRigidBodyCI);
  dynamics_world_->addRigidBody(rigid_body_);
}

Body::~Body()
{
  ROS_DEBUG_STREAM("Body delete " << name_);
  // if there is a constraint attached to this body, it
  // needs to be removed first
  for (std::map<std::string, Constraint*>::iterator it = constraints_.begin();
      it != constraints_.end(); ++it)
  {
    const std::string constraint_name = it->first;
    const Constraint* constraint = it->second;
    ROS_DEBUG_STREAM("Body " << name_ << " remove constraint " << constraint_name);
    // need to remove it from map from other Body
    if (constraint->body_a_->name_ != name_)
      constraint->body_a_->removeConstraint(constraint);
    if (it->second->body_b_->name_ != name_)
      constraint->body_b_->removeConstraint(constraint);

    // need to remove it from BulletSever constraints map
    parent_->removeConstraint(constraint);
    delete constraint;
  }

  for (size_t i = 0; i < marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
    marker_array_.markers[i].points.resize(0);
  }
  marker_array_pub_->publish(marker_array_);
  if (rigid_body_)
  {
    dynamics_world_->removeRigidBody(rigid_body_);
    delete rigid_body_->getMotionState();
    delete rigid_body_;
  }
  if (shape_)
    delete shape_;
  if (indices_)
    delete indices_;
  if (vertices_)
    delete vertices_;
  if (index_vertex_arrays_)
    delete index_vertex_arrays_;
}

// TODO(lucasw) pass in current time
void Body::update()
{
  if (!shape_)
    return;
  btTransform trans;
  // TODO(lucasw) instead of world transform, need to have a parent
  // specified and get the transform relative to that.
  rigid_body_->getMotionState()->getWorldTransform(trans);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(trans.getOrigin().getX(),
      trans.getOrigin().getY(),
      trans.getOrigin().getZ()));
  transform.setRotation(tf::Quaternion(trans.getRotation().getX(),
      trans.getRotation().getY(),
      trans.getRotation().getZ(),
      trans.getRotation().getW()));
  br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
    "map", name_));

  const int state = rigid_body_->getActivationState();
  if (state != state_)
  {
    state_ = state;
    for (size_t i = 0; i < marker_array_.markers.size(); ++i)
    {
      marker_array_.markers[i].color.r = state * 0.2;
      marker_array_.markers[i].color.g = 0.7 - state * 0.1;
      marker_array_.markers[i].color.b = 1.0 - state * 0.2;
    }
    marker_array_pub_->publish(marker_array_);
  }
  // ROS_INFO_STREAM("sphere height: " << trans.getOrigin().getY());
}

void Body::addConstraint(Constraint* constraint)
{
  ROS_DEBUG_STREAM(name_ << ": add constraint " << constraint->name_);
  constraints_[constraint->name_] = constraint;
}

// TODO(lucasw) make const
void Body::removeConstraint(const Constraint* constraint)
{
  // This doesn't delete the constraint just removes it (most likely
  // because it is about to be deleted from the other Body it is 
  // attached to.
  ROS_DEBUG_STREAM("Body " << name_ << ": remove constraint " << constraint->name_);
  constraints_.erase(constraint->name_);
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
    int tr[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
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
