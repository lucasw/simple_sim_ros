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
#include <bullet_server/Body.h>
#include <geometry_msgs/Pose.h>
#include <map>
#include <ros/ros.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class Body;

class BulletServer
{
  ros::NodeHandle nh_;
  ros::Subscriber body_sub_;
  void bodyCallback(const bullet_server::Body::ConstPtr& msg);
  tf::TransformBroadcaster br_;
  float period_;
  // ros::Publisher marker_pub_;
  ros::Publisher marker_array_pub_;

  btBroadphaseInterface* broadphase;
  btDefaultCollisionConfiguration* collision_configuration_;
  btCollisionDispatcher* dispatcher;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* dynamics_world_;

  btCollisionShape* ground_shape_;
  btDefaultMotionState* ground_motion_state_;
  btRigidBody* ground_rigid_body_;

  std::map<std::string, Body*> bodies_;

  int init();
public:
  BulletServer();
  ~BulletServer();
  void update();
};

class Body
{
  const std::string name_;
  tf::TransformBroadcaster* br_;
  // ros::Publisher* marker_pub_;
  ros::Publisher* marker_array_pub_;
  visualization_msgs::MarkerArray marker_array_;

  btCollisionShape* shape_;
  btRigidBody* rigid_body_;
  btDefaultMotionState* motion_state_;
  btDiscreteDynamicsWorld* dynamics_world_;
public:
  Body(const std::string name,
      unsigned int type,
      geometry_msgs::Pose pose,
      geometry_msgs::Vector3 scale,
      btDiscreteDynamicsWorld* dynamics_world,
      tf::TransformBroadcaster* br,
      ros::Publisher* marker_array_pub_);
  ~Body();

  void update();
};

BulletServer::BulletServer()
{
  init();
}

int BulletServer::init()
{
  broadphase = new btDbvtBroadphase();
  collision_configuration_ = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collision_configuration_);
  solver = new btSequentialImpulseConstraintSolver;
  dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher, broadphase,
      solver, collision_configuration_);
  dynamics_world_->setGravity(btVector3(0, 0, -10));

  ground_shape_ = new btStaticPlaneShape(btVector3(0, 0, 1), 1);
  ground_motion_state_ = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -1)));
  btRigidBody::btRigidBodyConstructionInfo
    ground_rigid_body_CI(0, ground_motion_state_, ground_shape_, btVector3(0, 0, 0));
  ground_rigid_body_ = new btRigidBody(ground_rigid_body_CI);
  dynamics_world_->addRigidBody(ground_rigid_body_);

  period_ = 1.0 / 60.0;

  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  // TODO(lucasw) make this a service
  // rostopic pub /add_body bullet_server/Body "{name: 'test6', pose: {position: {x: 0.201, y: 0.001, z: 10}, orientation: {w: 1}}}" -1
  body_sub_ = nh_.subscribe("add_body", 10, &BulletServer::bodyCallback, this);

  return 0;
}

void BulletServer::bodyCallback(const bullet_server::Body::ConstPtr& msg)
{
  if (bodies_.count(msg->name) > 0)
    delete bodies_[msg->name];

  bodies_[msg->name] = new Body(msg->name, msg->type, msg->pose,
      msg->scale, dynamics_world_, &br_, &marker_array_pub_);
}

void BulletServer::update()
{
  dynamics_world_->stepSimulation(period_, 10);
  for (std::map<std::string, Body*>::iterator it = bodies_.begin();
      it != bodies_.end(); ++it)
  {
    it->second->update();
  }
  ros::spinOnce();
  ros::Duration(period_).sleep();
}

BulletServer::~BulletServer()
{
  for (std::map<std::string, Body*>::iterator it = bodies_.begin();
      it != bodies_.end(); ++it)
  {
    delete it->second;
  }

  dynamics_world_->removeRigidBody(ground_rigid_body_);
  delete ground_rigid_body_->getMotionState();
  delete ground_rigid_body_;
  delete ground_shape_;

  delete dynamics_world_;
  delete solver;
  delete collision_configuration_;
  delete dispatcher;
  delete broadphase;
}

///////////////////////////////////////////////////////////////////////////////

// http://stackoverflow.com/questions/2535284/how-can-i-hash-a-string-to-an-int-using-c
unsigned long hash(const char *str) {
    unsigned long hash = 5381;
    int c;

    while (c = *str++) {
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
    }

    return hash;
}

Body::Body(const std::string name,
    unsigned int type,
    geometry_msgs::Pose pose,
    geometry_msgs::Vector3 scale,
    btDiscreteDynamicsWorld* dynamics_world,
    tf::TransformBroadcaster* br,
    ros::Publisher* marker_array_pub) :
  shape_(NULL),
  rigid_body_(NULL),
  name_(name),
  dynamics_world_(dynamics_world),
  br_(br),
  marker_array_pub_(marker_array_pub)
{
  // ROS_INFO_STREAM(name << " " << type << " " << pose);

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
  btScalar mass = 1;
  btVector3 fallInertia(0, 0, 0);
  shape_->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, motion_state_,
      shape_, fallInertia);
  rigid_body_ = new btRigidBody(fallRigidBodyCI);
  dynamics_world_->addRigidBody(rigid_body_);

  // TODO(lucasw) is it more efficient for every marker to have the same ns,
  // and have id be a hash of the name?
  for (size_t i = 0; i < marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].ns = "bullet_server"; // name;
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

Body::~Body()
{
  for (size_t i = 0; i < marker_array_.markers.size(); ++i)
    marker_array_.markers[i].action = visualization_msgs::Marker::DELETE;
  marker_array_pub_->publish(marker_array_);
  dynamics_world_->removeRigidBody(rigid_body_);
  if (shape_)
  {
    delete rigid_body_->getMotionState();
    delete rigid_body_;
  }
  if (shape_)
    delete shape_;
}

// TODO(lucasw) pass in current time
void Body::update()
{
  if (!shape_)
    return;
  btTransform trans;
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
  // ROS_INFO_STREAM("sphere height: " << trans.getOrigin().getY());
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

