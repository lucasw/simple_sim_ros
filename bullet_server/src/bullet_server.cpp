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

#include <bullet/btBulletDynamicsCommon.h>
#include <map>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class Body;

class BulletServer
{
  ros::NodeHandle nh_;
  tf::TransformBroadcaster br_;
  float period_;

  btBroadphaseInterface* broadphase;
  btDefaultCollisionConfiguration* collision_configuration_;
  btCollisionDispatcher* dispatcher;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* dynamics_world_;

  btCollisionShape* ground_shape_;
  btDefaultMotionState* ground_motion_state_;
  btRigidBody* ground_rigid_body_;

  Body* body_;

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

  btCollisionShape* shape_;
  btRigidBody* rigid_body_;
  btDefaultMotionState* motion_state_;
  btDiscreteDynamicsWorld* dynamics_world_;
public:
  Body(const std::string name,
      btDiscreteDynamicsWorld* dynamics_world,
      tf::TransformBroadcaster* br);
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

  body_ = new Body("falling_sphere1", dynamics_world_, &br_);

  period_ = 1.0 / 60.0;

  return 0;
}

void BulletServer::update()
{
  dynamics_world_->stepSimulation(period_, 10);
  body_->update();
  ros::spinOnce();
  ros::Duration(period_).sleep();
}

BulletServer::~BulletServer()
{
  delete body_;

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

Body::Body(const std::string name,
    btDiscreteDynamicsWorld* dynamics_world,
    tf::TransformBroadcaster* br) :
  name_(name),
  dynamics_world_(dynamics_world),
  br_(br)
{
  shape_ = new btSphereShape(1);
  motion_state_ = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1),
      btVector3(0, 0, 20)));
  btScalar mass = 1;
  btVector3 fallInertia(0, 0, 0);
  shape_->calculateLocalInertia(mass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, motion_state_,
      shape_, fallInertia);
  rigid_body_ = new btRigidBody(fallRigidBodyCI);
  dynamics_world_->addRigidBody(rigid_body_);
}

Body::~Body()
{
  dynamics_world_->removeRigidBody(rigid_body_);
  delete rigid_body_->getMotionState();
  delete rigid_body_;
  delete shape_;
}

// TODO(lucasw) pass in current time
void Body::update()
{
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

