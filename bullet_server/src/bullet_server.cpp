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

  btBroadphaseInterface* broadphase;
  btDefaultCollisionConfiguration* collisionConfiguration;
  btCollisionDispatcher* dispatcher;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* dynamicsWorld;

  btCollisionShape* groundShape;
  btDefaultMotionState* groundMotionState;
  btRigidBody* groundRigidBody;
 
  Body* body_;

  int init();
public:
  BulletServer();
  ~BulletServer();
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
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  solver = new btSequentialImpulseConstraintSolver;
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, 0, -10));

  groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 1);
  groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -1)));
  btRigidBody::btRigidBodyConstructionInfo
    groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
  groundRigidBody = new btRigidBody(groundRigidBodyCI);
  dynamicsWorld->addRigidBody(groundRigidBody);

  body_ = new Body("falling_sphere1", dynamicsWorld, &br_);

  const float period = 1.0 / 60.0;
  while (ros::ok())
  {
    dynamicsWorld->stepSimulation(period, 10);
    body_->update();
    ros::spinOnce();
    ros::Duration(period).sleep();
  }

  return 0;
}

BulletServer::~BulletServer()
{
  delete body_;

  dynamicsWorld->removeRigidBody(groundRigidBody);
  delete groundRigidBody->getMotionState();
  delete groundRigidBody;
  delete groundShape;

  delete dynamicsWorld;
  delete solver;
  delete collisionConfiguration;
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
      btVector3(0, 0, 50)));
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
}

