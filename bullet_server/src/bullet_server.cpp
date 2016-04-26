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
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

class BulletServer
{
  ros::NodeHandle nh_;
  tf::TransformBroadcaster br_;

  btBroadphaseInterface* broadphase;

        btDefaultCollisionConfiguration* collisionConfiguration;
        
        btCollisionDispatcher* dispatcher;

        btSequentialImpulseConstraintSolver* solver;

        btDiscreteDynamicsWorld* dynamicsWorld;

  #if 0
  // Build the broadphase
  btBroadphaseInterface* broadphase_;

  // Set up the collision configuration and dispatcher
  btDefaultCollisionConfiguration* collision_configuration_;
  btCollisionDispatcher* dispatcher_;

  // The actual physics solver
  btSequentialImpulseConstraintSolver* solver_;

  // The world.
  btDiscreteDynamicsWorld* dynamics_world_;
  #endif

  int init();
public:
  BulletServer();
};

BulletServer::BulletServer()
{
  init();
  #if 0
  // http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  collision_configuration_ = new btDefaultCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(collision_configuration_);

  solver_ = new btSequentialImpulseConstraintSolver();

  dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_, broadphase_,
      solver_, collision_configuration_);

  // TODO(lucasw) set this from param
  dynamics_world_->setGravity(btVector3(0, -10, 0));

  {
    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

    btDefaultMotionState* groundMotionState =
                    new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1),
                                                         btVector3(0, -1, 0)));
    btRigidBody::btRigidBodyConstructionInfo
                     groundRigidBodyCI(0, groundMotionState, groundShape,
                                       btVector3(0, 0, 0));
    btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);

    dynamics_world_->addRigidBody(groundRigidBody);
  }

  btRigidBody* fallRigidBody;
  {
    btDefaultMotionState* fallMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
    btCollisionShape* fallShape = new btSphereShape(1);
    btScalar mass = 1;
    btVector3 fallInertia(0, 0, 0);
    fallShape->calculateLocalInertia(mass, fallInertia);

    btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState,
                                                             fallShape, fallInertia);
    fallRigidBody = new btRigidBody(fallRigidBodyCI);
    dynamics_world_->addRigidBody(fallRigidBody);
  }

  const float rate = 60.0;
  const float period = 1.0 / rate;
  int i = 0;
  while (true)  // (ros::ok())
  {
    // float ms = getDeltaTimeMicroseconds();
    // float period = ms / 1e6;
    // ROS_INFO_STREAM(++i << " " << period << " " << dynamics_world_);
    dynamics_world_->stepSimulation(period, 1, period*2);

    btTransform trans;
    // fallRigidBody->getMotionState()->getWorldTransform(trans);
    // ROS_INFO_STREAM("sphere height: " << trans.getOrigin().getY());

    // ros::spinOnce();
  }
  #endif
}

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

int BulletServer::init()
{
  broadphase = new btDbvtBroadphase();
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  solver = new btSequentialImpulseConstraintSolver;
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, 0, -10));


  btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 1);
  btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -1)));
  btRigidBody::btRigidBodyConstructionInfo
    groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
  btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
  dynamicsWorld->addRigidBody(groundRigidBody);

  Body* body = new Body("falling_sphere1", dynamicsWorld, &br_);

  const float period = 1.0 / 60.0;
  while (ros::ok())
  {
    dynamicsWorld->stepSimulation(period, 10);
    body->update();
    ros::spinOnce();
    ros::Duration(period).sleep();
  }

  delete body;

  dynamicsWorld->removeRigidBody(groundRigidBody);
  delete groundRigidBody->getMotionState();
  delete groundRigidBody;
  delete groundShape;

  delete dynamicsWorld;
  delete solver;
  delete collisionConfiguration;
  delete dispatcher;
  delete broadphase;

  return 0;
}
