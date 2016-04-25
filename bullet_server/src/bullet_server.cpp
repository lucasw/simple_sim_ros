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

class BulletServer
{
  ros::NodeHandle nh_;

  // Build the broadphase
  btBroadphaseInterface* broadphase_;

  // Set up the collision configuration and dispatcher
  btDefaultCollisionConfiguration* collision_configuration_;
  btCollisionDispatcher* dispatcher_;

  // The actual physics solver
  btSequentialImpulseConstraintSolver* solver_;

  // The world.
  btDiscreteDynamicsWorld* dynamics_world_;
public:
  BulletServer();
};

BulletServer::BulletServer()
{
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  collision_configuration_ = new btDefaultCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(collision_configuration_);

  solver_ = new btSequentialImpulseConstraintSolver;

  dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_, broadphase_,
      solver_, collision_configuration_);
  dynamics_world_->setGravity(btVector3(0, -10, 0));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bullet_server");
  BulletServer bullet_server;
  ros::spin();
}
