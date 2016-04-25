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

#include <iostream>
#include <bullet/btBulletDynamicsCommon.h>
// #include <ros/ros.h>

class BulletServer
{
  // ros::NodeHandle nh_;

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
  // http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  collision_configuration_ = new btDefaultCollisionConfiguration();
  dispatcher_ = new btCollisionDispatcher(collision_configuration_);

  solver_ = new btSequentialImpulseConstraintSolver();

  dynamics_world_ = new btDiscreteDynamicsWorld(dispatcher_, broadphase_,
      solver_, collision_configuration_);

  // TODO(lucasw) set this from param
  dynamics_world_->setGravity(btVector3(0, -10, 0));

  #if 0
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
  #endif

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
}

int main(int argc, char** argv)
{
  // ros::init(argc, argv, "bullet_server");
  //  BulletServer bullet_server;


  // exact hello world
        btBroadphaseInterface* broadphase = new btDbvtBroadphase();

        btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
        btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

        btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

        btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

        dynamicsWorld->setGravity(btVector3(0, -10, 0));


        btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

        btCollisionShape* fallShape = new btSphereShape(1);


        btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
        btRigidBody::btRigidBodyConstructionInfo
                groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
        btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);
        dynamicsWorld->addRigidBody(groundRigidBody);


        btDefaultMotionState* fallMotionState =
                new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));
        btScalar mass = 1;
        btVector3 fallInertia(0, 0, 0);
        fallShape->calculateLocalInertia(mass, fallInertia);
        btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
        btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
        dynamicsWorld->addRigidBody(fallRigidBody);


        for (int i = 0; i < 300; i++) {
                dynamicsWorld->stepSimulation(1 / 60.f, 10);

                btTransform trans;
                fallRigidBody->getMotionState()->getWorldTransform(trans);

                std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
        }

        dynamicsWorld->removeRigidBody(fallRigidBody);
        delete fallRigidBody->getMotionState();
        delete fallRigidBody;

        dynamicsWorld->removeRigidBody(groundRigidBody);
        delete groundRigidBody->getMotionState();
        delete groundRigidBody;


        delete fallShape;

        delete groundShape;


        delete dynamicsWorld;
        delete solver;
        delete collisionConfiguration;
        delete dispatcher;
        delete broadphase;

        return 0;
}
