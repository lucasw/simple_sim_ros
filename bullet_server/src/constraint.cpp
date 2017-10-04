/**
  Copyright (C) 2016  Lucas Walter

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
#include <bullet_server/body.h>
#include <bullet_server/bullet_server.h>
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
      const bool enable_pos_pub,
      const bool enable_motor_sub,
      const bool disable_collisions_between_linked_bodies,
      btDiscreteDynamicsWorld* dynamics_world,
      ros::Publisher* marker_array_pub) :
    nh_(name),
    name_(name),
    body_a_(body_a),
    body_b_(body_b),
    max_motor_impulse_(max_motor_impulse),
    enable_pos_pub_(enable_pos_pub),
    enable_motor_sub_(enable_motor_sub),
    dynamics_world_(dynamics_world),
    marker_array_pub_(marker_array_pub)
{
  ROS_DEBUG_STREAM("new Constraint " << name << " " << body_a->name_ << " " << body_b->name_);
  // TODO(lucasw) pivot -> translation or similar
  const btVector3 pivot_in_a_bt(pivot_in_a.x, pivot_in_a.y, pivot_in_a.z);
  const btVector3 pivot_in_b_bt(pivot_in_b.x, pivot_in_b.y, pivot_in_b.z);
  const btVector3 axis_in_a_bt(axis_in_a.x, axis_in_a.y, axis_in_a.z);
  const btVector3 axis_in_b_bt(axis_in_b.x, axis_in_b.y, axis_in_b.z);
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
    if (enable_pos_pub_)
      pubs_["angular_pos"] = nh_.advertise<std_msgs::Float64>("angular_pos", 1);
    const std::string motor_name = "target_ang_motor_vel";
    command_[motor_name] = 0;
    if (enable_motor_sub_)
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

    if (enable_pos_pub_)
      pubs_["linear_pos"] = nh_.advertise<std_msgs::Float64>("linear_pos", 1);
    // body_sub_ = nh_.subscribe("add_body", 10, &BulletServer::bodyCallback, this);
    const std::string motor_name = "target_lin_motor_vel";
    command_[motor_name] = 0;
    if (enable_motor_sub_)
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
  dynamics_world_->addConstraint(constraint_, disable_collisions_between_linked_bodies);
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
    if (enable_pos_pub_)
    {
      std_msgs::Float64 msg;
      msg.data = slider->getLinearPos();
      pubs_["linear_pos"].publish(msg);
    }

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
    if (enable_pos_pub_)
    {
      std_msgs::Float64 msg;
      msg.data = hinge->getHingeAngle();
      pubs_["angular_pos"].publish(msg);
    }

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
