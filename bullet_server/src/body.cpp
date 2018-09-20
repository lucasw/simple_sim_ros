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

Body::Body(BulletServer* parent,
    const std::string name,
    unsigned int type,
    const float mass,
    const bool kinematic,
    geometry_msgs::Pose pose,
    geometry_msgs::Twist twist,
    geometry_msgs::Vector3 scale,
    const float friction,
    const float rolling_friction,
    btDiscreteDynamicsWorld* dynamics_world,
    tf::TransformBroadcaster* br,
    ros::Publisher* marker_array_pub,
    const std::string tf_prefix) :
  parent_(parent),
  vertices_(NULL),
  indices_(NULL),
  index_vertex_arrays_(NULL),
  shape_(NULL),
  rigid_body_(NULL),
  name_(name),
  dynamics_world_(dynamics_world),
  kinematic_linear_vel_(0.0, 0.0, 0.0),
  br_(br),
  marker_array_pub_(marker_array_pub),
  tf_prefix_(tf_prefix),
  state_(-1),
  new_transform_(false)
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
    marker.pose.orientation.w = 1.0;
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
      marker.pose.orientation.w = 0.70710678;
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
      marker.pose.orientation.w = 0.70710678;
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
  {
    ROS_ERROR_STREAM("unknown body type: " << type);
    return;
  }

  motion_state_ = new btDefaultMotionState(btTransform(
      btQuaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
      btVector3(pose.position.x, pose.position.y, pose.position.z)));
  // TODO(lucasw) provide in message
  btScalar scalar_mass = mass;

  btVector3 fall_inertia(0, 0, 0);
  shape_->calculateLocalInertia(scalar_mass, fall_inertia);
  ROS_INFO_STREAM(name_ << " " << scalar_mass
    << " " << fall_inertia.x()
    << " " << fall_inertia.y()
    << " " << fall_inertia.z());
  btRigidBody::btRigidBodyConstructionInfo construction_info(scalar_mass, motion_state_,
      shape_, fall_inertia);
  // TODO(lucasw) make construction_info message type to put these into
  construction_info.m_friction = friction;
  construction_info.m_rollingFriction = rolling_friction;
  rigid_body_ = new btRigidBody(construction_info);

  if (kinematic)
  {
    ROS_INFO_STREAM(name + " is kinematic, mass should be zero to work");
    rigid_body_->setCollisionFlags(
        rigid_body_->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
    rigid_body_->setActivationState(DISABLE_DEACTIVATION);
  }

  dynamics_world_->addRigidBody(rigid_body_);

  // ROS_INFO_STREAM("impulse " << msg->body << "\n" << msg->location << "\n" << msg->impulse);
  const btVector3 point_rel_body(0, 0, 0);
  const btVector3 impulse(twist.linear.x, twist.linear.y, twist.linear.z);
  rigid_body_->applyImpulse(impulse, point_rel_body);

  // TODO(lucasw) is it more efficient for every marker to have the same ns,
  // and have id be a hash of the name?
  for (size_t i = 0; i < marker_array_.markers.size(); ++i)
  {
    marker_array_.markers[i].ns = "bodies";  // name;
    marker_array_.markers[i].id = hash(name.c_str()) + i * 10000;
    marker_array_.markers[i].header.frame_id = tf_prefix_ + name;
    // marker_.header.stamp = ros::Time::now();
    marker_array_.markers[i].frame_locked = true;
    marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[i].color.r = 1.0;
    marker_array_.markers[i].color.g = 0.7;
    marker_array_.markers[i].color.a = 1.0;
    marker_array_.markers[i].lifetime = ros::Duration();
  }
  publishMarker();
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
    const float friction,
    const float rolling_friction,
    btDiscreteDynamicsWorld* dynamics_world,
    tf::TransformBroadcaster* br,
    ros::Publisher* marker_array_pub,
    const std::string tf_prefix) :
  parent_(parent),
  shape_(NULL),
  rigid_body_(NULL),
  name_(name),
  vertices_(NULL),
  indices_(NULL),
  index_vertex_arrays_(NULL),
  dynamics_world_(dynamics_world),
  br_(br),
  marker_array_pub_(marker_array_pub),
  tf_prefix_(tf_prefix)
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
    marker.header.frame_id = parent_->config_.frame_id;
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
    publishMarker();
  }
  #else

  // make triangle list marker and bvh triangle mesh
  {
    const size_t wd = image.size().width;
    const size_t ht = image.size().height;
    ROS_INFO_STREAM("making heightfield terrain " << wd << " " << ht
        << " " << resolution);

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.header.frame_id = parent_->config_.frame_id;
    // TODO(lucasw) if frame_id_ != parent_->frame_id_
    // then transform coords into this frame_id_
    marker.ns = "heightfield";
    marker.frame_locked = true;
    marker.id = hash(name.c_str());
    marker.color.r = 1.0;
    marker.color.g = 0.6;
    marker.color.a = 1.0;
    marker.pose.position.x = -static_cast<float>(wd) * resolution / 2.0;
    marker.pose.position.y = -static_cast<float>(ht) * resolution / 2.0;
    marker.pose.position.z = -max_height;
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
        const float height = image.at<uchar>(y, x) * height_scale;
        vertices_[x + y * wd].setValue(
            x * resolution,
            y * resolution,
            height);
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
    publishMarker();

    const int vert_stride = sizeof(btVector3);
    const int index_stride = 3 * sizeof(int);
    index_vertex_arrays_ = new btTriangleIndexVertexArray(total_triangles,
        indices_,
        index_stride,
        total_vertices,
        const_cast<btScalar*>(reinterpret_cast<const btScalar*>(&vertices_[0].x())),
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
  btRigidBody::btRigidBodyConstructionInfo construction_info(mass, motion_state_,
      shape_, fall_inertia);
  construction_info.m_friction = friction;
  construction_info.m_rollingFriction = rolling_friction;
  rigid_body_ = new btRigidBody(construction_info);
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

void Body::setTransform(const geometry_msgs::Transform& tr)
{
  transform_ = tr;
  new_transform_ = true;

  // TODO(lucasw) tried putting this in update() but apparently that is in
  // same thread.
  if (new_transform_)
  {
    // TODO(lucasw) if the transform frame is not the same as the
    // bullet frame then transform it.
    // But if that is done once, wouldn't there be a need to keep transforming
    // on every tickUpdate so this body can track that frame exactly?
    btTransform trans;
    btVector3 origin(transform_.translation.x,
        transform_.translation.y, transform_.translation.z);
    trans.setOrigin(origin);
    btQuaternion quat(transform_.rotation.x, transform_.rotation.y,
        transform_.rotation.z, transform_.rotation.w);
    trans.setRotation(quat);

    if (rigid_body_->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT)
    {
      rigid_body_->getMotionState()->setWorldTransform(trans);
    }
    else
    {
      // TODO(lucasw) add Twist to the service for this
      rigid_body_->setLinearVelocity(btVector3(0.0, 0.0, 0.0));
      rigid_body_->setWorldTransform(trans);
    }
    new_transform_ = false;
    ROS_DEBUG_STREAM(name_ << " new transform " << transform_);
  }

  #if 0
  // TODO(lucasw) is this needed?
  // probably should return true or false if timeout
  while (new_transform_)
  {
    // ros::Duration(0.2).sleep();
  }
  #endif
}

void Body::tickUpdate(btScalar time_step)
{
  // update kinematic object - is this going to be jerky?
  if (rigid_body_->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT)
  {
    btTransform trans;
    rigid_body_->getMotionState()->getWorldTransform(trans);
    trans.setOrigin(trans.getOrigin() + kinematic_linear_vel_ * time_step);
    rigid_body_->getMotionState()->setWorldTransform(trans);
  }
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
  // TODO(lucasw) "map" also should be a parameter
  br_->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
    "map", tf_prefix_ + name_));

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
    publishMarker();
  }
  // ROS_INFO_STREAM("sphere height: " << trans.getOrigin().getY());
}

void Body::publishMarker()
{
  marker_array_pub_->publish(marker_array_);
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
