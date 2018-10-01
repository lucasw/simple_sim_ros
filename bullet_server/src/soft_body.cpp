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
#include <bullet_server/bullet_server.h>
#include <bullet_server/body.h>
#include <bullet_server/constraint.h>
#include <bullet_server/soft_body.h>
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

bool isnan(const geometry_msgs::Point pt)
{
  if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))
    return true;
  return false;
}

bool isnan(const btVector3 pt)
{
  if (std::isnan(pt.x()) || std::isnan(pt.y()) || std::isnan(pt.z()))
    return true;
  return false;
}

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
    const float margin,
    const bool randomize_constraints,
    const uint8_t k_clusters,
    btSoftRigidDynamicsWorld* dynamics_world,
    tf::TransformBroadcaster* br,
    ros::Publisher* marker_array_pub) :
  parent_(parent),
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
    if (isnan(pos))
    {
      ROS_ERROR_STREAM("bad position " << pos);
      delete[] points;
      delete[] masses;
      return;
    }
    btScalar mass(nodes[i].mass);
    // this was segfaulting
    // soft_body_->appendNode(pos, mass);
    points[i] = pos;
    masses[i] = mass;
    // ROS_INFO_STREAM(pos << " " << mass);
  }

  soft_body_ = new btSoftBody(soft_body_world_info, nodes.size(), points, masses);
  // TODO(lucasw) all the nodes created above will have the default material properties,
  // not the one provided to this constructor- go through and override them?
  // Ultimately every node should have a material index into the materials vector.
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

  for (size_t i = 0; i < materials.size(); ++i)
  {
    btSoftBody::Material* pm = nullptr;
    // Overwrite default material/s (just one ought to be there, but make this generic
    // it seems the only point of m_materials[0] is to provide a default
    // for all the appendLinks/appendFaces etc.
    if (i < soft_body_->m_materials.size())
      pm = soft_body_->m_materials[i];
    else
      pm = soft_body_->appendMaterial();
    pm->m_kLST = materials[i].kLST;
    pm->m_kAST = materials[i].kAST;
    pm->m_kVST = materials[i].kVST;
    soft_body_->generateBendingConstraints(materials[i].bending_distance, pm);
    ROS_INFO_STREAM(name_ << " material " << pm->m_kLST);
  }

  soft_body_->generateClusters(k_clusters);
  // enable cluster collisions
  // TODO(lucasw) put all collision options into config
  // soft_body_->m_cfg.collisions += btSoftBody::fCollision::CL_RS;
  // soft_body_->m_cfg.collisions += btSoftBody::fCollision::CL_SS;

  for (size_t i = 0; i < links.size(); ++i)
  {
    btSoftBody::Material* pm = nullptr;
    if (links[i].material_ind < soft_body_->m_materials.size())
      pm = soft_body_->m_materials[links[i].material_ind];
    // TODO(lucasw) need to provide an optional material index
    // for each link, otherwise m_material[0] in the soft body is used.
    // With bcheckexist set to true redundant links ought
    // to be filtered out.
    soft_body_->appendLink(links[i].node_indices[0],
      links[i].node_indices[1], pm, true);
  }
  for (size_t i = 0; i < faces.size(); ++i)
  {
    btSoftBody::Material* pm = nullptr;
    if (faces[i].material_ind < soft_body_->m_materials.size())
      pm = soft_body_->m_materials[faces[i].material_ind];
    soft_body_->appendFace(faces[i].node_indices[0],
      faces[i].node_indices[1],
      faces[i].node_indices[2], pm);
  }
  for (size_t i = 0; i < tetras.size(); ++i)
  {
    btSoftBody::Material* pm = nullptr;
    if (tetras[i].material_ind < soft_body_->m_materials.size())
      pm = soft_body_->m_materials[tetras[i].material_ind];
    soft_body_->appendTetra(tetras[i].node_indices[0],
      tetras[i].node_indices[1],
      tetras[i].node_indices[2],
      tetras[i].node_indices[3], pm);
  }

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

  if (randomize_constraints)
    soft_body_->randomizeConstraints();
  ROS_INFO_STREAM(name_ << ", margin " << margin);
  if (margin > 0)
    soft_body_->getCollisionShape()->setMargin(margin);

  marker_array_.markers.resize(COUNT);
  {
    visualization_msgs::Marker marker;
    // TODO(lucasw) also have a LINES and TRIANGLE_LIST marker
    marker.type = visualization_msgs::Marker::POINTS;
    // rotating the z axis to the y axis is a -90 degree around the axis axis (roll)
    // KDL::Rotation(-M_PI_2, 0, 0)?
    // tf::Quaternion quat = tf::createQuaternionFromRPY();
    // tf::Matrix3x3(quat)
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.ns = "nodes";
    // marker_.header.stamp = ros::Time::now();
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // TODO(lucasw) could turn this into function
    marker.id = hash(name_.c_str());
    marker.header.frame_id = parent_->config_.frame_id;
    marker.color.r = 0.45;
    marker.color.g = 0.4;
    marker.color.b = 0.65;
    marker_array_.markers[NODES] = marker;
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
    marker.scale.x = 0.0005;
    marker.scale.y = 0.0005;
    marker.scale.z = 0.0005;
    marker.ns = "links";
    // marker_.header.stamp = ros::Time::now();
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    // TODO(lucasw) could turn this into function
    marker.id = hash((name_ + "lines").c_str());
    marker.header.frame_id = parent_->config_.frame_id;
    marker.color.r = 0.3;
    marker.color.g = 0.67;
    marker.color.b = 0.65;
    marker_array_.markers[LINKS] = marker;
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
    marker_array_.markers[ANCHORS] = marker;

    marker.ns = "anchor_pivots";
    marker.color.r = 0.67;
    marker.color.g = 0.87;
    marker.color.b = 0.45;
    marker_array_.markers[ANCHOR_PIVOTS] = marker;
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
    marker_array_.markers[TETRAS] = marker;
  }

  // face markers
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
    marker.ns = "faces";
    marker.frame_locked = true;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    marker.id = hash((name_ + "faces").c_str());
    marker.header.frame_id = "map";
    marker.color.r = 0.15;
    marker.color.g = 0.37;
    marker.color.b = 0.55;
    marker_array_.markers[FACES] = marker;
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

  btSoftBody::tNodeArray& nodes(soft_body_->m_nodes);
  marker_array_.markers[NODES].points.clear();
  for (size_t i = 0; i < nodes.size(); ++i)
  {
    geometry_msgs::Point pt;
    pt.x = nodes[i].m_x.getX();
    pt.y = nodes[i].m_x.getY();
    pt.z = nodes[i].m_x.getZ();
    marker_array_.markers[NODES].points.push_back(pt);

    if (isnan(pt))
    {
      ROS_ERROR_STREAM(name_ << " bad node " << i << " " << pt);
      return;
    }
  }

  btSoftBody::tLinkArray& links(soft_body_->m_links);
  marker_array_.markers[LINKS].points.clear();
  for (size_t i = 0; i < links.size(); ++i)
  {
    geometry_msgs::Point pt1;
    pt1.x = links[i].m_n[0]->m_x.getX();
    pt1.y = links[i].m_n[0]->m_x.getY();
    pt1.z = links[i].m_n[0]->m_x.getZ();
    marker_array_.markers[LINKS].points.push_back(pt1);

    geometry_msgs::Point pt2;
    pt2.x = links[i].m_n[1]->m_x.getX();
    pt2.y = links[i].m_n[1]->m_x.getY();
    pt2.z = links[i].m_n[1]->m_x.getZ();
    marker_array_.markers[LINKS].points.push_back(pt2);

    if (isnan(pt1) || isnan(pt2))
    {
      ROS_ERROR_STREAM("bad link " << i << " " << pt1 << " " << pt2);
      return;
    }
  }

  btSoftBody::tTetraArray& tetras(soft_body_->m_tetras);
  marker_array_.markers[TETRAS].points.clear();
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
        marker_array_.markers[TETRAS].points.push_back(pt);
      }
    }
  }

  // faces
  btSoftBody::tFaceArray& faces(soft_body_->m_faces);
  marker_array_.markers[FACES].points.clear();
  for (size_t i = 0; i < faces.size(); ++i)
  {
    for (size_t k = 0; k < 3; ++k)
    {
      const btSoftBody::Node* node = faces[i].m_n[k];
      geometry_msgs::Point pt;
      pt.x = node->m_x.getX();
      pt.y = node->m_x.getY();
      pt.z = node->m_x.getZ();
      marker_array_.markers[FACES].points.push_back(pt);
    }
  }

  btSoftBody::tAnchorArray& anchors(soft_body_->m_anchors);
  marker_array_.markers[ANCHORS].points.clear();
  marker_array_.markers[ANCHOR_PIVOTS].points.clear();
  for (size_t i = 0; i < anchors.size(); ++i)
  {
    geometry_msgs::Point pt1;
    pt1.x = anchors[i].m_node->m_x.getX();
    pt1.y = anchors[i].m_node->m_x.getY();
    pt1.z = anchors[i].m_node->m_x.getZ();
    marker_array_.markers[ANCHORS].points.push_back(pt1);

    btTransform trans;
    anchors[i].m_body->getMotionState()->getWorldTransform(trans);

    btVector3 world_point = trans * anchors[i].m_local;
    geometry_msgs::Point pt2;
    pt2.x = world_point.getX();
    pt2.y = world_point.getY();
    pt2.z = world_point.getZ();
    marker_array_.markers[ANCHORS].points.push_back(pt2);
    marker_array_.markers[ANCHOR_PIVOTS].points.push_back(pt2);

    geometry_msgs::Point pt3;
    pt3.x = trans.getOrigin().getX();
    pt3.y = trans.getOrigin().getY();
    pt3.z = trans.getOrigin().getZ();
    marker_array_.markers[ANCHOR_PIVOTS].points.push_back(pt3);
  }

  marker_array_pub_->publish(marker_array_);
}
