#include <bullet/btBulletDynamicsCommon.h>
// #include <geometry_msgs/Point.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

class Raycast
{
public:
  Raycast(const std::string name, const std::string frame_id,
      const geometry_msgs::Point start, const geometry_msgs::Point end,
      const std::string topic_name,
      ros::NodeHandle& nh,
      btDiscreteDynamicsWorld* dynamics_world);

  bool update(tf2_ros::Buffer& tf_buffer);
private:
  ros::Publisher point_cloud_pub_;

  const std::string name_;
  const std::string frame_id_;
  // both vectors are in frame_id_ tf frame
  // TODO(lucasw) each raycast needs to be able to have any number
  // of rays (they all have to be in the same frame though),
  // make vector of pairs of btVectors.
  // const btVector3 start_;
  // const btVector3 end_;
  const geometry_msgs::Point start_;
  const geometry_msgs::Point end_;

  const btDiscreteDynamicsWorld* dynamics_world_;
};

