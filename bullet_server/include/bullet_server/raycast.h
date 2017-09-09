#include <bullet/btBulletDynamicsCommon.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>

class Raycast
{
public:
  Raycast(const std::string name, const std::string frame_id,
      const geometry_msgs::Point start, const geometry_msgs::Point end);

  const std::string name_;
  const std::string frame_id_;
  // both are in frame_id_
  // const
  btVector3 start_;
  const btVector3 end_;
};

