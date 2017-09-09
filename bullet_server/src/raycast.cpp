/**
  Copyright 2017 Lucas Walter

*/

#include <bullet_server/raycast.h>
#include <geometry_msgs/Point.h>

Raycast::Raycast(const std::string name, const std::string frame_id,
      const geometry_msgs::Point start, const geometry_msgs::Point end) :
    name_(name),
    frame_id_(frame_id),
    start_(start.x, start.y, start.x),
    end_(end.x, end.y, end.z)
{
}
