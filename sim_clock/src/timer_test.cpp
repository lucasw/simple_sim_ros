#include <ros/ros.h>

ros::Time start;
ros::WallTime wall_start;

void printTime(const ros::TimerEvent& e, const std::string prefix)
{
  // ROS_INFO_STREAM(
  std::cout <<
      prefix
      << ", wall: " << ros::WallTime::now() - wall_start
      << ", now: " << ros::Time::now() - start
      << ", expected: " << (e.current_expected - start).toSec()
      << ", real: " << (e.current_real - start).toSec()
      << std::endl;
  // );
}

void callback1(const ros::TimerEvent& e)
{
  printTime(e, "callback 1");
}

void callback2(const ros::TimerEvent& e)
{
  printTime(e, "callback 2");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;

  start = ros::Time::now();
  // don't get the wall time until a clock has been received
  ros::Duration(0.0001).sleep();
  wall_start = ros::WallTime::now();

  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);
  ros::Timer timer2 = n.createTimer(ros::Duration(1.0), callback2);
  ros::spin();

  return 0;
}
