#include <ros/ros.h>

ros::Time start;

void callback1(const ros::TimerEvent& e)
{
  ROS_INFO_STREAM("Callback 1 triggered "
      << ros::Time::now() - start << " "
      << (e.current_expected - start).toSec() << " "
      << (e.current_real - start).toSec());
}

void callback2(const ros::TimerEvent& e)
{
  ROS_INFO_STREAM("Callback 2 triggered "
      << ros::Time::now() - start << " "
      << (e.current_expected - start).toSec() << " "
      << (e.current_real - start).toSec());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;

  start = ros::Time::now();

  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);
  ros::Timer timer2 = n.createTimer(ros::Duration(1.0), callback2);
  ros::spin();

  return 0;
}
