/**
Copyright 2017 Lucas Walter
Port of sim_clock.py

*/

#include <dynamic_reconfigure/server.h>
#include <rosgraph_msgs/Clock.h>
#include <sim_clock/ClockConfig.h>
#include <std_msgs/Float32.h>

class SimClock
{
public:
  SimClock();

private:
  void update(const ros::WallDuration dt);
  bool first_single_;
  void singleDt(const ros::WallTimerEvent& e);
  void step(const std_msgs::Float32ConstPtr& msg);

  sim_clock::ClockConfig config_;
  boost::recursive_mutex dr_mutex_;
  typedef dynamic_reconfigure::Server<sim_clock::ClockConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  void reconfigureCallback(
      sim_clock::ClockConfig& config,
      uint32_t level);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double cur_time_;
  double extra_time_;
  ros::WallDuration sleep_time_;
  ros::WallTimer timer_;
};

SimClock::SimClock() :
  first_single_(false),
  nh_private_("~"),
  cur_time_(0.0),  // TODO(lucasw) allow this to be set
  extra_time_(0.0),
  sleep_time_(0.0)
{
  pub_ = nh_.advertise<rosgraph_msgs::Clock>("clock", 10);
  // dynamic reconfigure init
  {
    reconfigure_server_.reset(
        new ReconfigureServer(dr_mutex_, nh_private_));
    dynamic_reconfigure::Server<sim_clock::ClockConfig>::CallbackType scrc =
      boost::bind(&SimClock::reconfigureCallback, this, _1, _2);
    reconfigure_server_->setCallback(scrc);
  }

  sub_ = nh_private_.subscribe("step", 10, &SimClock::step, this);
  //
}

// TODO(lucasw) the names of update and singleDt should maybe be reversed
void SimClock::update(const ros::WallDuration dt)
{
  cur_time_ += dt.toSec();  // * config_.real_time_factor;
  // TODO(lucasw) time values can't be negative, so reverse
  // time isn't going to work well without a large positive offset.
  rosgraph_msgs::Clock msg;
  ros::Time time;
  msg.clock = time.fromSec(cur_time_);
  pub_.publish(msg);
  extra_time_ = 0.0;
}

void SimClock::singleDt(const ros::WallTimerEvent& e)
{
  // on the first call since init or resuming last_real will be zero
  // or very far in the past
  if (!first_single_)
  {
    update(ros::WallDuration(config_.dt));
    first_single_ = true;
  }
  else
  {
    update((e.current_real - e.last_real) * config_.real_time_factor);
  }
}

// trigger the update from a message optionally or the
// One example of a message would be from a cpu intensive node that has
// finished processing
void SimClock::step(const std_msgs::Float32ConstPtr& msg)
{
  // TODO(lucasw) want to keep a self.target_time around
  // so that sequential calls to step will not round to nearest dt,
  const double target_time = cur_time_ + msg->data - extra_time_;
  ROS_INFO_STREAM(target_time << " " << cur_time_ << " "
      << msg->data << " " << extra_time_);
  // TODO(lucasw) maybe a do-while, so one dt is always going to happen?
  ros::WallTime start_time = ros::WallTime::now();
  // do at least a single update even if extra_time would otherwise prevent it
  do
  {
    sleep_time_.sleep();
    const ros::WallTime end_time = ros::WallTime::now();
    const ros::WallDuration dt = (end_time - start_time);
    update(dt * config_.real_time_factor);
    start_time = end_time;
  }
  while ((cur_time_ < target_time) && (!config_.play_pause) && ros::ok());
  // subtract this from target time if next update is a step, otherwise zero it out
  extra_time_ = cur_time_ - target_time;
}

void SimClock::reconfigureCallback(
      sim_clock::ClockConfig& config,
      uint32_t level)
{
  if (!config.play_pause)
  {
    if (config_.play_pause)
    {
      first_single_ = false;
      timer_.stop();
    }

    if (config.single_step)
    {
      const std_msgs::Float32Ptr msg(new std_msgs::Float32);
      msg->data = config.step_time;
      step(msg);
      config.single_step = false;
    }
    else if (config.single_dt)
    {
      update(ros::WallDuration(config.dt));
      config.single_dt = false;
    }
  }
  else
  {
    ros::WallDuration sleep_time = ros::WallDuration(std::abs(config.dt / config.real_time_factor));
    if (sleep_time != sleep_time_)
    {
      ROS_INFO_STREAM(sleep_time << " " << config.dt << " " << config.real_time_factor
          << " " << cur_time_);
      sleep_time_ = sleep_time;
      if (!timer_.isValid())
        timer_ = nh_.createWallTimer(sleep_time, &SimClock::singleDt, this);
      else
        timer_.setPeriod(sleep_time);
    }
    // if (!config_.play_pause)
    timer_.start();
  }
  config_ = config;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_clock");
  SimClock sim_clock;
  ros::spin();
}
