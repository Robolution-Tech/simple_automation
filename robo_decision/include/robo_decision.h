#ifndef ROBO_DECISION_H
#define ROBO_DECISION_H

#include "json.hpp"
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <string>

namespace robo_decision
{

  enum class State
  {
    INIT,       // System init
    RUNNING,    // Running to A (unloaded) or B (with payloads)
    EXCAVATION, // Excavation mode
    RETURNNING,
    ABORT,
  };

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      MoveBaseClient;

  class RoboDecision
  {
  public:
    RoboDecision(const std::string &config_path);
    ~RoboDecision();

    nlohmann::json config_reader;
    ros::NodeHandle nh;

    ros::Subscriber cmd_vel_sub;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber user_command_sub;
    ros::Subscriber processing_sub;
    ros::Subscriber safety_sub;
    MoveBaseClient ac;

    State system_state = State::INIT;

    void Run();
    void Run_excavation();
    void Cmd_vel_callback(const geometry_msgs::Twist &msg);
    void User_command_callback(const std_msgs::Int8 &msg);
    void Processing_topic_callback(const geometry_msgs::PoseStamped &msg);
    void Safety_topic_callback(const std_msgs::Bool &msg);

  private:
    // Member varaibles
    std::string cmd_vel_sub_topic;
    std::string navigation_server_name;
    std::string user_command_topic;
    std::string processing_topic;
    std::string cmd_vel_pub_topic;

    std::string safety_topic;
    move_base_msgs::MoveBaseGoal processed_naviation_goal =
        move_base_msgs::MoveBaseGoal();
    geometry_msgs::Twist stop_vehicle_twist = geometry_msgs::Twist();
    move_base_msgs::MoveBaseGoal return_to_home_goal =
        move_base_msgs::MoveBaseGoal();
    int received_command = 0; // Defualt mode
    bool system_inited = false;
    geometry_msgs::Twist received_cmd_vel;
    bool safe_to_proceed = true; // TODO: check if safe to proceed?
    bool found_target = false;
    // Member functions
    void Init_return_to_home(const geometry_msgs::PoseStamped &home_location);
    void Init_stop_vehicle_twist();
  };
} // namespace robo_decision

#endif
