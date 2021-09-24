#ifndef ROBO_DECISION_H
#define ROBO_DECISION_H

#include "json.hpp"
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <mutex>
#include <ros/ros.h>
#include <sstream>
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

    // Subscribers:
    ros::Subscriber cmd_vel_sub;
    ros::Subscriber user_command_sub;
    ros::Subscriber processing_sub;
    ros::Subscriber safety_sub;
    ros::Subscriber hard_code_sub;
    ros::Subscriber hard_code_cmd_vel_sub;

    // Publishers:
    ros::Publisher robo_decision_system_state_pub;
    ros::Publisher cmd_vel_pub;

    // Client:
    MoveBaseClient ac;

    State system_state = State::INIT;

    // Main threads:
    void Run();
    void Run_excavation();

    // Sub - callbacks:
    void Cmd_vel_callback(const geometry_msgs::Twist &msg);
    void User_command_callback(const std_msgs::Int8 &msg);
    void Processing_topic_callback(const geometry_msgs::PoseStamped &msg);
    void Safety_topic_callback(const std_msgs::Bool &msg);
    void Hard_code_callback(const std_msgs::Bool &msg);
    void Hard_code_cmd_vel_callback(const geometry_msgs::Twist &msg);

  private:
    // Topic names
    std::string cmd_vel_sub_topic;
    std::string navigation_server_name;
    std::string user_command_topic;
    std::string processing_topic;
    std::string cmd_vel_pub_topic;
    std::string robo_decision_system_state_topic;
    std::string hard_code_topic;
    std::string hard_code_cmd_vel_topic;
    std::string safety_topic;

    // Navigation/cmd_vel
    move_base_msgs::MoveBaseGoal processed_naviation_goal =
        move_base_msgs::MoveBaseGoal();
    geometry_msgs::Twist stop_vehicle_twist = geometry_msgs::Twist();
    move_base_msgs::MoveBaseGoal return_to_home_goal =
        move_base_msgs::MoveBaseGoal();
    geometry_msgs::Twist received_cmd_vel;

    // State variables
    int received_command = 0; // Defualt mode
    bool safe_to_proceed = true;
    bool found_target = false;
    bool excavation_mode = false;
    bool resume_after_stop = true;
    bool is_system_ok = false;
    bool is_procecssed_navigation_goal_received = false;
    bool is_cmd_vel_received = false;

    // Member functions
    void Init_return_to_home(const geometry_msgs::PoseStamped &home_location);
    void Init_stop_vehicle_twist();
    bool Check_system_ok();
  };
} // namespace robo_decision

#endif
