#include "include/robo_decision.h"

namespace robo_decision
{
  RoboDecision::RoboDecision(const std::string &config_path)
      : ac("move_base", true)
  {
    system_state = State::INIT;
    if (!boost::filesystem::exists(config_path))
    {
      ROS_ERROR("Can't open config file!");
      system_state = State::ABORT;
    }
    else
    {
      std::stringstream ss;
      ss << "Reading config file from " << config_path << " ...";
      std::cout << ss.str() << std::endl;
      std::ifstream i(config_path);
      i >> config_reader;
      cmd_vel_pub_topic = config_reader["cmd_vel_pub_topic"];
      cmd_vel_sub_topic = config_reader["cmd_vel_sub_topic"];
      /* navigation_server_name = config_reader["navigation_server_name"]; */
      user_command_topic = config_reader["user_command_topic"];
      processing_topic = config_reader["processing_topic"];
      safety_topic = config_reader["safety_topic"];
      excavation_mode = bool(config_reader["excavation_mode"]);
    }
  };
  RoboDecision::~RoboDecision() = default;

  void RoboDecision::Run()
  {
    while (ros::ok())
    {
      // TODO: Complete the expression for different state:
      switch (system_state)
      {
      case State::INIT:
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_pub_topic, 1000);
        cmd_vel_sub = nh.subscribe(cmd_vel_sub_topic, 1000,
                                   &RoboDecision::Cmd_vel_callback, this);
        user_command_sub = nh.subscribe(
            user_command_topic, 1000, &RoboDecision::User_command_callback, this);
        processing_sub =
            nh.subscribe(processing_topic, 1000,
                         &RoboDecision::Processing_topic_callback, this);
        safety_sub = nh.subscribe(safety_topic, 1000,
                                  &RoboDecision::Safety_topic_callback, this);
        while (!ac.waitForServer(ros::Duration(5.0)) && ros::ok())
        {
          ROS_INFO("Waiting for the move_base action server to come up");
        }
        ROS_INFO("System initalized successfully.");
        system_inited = true;
        ROS_INFO("Now waiting for incoming commands.");
        // Init_return_to_home();
        Init_stop_vehicle_twist();
        break;

      case State::RUNNING:
        // check if the new navigation goal is received:
        if (safe_to_proceed && system_inited)
        {
          cmd_vel_pub.publish(received_cmd_vel);
          if (found_target ||
              ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("Goal achieved ...");
            ac.cancelAllGoals();
            if (excavation_mode)
            {
              ROS_INFO("Now entering into excavation mode");
              system_state = State::EXCAVATION;
            }
          }
        }
        break;

      case State::ABORT:
        /* system_stop = true; */
        if (system_inited)
        {
          ROS_INFO("Abortiong task ...");
          ac.cancelAllGoals();
          ROS_INFO("Stop the vehicle ...");
          cmd_vel_pub.publish(stop_vehicle_twist);
        }
        break;

      case State::RETURNNING:
        if (safe_to_proceed && system_inited)
        {
          cmd_vel_pub.publish(received_cmd_vel);
        }
        break;

      case State::EXCAVATION:
        if (safe_to_proceed && system_inited)
        {
          Run_excavation();
        }
        break;

      default:
        if (system_inited)
        {
          system_state = State::ABORT;
          ac.cancelAllGoals();
          cmd_vel_pub.publish(stop_vehicle_twist);
        }
      }
      ros::spinOnce();
    }
  }

  void RoboDecision::Cmd_vel_callback(const geometry_msgs::Twist &msg)
  {
    received_cmd_vel = msg;
  }

  void RoboDecision::User_command_callback(const std_msgs::Int8 &msg)
  {
    received_command = msg.data;
    switch (received_command)
    {
    case 1: // Task start
      ROS_INFO("Running command received ...");
      if (system_inited)
      {
        system_state = State::RUNNING;
        ac.cancelAllGoals();
        ROS_INFO("Sending Navigation Goal ...");
        ac.sendGoal(processed_naviation_goal);
      }
      break;

    case 2: // Return home
      ROS_INFO("Returning home comand received ...");
      if (system_inited)
      {
        system_state = State::RETURNNING;
        ac.cancelAllGoals();
        ROS_INFO("Now updaing navigation goal to home ...");
        ac.sendGoal(return_to_home_goal);
      }
      break;

    case 3: // Abort mission / (stop)
      ROS_INFO("Abort command received ...");
      system_state = State::ABORT;
      break;

    default:
      system_state = State::ABORT;
      break;
    }
  }

  void RoboDecision::Processing_topic_callback(
      const geometry_msgs::PoseStamped &msg)
  {
    processed_naviation_goal.target_pose.header.frame_id = msg.header.frame_id;
    processed_naviation_goal.target_pose.header.stamp = ros::Time::now();
    processed_naviation_goal.target_pose.pose = msg.pose;
    /* if (system_state == State::RUNNING) { */
    /*   ROS_INFO("Sending Navigation Goal ..."); */
    /*   ac.sendGoal(processed_naviation_goal); */
    /* } */
  }

  void RoboDecision::Init_return_to_home(
      const geometry_msgs::PoseStamped &home_location)
  {
    return_to_home_goal = move_base_msgs::MoveBaseGoal();
    return_to_home_goal.target_pose.header.frame_id =
        home_location.header.frame_id;
    return_to_home_goal.target_pose.header.stamp = ros::Time::now();
    return_to_home_goal.target_pose.pose = home_location.pose;
  }

  void RoboDecision::Init_stop_vehicle_twist()
  {
    stop_vehicle_twist.linear.x = 0;
    stop_vehicle_twist.linear.y = 0;
    stop_vehicle_twist.linear.z = 0;
    stop_vehicle_twist.angular.x = 0;
    stop_vehicle_twist.angular.y = 0;
    stop_vehicle_twist.angular.z = 0;
  }

  void RoboDecision::Safety_topic_callback(const std_msgs::Bool &msg)
  {
    safe_to_proceed = msg.data;
    if (!safe_to_proceed)
    {
      system_state = State::ABORT;
    }
  }

  // TODO: Hard-coded sequence
  void RoboDecision::Run_excavation() {}
} // namespace robo_decision
