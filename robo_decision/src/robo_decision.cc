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
      robo_decision_system_state_topic =
          config_reader["robo_decision_system_state_topic"];
      excavation_mode = bool(config_reader["excavation_mode"]);
      hard_code_topic = config_reader["hard_code_topic"];
      hard_code_cmd_vel_topic = config_reader["hard_code_cmd_vel_topic"];
      resume_after_stop = bool(config_reader["resume_after_stop"]);
      Setup_topics();
    }
  };
  RoboDecision::~RoboDecision() = default;

  void RoboDecision::Run()
  {
    while (ros::ok())
    {
      // TODO: Complete the expression for different state:
      is_system_ok = Check_system_ok();
      // robo_decision_system_state_pub.publish(int(system_state));
      switch (system_state)
      {
      case State::INIT:
        // Waiting for move_base action server to come up
        while (!ac.waitForServer(ros::Duration(5.0)) && ros::ok())
        {
          ROS_INFO("Waiting for the move_base action server to come up");
        }
        ROS_INFO("System initalized successfully.");
        ROS_INFO("Now waiting for incoming commands.");
        // Init_return_to_home();
        Init_stop_vehicle_twist();
        is_system_ok = Check_system_ok();
        break;

      case State::RUNNING:
        if (!is_system_ok)
        {
          break;
        }
        // Check if hard_code mode is on
        if (excavation_mode)
        {
          system_state = State::EXCAVATION;
          ROS_INFO("Now entering into excavation mode");
        }
        if (is_procecssed_navigation_goal_received)
        {
          ROS_INFO("Updating Navigation Goal ...");
          ac.cancelAllGoals();
          ac.sendGoal(processed_naviation_goal);
          if (is_cmd_vel_received && safe_to_proceed)
          {
            cmd_vel_pub.publish(received_cmd_vel);
          }
          else if (!safe_to_proceed)
          {
            ROS_ERROR("Not safe to proceed"); // TODO: maybe add unsafe state?
            cmd_vel_pub.publish(stop_vehicle_twist);
          }
        }
        break;

      case State::ABORT:
        /* system_stop = true; */
        if (is_system_ok)
        {
          ROS_INFO("Abortiong task ...");
          ac.cancelAllGoals();
          ROS_INFO("Stop the vehicle ...");
          cmd_vel_pub.publish(stop_vehicle_twist);
        }
        break;

      case State::RETURNNING:
        if (!is_system_ok)
        {
          break;
        }
        if (is_cmd_vel_received && safe_to_proceed)
        {
          cmd_vel_pub.publish(received_cmd_vel);
        }
        else if (!safe_to_proceed)
        {
          ROS_ERROR("Not safe to proceed"); // TODO: maybe add unsafe state?
          cmd_vel_pub.publish(stop_vehicle_twist);

          if (safe_to_proceed && is_system_ok)
          {
            cmd_vel_pub.publish(received_cmd_vel);
          }
          break;

        case State::EXCAVATION:
          // currently let other node to handle it.
          // if (safe_to_proceed && is_system_ok)
          // {
          //   Run_excavation();
          // }
          break;

        default:
          if (is_system_ok)
          {
            system_state = State::ABORT;
            ac.cancelAllGoals();
            cmd_vel_pub.publish(stop_vehicle_twist);
          }
        }
        ros::spinOnce();
      }
    }
  }

  void RoboDecision::Cmd_vel_callback(const geometry_msgs::Twist &msg)
  {
    is_cmd_vel_received = true;
    received_cmd_vel = msg;
  }

  void RoboDecision::User_command_callback(const std_msgs::Int8 &msg)
  {
    received_command = msg.data;
    switch (received_command)
    {
    case 1: // Task start
      ROS_INFO("Running command received ...");
      if (is_system_ok)
      {
        system_state = State::RUNNING;
        is_procecssed_navigation_goal_received = false;
        is_cmd_vel_received = false;
      }
      break;

    case 2: // Return home
      ROS_INFO("Returning home comand received ...");
      if (is_system_ok)
      {
        system_state = State::RETURNNING;
        ac.cancelAllGoals();
        ROS_INFO("Now updaing navigation goal to home ...");
        is_cmd_vel_received = false;
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
    is_procecssed_navigation_goal_received = true;
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
    if (!safe_to_proceed && resume_after_stop)
    {
      system_state = State::ABORT;
    }
  }

  // TODO: Hard-coded sequence
  void RoboDecision::Run_excavation()
  {
  }

  void RoboDecision::Hard_code_callback(const std_msgs::Bool &msg)
  {
    excavation_mode = bool(msg.data);
  }

  void RoboDecision::Hard_code_cmd_vel_callback(
      const geometry_msgs::Twist &msg)
  {
    received_cmd_vel = msg;
  }

  // TODO: check system running status
  bool RoboDecision::Check_system_ok()
  {
    if (!ac.isServerConnected())
    {
      return false;
    }
    return true;
  }

  void RoboDecision::Setup_topics()
  {
    // set up publishers:
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_pub_topic, 1000);
    robo_decision_system_state_pub =
        nh.advertise<std_msgs::Int8>(robo_decision_system_state_topic, 1000);

    // set up subscribers:
    cmd_vel_sub = nh.subscribe(cmd_vel_sub_topic, 1000,
                               &RoboDecision::Cmd_vel_callback, this);
    user_command_sub = nh.subscribe(
        user_command_topic, 1000, &RoboDecision::User_command_callback, this);
    processing_sub =
        nh.subscribe(processing_topic, 1000,
                     &RoboDecision::Processing_topic_callback, this);
    safety_sub = nh.subscribe(safety_topic, 1000,
                              &RoboDecision::Safety_topic_callback, this);
    hard_code_sub = nh.subscribe(hard_code_topic, 1000,
                                 &RoboDecision::Hard_code_callback, this);
    hard_code_cmd_vel_sub =
        nh.subscribe(hard_code_cmd_vel_topic, 1000,
                     &RoboDecision::Hard_code_cmd_vel_callback, this);
  }
} // namespace robo_decision
