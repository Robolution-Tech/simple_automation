/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include <limits>
#include <pluginlib/class_list_macros.h>
#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <ctime>

namespace pointcloud_to_laserscan
{
PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet()
{
}

void PointCloudToLaserScanNodelet::onInit()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("target_frame", target_frame_, "");
  private_nh_.param<std::string>("lidar_left_topic", lidar_left_topic_, "");
  private_nh_.param<std::string>("lidar_right_topic", lidar_right_topic_, "");
  private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
  private_nh_.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_height", max_height_, std::numeric_limits<double>::max());

  private_nh_.param<double>("angle_min", angle_min_, -M_PI);
  private_nh_.param<double>("angle_max", angle_max_, M_PI);
  private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
  private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
  private_nh_.param<double>("range_min", range_min_, 0.0);
  private_nh_.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
  private_nh_.param<double>("inf_epsilon", inf_epsilon_, 1.0);

  int concurrency_level;
  private_nh_.param<int>("concurrency_level", concurrency_level, 1);
  private_nh_.param<bool>("use_inf", use_inf_, true);

  std::cout<< "start" << std::endl;
  // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
  if (concurrency_level == 1)
  {
    nh_ = getNodeHandle();
  }
  else
  {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0)
  {
    input_queue_size_ = concurrency_level;
  }
  else
  {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }

  // if pointcloud target frame specified, we need to filter by transform availability
  // if only left(first) lidar is avaliable
  if (!target_frame_.empty() && lidar_right_topic_.empty())
  {
    std::cout<< "lidar left only" << std::endl;
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    lidar_left_message_filter_.reset(new MessageFilter(lidar_left_sub_, *tf2_, target_frame_, input_queue_size_, nh_));
    lidar_left_message_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    lidar_left_message_filter_->registerFailureCallback(boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
  }
  else if (target_frame_.empty())  // otherwise setup direct subscription
  {
    std::cout<< "target frame empty" << std::endl;
    lidar_left_sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
  }
  else{
    std::cout<< "two cloud combine" << std::endl;
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    lidar_left_message_filter_.reset(new MessageFilter(lidar_left_sub_, *tf2_, target_frame_, input_queue_size_, nh_));
    lidar_right_message_filter_.reset(new MessageFilter(lidar_right_sub_, *tf2_, target_frame_, input_queue_size_, nh_));

    sync_.reset(new Sync(appTimeSyncPolicy_(10),*lidar_left_message_filter_,*lidar_right_message_filter_));
    sync_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloud_combine, this, _1,_2));
    // timeSynchronizer_ = new message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>(*message_filter_,*lidar_right_message_filter_,1);
    // timeSynchronizer_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloud_combine, this, _1,_2));
  }

  pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
                                               boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));

  cloud_merged_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloud_merged", 1, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
                                               boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
}

void PointCloudToLaserScanNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if ((pub_.getNumSubscribers() > 0 || cloud_merged_pub_.getNumSubscribers()>0) && lidar_left_sub_.getSubscriber().getNumPublishers() == 0 && lidar_right_sub_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
    lidar_left_sub_.subscribe(nh_, lidar_left_topic_, input_queue_size_);
    lidar_right_sub_.subscribe(nh_, lidar_right_topic_, input_queue_size_);
  }else{
    printf("no subscriber");
  }
}

void PointCloudToLaserScanNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0 && cloud_merged_pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
    lidar_left_sub_.unsubscribe();
    lidar_right_sub_.unsubscribe();
  }
}

void PointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
                                             tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
                                                                             << lidar_left_message_filter_->getTargetFramesString()
                                                                             << " at time " << cloud_msg->header.stamp
                                                                             << ", reason: " << reason);
}

void PointCloudToLaserScanNodelet::cloud_combine(const sensor_msgs::PointCloud2ConstPtr& left_cloud_msg, const sensor_msgs::PointCloud2ConstPtr& right_cloud_msg){
  // std::cout<< "in the cloud combine" << std::endl;
  sensor_msgs::PointCloud2 cloud_merge;
  sensor_msgs::PointCloud2Ptr cloud_left;
  sensor_msgs::PointCloud2Ptr cloud_right;
  sensor_msgs::PointCloud2ConstPtr cloud_out;
  cloud_left.reset(new sensor_msgs::PointCloud2);
  cloud_right.reset(new sensor_msgs::PointCloud2);
  // std::cout<< "1" << std::endl;
  // Transform cloud if necessary
  tf2_->transform(*left_cloud_msg, *cloud_left, target_frame_, ros::Duration(tolerance_));
  tf2_->transform(*right_cloud_msg, *cloud_right, target_frame_, ros::Duration(tolerance_));
  
  // std::cout<< "2" << std::endl;
  //combine two transformed point cloud
  pcl::concatenatePointCloud(*cloud_left, *cloud_right, cloud_merge);


  // std::cout<< "3" << std::endl;
  cloud_out.reset(new sensor_msgs::PointCloud2(cloud_merge));
  //create laser scan output
  sensor_msgs::LaserScan output;
  output.header = left_cloud_msg->header;
  output.header.frame_id = target_frame_;

  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;
  // std::cout<< "4" << std::endl;
  // determine amount of rays to create
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_)
  {
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else
  {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  // std::clock_t start;
  // double duration;
  // start = std::clock();

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
       iter_z(*cloud_out, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_)
    {
      NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_)
    {
      NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_)
    {
      NODELET_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
      NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - output.angle_min) / output.angle_increment;
    if (range < output.ranges[index])
    {
      output.ranges[index] = range;
    }
  }

  // duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
  // std::cout<<"printf: "<< duration <<'\n';

  pub_.publish(output);
  cloud_merged_pub_.publish(cloud_merge);


}

void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // build laserscan output
  sensor_msgs::LaserScan output;
  output.header = cloud_msg->header;
  if (!target_frame_.empty())
  {
    output.header.frame_id = target_frame_;
  }

  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_)
  {
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  }
  else
  {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  sensor_msgs::PointCloud2ConstPtr cloud_out;
  sensor_msgs::PointCloud2Ptr cloud;

  // Transform cloud if necessary
  if (!(output.header.frame_id == cloud_msg->header.frame_id))
  {
    try
    {
      cloud.reset(new sensor_msgs::PointCloud2);
      tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tolerance_));
      cloud_out = cloud;
    }
    catch (tf2::TransformException& ex)
    {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
      return;
    }
  }
  else
  {
    cloud_out = cloud_msg;
  }

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
       iter_z(*cloud_out, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_)
    {
      NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_)
    {
      NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_)
    {
      NODELET_DEBUG("rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_, *iter_x,
                    *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max)
    {
      NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - output.angle_min) / output.angle_increment;
    if (range < output.ranges[index])
    {
      output.ranges[index] = range;
    }
  }
  pub_.publish(output);
}
}  // namespace pointcloud_to_laserscan

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet)