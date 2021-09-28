#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

class pointcloud_to_laserscan{
	private:
		ros::Subscriber lidar1_sub_;
		ros::Subscriber lidar2_sub_;
	
	public:
		void pointcloud_to_laserscan(ros::NodeHandle *nh);
		//lidar1_sub_ = nh->subscribe("/lidar1_topic", 1000,&pointcloud_to_laserscan::callback_lidar, this);

		bool callback_lidar(const 

}
