/*
PCD2Laser
Converts a PointCloud2 message to a LaserScan message
*/
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>

namespace pcd2laser
{
class PCD2Laser : public nodelet::Nodelet
{
private:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher laser_pub_;
    ros::Subscriber pcd_sub_;

    std::string laser_topic_, pcd_topic_;
    std::string frame_id_;
    double height_max_, height_min_, angle_max_, angle_min_, angle_increment_, range_max_, range_min_;

    void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

public:
    virtual void onInit();
};

} // namespace pcd2laser
