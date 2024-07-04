#include "pcd2laser/pcd2laser.hpp"

namespace pcd2laser
{
void PCD2Laser::pcdCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    sensor_msgs::LaserScan laser_msg;
    laser_msg.header = msg->header;
    laser_msg.header.frame_id = frame_id_;
    laser_msg.angle_min = angle_min_;
    laser_msg.angle_max = angle_max_;
    laser_msg.angle_increment = angle_increment_;
    laser_msg.range_min = range_min_;
    laser_msg.range_max = range_max_;
    
    unsigned int num_ranges = (angle_max_ - angle_min_) / angle_increment_;
    laser_msg.ranges.resize(num_ranges, std::numeric_limits<float>::quiet_NaN());

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    for (const auto &point : cloud.points) {
        if (point.z < height_min_ || point.z > height_max_) continue;
        double angle = atan2(point.y, point.x);
        if (angle < angle_min_ || angle > angle_max_) continue;
        unsigned int index = (angle - angle_min_) / angle_increment_;
        if (index >= num_ranges) continue;
        double range = sqrt(point.x * point.x + point.y * point.y);
        if (range < range_min_ || range > range_max_) continue;
        if (std::isnan(laser_msg.ranges[index]) || range < laser_msg.ranges[index]) {
            laser_msg.ranges[index] = range;
        }
    }
    laser_pub_.publish(laser_msg);
}

void PCD2Laser::onInit()
{
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    pnh_.param<std::string>("laser_topic", laser_topic_, "laser");
    pnh_.param<std::string>("pcd_topic", pcd_topic_, "pcd");
    pnh_.param<std::string>("frame_id", frame_id_, "laser");
    pnh_.param<double>("height_max", height_max_, 1.0);
    pnh_.param<double>("height_min", height_min_, 0.0);
    pnh_.param<double>("angle_max", angle_max_, 2.0 * M_PI);
    pnh_.param<double>("angle_min", angle_min_, -2.0 * M_PI);
    pnh_.param<double>("angle_increment", angle_increment_, 0.01);
    pnh_.param<double>("range_max", range_max_, 100.0);
    pnh_.param<double>("range_min", range_min_, 0.0);

    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>(laser_topic_, 1);
    pcd_sub_ = nh_.subscribe(pcd_topic_, 1, &PCD2Laser::pcdCallback, this);
}

} // namespace pcd2laser

PLUGINLIB_EXPORT_CLASS(pcd2laser::PCD2Laser, nodelet::Nodelet)