#include "pcd2laser/pcd2laser.hpp"

namespace pcd2laser
{
PCD2Laser::PCD2Laser() : nh_(""), pnh_("~")
{
    pnh_.param<std::string>("laser_topic", laser_topic_, "laser");
    pnh_.param<std::string>("pcd_topic", pcd_topic_, "pcd");
    pnh_.param<std::string>("frame_id", frame_id_, "laser_frame");
    pnh_.param<double>("height_max", height_max_, 1.0);
    pnh_.param<double>("height_min", height_min_, 0.0);
    pnh_.param<double>("angle_max", angle_max_, 2 * M_PI);
    pnh_.param<double>("angle_min", angle_min_, 2 * -M_PI);
    pnh_.param<double>("angle_increment", angle_increment_, 0.01);
    pnh_.param<double>("range_max", range_max_, 100.0);
    pnh_.param<double>("range_min", range_min_, 0.0);

    laser_pub_ = nh_.advertise<sensor_msgs::LaserScan>(laser_topic_, 1);
    pcd_sub_ = nh_.subscribe(pcd_topic_, 1, &PCD2Laser::pcdCallback, this);
}

PCD2Laser::~PCD2Laser()
{
}

void PCD2Laser::onInit()
{
}

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

    // Fill the ranges
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    std::vector<float> ranges;
    for (double angle = angle_min_; angle < angle_max_; angle += angle_increment_)
    {
        double x = range_max_ * cos(angle);
        double y = range_max_ * sin(angle);
        double z = height_max_;

        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            if (cloud->points[i].x < x && cloud->points[i].x > 0 && cloud->points[i].y < y && cloud->points[i].y > 0 && cloud->points[i].z < z && cloud->points[i].z > height_min_)
            {
                double range = sqrt(pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2));
                if (range < range_max_)
                {
                    ranges.push_back(range);
                }
            }
        }
    }

    laser_msg.ranges = ranges;
    laser_pub_.publish(laser_msg);
}

} // namespace pcd2laser

PLUGINLIB_EXPORT_CLASS(pcd2laser::PCD2Laser, nodelet::Nodelet)