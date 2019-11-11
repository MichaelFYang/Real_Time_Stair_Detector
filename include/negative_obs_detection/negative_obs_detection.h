/*
Negtive Obstable Detection
Author: Fan Yang (fanyang2@cs.cmu.edu)
Organization: CMU Sub-T Explorer Team
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>


class NegObsDetect
{
public:
    NegObsDetect();
    void Loop();
private:
    // ROS params
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher negative_object_border_pub_;
    // Point Cloud Value
    pcl::PointCloud<pcl::PointXYZI>::Ptr frontier_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_filtered_;
	pcl::PointCloud<pcl::PointXYZI>::Ptr laser_frontier_filtered_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_collision_cloud_;
	pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_frontier_cloud_;
    // Operation Functions
    void Initialization();
    void CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg);
};