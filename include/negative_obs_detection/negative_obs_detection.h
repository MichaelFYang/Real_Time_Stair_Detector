/*
Negtive Obstable Detection
Author: Fan Yang (fanyang2@cs.cmu.edu)
Organization: CMU Sub-T Explorer Team
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <string>

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;


class NegObsDetect
{
public:
    NegObsDetect();
    void Loop();
private:
    // ROS Valuables
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher negative_object_border_pub_;
    // params
    std::string goal_topic_, laser_topic_sub_, laser_topic_pub_, neg_obs_pub_; 
    double laserVoxelSize_;
    // Point Cloud Value
    PointCloudPtr laser_cloud_;
    PointCloudPrt laser_cloud_image_;
    PointCloudPtr ground_cloud_;
    PointCloudPtr neg_obs_cloud_;

    pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter_;

    // Operation Functions
    void Initialization();
    void CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg);
    void TransCloudFrame();
    void GroundSegmentation();
    void CloudImageProjection();
    // Tool Functions
    void LeftRotatePoint(pcl::PointXYZI &pnt);
    

};