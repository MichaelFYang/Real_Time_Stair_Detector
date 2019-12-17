/*
Negtive Obstable Detection
Author: Fan Yang (fanyang2@cs.cmu.edu)
Organization: CMU Sub-T Explorer Team
*/

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>
#include <unordered_set>
#include <tf/transform_broadcaster.h>
#include <queue> 
#include <math.h>

#include <string>

#define N_SCAN 16
#define ANG_RES_Y 2.0
#define ANG_RES_X  0.2
#define ANG_BOTTOM 15.1
#define HORIZON_SCAN  1800


typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

struct Point3D {
    float x;
    float y;
    float z;
    Point3D() {}
    Point3D(float _x, float _y, float _z): x(_x), y(_y), z(_z) {}
    bool operator ==(const Point3D& pt) const
    {
        return x == pt.x && y == pt.y && z == pt.z;
    }

    float operator *(const Point3D& pt) const
    {
        return x * pt.x + y * pt.y + z * pt.z;
    }

    Point3D operator *(const float factor) const
    {
        return Point3D(x*factor, y*factor, z*factor);
    }

    Point3D operator /(const float factor) const
    {
        return Point3D(x/factor, y/factor, z/factor);
    }

    Point3D operator +(const Point3D& pt) const
    {
        return Point3D(x+pt.x, y+pt.y, z+pt.z);
    }
    Point3D operator -(const Point3D& pt) const
    {
        return Point3D(x-pt.x, y-pt.y, z-pt.z);
    }
};


class NegObsDetect
{
public:
    NegObsDetect();
    void Loop();
private:
    // ROS Valuables
    ros::NodeHandle nh_;
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher ground_cloud_pub_;
    ros::Publisher cloud_image_pub_;
    ros::Publisher negative_object_border_pub_;
    // params
    std::string laser_topic_sub_, odom_topic_sub_, ground_topic_pub_, neg_obs_topic_pub_, cloud_image_topic_pub_;
    std::string odom_frame_id_;
    double slope_thresh_;
    double robot_heading_;
    nav_msgs::Odometry odom_;
    Point3D robot_pos_;
    PointType nanPoint_;
    // TF tree
    // Point Cloud Value
    PointCloudPtr laser_cloud_;
    PointCloudPtr laser_cloud_image_;
    PointCloudPtr laser_cloud_image_world_;
    PointCloudPtr ground_cloud_;
    PointCloudPtr neg_obs_cloud_;
    sensor_msgs::PointCloud2ConstPtr cloud_msg_;
    sensor_msgs::PointCloud2 ground_ros_cloud_;
    sensor_msgs::PointCloud2 cloud_image_ros_cloud_;
    sensor_msgs::PointCloud2 neg_obs_ros_cloud_; 

    // Operation Functions
    void Initialization();
    void TopicHandle();
    void CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg);
    void OdomHandler(const nav_msgs::Odometry odom_msg);
    void TransCloudFrame();
    void GroundSegmentation();
    void CloudImageProjection();
    // Tool Functions
    void LeftRotatePoint(pcl::PointXYZI &pnt);
    void RightRotatePointToWorld(pcl::PointXYZI &pnt);
    

};