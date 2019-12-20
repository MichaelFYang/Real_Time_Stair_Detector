/*
Negtive Obstable Detection
Author: Fan Yang (fanyang2@cs.cmu.edu)
Organization: CMU Sub-T Explorer Team
*/

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>
#include <unordered_set>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <math.h>
#include <deque> 
#include <math.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#define N_SCAN 16 // N_SCAN/2 matters
#define LIDAR_H 1.0
#define ANG_RES_Y 2.0
#define ANG_RES_X  0.2
#define KERNEL_SIZE 5
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
    ros::Publisher stair_center_pub_;
    ros::ServiceServer kenerl_service_;
    // params
    std::string laser_topic_sub_, odom_topic_sub_, ground_topic_pub_, stair_topic_pub_, cloud_image_topic_pub_, kernel_server_topic_;
    std::string odom_frame_id_;
    std::vector<Point3D> kernel_elem_;
    std::string kernel_filename_;
    bool is_kernel_, is_inited_;
    float slope_thresh_;
    float flat_thresh_;
    double robot_heading_;
    float correlation_thred_;
    int col_filter_size_;
    int frame_filter_size_;
    std::vector<std::vector<Point3D> > elem_matrix_;
    std::vector<Point3D> elem_score_;
    std::deque<std::vector<Point3D> > frame_elem_score_;
    nav_msgs::Odometry odom_;
    Point3D robot_pos_;
    PointType nanPoint_;
    // TF tree
    // Point Cloud Value
    PointCloudPtr laser_cloud_;
    PointCloudPtr laser_cloud_image_;
    PointCloudPtr laser_cloud_image_world_;
    PointCloudPtr ground_cloud_;
    PointCloudPtr stair_center_cloud_;
    sensor_msgs::PointCloud2ConstPtr cloud_msg_;
    sensor_msgs::PointCloud2 ground_ros_cloud_;
    sensor_msgs::PointCloud2 cloud_image_ros_cloud_;
    sensor_msgs::PointCloud2 stair_center_ros_cloud_; 

    // Operation Functions
    void Initialization();
    void TopicHandle();
    bool KernelGeneration(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res);
    void NeighberAngleUpdate(std::size_t col, std::size_t row, float& angle_down, float& angle_up);
    void FilterColumn();
    void FilterFrames();
    void ReadKernelFile();
    void NormColElem(std::vector<Point3D> &elem_col);
    void TransToWorld(pcl::PointXYZI &pnt);
    void CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg);
    void OdomHandler(const nav_msgs::Odometry odom_msg);
    void TransCloudFrame();
    void GroundSegmentation();
    void CloudImageProjection();
    void SimularityCalculation();
    // Tool Functions
    void LeftRotatePoint(pcl::PointXYZI &pnt);
    void RightRotatePointToWorld(pcl::PointXYZI &pnt);
    float Sigmoid(float x);
    float ReLu(float x);
    

};