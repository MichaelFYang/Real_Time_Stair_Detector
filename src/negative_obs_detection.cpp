#include "negative_obs_detection/negative_obs_detection.h"

/* --------------------------------------------------------------- */

NegObsDetect::NegObsDetect() {
/* TODO! */
}

void NegObsDetect::Initialization() {
/* TODO!*/ 
    // Initai ROS params
    if (!nh_.getParam("vb_goal_topic",goal_topic_)) {
        goal_topic_ = "/way_point";
    } 
    if (!nh_.getParam("laserVoxelSize", laserVoxelSize_))
        laserVoxelSize_ = 0.05;
    // Allocate Memory for PointClouds
    laser_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    ground_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    neg_obs_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    neg_obs_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    laserDwzFilter_.setLeafSize(laserVoxelSize,laserVoxelSize,laserVoxelSize);

}

void NegObsDetect::CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg) {
/* Callback function of raw point cloud */
    laser_cloud_->clear();
    pcl::fromROSMsg(*laser_msg, *laser_cloud_);
    this->TransCloudFrame();
}

void NegObsDetect::TransCloudFrame() {
/* Source credit: http://pointclouds.org/documentation/tutorials/passthrough.php */
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_temp->clear();
    std::size_t laser_cloud_size = laser_cloud_->points.size();
    pcl::PointXYZI point;
    for (std::size_t i=0; i<laser_cloud_size; i++) {
        point = laser_cloud_->points[i];
        point.intensity = 1.0;
        this->LeftRotatePoint(point);
        laser_cloud_temp->points.push_back(point);
    }
    laserDwzFilter_->clear();
    laserDwzFilter.setInputCloud(laser_cloud_temp);
    laserDwzFilter.filter(*laser_cloud_);

}

void NegObsDetect::LeftRotatePoint(pcl::PointXYZI &pnt) {
/* Credit:https://bitbucket.org/cmusubt/misc_utils/src/master/src/misc_utils.cpp */
    float tmp_z = pnt.z;
    pnt.z = pnt.y;
    pnt.y = pnt.x;
    pnt.x = tmp_z;
}

void NegObsDetect::CloudImageProjection() {
/* TODO -> Make a Project PointCloud into a image [row, col] for BSF search*/
}

void NegObsDetect::GroundSegmentation() {
/* TODO! Segment Ground PointCloud -> Operation on laser_cloud_image */
    if (laser_cloud_image_->empty()) {
        std::cout<<"The Point Cloud is Empty now!"
        return;
    }
    std::size_t laser_cloud_size = laser_cloud_image_->points.size();
    for (std::size_t i=0; i<laser_cloud_size; i++) {
        // TODO!
    }
}








