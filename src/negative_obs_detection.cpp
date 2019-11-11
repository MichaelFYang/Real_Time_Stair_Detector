#include "negative_obs_detection/negative_obs_detection.h"

/* --------------------------------------------------------------- */

NegObsDetect::NegObsDetect() {
/* TODO! */
}

void NegObsDetect::Initialization() {
/* TODO!*/  
}

void NegObsDetect::CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg) {
/* TODO!*/
    laser_cloud_->clear();
    pcl::fromROSMsg(*laser_msg, *laser_cloud_);
    this->LaserCloudFilter(laser_cloud_filtered_);
    kdtree_collision_cloud_->setInputCloud(laser_cloud_filtered_);
}




