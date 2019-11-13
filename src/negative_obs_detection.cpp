#include "negative_obs_detection/negative_obs_detection.h"

/* --------------------------------------------------------------- */

NegObsDetect::NegObsDetect() {
/* TODO! */
     // Initai ROS params
    if (!nh_.getParam("vb_goal_topic",goal_topic_)) {
        goal_topic_ = "/way_point";
    } 
    if (!nh_.getParam("laserVoxelSize", laserVoxelSize_))
        laserVoxelSize_ = 0.05;
    this->Initialization();
}

void NegObsDetect::Initialization() {
/* TODO!*/ 
    // Allocate Memory for PointClouds
    laser_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ground_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    neg_obs_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    neg_obs_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    nanPoint_.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

}

void NegObsDetect::CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg) {
/* Callback function of raw point cloud */
    laser_cloud_->clear();
    laser_cloud_image->clear();
    pcl::fromROSMsg(*laser_msg, *laser_cloud_);
    this->TransCloudFrame();
    laser_cloud_imgae.resize();
}

void VB_Planner::OdomHandler(const nav_msgs::Odometry odom_msg) {
/* Odom Callback function */
    odom_ = odom_msg;
    robot_pos_.x = odom_.pose.pose.position.x;
    robot_pos_.y = odom_.pose.pose.position.y;
    robot_pos_.z = odom_.pose.pose.position.z;
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
    laser_cloud_ = laser_cloud_temp;
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
    std::fill(laser_cloud_image_->points.begin(), laser_cloud_image_->points.end(), nanPoint);
    float verticalAngle, horizonAngle, range;
    std::size_t rowIdn, columnIdn, index, cloudSize; 
    PointType thisPoint;

    cloudSize = laser_cloud_->points.size();

    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = laser_cloud_->points[i].x - robot_pos_.x;
        thisPoint.y = laser_cloud_->points[i].y - robot_pos_.y;
        thisPoint.z = laser_cloud_->points[i].z - robot_pos_.z;

        if (thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z < 4) {
            continue;
        }

        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + ang_bottom) / ANG_RES_Y;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        columnIdn = -round((horizonAngle-90.0)/ANG_RES_X) + HORIZON_SCAN/2;
        if (columnIdn >= HORIZON_SCAN)
            columnIdn -= HORIZON_SCAN;

        if (columnIdn < 0 || columnIdn >= HORIZON_SCAN)
            continue;

        // thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
        thisPoint.intensity = (float)rowIdn;
        index = columnIdn  + rowIdn * HORIZON_SCAN;
        laser_cloud_image_->points[index] = thisPoint;
    }
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








