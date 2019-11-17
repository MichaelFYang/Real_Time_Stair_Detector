#include "negative_obs_detection/negative_obs_detection.h"

/* --------------------------------------------------------------- */

NegObsDetect::NegObsDetect() {
/* TODO! */
     // Initai ROS params
    if (!nh_.getParam("neg_obs/laser_topic_sub",laser_topic_sub_)) {
        laser_topic_sub_ = "/velodyne_cloud_registered";
    }
    if (!nh_.getParam("neg_obs/odom_topic_sub",odom_topic_sub_)) {
        odom_topic_sub_ = "/integrated_to_map";
    }
    if (!nh_.getParam("neg_obs/cloud_image_topic_pub",cloud_image_topic_pub_)) {
       cloud_image_topic_pub_ = "/neg_detect/cloud_image";
    }
    if (!nh_.getParam("ground_topic_pub",ground_topic_pub_)) {
        ground_topic_pub_ = "/neg_detect/ground_cloud";
    } 
    if (!nh_.getParam("neg_obs_topic_pub", neg_obs_topic_pub_)) {
        neg_obs_topic_pub_ = "/neg_detect/neg_obstacle";
    }
    if (!nh_.getParam("slope_thresh", slope_thresh_)) {
    slope_thresh_ = 5.0;
    }
    
    this->Initialization();
}

void NegObsDetect::Loop() {
/* TODO! */
    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_pub_,1);
    negative_object_border_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(neg_obs_topic_pub_,1);
    cloud_image_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_image_topic_pub_,1);
    point_cloud_sub_ = nh_.subscribe(laser_topic_sub_,1,&NegObsDetect::CloudHandler,this);
    odom_sub_ = nh_.subscribe(odom_topic_sub_,1,&NegObsDetect::OdomHandler,this);

    ros::Rate rate(1);
    while(ros::ok())
    {
        ros::spinOnce(); // process all callback function
        std::cout<<"Debug Here: 1"<<std::endl;
        if (!laser_cloud_->empty()) {
        //process
            this->CloudImageProjection();
            this->GroundSegmentation();
            std::cout<<"Debug Here: 3"<<std::endl;
            this->TopicHandle();
        }else std::cout<<"The Point Cloud is Empty now!"<<std::endl;
        rate.sleep();
    }
}

void NegObsDetect::TopicHandle() {
/* cloud type transpose and msg publish*/
    pcl::toROSMsg(*ground_cloud_, ground_ros_cloud_);
    pcl::toROSMsg(*neg_obs_cloud_, neg_obs_ros_cloud_);
    // std::cout<<"Frame Id: "<<cloud_msg_->header.frame_id<<std::endl;
    ground_ros_cloud_.header = cloud_msg_->header;
    neg_obs_ros_cloud_.header = cloud_msg_->header;
    point_cloud_pub_.publish(ground_ros_cloud_);
    negative_object_border_pub_.publish(neg_obs_ros_cloud_);
}

void NegObsDetect::Initialization() {
/* TODO!*/ 
    // Allocate Memory for PointClouds
    laser_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_image_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_image_world_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    ground_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    neg_obs_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    
    nanPoint_.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.intensity = -1;
    std::cout<<"Initialize Successful"<<std::endl;

}

void NegObsDetect::CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg) {
/* Callback function of raw point cloud */
    laser_cloud_->clear();
    laser_cloud_image_->clear();
    // std::cout<<"Debug Here: 2"<<std::endl;
    cloud_msg_ = cloud_msg;
    pcl::fromROSMsg(*cloud_msg, *laser_cloud_);
    this->TransCloudFrame();
    laser_cloud_image_->points.resize(N_SCAN*HORIZON_SCAN);
    laser_cloud_image_world_->points.resize(N_SCAN*HORIZON_SCAN);
}

void NegObsDetect::OdomHandler(const nav_msgs::Odometry odom_msg) {
/* Odom Callback function */
    odom_ = odom_msg;
    odom_frame_id_ = odom_msg.header.frame_id;
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

void NegObsDetect::RightRotatePointToWorld(pcl::PointXYZI &pnt) {
/* Credit:https://bitbucket.org/cmusubt/misc_utils/src/master/src/misc_utils.cpp */
    float tmp_x = pnt.x + robot_pos_.x;
    pnt.x = pnt.y + robot_pos_.y;
    pnt.y = pnt.z + robot_pos_.z;
    pnt.z = tmp_x;
}

void NegObsDetect::CloudImageProjection() {
/* TODO -> Make a Project PointCloud into a image [row, col] for BSF search*/
    std::fill(laser_cloud_image_->points.begin(), laser_cloud_image_->points.end(), nanPoint_);
    std::fill(laser_cloud_image_world_->points.begin(), laser_cloud_image_world_->points.end(), nanPoint_);
    float verticalAngle, horizonAngle, range;
    std::size_t rowIdn, columnIdn, index, cloudSize; 
    PointType thisPoint;

    cloudSize = laser_cloud_->points.size();
    PointType temp_point;

    for (size_t i = 0; i < cloudSize; ++i){

        thisPoint.x = laser_cloud_->points[i].x - robot_pos_.x;
        thisPoint.y = laser_cloud_->points[i].y - robot_pos_.y;
        thisPoint.z = laser_cloud_->points[i].z - robot_pos_.z;

        if (thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z < 4) {
            continue;
        }

        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle +ANG_BOTTOM) / ANG_RES_Y;
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
        temp_point = thisPoint;
        this->RightRotatePointToWorld(temp_point);
        laser_cloud_image_world_->points[index] = temp_point;
    }
    pcl::toROSMsg(*laser_cloud_image_world_, cloud_image_ros_cloud_);
    cloud_image_ros_cloud_.header = cloud_msg_->header;
    cloud_image_pub_.publish(cloud_image_ros_cloud_);
}

void NegObsDetect::GroundSegmentation() {
/* Segment Ground PointCloud -> Operation on laser_cloud_image */
    ground_cloud_->clear();
    neg_obs_cloud_->clear();
    std::size_t id_cur, id_check;
    float diff_x, diff_y, diff_z;
    double angle;
    std::size_t laser_cloud_size = laser_cloud_image_->points.size();
    std::unordered_set<std::size_t> visited_set;
    std::queue<std::size_t> ground_queue;

    int cur_row, cur_col, check_row, check_col;
    int dx[8] = {-1,-1,-1,0,0,1,1,1};
    int dy[8] = {-1,0,1,-1,1,-1,0,1};
    /* Start Point*/
    cur_row = 0;
    cur_col = int(HORIZON_SCAN/2);
    /* Start Point*/
    visited_set.clear();
    id_cur = cur_col + cur_row * HORIZON_SCAN;
    ground_queue.push(id_cur);
    if (laser_cloud_image_->points[id_cur].x == std::numeric_limits<float>::quiet_NaN()) {
        std::cout<<"Initial groud point is invaild!"<<std::endl;
        return;
    }
    ground_cloud_->points.push_back(laser_cloud_image_->points[id_cur]);
    while (!ground_queue.empty()) {
        std::cout<<"current id: "<<id_cur<<std::endl;
        id_cur = ground_queue.back();
        std::cout<<"Size of ground_queue: "<<ground_queue.size()<<std::endl;
        visited_set.insert(id_cur);
        ground_queue.pop();
        cur_col = id_cur % HORIZON_SCAN;
        cur_row = (int)(id_cur/HORIZON_SCAN);
        for (int i=0; i<8; i++) {
            // calculate check id, check row id , check col id;
            check_row = cur_row + dx[i];
            check_col = cur_col + dy[i];
            if (check_row < 1 || check_row >= int(N_SCAN/2)) continue;
            if (check_col < 0) check_col = HORIZON_SCAN - 1;
            if (check_col >= HORIZON_SCAN) check_col = 0;
            id_check = check_col + check_row * HORIZON_SCAN;
            // Terrain Angle Calculation 
            if (visited_set.count(id_check)) continue;
            // visited_set.insert(id_check);
            diff_x = laser_cloud_image_->points[id_check].x - laser_cloud_image_->points[id_cur].x;
            diff_x = laser_cloud_image_->points[id_check].x - laser_cloud_image_->points[id_cur].x;
            diff_x = laser_cloud_image_->points[id_check].x - laser_cloud_image_->points[id_cur].x;
            angle = atan2(diff_z, sqrt(diff_x*diff_x + diff_y*diff_y) ) * 180 / M_PI;
            std::cout<<"Adjecent angle: "<<angle<<std::endl;
            if (abs(angle) <= slope_thresh_) {
                PointType temp_point;
                temp_point = laser_cloud_image_->points[id_check];
                this->RightRotatePointToWorld(temp_point);
                ground_cloud_->points.push_back(temp_point);
                ground_queue.push(id_check);
            }else if(angle < 0) { // negative obstacle
                PointType temp_point;
                temp_point = laser_cloud_image_->points[id_check];
                this->RightRotatePointToWorld(temp_point);
                neg_obs_cloud_->points.push_back(temp_point);
            }
        }
    }
}








