#include "negative_obs_detection/negative_obs_detection.h"

/* --------------------------------------------------------------- */

NegObsDetect::NegObsDetect() {
/* TODO! */
     // Initai ROS params
    if (!nh_.getParam("/neg_obs_detection/correlation_thred",correlation_thred_)) {
        correlation_thred_ = 0.135;
    }
    if (!nh_.getParam("/neg_obs_detection/laser_topic_sub",laser_topic_sub_)) {
        laser_topic_sub_ = "/velodyne_cloud_registered";
    }
    if (!nh_.getParam("/neg_obs_detection/odom_topic_sub",odom_topic_sub_)) {
        odom_topic_sub_ = "/integrated_to_map";
    }
    if (!nh_.getParam("/neg_obs_detection/cloud_image_topic_pub",cloud_image_topic_pub_)) {
       cloud_image_topic_pub_ = "/neg_detect/cloud_image";
    }
    if (!nh_.getParam("/neg_obs_detection/ground_topic_pub",ground_topic_pub_)) {
        ground_topic_pub_ = "/neg_detect/ground_cloud";
    } 
    if (!nh_.getParam("/neg_obs_detection/neg_obs_topic_pub", stair_topic_pub_)) {
        stair_topic_pub_ = "/neg_detect/stair_topic_pub";
    }
    if (!nh_.getParam("/neg_obs_detection/kernel_server_topic",kernel_server_topic_)) {
        kernel_server_topic_ = "/neg_detect/kernel_serve_topic";
    }
    if (!nh_.getParam("/neg_obs_detection/kernel_filename",kernel_filename_)) {
        kernel_filename_ = "stair_kernel.txt";
    }
    if (!nh_.getParam("/neg_obs_detection/slope_thresh", slope_thresh_)) {
        slope_thresh_ = 10.0;
    }
    
    this->Initialization();
}

void NegObsDetect::Loop() {
/* TODO! */
    ground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_pub_,1);
    stair_center_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(stair_topic_pub_,1);
    cloud_image_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_image_topic_pub_,1);
    point_cloud_sub_ = nh_.subscribe(laser_topic_sub_,1,&NegObsDetect::CloudHandler,this);
    odom_sub_ = nh_.subscribe(odom_topic_sub_,1,&NegObsDetect::OdomHandler,this);
    kenerl_service_ = nh_.advertiseService(kernel_server_topic_,&NegObsDetect::KernelGeneration,this);

    ros::Rate rate(1);
    this->ReadKernelFile();
    if (!is_kernel_) return;
    while(ros::ok())
    {
        ros::spinOnce(); // process all callback function
        // std::cout<<"Debug Here: 1"<<std::endl;
        if (!laser_cloud_->empty()) {
        //process
            this->CloudImageProjection();
            this->GroundSegmentation();
            // std::cout<<"Debug Here: 3"<<std::endl;
            this->SimularityCalculation();
            this->TopicHandle();
        }else std::cout<<"The Point Cloud is Empty now!"<<std::endl;
        rate.sleep();
    }
}

void NegObsDetect::TopicHandle() {
/* cloud type transpose and msg publish*/
    pcl::toROSMsg(*ground_cloud_, ground_ros_cloud_);
    pcl::toROSMsg(*stair_center_cloud_, stair_center_ros_cloud_);
    // std::cout<<"Frame Id: "<<cloud_msg_->header.frame_id<<std::endl;
    ground_ros_cloud_.header = cloud_msg_->header;
    stair_center_ros_cloud_.header = cloud_msg_->header;
    ground_cloud_pub_.publish(ground_ros_cloud_);
    stair_center_pub_.publish(stair_center_ros_cloud_);
}

void NegObsDetect::Initialization() {
/* TODO!*/ 
    // Allocate Memory for PointClouds
    laser_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_image_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_image_world_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    ground_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    stair_center_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    
    nanPoint_.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.intensity = -1;
    elem_martix_.resize(HORIZON_SCAN);
    for(std::size_t i=0; i<HORIZON_SCAN; i++) {
        elem_martix_[i].resize(int(N_SCAN/2));
    }
    elem_score_.resize(HORIZON_SCAN);
    is_kernel_ = false;
    std::cout<<"Initialize Successful"<<std::endl;

}

void NegObsDetect::CloudHandler(const sensor_msgs::PointCloud2ConstPtr cloud_msg) {
/* Callback function of raw point cloud */
    laser_cloud_->clear();
    laser_cloud_image_->clear();
    laser_cloud_image_world_->clear();
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

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geo_quat = odom_msg.pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);
    robot_heading_ = yaw * 180 / M_PI;

}

void NegObsDetect::TransCloudFrame() {
/* Source credit: http://pointclouds.org/documentation/tutorials/passthrough.php */
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_temp->clear();
    std::size_t laser_cloud_size = laser_cloud_->points.size();
    pcl::PointXYZI point;
    for (std::size_t i=0; i<laser_cloud_size; i++) {
        point = laser_cloud_->points[i];
        point.intensity = laser_cloud_->points[i].intensity;
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

void NegObsDetect::TransToWorld(pcl::PointXYZI &pnt) {
    pnt.x += robot_pos_.x;
    pnt.y += robot_pos_.y;
    pnt.z += robot_pos_.z;
}

void NegObsDetect::RightRotatePointToWorld(pcl::PointXYZI &pnt) {
/* Credit:https://bitbucket.org/cmusubt/misc_utils/src/master/src/misc_utils.cpp */
    float tmp_x = pnt.x;
    pnt.x = pnt.y;
    pnt.y = pnt.z;
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

        rowIdn = laser_cloud_->points[i].intensity;
        // std::cout<<"Intensity: "<<rowIdn<<std::endl;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        columnIdn = -round((horizonAngle-90.0+robot_heading_)/ANG_RES_X) + HORIZON_SCAN/2;
        if (columnIdn >= HORIZON_SCAN)
            columnIdn -= HORIZON_SCAN;

        if (columnIdn < 0 || columnIdn >= HORIZON_SCAN)
            continue;

        // thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;
        thisPoint.intensity = (float)rowIdn;
        index = columnIdn  + rowIdn * HORIZON_SCAN;
        this->TransToWorld(thisPoint);
        laser_cloud_image_->points[index] = thisPoint;
        temp_point = thisPoint;
        this->RightRotatePointToWorld(temp_point);
        laser_cloud_image_world_->points[index] = temp_point;
    }
    pcl::toROSMsg(*laser_cloud_image_world_, cloud_image_ros_cloud_);
    cloud_image_ros_cloud_.header = cloud_msg_->header;
    cloud_image_pub_.publish(cloud_image_ros_cloud_);
}

void NegObsDetect::NormColElem(std::vector<float> &elem_col) {
    float sum = 0;
    for (std::size_t i=0; i<elem_col.size(); i++) {
        if (isnan(elem_col[i])) {
            elem_col[i] = 0.0;
        }
        else elem_col[i] = this->Sigmoid(elem_col[i]);
        sum += elem_col[i]; 
    }
    for (std::size_t i=0; i<elem_col.size(); i++) {
        elem_col[i] = elem_col[i]/sum;
    }
}

void NegObsDetect::SimularityCalculation() {
    stair_center_cloud_->clear();
    PointType temp_point;
    for (std::size_t i=0; i<HORIZON_SCAN; i++) {
        float temp_score = 0.0;
        for (std::size_t k=0; k<int(N_SCAN/2); k++) {
            temp_score += elem_martix_[i][k] * kernel_elem_[k];
        }
        elem_score_[i] = temp_score;
        if (elem_score_[i] > correlation_thred_) {
            int id_check = i + int(N_SCAN/4)*HORIZON_SCAN;
            temp_point = laser_cloud_image_->points[id_check];
            this->RightRotatePointToWorld(temp_point);
            stair_center_cloud_->push_back(temp_point);
        }
    }
    std::cout<<"Center Simularity Score: "<<elem_score_[int(HORIZON_SCAN/2)]<<std::endl;
}

void NegObsDetect::GroundSegmentation() {
/* Segment Ground PointCloud -> Operation on laser_cloud_image */
    ground_cloud_->clear();
    std::size_t id_cur, id_check, cur_row, cur_col;
    double angle;
    PointType temp_point;
    std::size_t laser_cloud_size = laser_cloud_image_->points.size();
    /* Start Point*/
    cur_row = 0;
    cur_col = int(HORIZON_SCAN/2);
    /* Start Point*/
    id_cur = cur_col + cur_row * HORIZON_SCAN;
    // std::cout<<"Center Point X: "<<laser_cloud_image_->points[id_cur].x<<"; -- Y: "<<laser_cloud_image_->points[id_cur].y<<"; -- Z: "<<laser_cloud_image_->points[id_cur].z<<std::endl;
    for (std::size_t k=0; k<HORIZON_SCAN; k++) {
        for (std::size_t i=0; i<int(N_SCAN/2); i++) {
            int id_check = k + i*HORIZON_SCAN;
            temp_point = laser_cloud_image_->points[id_check];
            elem_martix_[k][i] = temp_point.z - robot_pos_.z;
            if (k == int(HORIZON_SCAN/2)) {
                this->RightRotatePointToWorld(temp_point);
                ground_cloud_->points.push_back(temp_point);
            } 
        }
        this->NormColElem(elem_martix_[k]);
    }
}

bool NegObsDetect::KernelGeneration(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res) {
    int center_col = int(HORIZON_SCAN/2);
    std::ofstream FILE(kernel_filename_);
    for (std::size_t i=0; i<elem_martix_[center_col].size(); i++) {
        FILE << elem_martix_[center_col][i];
        FILE << "\n";
    }
    FILE.close();
    return true;
}

void NegObsDetect::ReadKernelFile() {
    // file the kernel from kernel file
    std::fstream FILE(kernel_filename_);
    kernel_elem_.clear();
    std::string line;
    if (!FILE.good()) {
        std::cout<<"Cannot Find Kernel File in Path: "<<kernel_filename_<<std::endl;
        return;
    }
    while(std::getline(FILE, line)) {
        std::istringstream is(line);
        float elem;
        is >> elem;
        kernel_elem_.push_back(elem);
    }
    is_kernel_ = true;
    FILE.close();
}

float NegObsDetect::Sigmoid(float x)
//Credit Kiyoshi Kawaguchi 2000-06-17; http://www.ece.utep.edu/research/webfuzzy/docs/kk-thesis/kk-thesis-html/node72.html
{
    float exp_value;
    float return_value;

    /*** Exponential calculation ***/
    exp_value = exp((double) -x);

    /*** Final sigmoid value ***/
    return_value = 1 / (1 + exp_value);

    return return_value;
}








