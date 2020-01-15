#include "negative_obs_detection/negative_obs_detection.h"

/* --------------------------------------------------------------- */

NegObsDetect::NegObsDetect() {
/* TODO! */
     // Initai ROS params
    if (!nh_.getParam("/neg_obs_detection/correlation_thred",correlation_thred_)) {
        correlation_thred_ = 0.85;
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
    if (!nh_.getParam("/neg_obs_detection/is_stair_topic_pub", is_stair_topic_pub_)) {
        is_stair_topic_pub_ = "/neg_detect/is_stair_topic_pub";
    }
    if (!nh_.getParam("/neg_obs_detection/stair_poses_topic_pub", stair_poses_topic_pub_)) {
        stair_poses_topic_pub_ = "/neg_detect/stair_poses_topic_pub";
    }
    if (!nh_.getParam("/neg_obs_detection/kernel_server_topic",kernel_server_topic_)) {
        kernel_server_topic_ = "/neg_detect/kernel_serve_topic";
    }
    if (!nh_.getParam("/neg_obs_detection/kernel_filename",kernel_filename_)) {
        kernel_filename_ = "/stair_kernel.txt";
    }
    if (!nh_.getParam("/neg_obs_detection/slope_thresh", slope_thresh_)) {
        slope_thresh_ = 50.0;
    }
    if (!nh_.getParam("/neg_obs_detection/kMeans_iters", kMeans_iters_)) {
        kMeans_iters_ = 10;
    }
    if (!nh_.getParam("/neg_obs_detection/flat_thresh", flat_thresh_)) {
        flat_thresh_ = 10.0;
    }
    if (!nh_.getParam("/neg_obs_detection/col_filter_size", col_filter_size_)) {
        col_filter_size_ = 25;
    }
    if (!nh_.getParam("/neg_obs_detection/frame_filter_size", frame_filter_size_)) {
        frame_filter_size_ = 20;
    }
    if (!nh_.getParam("/neg_obs_detection/cluster_filter_size", cluster_filter_size_)) {
        cluster_filter_size_ = 10;
    }
    if (!nh_.getParam("/neg_obs_detection/cluster_radius", cluster_radius_)) {
        cluster_radius_ = 2.0;
    }
    this->Initialization();
}

void NegObsDetect::Loop() {
/* TODO! */
    ground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_pub_,1);
    stair_center_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(stair_topic_pub_,1);
    cloud_image_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_image_topic_pub_,1);
    is_stair_pub_ = nh_.advertise<std_msgs::Bool>(is_stair_topic_pub_,1);
    stair_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>(stair_poses_topic_pub_,1);
    point_cloud_sub_ = nh_.subscribe(laser_topic_sub_,1,&NegObsDetect::CloudHandler,this);
    odom_sub_ = nh_.subscribe(odom_topic_sub_,1,&NegObsDetect::OdomHandler,this);
    kenerl_service_ = nh_.advertiseService(kernel_server_topic_,&NegObsDetect::KernelGeneration,this);
    ros::Rate rate(5);
    this->ReadKernelFile();
    if (!is_kernel_) return;
    while(ros::ok())
    {
        ros::spinOnce(); // process all callback function
        // std::cout<<"Debug Here: 1"<<std::endl;
        if (!laser_cloud_->empty()) {
        //process
            is_stair_ = false;
            this->CloudImageProjection();
            this->GroundSegmentation();
            // std::cout<<"Debug Here: 3"<<std::endl;
            this->SimularityCalculation();
            if(is_inited_) this->TopicHandle();
        }else std::cout<<"The Point Cloud is Empty now!"<<std::endl;
        rate.sleep();
    }
}

void NegObsDetect::TopicHandle() {
/* cloud type transpose and msg publish*/
    pcl::toROSMsg(*ground_cloud_, ground_ros_cloud_);
    pcl::toROSMsg(*filtered_stair_cloud_, stair_center_ros_cloud_);
    // std::cout<<"Frame Id: "<<cloud_msg_->header.frame_id<<std::endl;
    std_msgs::Bool is_stair_ros;
    is_stair_ros.data = is_stair_;
    ground_ros_cloud_.header = cloud_msg_->header;
    stair_center_pose_array_.header = cloud_msg_->header;
    stair_center_ros_cloud_.header = cloud_msg_->header;
    ground_cloud_pub_.publish(ground_ros_cloud_);
    stair_center_pub_.publish(stair_center_ros_cloud_);
    is_stair_pub_.publish(is_stair_ros);
    stair_poses_pub_.publish(stair_center_pose_array_);
}

void NegObsDetect::Initialization() {
/* TODO!*/ 
    // Allocate Memory for PointClouds
    laser_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_image_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    laser_cloud_image_world_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    ground_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    stair_center_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    filtered_stair_cloud_ = PointCloudPtr(new pcl::PointCloud<pcl::PointXYZI>());
    kdtree_stair_cloud_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    
    nanPoint_.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint_.intensity = -1;
    elem_matrix_.resize(HORIZON_SCAN);
    for(std::size_t i=0; i<HORIZON_SCAN; i++) {
        elem_matrix_[i].resize(int(N_SCAN/2));
    }
    elem_score_.resize(HORIZON_SCAN);
    is_kernel_ = false;
    is_inited_ = false;
    frame_elem_score_.clear();
    std::cout<<"Initialize Successful"<<std::endl;
    is_stair_ = false;
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

void NegObsDetect::KMeansCluster(PointCloudPtr filtered_stair_cloud) {
/* Generate Cluster Center -- Max cluster -> 3*/
    PointType c0, c1, c2, c_max;
    geometry_msgs::Pose tmp_pose;
    std::size_t cloudSize = filtered_stair_cloud->points.size();
    // initialize
    c0 = filtered_stair_cloud->points[0];
    c1 = filtered_stair_cloud->points[1];
    c2 = filtered_stair_cloud->points[2];
    int iter = 0;
    while (iter < kMeans_iters_) {
        CenterCloud pc0, pc1, pc2;
        pc0.add(c0);
        pc1.add(c1);
        pc2.add(c2);
        for (std::size_t i=0; i<cloudSize; i++) {
            PointType check_point = filtered_stair_cloud->points[i];
            this->UpdateClusterCloud(check_point,c0,c1,c2,pc0,pc1,pc2); // add the check_point to the centercloud with min dist.
        }
        // update center
        pc0.get(c0);
        pc1.get(c1);
        pc2.get(c2);
        c_max = this->UpdateCenterMax(pc0,pc1,pc2);
        iter ++;
    }
    tmp_pose.position.x = c_max.x;
    tmp_pose.position.y = c_max.y;
    tmp_pose.position.z = c_max.z;
    stair_center_pose_array_.poses.push_back(tmp_pose);
}

void NegObsDetect::UpdateClusterCloud(const PointType &check_point, const PointType &c0, const PointType &c1, const PointType &c2, CenterCloud &pc0, CenterCloud &pc1, CenterCloud &pc2) {
    /* TODO! Add the checkpoint to the nearest centeriod point cloud */
    float d0,d1,d2;
    float min_dist = 0;
    d0 = (check_point.x - c0.x) * (check_point.x - c0.x) + (check_point.y - c0.y) * (check_point.y - c0.y) + (check_point.z - c0.z) * (check_point.z - c0.z);
    d1 = (check_point.x - c1.x) * (check_point.x - c1.x) + (check_point.y - c1.y) * (check_point.y - c1.y) + (check_point.z - c1.z) * (check_point.z - c1.z);
    d2 = (check_point.x - c2.x) * (check_point.x - c2.x) + (check_point.y - c2.y) * (check_point.y - c2.y) + (check_point.z - c2.z) * (check_point.z - c2.z);
    min_dist = d0;
    if (min_dist > d1) min_dist = d1;
    if (min_dist > d2) min_dist = d2;
    if (min_dist == d0) pc0.add(check_point);
    if (min_dist == d1) pc1.add(check_point);
    if (min_dist == d2) pc2.add(check_point);
}

PointType NegObsDetect::UpdateCenterMax(const CenterCloud &pc0, const CenterCloud &pc1, const CenterCloud &pc2) {
    /* TODO! */
    float n0,n1,n2;
    PointType c_max;
    n0 = pc0.getSize();
    n1 = pc1.getSize();
    n2 = pc2.getSize();
    float max_num = n0;
    if (n1 > max_num) max_num = n1;
    if (n2 > max_num) max_num = n2;
    if (max_num == n0) pc0.get(c_max);
    if (max_num == n1) pc1.get(c_max);
    if (max_num == n2) pc2.get(c_max);
    return c_max; 
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

        columnIdn = -round((horizonAngle-90.0)/ANG_RES_X) + HORIZON_SCAN/2;
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

std::vector<Point3D> NegObsDetect::NormColElem(const std::vector<Point3D> &elem_col, bool is_process) {
    std::vector<Point3D> new_elem_col;
    float sum_x = 0;
    float sum_z = 0;
    float num_row = elem_col.size();
    float counter_x = 0;
    float counter_z = 0;
    float decay_factore_x, decay_factore_z;
    new_elem_col = elem_col;
    for (std::size_t i=0; i<elem_col.size(); i++) {
        if(is_process) {
            if (isnan(elem_col[i].x)) {
                new_elem_col[i].x = 0.0;
                counter_x ++;
            }else {
                new_elem_col[i].x = this->ReLu(elem_col[i].x);
            }
            if (isnan(elem_col[i].z)) {
                new_elem_col[i].z = 0.0;
                counter_z ++;
            }else {
                new_elem_col[i].z = this->ReLu(elem_col[i].z);
            }
        }
        sum_x += elem_col[i].x*elem_col[i].x; 
        sum_z += elem_col[i].z*elem_col[i].z;  
    }
    if (sum_x != 0 ) {
        decay_factore_x = (num_row - counter_x) / num_row;
        for (std::size_t i=0; i<elem_col.size(); i++) {     
            new_elem_col[i].x = new_elem_col[i].x/sqrt(sum_x) *decay_factore_x;
        }
    }
    if (sum_z != 0 ) {
        decay_factore_z = (num_row - counter_z) / num_row;
        for (std::size_t i=0; i<elem_col.size(); i++) {
            new_elem_col[i].z = new_elem_col[i].z/sqrt(sum_z)*decay_factore_z;
        }
    }
    return new_elem_col;
}

void NegObsDetect::SimularityCalculation() {
    stair_center_cloud_->clear();
    stair_center_pose_array_.poses.clear();
    PointType temp_point;
    int num_windows = int(N_SCAN/2) - KERNEL_SIZE + 1;
    for (std::size_t i=0; i<HORIZON_SCAN; i++) {
        float max_z = 0.0;
        float max_dist = 0.0;
        for (int n=0; n<num_windows; n++) {
            float temp_score_z = 0.0;
            float temp_score_dist = 0.0;
            std::vector<Point3D> new_elem_col = std::vector<Point3D>(elem_matrix_[i].begin()+n, elem_matrix_[i].begin()+n+KERNEL_SIZE);
            new_elem_col = this->NormColElem(new_elem_col, false);
            for (std::size_t k=0; k<KERNEL_SIZE; k++) {
                temp_score_z += new_elem_col[k].z * kernel_elem_[k].z;
                temp_score_dist += new_elem_col[k].x * kernel_elem_[k].x;
            }
            if (temp_score_z > max_z) {
                max_z = temp_score_z;
            }
            if (temp_score_dist > max_dist) {
                max_dist = temp_score_dist;
            }
        }
        elem_score_[i].x = max_dist;
        elem_score_[i].z = max_z;
    }
    this->FilterColumn();
    this->FilterFrames();
    if (is_inited_) {
        for (std::size_t i=0; i<HORIZON_SCAN; i++) {
            if (elem_score_[i].z > correlation_thred_) {
                int id_check = i + int(N_SCAN/4)*HORIZON_SCAN;
                temp_point = laser_cloud_image_->points[id_check];
                if (!isnan(temp_point.x)) {
                    this->RightRotatePointToWorld(temp_point);
                    stair_center_cloud_->push_back(temp_point);
                }
            }
        }
        std::cout<<"Center Simularity Score X: "<<elem_score_[int(HORIZON_SCAN/2)].x <<" -- Center Simularity Score Z: "<<elem_score_[int(HORIZON_SCAN/2)].z<<std::endl;
    }
    filtered_stair_cloud_->clear();
    if (stair_center_cloud_->points.size() > 2*cluster_filter_size_) {
        this->ClusterFilter();
        this->KMeansCluster(filtered_stair_cloud_);
        is_stair_ = true;
    }
}

void NegObsDetect::ClusterFilter() {
    // Credit: Chao C,.
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;
    std::size_t cloudSize = stair_center_cloud_->points.size();
    kdtree_stair_cloud_->setInputCloud(stair_center_cloud_);
    PointType thisPoint;
    for (std::size_t i=0; i<cloudSize; i++) {
        thisPoint = stair_center_cloud_->points[i];
        kdtree_stair_cloud_->radiusSearch(thisPoint, cluster_radius_, pointSearchInd, pointSearchSqDis);
        if (pointSearchInd.size() > cluster_filter_size_) {
            filtered_stair_cloud_->push_back(thisPoint);
        } 
    }
}

void NegObsDetect::GroundSegmentation() {
/* Segment Ground PointCloud -> Operation on laser_cloud_image */
    ground_cloud_->clear();
    PointType temp_point;
    float angle_down, angle_up;
    std::size_t laser_cloud_size = laser_cloud_image_->points.size();
    /* Start Point*/
    for (std::size_t k=0; k<HORIZON_SCAN; k++) {
        for (std::size_t i=0; i<int(N_SCAN/2); i++) {
            this->NeighberAngleUpdate(k, i, angle_down, angle_up);
            int id_check = k + i*HORIZON_SCAN;
            temp_point = laser_cloud_image_->points[id_check];
            if (angle_down < slope_thresh_ && angle_up < slope_thresh_ && angle_down > flat_thresh_ && angle_up > flat_thresh_) {
                elem_matrix_[k][i].x = sqrt((temp_point.x-robot_pos_.x)*(temp_point.x-robot_pos_.x)+(temp_point.y-robot_pos_.y)*(temp_point.y-robot_pos_.y));
                elem_matrix_[k][i].y = 0; // DO NOT USE Y
                elem_matrix_[k][i].z = temp_point.z - robot_pos_.z + LIDAR_H;
            }else {
                elem_matrix_[k][i].x = 0;
                elem_matrix_[k][i].y = 0; // DO NOT USE Y
                elem_matrix_[k][i].z = 0;
            }
            
            if (k % 100 == 0) {
                this->RightRotatePointToWorld(temp_point);
                ground_cloud_->points.push_back(temp_point);
            } 
        }
        elem_matrix_[k] = this->NormColElem(elem_matrix_[k], true);
    }
}

void NegObsDetect::FilterColumn() {
    for (int i=0; i<HORIZON_SCAN; i++) {
        int begin_id = std::max(0,i-col_filter_size_);
        float sum_x = 0;
        float sum_z = 0;
        for (int k=0; k<2*col_filter_size_+1; k++) {
            int check_id = (begin_id + k) % HORIZON_SCAN;
            sum_x += elem_score_[i].x;
            sum_z += elem_score_[i].z;
        }
        sum_x = sum_x / (2*col_filter_size_+1);
        sum_z = sum_z / (2*col_filter_size_+1);
        elem_score_[i].x = sum_x;
        elem_score_[i].z = sum_z;
    }
}

void NegObsDetect::FilterFrames() {
    if (frame_elem_score_.size() >= frame_filter_size_) {
        frame_elem_score_.pop_front();
        is_inited_ = true;
    }
    frame_elem_score_.push_back(elem_score_);
    int num_frames = frame_elem_score_.size();
    for (int i=0; i<HORIZON_SCAN; i++) { // column loop
        float sum_x = 0;
        float sum_z = 0;
        int counter_x = 0;
        int counter_z = 0;
        for (int k=0; k<num_frames; k++) { //frame loop
            if (frame_elem_score_[k][i].x != 0) {
                sum_x += frame_elem_score_[k][i].x;
                counter_x ++;
            }
            if (frame_elem_score_[k][i].z != 0) {
                sum_z += frame_elem_score_[k][i].z;
                counter_z ++;
            }
        }
        if (counter_x != 0) {
            sum_x = sum_x / counter_x;
            elem_score_[i].x = sum_x;
        }
        if (counter_z != 0) {
            sum_z = sum_z / counter_z;
            elem_score_[i].z = sum_z;
        }
    }
}

void NegObsDetect::NeighberAngleUpdate(std::size_t col, std::size_t row, float& angle_down, float& angle_up) {
    PointType check_point, down_point, up_point;
    int id_check = col + row*HORIZON_SCAN;
    int id_down, id_up;
    float diff_x_down, diff_y_down, diff_z_down,diff_x_up, diff_y_up, diff_z_up;
    if (row == 0){
        id_down = col + row*HORIZON_SCAN;
        id_check = col + (row+1)*HORIZON_SCAN;
        id_up = col + (row+2)*HORIZON_SCAN;
    }else{
        id_down = col + (row-1)*HORIZON_SCAN;
        id_up = col + (row+1)*HORIZON_SCAN;
    }
    check_point = laser_cloud_image_->points[id_check];
    down_point = laser_cloud_image_->points[id_down];
    up_point = laser_cloud_image_->points[id_up];
    diff_x_down = check_point.x - down_point.x;
    diff_y_down = check_point.y - down_point.y;
    diff_z_down = check_point.z - down_point.z;
    diff_x_up = up_point.x - check_point.x;
    diff_y_up = up_point.y - check_point.y;
    diff_z_up = up_point.z - check_point.z;
    angle_down = fabs(atan2(diff_z_down, sqrt(diff_x_down*diff_x_down + diff_y_down*diff_y_down))*180/M_PI);
    angle_up = fabs(atan2(diff_z_up, sqrt(diff_x_up*diff_x_up + diff_y_up*diff_y_up))*180/M_PI); 
}

bool NegObsDetect::KernelGeneration(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res) {
    int center_col = int(HORIZON_SCAN/2);
    std::ofstream FILE(kernel_filename_);
    for (std::size_t i=0; i<elem_matrix_[center_col].size(); i++) {
        FILE << elem_matrix_[center_col][i].x<<" ";
        FILE << elem_matrix_[center_col][i].z;
        FILE << "\n";
    }
    FILE.close();
    return true;
}

void NegObsDetect::ReadKernelFile() {
    // file the kernel from kernel file
    kernel_filename_ = ros::package::getPath("neg_obs_detection") + "/data/" + kernel_filename_;
    std::fstream FILE(kernel_filename_);
    kernel_elem_.clear();
    std::string line;
    if (!FILE.good()) {
        std::cout<<"Cannot Find Kernel File in Path: "<<kernel_filename_<<std::endl;
        return;
    }
    while(std::getline(FILE, line)) {
        std::istringstream is(line);
        Point3D elem;
        is >> elem.x;
        is >> elem.z;
        kernel_elem_.push_back(elem);
    }
    is_kernel_ = true;
    FILE.close();
}

float NegObsDetect::ReLu(float x) {
    if (x>0) return x;
    else return 0;
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








