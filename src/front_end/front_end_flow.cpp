/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/front_end/front_end_flow.hpp"

#include "glog/logging.h"

#include "lidar_localization/tools/file_manager.hpp"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    //5个订阅
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "imu_link", "velo_link");
    //五个发布
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "/map");
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", 100, "/map");
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", 100, "/map");
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 100);

    //前端这里进入FrontEnd()-front_end.hpp 进行匹配算法 registration配置，初始化
    front_end_ptr_ = std::make_shared<FrontEnd>();
    //指针重置
    local_map_ptr_.reset(new CloudData::CLOUD());
    global_map_ptr_.reset(new CloudData::CLOUD());
    current_scan_ptr_.reset(new CloudData::CLOUD());
}

bool FrontEndFlow::Run() {
    // 读数据 进行线性插值 
    if (!ReadData())
        return false;
    // 获得lidar_to_imu 外参 以四元素表示
    if (!InitCalibration()) 
        return false;
    //初始化gps
    if (!InitGNSS())
        return false;
    //检测是否全部都有数据
    while(HasData()) {
        //判断数据是否在一定相隔时间内
        if (!ValidData())
            continue;
        // GNSS的定位
        UpdateGNSSOdometry();
        // 更新激光里程计 进入updata 调用registration的scanmatch 和 VoxelFilter 的filter滤波
        if (UpdateLaserOdometry()) {
            // 算完里程计发布结果    
            PublishData();
            SaveTrajectory();
        }
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    // 所有接收的数据保存在 下列容器中 
    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;
    // 放置数据 将信新的数据放置 到unsynced_imu_ 、unsynced_velocity_ 、unsynced_gnss_ 中
    imu_sub_ptr_->ParseData(unsynced_imu_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if (cloud_data_buff_.size() == 0)
        return false;
    // 获得点云的时间
    double cloud_time = cloud_data_buff_.front().time;
    // 时间戳对齐的变量放置到 imu_data_buff_ 判断时间相隔是否在适合范围  进行线性插值 完成后返回true
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);
    // 如果没有初始化 若要有一个传感器不同步  则return false  并且删除 点云数据
    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_velocity || !valid_gnss) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    return true;
}

// 获取外参数
bool FrontEndFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        //通过 lidar_to_imu_ 一个一个旋转轴—-》算出求出欧拉角--》四元素矩阵
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

//初始化gps
bool FrontEndFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        //重置 gps 
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool FrontEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (velocity_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;
    
    return true;
}
//有效数据，保证数据同步
bool FrontEndFlow::ValidData() {
    // 读取头数据
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();
    // 计算时间差
    double d_time = current_cloud_data_.time - current_imu_data_.time;
    //如果时间低于-0.05 清除点云缓冲数据 重新计算获取
    if (d_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }
    //如果时间低大于0.05 清除IMU vel gps缓冲数据  重新计算获取
    if (d_time > 0.05) {
        imu_data_buff_.pop_front();
        velocity_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
        return false;
    }
    //如果时间低大于-0.05 和小于0.05清除所有缓冲数据  重新计算获取  计算雷达里程计
    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

//利用gnss和imu 初始化里程计
bool FrontEndFlow::UpdateGNSSOdometry() {
    gnss_odometry_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0,3) = current_gnss_data_.local_E;
    gnss_odometry_(1,3) = current_gnss_data_.local_N;
    gnss_odometry_(2,3) = current_gnss_data_.local_U;
    //四元素转化为矩阵    current_imu_data_为imu_data_buff_.front() 缓冲数据
    gnss_odometry_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();

    //求变换矩阵
    gnss_odometry_ *= lidar_to_imu_; 

    return true;
}

// 返回是否匹配成功      计算结果保存在 laser_odometry_  
bool FrontEndFlow::UpdateLaserOdometry() {
    static bool front_end_pose_inited = false;
    // 初始化  
    if (!front_end_pose_inited) {
        front_end_pose_inited = true;
        front_end_ptr_->SetInitPose(gnss_odometry_);
        //进入ndt的扫描入口
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }
    // 正常运行，连续运行laser_odometry_ 啥意思？？？？？
    laser_odometry_ = Eigen::Matrix4f::Identity();    // 单位阵
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {
    //gnss
    gnss_pub_ptr_->Publish(gnss_odometry_);
    laser_odom_pub_ptr_->Publish(laser_odometry_);
    //点云
    front_end_ptr_->GetCurrentScan(current_scan_ptr_);
    cloud_pub_ptr_->Publish(current_scan_ptr_);
    //局部地图
    if (front_end_ptr_->GetNewLocalMap(local_map_ptr_))
        local_map_pub_ptr_->Publish(local_map_ptr_);

    return true;
}

bool FrontEndFlow::SaveTrajectory() {
    static std::ofstream ground_truth, laser_odom;
    static bool is_file_created = false;
    if (!is_file_created) {
        if (!FileManager::CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
            return false;
        if (!FileManager::CreateFile(ground_truth, WORK_SPACE_PATH + "/slam_data/trajectory/ground_truth.txt"))
            return false;
        if (!FileManager::CreateFile(laser_odom, WORK_SPACE_PATH + "/slam_data/trajectory/laser_odom.txt"))
            return false;
        is_file_created = true;
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            ground_truth << gnss_odometry_(i, j);
            laser_odom << laser_odometry_(i, j);
            if (i == 2 && j == 3) {
                ground_truth << std::endl;
                laser_odom << std::endl;
            } else {
                ground_truth << " ";
                laser_odom << " ";
            }
        }
    }

    return true;
}

bool FrontEndFlow::SaveMap() {
    return front_end_ptr_->SaveMap();
}
//发布地图
bool FrontEndFlow::PublishGlobalMap() {
    if (front_end_ptr_->GetNewGlobalMap(global_map_ptr_)) { 
        //发布全局地图并重置
        global_map_pub_ptr_->Publish(global_map_ptr_);
        global_map_ptr_.reset(new CloudData::CLOUD());
    }
    return true;
}
}