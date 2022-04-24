/*
 * @Description: 前端里程计算法
 * @Author: Ren Qian
 * @Date: 2020-02-04 18:53:06
 */
#include "lidar_localization/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
    //1、三个指针：局部地图、全局地图以及点云结果 2、参数初始化主函数
FrontEnd::FrontEnd()
    :local_map_ptr_(new CloudData::CLOUD()),
     global_map_ptr_(new CloudData::CLOUD()),
     result_cloud_ptr_(new CloudData::CLOUD()) {
    
    InitWithConfig();
}

//首先产生YAML::Node类型的文件路径节点，然后分为三个步骤：分别初始化路径、匹配、滤波。
bool FrontEnd::InitWithConfig() {
    // 1.产生文件路径节点config_node
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
   
    // 2.初始化路径
    InitDataPath(config_node);
    // 3.初始化匹配
    InitRegistration(registration_ptr_, config_node);
    // 4.三个初始化滤波
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    InitFilter("display", display_filter_ptr_, config_node);

    return true;
}

//读取参数表config.yaml

//局部地图初始化参数
bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

bool FrontEnd::InitDataPath(const YAML::Node& config_node) {
    // 1.获得data_path_转为string类型
    data_path_ = config_node["data_path"].as<std::string>();
    if (data_path_ == "./") {
        data_path_ = WORK_SPACE_PATH;
    }
    data_path_ += "/slam_data";
     // 2.推断data_path_是否为文件夹
    if (boost::filesystem::is_directory(data_path_)) {
        boost::filesystem::remove_all(data_path_);
    }
     // 3.创建文件夹，日志记录地图点云存放地址
    boost::filesystem::create_directory(data_path_);
    //如果创建不成功
    if (!boost::filesystem::is_directory(data_path_)) {
        LOG(WARNING) << "文件夹 " << data_path_ << " 未创建成功!";
        return false;
    } else {
        // 日志记录器LOG
        LOG(INFO) << "地图点云存放地址：" << data_path_;
    }
   // 4.创建文件夹，日志记录关键帧点云存放地址
    std::string key_frame_path = data_path_ + "/key_frames";
    boost::filesystem::create_directory(data_path_ + "/key_frames");
    //如果创建不成功
    if (!boost::filesystem::is_directory(key_frame_path)) {
        LOG(WARNING) << "文件夹 " << key_frame_path << " 未创建成功!";
        return false;
    } else {
        LOG(INFO) << "关键帧点云存放地址：" << key_frame_path << std::endl << std::endl;
    }

    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    // 点云匹配方式NDT
    LOG(INFO) << "点云匹配方式为：" << registration_method;
    //后期修改为icp方式验证在这里
    if (registration_method == "ICP") {
        //返回指针ICP匹配方法
        registration_ptr = std::make_shared<ICPRegistration>(config_node[registration_method]);
    } 
    else if (registration_method == "NDT") {
        //返回指针NDT匹配方法
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    }
    else
    {
        LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

//验证体素滤波方法
bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << filter_user << "选择的滤波方法为：" << filter_mothod;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

//位姿里程计更新
bool FrontEnd::Update(const CloudData& cloud_data, Eigen::Matrix4f& cloud_pose) {
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> indices;
    // 滤除NaN点   
    //pcl::removeNaNFromPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, std::vector<int> &index)
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);
    // 体素滤波  降采样   
    CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.cloud_ptr, filtered_cloud_ptr);
    //创建定位变量
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;  //Eigen::Matrix4f::Identity();单位矩阵
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;
        //// 更新匹配地图 
        UpdateWithNewFrame(current_frame_);
        //把当前点云位姿赋值给上一时刻 点云位姿
        cloud_pose = current_frame_.pose;
        return true;
    }

    // 不是第一帧，就正常匹配
    // 匹配结果放置在 current_frame_
    //CLOUD_PTR& input_source, Matrix4f& predict_pose,  CLOUD_PTR& result_cloud_ptr, Matrix4f& result_pose) {
    //进入调用registratin 方式  ICP 或者 NDT
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr_, current_frame_.pose);
    cloud_pose = current_frame_.pose;

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;     // 增量 变换矩阵  
    predict_pose = current_frame_.pose * step_pose;            // 更新预测量     匀速运动假设    
    last_pose = current_frame_.pose;                           // 保存当前

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        UpdateWithNewFrame(current_frame_);    // 如果是新的关键帧则用来更新  匹配地图
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

// 更新匹配地图  
bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    // 把关键帧点云存储到硬盘里，节省内存
    std::string file_path = data_path_ + "/key_frames/key_frame_" + std::to_string(global_map_frames_.size()) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr);

    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
    
    // 更新局部地图
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        //关键帧数量数量大于10 进行滤波 并存储结果
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    // 保存所有关键帧信息在容器里
    // 存储之前，点云要先释放，因为已经存到了硬盘里，不释放也达不到节省内存的目的
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD());
    global_map_frames_.push_back(key_frame);

    return true;
}

bool FrontEnd::SaveMap() {
    //重置全局地图指针
    global_map_ptr_.reset(new CloudData::CLOUD());
    //定义关键帧
    std::string key_frame_path = "";
    //调用sensor_data.cpp 的cloud_data.hpp 的类 创建关键帧和 转换点云指针
    CloudData::CLOUD_PTR key_frame_cloud_ptr(new CloudData::CLOUD());
    CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());

    for (size_t i = 0; i < global_map_frames_.size(); ++i) {
        key_frame_path = data_path_ + "/key_frames/key_frame_" + std::to_string(i) + ".pcd";
        //调用pcl 读取pcd文件和点云 
        pcl::io::loadPCDFile(key_frame_path, *key_frame_cloud_ptr);
        //点云刚体变换
        pcl::transformPointCloud(*key_frame_cloud_ptr, 
                                *transformed_cloud_ptr, 
                                global_map_frames_.at(i).pose);
        //全局地图添加
        *global_map_ptr_ += *transformed_cloud_ptr;
    }
    
    //定义地图文件路径
    std::string map_file_path = data_path_ + "/map.pcd";

    //将点云写进文件
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr_);
    //新地图完成
    has_new_global_map_ = true;

    return true;
}

bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR& local_map_ptr) {
    if (has_new_local_map_) {
        display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
        return true;
    }
    return false;
}

bool FrontEnd::GetNewGlobalMap(CloudData::CLOUD_PTR& global_map_ptr) {
    if (has_new_global_map_) {
        has_new_global_map_ = false;
        //调用pcl体素滤波
        display_filter_ptr_->Filter(global_map_ptr_, global_map_ptr);
        global_map_ptr_.reset(new CloudData::CLOUD());
        return true;
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR& current_scan_ptr) {
    display_filter_ptr_->Filter(result_cloud_ptr_, current_scan_ptr);
    return true;
}
}