/*
 * @Description: voxel filter 模块实现
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:53:20
 */
#include "lidar_localization/models/cloud_filter/voxel_filter.hpp"

#include "glog/logging.h"

namespace lidar_localization {

// 构造函数初始化
VoxelFilter::VoxelFilter(const YAML::Node& node) {
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();
    
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

//设置滤波参数
bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    LOG(INFO) << "Voxel Filter 的参数为：" << std::endl
              << leaf_size_x << ", "
              << leaf_size_y << ", "
              << leaf_size_z 
              << std::endl << std::endl;

    return true;
}
//体素格滤波器可以达到向下采样同时不破坏点云本身几何结构的功能
bool VoxelFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
    //pcl输入点云
    voxel_filter_.setInputCloud(input_cloud_ptr);
    //pcl体素滤波
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}
} 