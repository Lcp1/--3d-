/*
 * @Description: voxel filter 模块
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:37:49
 */
#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>
#include "lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_localization {
class VoxelFilter: public CloudFilterInterface {
  public:
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);
    //后面的override表明当前派生类函数重写了基类的虚函数，防止代码中出现意外的继承行为？？？？？？？？？
    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    // 设置滤波参数 
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<CloudData::POINT> voxel_filter_;  //创建滤波对象
};
}
#endif