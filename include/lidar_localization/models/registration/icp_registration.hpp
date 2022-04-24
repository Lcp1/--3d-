/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 */
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include <pcl/registration/icp.h>
#include "lidar_localization/models/registration/registration_interface.hpp"

namespace lidar_localization {
class ICPRegistration: public RegistrationInterface {
  public:
    ICPRegistration(const YAML::Node& node);
    ICPRegistration(float MCD, float TEps, float EFEps, int max_iter);
    //输入目标点云
    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    //扫描匹配
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
  //设置参数
    bool SetRegistrationParam(float MCD, float TEps, float EFEps, int max_iter);

  private:
    pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_;
};
}

#endif