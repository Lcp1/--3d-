/*
 * @Description: ICP 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 */
#include "lidar_localization/models/registration/icp_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization {
//最主要的类成员函数SetInputTarget()和ScanMatch()，
//另外的类成员函数设置滤波参数SetRegistrationParam()，
//最主要的类成员变量 icp_ptr_。
ICPRegistration::ICPRegistration(const YAML::Node& node)
    :icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {
    
   // float res = node["res"].as<float>();
   // float step_size = node["step_size"].as<float>();
   // float trans_eps = node["trans_eps"].as<float>();
   // int max_iter = node["max_iter"].as<int>();
      float MCD = node["MCD"].as<float>();
      float TEps = node["TEps"].as<float>();
      float EFEps = node["EFEps"].as<float>();
      int max_iter = node["max_iter"].as<int>();
    SetRegistrationParam(MCD, TEps, EFEps, max_iter);
}

ICPRegistration::ICPRegistration(float MCD, float TEps, float EFEps, int max_iter)
    :icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(MCD, TEps, EFEps, max_iter);
}

bool ICPRegistration::SetRegistrationParam(float MCD, float TEps, float EFEps, int max_iter) {
    //icp_ptr_->setResolution(res);//网格设置大小
   // icp_ptr_->setStepSize(step_size);//牛顿法优化的最大步长
    //判断是否收敛到设定阀值
    icp_ptr_->setMaxCorrespondenceDistance(100);//(MCD);
    icp_ptr_->setEuclideanFitnessEpsilon(1e-6);//(TEps);
    icp_ptr_->setTransformationEpsilon(1e-6);//(EFEps);//连续转换之间允许最大差值
    icp_ptr_->setMaximumIterations(max_iter);//最大迭代次数
    icp_ptr_->setRANSACIterations(0);

    LOG(INFO) << "ICP 的匹配参数为：" << std::endl
              << " MCD: " << MCD << ", "
              << "TEps: " << TEps << ", "
              << "EFEps: " << EFEps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    icp_ptr_->setInputTarget(input_target);

    return true;
}

bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    //读取输入点云
    icp_ptr_->setInputSource(input_source);
    //对齐 校准
    icp_ptr_->align(*result_cloud_ptr, predict_pose);
    //获得最终结果
    result_pose = icp_ptr_->getFinalTransformation();

    return true;
}
}