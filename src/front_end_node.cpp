/*
 * @Description: 前端里程计的node文件
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include "glog/logging.h"

#include <lidar_localization/saveMap.h>
#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/front_end/front_end_flow.hpp"

using namespace lidar_localization;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

// ros server 的回调 PublishGlobalMap调用pcl滤波
//_front_end_flow_ptr->SaveMap();调用 FrontEnd::SaveMap()
bool save_map_callback(saveMap::Request &request, saveMap::Response &response) {
    response.succeed = _front_end_flow_ptr->SaveMap();
    _front_end_flow_ptr->PublishGlobalMap();
    return response.succeed;
}

int main(int argc, char *argv[]) {
    //初始化日志
    google::InitGoogleLogging(argv[0]);
    //build文件后 得到的工作路径
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    //除了glog日志文件之外是否需要标准输出
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    //新建ros 句柄
    ros::NodeHandle nh;
    //新建ros 服务 接收到服务save——map 回调保存地图rosservice call /save_map
    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
    //定义一个FrontEndFlow类的 句柄
    _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);
    
    // 前端线程
    ros::Rate rate(100);
    while (ros::ok()) {
        //ros::spin() 在调用后不会再返回，也就是你的主程序到这儿就不往下执行了，而 ros::spinOnce() 后者在调用后还可以继续执行之后的程序。
        ros::spinOnce();

        _front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}

//