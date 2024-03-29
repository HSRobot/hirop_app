#pragma once

#include <iostream>

#include <hirop/datamanager/file_datawriter.h>
#include <hirop/datamanager/posedata.h>
#include <hirop/nav/mobile_robot.h>
#include <hirop/nav/autorunmotion.h>

#include <ros/ros.h>
#include <vision_bridge/detection.h>
#include <vision_bridge/ObjectArray.h>

#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;
using namespace hirop::data_manager;
using namespace hirop::navigation;
using namespace moveit::planning_interface;

#define Y_SHIFTING 0.1
#define X_SHIFTING 0.1

/**
 *  ROW 表示一行有多少个物体
 */
#define ROW 4

namespace hirop_app{

class AutoMovePickApp{

public:

    /**
     * @brief AutoMovePickApp   构造函数
     */
    AutoMovePickApp();

    /**
     * @brief moveto    运动到指定点
     * @param flag      地点的名称
     * @return          0 成功 -1失败
     */
    int moveto(std::string flag);

    /**
     * @brief detection 识别指定物体
     * @param object    被识别的物体名称
     * @return          0 识别成功 -1 识别失败
     */
    int detection(std::string object);

    /**
     * @brief pick  执行抓取
     * @return      0 抓取成功 -1 抓取失败
     */
    int pick();

    /**
     * @brief place 执行放置动作，每次都会更新放置的位置
     * @return      0 放置成功 -1 放置失败
     */
    int place();

    /**
     * @brief look  观察一次周围环境
     * @return      0 正常观察 -1 观察出错
     */
    int look();

    /**
     * @brief cleanPcl  清空之前观察的点云
     * @return          0 清空成功 -1 清空失败
     */
    int cleanPcl();

    /**
     * @brief armMoveTo
     * @param name
     * @return
     */
    int armMoveTo(std::string name);

    /**
     * @brief clearOctomap
     * @return
     */
    int clearOctomap();

private:

    /**
     * @brief getMapFlagPose    获取地图上flag的pose
     * @param flag              flag名称
     * @return                  位姿
     */
    PoseData *getMapFlagPose(std::string flag);

    /**
     * @brief detectionResCallback  识别结果的回调函数
     * @param msg                   识别结果
     */
    void detectionResCallback(const vision_bridge::ObjectArray::ConstPtr& msg);

    /**
     * @brief initMoveit
     * @return  0 success -1 error
     */
    int initMoveit();

    /**
     * @brief initPlace     初始化放置相关的参数
     */
    void initPlace();

private:

    /**
     * @brief currentMap    当前地图
     */
    std::string currentMap;

    /**
     * @brief _n            节点句柄
     */
    ros::NodeHandle _n;

    /**
     * @brief _detectionClinet  检测服务的客户端
     */
    ros::ServiceClient _detectionClinet;

    /**
     * @brief _pickClient  抓取服务的客户端
     */
    ros::ServiceClient _pickClient;

    /**
     * @brief _placeClient  放置服务的客户端
     */
    ros::ServiceClient _placeClient;

    /**
     * @brief _setPoseClient  设置目标位姿的客户端
     */
    ros::ServiceClient _setPoseClient;

    /**
     * @brief _setPoseClient  设置放置位姿的客户端
     */
    ros::ServiceClient _setPlacePoseClient;

    /**
     * @brief _clearOctomapClient   clear movit octomap client
     */
    ros::ServiceClient _clearOctomapClient;

    /**
     * @brief _lookClient  感知模块的客户端
     */
    ros::ServiceClient _lookClient;

    /**
     * @brief _cleanPclClient  清除点云的客户端
     */
    ros::ServiceClient _cleanPclClient;

    /**
     * @brief _objectSub    物体检测成功的话题
     */
    ros::Subscriber _objectSub;

    /**
     * @brief objectPose    物体的相机坐标
     */
    geometry_msgs::PoseStamped _objectCamPose;

    /**
     * @brief _haveObject   标志当前是否有物体已经被识别到
     */
    bool _haveObject;

    /**
     * @brief _worldPose    save the world pose
     */
    geometry_msgs::PoseStamped worldPose;

    /**
     * @brief _placePose     保存放置的初始位姿
     */
    geometry_msgs::PoseStamped _placePose;

    /**
     * @brief _move_group   moevit的接口
     */
    MoveGroupInterface *_move_group;

    /**
     * @brief _placeCount    放置的计数
     */
    int _placeCount;
};

}
