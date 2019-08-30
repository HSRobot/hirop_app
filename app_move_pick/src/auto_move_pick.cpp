#include "auto_move_pick.h"

#include <pick_place_bridge/set_pick.h>
#include <pick_place_bridge/ros_pick_run.h>

#include <hirop_msgs/Look.h>
#include <hirop_msgs/CleanPCL.h>

#include <tf/transform_listener.h>

using namespace hirop_app;

AutoMovePickApp::AutoMovePickApp(){

    _haveObject = false;

    currentMap = "map1";

    initMoveit();

    _detectionClinet = _n.serviceClient<vision_bridge::detection>("detection");
    _pickClient = _n.serviceClient<pick_place_bridge::ros_pick_run>("pick_excute");
    _setPoseClient = _n.serviceClient<pick_place_bridge::set_pick>("set_pick_pose");
    _lookClient =  _n.serviceClient<hirop_msgs::Look>("look");
    _cleanPclClient =  _n.serviceClient<hirop_msgs::CleanPCL>("clean_pcl");

    _objectSub = _n.subscribe("/object_array", 1, &AutoMovePickApp::detectionResCallback, this);
}

int AutoMovePickApp::detection(std::string object){

    vision_bridge::detection detection_srv;
    detection_srv.request.objectName = object;

    detection_srv.request.detectorName = "Yolo6dDetector";
    detection_srv.request.detectorType = 1;
    detection_srv.request.detectorConfig = "";

    armMoveTo("detection");

    bool ret;
    ret = _detectionClinet.call(detection_srv);

    if(detection_srv.response.result || !ret){
        std::cout << "detection error" << std::endl;
        return -1;
    }

    return 0;
}

int AutoMovePickApp::moveto(std::string flag){

    PoseData* data;

    data = getMapFlagPose(flag);

    /**
     * @brief 获取移动底盘的实例
     */
    MobileRobot *robot = MobileRobot::getInstance();

    /**
     * 创建一个导航运动
     */
    AutoRunMotion *motion = new AutoRunMotion();

    /**
     *  设置目标点
     */
    motion->setTargetPose(data->pose);

    /**
     *  执行运动
     */
    robot->runMotion(motion);

    delete data;
    delete motion;

    return 0;
}

void AutoMovePickApp::detectionResCallback(const vision_bridge::ObjectArray::ConstPtr& msg){
    /**
     *  只是将位姿保存下来
     */
    _objectCamPose = msg->objects[0].pose;

    tf::TransformListener listener;

    for(int i = 0; i < 10; i++){

        try{
            listener.waitForTransform("robot_base_link", _objectCamPose.header.frame_id, ros::Time(0), ros::Duration(1.0));
            listener.transformPose("robot_base_link", _objectCamPose, worldPose);
            break;
        }catch (tf::TransformException &ex) {
            /**
         * @brief 获取变换关系失败，等待1秒后继续获取坐标变换
         */
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

    }

    worldPose.pose.orientation.x = 0;
    worldPose.pose.orientation.y = 0;
    worldPose.pose.orientation.z = 0;
    worldPose.pose.orientation.w = 1;

    std::cout << "detection object :  x = " << worldPose.pose.position.x << std::endl;

    _haveObject = true;
}

PoseData* AutoMovePickApp::getMapFlagPose(std::string flag){

    FileDataWriter write;

    DataUri uri(flag);
    uri.addFlag("MAP");
    uri.addFlag(currentMap);
    uri.addFlag("flags");

    PoseData *data = (PoseData *)write.loadData(uri);

    return data;
}

int AutoMovePickApp::pick(){

    bool ret;

    pick_place_bridge::set_pick objectPoseSrv;
    objectPoseSrv.request.pickPos = worldPose;

    /**
     *  设置抓取目标的位姿
     */
    ret = _setPoseClient.call(objectPoseSrv);

    if( !objectPoseSrv.response.isSetFinsh || !ret){
        std::cout << "set object pose error" << std::endl;
        return -1;
    }

    pick_place_bridge::ros_pick_run pickSrv;
    ret = _pickClient.call(pickSrv);

    armMoveTo("pick");

    if( !pickSrv.response.isPickFinsh || !ret){
        std::cout << "pick object error" << std::endl;
        return -1;
    }

    return 0;
}

int AutoMovePickApp::look(){
    bool ret;
    hirop_msgs::Look lookSrv;

    ret = _lookClient.call(lookSrv);

    if( lookSrv.response.reuslt || !ret){
        std::cout << "look error" << std::endl;
        return -1;
    }

    return 0;
}

int AutoMovePickApp::cleanPcl(){

    bool ret;
    hirop_msgs::CleanPCL cleanSrv;

    ret = _cleanPclClient.call(cleanSrv);

    if( cleanSrv.response.result || !ret){
        std::cout << "clean pcl error" << std::endl;
        return -1;
    }

    return 0;
}

int AutoMovePickApp::armMoveTo(std::string name){

    MoveItErrorCode result;
    for(int i = 0; i < 2; i++){
        _move_group->setNamedTarget(name);
        result = _move_group->move();

        if(result == MoveItErrorCode::SUCCESS){
            return 0;
        }
        std::cout << "ARM Move error" << std::endl;
    }

    if(result != MoveItErrorCode::SUCCESS)
        return -1;

    return 0;
}

int AutoMovePickApp::initMoveit(){

    _move_group = new MoveGroupInterface("arm");
    _move_group->setPoseReferenceFrame("robot_base_link");
    _move_group->setPlannerId("RRTConnect");
    _move_group->setPlanningTime(3);

    return 0;
}
