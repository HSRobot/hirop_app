#include "auto_move_pick.h"

#include <pick_place_bridge/set_pick.h>
#include <pick_place_bridge/ros_pick_run.h>

#include <hirop_msgs/Look.h>
#include <hirop_msgs/CleanPCL.h>

using namespace hirop_app;

AutoMovePickApp::AutoMovePickApp(){

    _haveObject = false;

    currentMap = "map1";

    _detectionClinet = _n.serviceClient<vision_bridge::detection>("detection");
    _pickClient = _n.serviceClient<pick_place_bridge::ros_pick_run>("pick_excute");
    _setPoseClient = _n.serviceClient<pick_place_bridge::set_pick>("set_pick_pose");
    _lookClient =  _n.serviceClient<hirop_msgs::Look>("look");
    _cleanPclClient =  _n.serviceClient<hirop_msgs::CleanPCL>("clean_pcl");

//    std::cout << "waiting detection server ... " << std::endl;

//    if(_detectionClinet.waitForExistence(ros::Duration(5))){
//        std::cout << "detection server was ready ^_^ " << std::endl;
//    }else{
//        std::cout << "detection server has some worry ready -.-" << std::endl;
//    }

    _objectSub = _n.subscribe("/object_array", 1, &AutoMovePickApp::detectionResCallback, this);
}

int AutoMovePickApp::detection(std::string object){

    vision_bridge::detection detection_srv;
    detection_srv.request.objectName = object;

    detection_srv.request.detectorName = "Yolo6dDetector";
    detection_srv.request.detectorType = 1;
    detection_srv.request.detectorConfig = "";

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
}

void AutoMovePickApp::detectionResCallback(const vision_bridge::ObjectArray::ConstPtr& msg){
    /**
     *  只是将位姿保存下来
     */
    _objectCamPose = msg->objects[0].pose;

    std::cout << "detection object :  x = " << _objectCamPose.pose.position.x << std::endl;

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
    objectPoseSrv.request.pickPos = _objectCamPose;

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
