#include "auto_move_pick.h"
#include "timer.h"

#include <pick_place_bridge/set_pick.h>
#include <pick_place_bridge/set_place.h>
#include <pick_place_bridge/ros_pick_run.h>
#include <pick_place_bridge/ros_place_run.h>

#include <hirop_msgs/Look.h>
#include <hirop_msgs/CleanPCL.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>

using namespace hirop_app;

AutoMovePickApp::AutoMovePickApp(){

    _haveObject = false;
    currentMap = "map1";
    _placeCount = 0;

    initMoveit();
    initPlace();

    _detectionClinet = _n.serviceClient<vision_bridge::detection>("detection");
    _pickClient = _n.serviceClient<pick_place_bridge::ros_pick_run>("pick_excute");
    _placeClient = _n.serviceClient<pick_place_bridge::ros_place_run>("place_excute");
    _setPlacePoseClient = _n.serviceClient<pick_place_bridge::set_place>("set_place_pose");
    _setPoseClient = _n.serviceClient<pick_place_bridge::set_pick>("set_pick_pose");
    _lookClient =  _n.serviceClient<hirop_msgs::Look>("look");
    _cleanPclClient =  _n.serviceClient<hirop_msgs::CleanPCL>("clean_pcl");
    _clearOctomapClient = _n.serviceClient<std_srvs::Empty>("clear_octomap");

    _objectSub = _n.subscribe("/object_array", 1, &AutoMovePickApp::detectionResCallback, this);
}

int AutoMovePickApp::detection(std::string object){

    Timer timer("detection");

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

    Timer timer("navigation");

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

    std::cout << "detection object :  x = " << worldPose.pose.position.x << std::endl;
    std::cout << "detection object :  y = " << worldPose.pose.position.x << std::endl;
    std::cout << "detection object :  z = " << worldPose.pose.position.x << std::endl;

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

    Timer timer("pick");

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

    Timer timer("look");

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

int AutoMovePickApp::clearOctomap(){

    std_srvs::Empty empty;

    bool ret;
    ret = _clearOctomapClient.call(empty);

    if(!ret){
        std::cout << "clean octomap error" << std::endl;
        return -1;
    }

    return 0;
}

int AutoMovePickApp::initMoveit(){

    _move_group = new MoveGroupInterface("arm");
    _move_group->setPoseReferenceFrame("robot_base_link");
    _move_group->setPlannerId("RRTConnect");
    _move_group->setPlanningTime(3);

    return 0;
}

void AutoMovePickApp::initPlace(){

    _placePose.header.frame_id = "base_link";
    _placePose.pose.position.x = 0.347480040602;
    _placePose.pose.position.y = 0.188192476405;
    _placePose.pose.position.z = 0.428161434575;
    _placePose.pose.orientation.x =  -0.000287358026941;
    _placePose.pose.orientation.y =  0.70659545853;
    _placePose.pose.orientation.z =  0.000385127082133;
    _placePose.pose.orientation.w =  0.707617571212;

}

int AutoMovePickApp::place(){

    Timer timer("place");

    int rows = 0;
    int cols = 0;
    int ret;

    /**
     *  根据物体个数来更新摆放的位置
     */
    rows = _placeCount / ROW;
    cols = _placeCount % ROW;

    pick_place_bridge::set_place placePoseSrv;
    placePoseSrv.request.placePos = _placePose;

    /**
     *  根据物体个数来更新摆放的位置
     */
    rows = _placeCount / ROW;
    cols = _placeCount % ROW;
    placePoseSrv.request.placePos.pose.position.x += rows * X_SHIFTING;
    placePoseSrv.request.placePos.pose.position.y += cols * Y_SHIFTING;

    ret = _setPlacePoseClient.call(placePoseSrv);
    if(!ret){
        std::cout << "set place pose error" << std::endl;
        return -1;
    }

    pick_place_bridge::ros_place_run placeSrv;
    ret = _placeClient.call(placeSrv);
    if(!ret){
        std::cout << "place error" << std::endl;
        return -1;
    }

    /**
     *  更新放置计数
     */
    _placeCount ++;

    return 0;
}
