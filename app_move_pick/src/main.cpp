#include <iostream>

#include <ros/ros.h>

#include "auto_move_pick.h"

using namespace hirop_app;

int main(int argc, char *argv[]){

    ros::init(argc, argv, "move_pick_app");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    /**
     * @brief app   应用的实例
     */
    AutoMovePickApp app;
    int model;
    std::string flag;

    std::cout << "请输入指令：" << std::endl;
    std::cout << "1，移动底盘目标点运动" << std::endl;
    std::cout << "2，识别牛奶盒" << std::endl;
    std::cout << "3，构建点云" << std::endl;
    std::cout << "4，清除点云" << std::endl;
    std::cout << "5，抓取目标" << std::endl;

    std::cin >> model;

    while(!ros::isShuttingDown()){

        switch(model){

        case 1:
            std::cout << "请输入目标位置" << std::endl;
            std::cin >> flag;
            app.moveto(flag);
            break;

        case 2:
            app.detection("Yolo6dDetector");
            break;

        case 3:
            app.look();
            break;

        case 4:
            app.cleanPcl();
            break;

        case 5:
            app.pick();
            break;
        }
        std::cout << "任务执行完毕" << std::endl;
        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << "请输入指令：" << std::endl;
        std::cout << "1，移动底盘目标点运动" << std::endl;
        std::cout << "2，识别牛奶盒" << std::endl;
        std::cout << "3，构建点云" << std::endl;
        std::cout << "4，清除点云" << std::endl;
        std::cout << "5，抓取目标" << std::endl;
    }

    return 0;
}
