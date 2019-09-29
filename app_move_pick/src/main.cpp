#include <iostream>
#include <ros/ros.h>
#include "auto_move_pick.h"

#include <tf/transform_listener.h>

using namespace hirop_app;

void testTF(){

    tf::TransformListener listener;
    tf::StampedTransform transform;

    /**
     * 获取当前相机系和世界坐标系的变换关系
     */
    try{
        listener.waitForTransform("d435i_color_optical_frame", "d435i_link", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("d435i_color_optical_frame", "d435i_link", ros::Time(0), transform);
    }catch (tf::TransformException &ex) {
        /**
         * @brief 获取变换关系失败，等待1秒后继续获取坐标变换
         */
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    std::cout << "robot_base_link -> d435i_link "<< std::endl
              << "X = " << transform.getOrigin().getX() << std::endl
              << "Y = " << transform.getOrigin().getY() << std::endl
              << "Z = " << transform.getOrigin().getZ() << std::endl
              << "QX = " << transform.getRotation().getX() << std::endl
              << "QY = " << transform.getRotation().getY() << std::endl
              << "QZ = " << transform.getRotation().getZ() << std::endl
              << "QW = " << transform.getRotation().getW() << std::endl;
}

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

    testTF();

    while(!ros::isShuttingDown()){

        std::cout << "任务执行完毕" << std::endl;
        std::cout << "---------------------------------------------------" << std::endl;
        std::cout << "请输入指令：" << std::endl;
        std::cout << "0，移动抓取" << std::endl;
        std::cout << "1，移动底盘目标点运动" << std::endl;
        std::cout << "2，识别牛奶盒" << std::endl;
        std::cout << "3，构建点云" << std::endl;
        std::cout << "4，清除点云" << std::endl;
        std::cout << "5，抓取目标" << std::endl;
        std::cout << "6，放置目标" << std::endl;
        std::cout << "7，执行连续抓取" << std::endl;
        std::cout << "8，退出" << std::endl;


        std::cin >> model;

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

        case 6:
            app.place();
            break;

        case 7:

            while(!app.detection("Yolo6dDetector")){

                if(app.look())
                    break;

                if(app.pick())
                    break;

                if(app.moveto("pick"))
                    break;

                if(app.place())
                    break;

                if(app.armMoveTo("detection"))
                    break;

                if(app.clearOctomap())
                    break;

                if(app.cleanPcl())
                    break;
            }
            break;

        case 8:
            return 0;
            break;

        case 0:
            if(app.moveto("table"))
                break;

            if(app.detection("Yolo6dDetector"))
                break;

            sleep(0.5);

            if(app.look())
                break;

            if(app.pick())
                break;

            if(app.moveto("home"))
                break;

            if(app.armMoveTo("detection"))
                break;

            if(app.clearOctomap())
                break;

            if(app.clearOctomap())
                break;

            break;
        }
    }

    return 0;
}
