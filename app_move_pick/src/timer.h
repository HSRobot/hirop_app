#pragma once

#include <ros/ros.h>

class Timer{

public:
    /**
     * @brief Timer 在构造函数中获取当前的时间
     * @param name  计时器的名称
     */
    Timer(std::string name) : _name(name){
        start = ros::Time::now();
    }

    /**
     * @brief ~Timer 在析构函数中计算从构造到析构的间隔
     */
    ~Timer(){
        std::cout << _name << " time = " << ros::Time::now().toSec() - start.toSec() << std::endl;
    }

private:
    std::string _name;
    ros::Time start;
};
