/*
 * @Author: Chuangbin Chen
 * @Date: 2020-03-22 22:19:32
 * @LastEditTime: 2020-03-26 20:10:33
 * @LastEditors: Do not edit
 * @Description: 
 */
#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <ros/ros.h>
#include <map>
#include "../feature_manager.h"

using namespace Eigen;
using namespace std;

/**
 * @description: 图像帧类可由图像帧的特征点与时间戳构造，
 *               此外还保存了位姿Rt，预积分对象pre_integration，是否是关键帧。
 * @return: 
 */
class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
        {
            points = _points;
        };
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>> > > points;
        double t;
        Matrix3d R;
        Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);