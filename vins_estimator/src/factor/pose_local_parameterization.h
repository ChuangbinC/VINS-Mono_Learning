/*
 * @Author: Chuangbin Chen
 * @Date: 2020-03-22 22:19:32
 * @LastEditTime: 2020-03-26 16:38:18
 * @LastEditors: Do not edit
 * @Description: 
 */
#pragma once

#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

/**
 * @description: 对于四元数或者旋转矩阵这种使用过参数化表示旋转的方式，它们是不支持广义的加法（因为使用普通的加法就会打破其 constraint，比如旋转矩阵加旋转矩阵得到的就不再是旋转矩阵），
 * 所以我们在使用ceres对其进行迭代更新的时候就需要自定义其更新方式了，具体的做法是实现一个参数本地化的子类，需要继承于LocalParameterization，LocalParameterization是纯虚类，
 * 所以我们继承的时候要把所有的纯虚函数都实现一遍才能使用该类生成对象.
 * https://blog.csdn.net/hzwwpgmwy/article/details/86490556
 * 
 * 这里的pose包括q，p，其中q是四元数代表旋转，参数量为4，实际自由度为3
 * @param {type} 
 * @return: 
 */
class PoseLocalParameterization : public ceres::LocalParameterization
{
    // 实现优化变量的更新
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    // 表示参数 x 的自由度（可能有冗余），比如四元数的自由度是4，旋转矩阵的自由度是9
    virtual int GlobalSize() const { return 7; };
    //表示 Δx 所在的正切空间（tangent space）的自由度，例如四元数计算旋转增量只需要取向量部分，因此自由度为3
    virtual int LocalSize() const { return 6; };
};
