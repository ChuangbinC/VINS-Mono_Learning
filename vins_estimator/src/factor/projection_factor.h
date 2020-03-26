/*
 * @Author: Chuangbin Chen
 * @Date: 2020-03-22 22:19:32
 * @LastEditTime: 2020-03-26 11:41:47
 * @LastEditors: Do not edit
 * @Description: 
 */
#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"
/**
 * @description: 2表示残差的自由度[u,v]
 * 7表示[p,q]的自由度，分别是第一个空间点[p^w_i,q^w_i]，第二个空间点[p^w_j,q^w_j]，相机和IMU外参[p^c_b,q^c_b]
 * 1表示逆深度
 * @param {type} 
 * @return: 
 */
class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 7, 1>
{
  public:
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
    void check(double **parameters);

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Matrix<double, 2, 3> tangent_base;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
