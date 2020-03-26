/*
 * @Author: Chuangbin Chen
 * @Date: 2020-03-22 22:19:32
 * @LastEditTime: 2020-03-25 16:35:06
 * @LastEditors: Do not edit
 * @Description: 关于边缘化策略的类
 */
#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <cstdlib>
#include <pthread.h>
#include <ceres/ceres.h>
#include <unordered_map>

#include "../utility/utility.h"
#include "../utility/tic_toc.h"

const int NUM_THREADS = 4;

struct ResidualBlockInfo
{
    ResidualBlockInfo(ceres::CostFunction *_cost_function,ceres::LossFunction *_loss_function, std::vector<double *> _parameter_blocks, std::vector<int> _drop_set)
        : cost_function(_cost_function), loss_function(_loss_function), parameter_blocks(_parameter_blocks), drop_set(_drop_set) {}

    void Evaluate();

    ceres::CostFunction *cost_function;
    //其中parameter_block_sizes每个优化变量块的变量大小，以IMU残差为例，为[7,9,7,9]
    ceres::LossFunction *loss_function;
    //优化变量数据
    std::vector<double *> parameter_blocks;
    //待marg的优化变量id
    std::vector<int> drop_set;
    //Jacobian
    double **raw_jacobians;
    std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
    //残差，IMU：15×1,视觉：2×1
    Eigen::VectorXd residuals;

    int localSize(int size)
    {
        return size == 7 ? 6 : size;
    }
};

struct ThreadsStruct
{
    std::vector<ResidualBlockInfo *> sub_factors;
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
    std::unordered_map<long, int> parameter_block_size; //global size
    std::unordered_map<long, int> parameter_block_idx; //local size
};

class MarginalizationInfo
{
  public:
    ~MarginalizationInfo();
    int localSize(int size) const;
    int globalSize(int size) const;
    
    //加残差块相关信息(优化变量、待marg的变量)
    void addResidualBlockInfo(ResidualBlockInfo *residual_block_info);
    
    //计算每个残差对应的Jacobian，并更新parameter_block_data
    void preMarginalize();

    //pos为所有变量维度，m为需要marg掉的变量，n为需要保留的变量
    void marginalize();
    std::vector<double *> getParameterBlocks(std::unordered_map<long, double *> &addr_shift);
    
    //所有观测项
    std::vector<ResidualBlockInfo *> factors;

    
    //m为要marg掉的变量个数，也就是parameter_block_idx的总localSize，以double为单位，VBias为9，PQ为6，T为6，t为1
    // VBias 表示 speed(3) Bas(3) Bgs(3) 详情见Vector2double
    // PQ 表示 pose(3) q(4),至于为什么这里是6，可能跟下面的7to6有关
    //n为要保留下的优化变量的变量个数,n=localSize(parameter_block_size) – m
    int m, n;

    //<优化变量内存地址，localSize>
    std::unordered_map<long, int> parameter_block_size; //global size
    int sum_block_size;
    //<待marg的优化变量内存地址，在parameter_block_size中的id,以double为单位>
    std::unordered_map<long, int> parameter_block_idx; //local size

    //<优化变量内存地址，数据>
    std::unordered_map<long, double *> parameter_block_data;

    std::vector<int> keep_block_size; //global size
    std::vector<int> keep_block_idx;  //local size
    std::vector<double *> keep_block_data;

    Eigen::MatrixXd linearized_jacobians;
    Eigen::VectorXd linearized_residuals;
    const double eps = 1e-8;

};

class MarginalizationFactor : public ceres::CostFunction
{
  public:
    MarginalizationFactor(MarginalizationInfo* _marginalization_info);
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    MarginalizationInfo* marginalization_info;
};
