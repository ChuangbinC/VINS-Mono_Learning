/*
 * @Author: Chuangbin Chen
 * @Date: 2020-03-22 22:19:32
 * @LastEditTime: 2020-05-02 00:40:13
 * @LastEditors: Do not edit
 * @Description: 
 */
#include "initial_alignment.h"

/**
 * @description: 陀螺仪偏置校正
 *               根据视觉SFM的结果来校正陀螺仪Bias -> Paper V-B-1
 *               主要是将相邻帧之间SFM求解出来的旋转矩阵与IMU预积分的旋转量对齐
 *               注意得到了新的Bias后对应的预积分需要repropagate
 * @param[in]   all_image_frame所有图像帧构成的map,图像帧保存了位姿、预积分量和关于角点的信息
 * @param[out]  Bgs 陀螺仪偏置
 * @return      void
 */
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();

        //R_ij = (R^c0_bk)^-1 * (R^c0_{bk+1}) 转换为四元数 q_ij = (q^c0_bk)^-1 * (q^c0_bk+1)
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        //tmp_A = J_j_bw
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(O_R, O_BG);
        //tmp_b = 2 * (r^bk_bk+1)^-1 * (q^c0_bk)^-1 * (q^c0_bk+1)
        //      = 2 * (r^bk_bk+1)^-1 * q_ij
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();
        //tmp_A * delta_bg = tmp_b
        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;

    }
    //LDLT方法
    
    delta_bg = A.ldlt().solve(b);
    ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());

    for (int i = 0; i <= WINDOW_SIZE; i++)
        Bgs[i] += delta_bg;
    int i=0;
    // 重新预积分
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        // 为什么这里只利用到了bgs[0]，因为这里的bgs[i]都被初始化为同一个初始值 delta_bg
        // cout << "Bgs["<< i << "] is " <<Bgs[i++]<< endl;
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }
}

/**
 * @description: 在半径为G的半球找到切面的一对正交基 -> Algorithm 1  
 * @param {type} 
 * @return: 
 */
MatrixXd TangentBasis(Vector3d &g0)
{
    Vector3d b, c;
    Vector3d a = g0.normalized();
    Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}

/**
 * @description: 重力细化，在其切线空间上用两个变量重新参数化重力 -> Paper V-B-3 
                 g^ = ||g|| * (g^-) + w1b1 + w2b2 
 * @param[in]    all_image_frame 所有图像帧构成的map,图像帧保存了位姿，预积分量和关于角点的信息
 * @param[out]   g 重力加速度
 * @param[out]   x 待优化变量，窗口中每帧的速度V[0:n]、二自由度重力参数w[w1,w2]^T、尺度s
 * @return: 
 */
void RefineGravity(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    //g0 = (g^-)*||g||
    Vector3d g0 = g.normalized() * G.norm();
    
    Vector3d lx, ly;
    //VectorXd x;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    // 迭代四次更加精准
    for(int k = 0; k < 4; k++)
    {
        //lxly = b = [b1,b2]
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0);
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);

            MatrixXd tmp_A(6, 9);
            tmp_A.setZero();
            VectorXd tmp_b(6);
            tmp_b.setZero();

            double dt = frame_j->second.pre_integration->sum_dt;

            // tmp_A(6,9) = [-I*dt           0             (R^bk_c0)*dt*dt*b/2   (R^bk_c0)*((p^c0_ck+1)-(p^c0_ck))  ] 
            //              [ -I    (R^bk_c0)*(R^c0_bk+1)      (R^bk_c0)*dt*b                  0                    ]
            // tmp_b(6,1) = [ (a^bk_bk+1)+(R^bk_c0)*(R^c0_bk+1)*p^b_c-p^b_c - (R^bk_c0)*dt*dt*||g||*(g^-)/2 , (b^bk_bk+1)-(R^bk_c0)dt*||g||*(g^-)]^T
            // tmp_A * x = tmp_b 求解最小二乘问题
            tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0] - frame_i->second.R.transpose() * dt * dt / 2 * g0;

            tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;


            Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
            //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
            //MatrixXd cov_inv = cov.inverse();
            cov_inv.setIdentity();

            MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();

            A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            b.tail<3>() += r_b.tail<3>();

            A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
            //QUES:为什么要乘1000，关于单位的吗？
            A = A * 1000.0;
            b = b * 1000.0;
            x = A.ldlt().solve(b);

            //dg = [w1,w2]^T
            VectorXd dg = x.segment<2>(n_state - 3);
            g0 = (g0 + lxly * dg).normalized() * G.norm();
            //double s = x(n_state - 1);
    }   
    g = g0;
}

/**
 * @description: 计算尺度，重力加速度和速度，速度、重力向量和尺度初始化Paper -> V-B-2
 *               相邻帧之间的位置和速度与IMU预积分出来的位置和速度对齐，求解最小二乘
 *               重力细化 -> Paper V-B-3    
 * @param[in]    all_image_frame 所有图像帧构成的map,图像帧保存了位姿，预积分量和关于角点的信息
 * @param[out]   g 重力加速度
 * @param[out]   x 待优化变量，窗口中每帧的速度V[0:n]、重力g、尺度s
 * @return: 
 */
bool LinearAlignment(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x)
{
    int all_frame_count = all_image_frame.size();
    //优化量x的总维度
    int n_state = all_frame_count * 3 + 3 + 1;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        double dt = frame_j->second.pre_integration->sum_dt;

        // tmp_A(6,10) = H^bk_bk+1 = [-I*dt           0             (R^bk_c0)*dt*dt/2   (R^bk_c0)*((p^c0_ck+1)-(p^c0_ck))  ] 
        //                           [ -I    (R^bk_c0)*(R^c0_bk+1)      (R^bk_c0)*dt                  0                    ]
        // tmp_b(6,1 ) = z^bk_bk+1 = [ (a^bk_bk+1)+(R^bk_c0)*(R^c0_bk+1)*p^b_c-p^b_c , (b^bk_bk+1)]^T
        // tmp_A * x = tmp_b 求解最小二乘问题
        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC[0] - TIC[0];
        //cout << "delta_p   " << frame_j->second.pre_integration->delta_p.transpose() << endl;
        tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity();
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v   " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();

        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        b.tail<4>() += r_b.tail<4>();

        A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    double s = x(n_state - 1) / 100.0;
    ROS_DEBUG("estimated scale: %f", s);
    // 取求解出来的g，这里的g应该是在c0坐标系上的
    // 由于最后一个变量为s，g有三个变量，因此从倒数第4个开始取
    g = x.segment<3>(n_state - 4);
    ROS_DEBUG_STREAM(" result g     " << g.norm() << " " << g.transpose());
    //如果重力加速度与参考值差太大或者尺度为负则说明计算错误
    if(fabs(g.norm() - G.norm()) > 1.0 || s < 0)
    {
        return false;
    }
    //重力细化，重力细化过程中又重复上面的求解最小二乘，不过里面的参数会有变化
    RefineGravity(all_image_frame, g, x);
    // 这个是因为上面求解的时候除了100，算出的结果变大100倍，所以现在除以100变回原来结果
    s = (x.tail<1>())(0) / 100.0;
    // x中最后一个变量为s
    (x.tail<1>())(0) = s;
    ROS_DEBUG_STREAM(" refine     " << g.norm() << " " << g.transpose());
    if(s < 0.0 )
        return false;   
    else
        return true;
}

/**
 * @description: 视觉和IMU对齐
 * @param {type} 
 * @return: 
 */
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x)
{
    //计算陀螺仪偏置
    solveGyroscopeBias(all_image_frame, Bgs);

    //计算尺度，重力加速度和速度
    if(LinearAlignment(all_image_frame, g, x))
        return true;
    else 
        return false;
}
