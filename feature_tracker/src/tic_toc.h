/*
 * @Author: Chuangbin Chen
 * @Date: 2020-03-22 22:19:31
 * @LastEditTime: 2020-03-24 16:11:01
 * @LastEditors: Do not edit
 * @Description: 计算程序运算时间的类
 */
#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
