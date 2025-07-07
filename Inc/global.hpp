#pragma once

#include <array>
#include <mutex>

// 声明：全局最新关节角度数组（在 main.cpp 里定义一次）
extern std::array<double, 7> g_latest_q;

// 声明：保护 g_latest_q 的互斥量
extern std::mutex g_q_mutex;
