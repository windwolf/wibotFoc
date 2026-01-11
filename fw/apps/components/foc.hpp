#pragma once

#include "buffer.hpp"
#include "dsp/controller/pid.hpp"
#include "math/index.hpp"
#include "pp/controller/pid-node.hpp"

namespace wibot::motor {

using namespace wibot;

struct Sensor {
    Buffer16<3> uAbcADC;  // 三相电压ADC采样值
    Buffer16<3> iAbcADC;  // 三相电流ADC采样值

    Vector3f uAbc;      // 三相电压
    Vector3f iAbc;      // 三相电流
    f32      theta;     // 转子角度 (弧度)
    f32      omega;     // 角速度 (弧度/秒)
    f32      position;  // 位置 (单位根据编码器而定)
};

struct Config {
    Pid::Config idPid;
    Pid::Config iqPid;
};

/**
 * @brief FOC (Field Oriented Control) 磁场定向控制器
 * 
 * FOC控制器流程图 (横向):
 * ┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
 * │                                                  FOC 控制流程                                                        │
 * └──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
 * 
 *   ┌─────────┐      ┌─────────┐      ┌──────────┐      ┌─────────┐      ┌─────────┐      ┌─────────┐      ┌──────────┐
 *   │  输入   │ ───> │ Clarke  │ ───> │   Park   │ ───> │ PID控制 │ ───> │ InvPark │ ───> │  SVPWM  │ ───> │   三相   │
 *   │ 目标值  │      │  变换   │      │   变换   │      │  (dq轴) │      │  变换   │      │  调制   │      │   输出   │
 *   └─────────┘      └─────────┘      └──────────┘      └─────────┘      └─────────┘      └─────────┘      └──────────┘
 *       │                │                  │                │                │                 │               │
 *   id*, iq*        iABC ───> iαβ      iαβ ───> idq      idq, id*, iq*   udq ───> uαβ      uαβ ───> PWM   PWM ───> 电机
 *   (d轴/q轴电流)   (三相电流)  (静止坐标)   (旋转坐标)   PID输出udq    (静止坐标)      (占空比)       (三相驱动)
 *       │                                  ▲                                  │
 *       │                                  │                                  │
 *       └──────────────────────────────────┼──────────────────────────────────┘
 *                                          │
 *                                    ┌─────────┐
 *                                    │ 转子位置│  ← 编码器/传感器反馈
 *                                    │  theta  │     (电角度)
 *                                    └─────────┘
 * 
 * 详细步骤说明:
 * 
 * 1. 输入目标值
 *    - id* : d轴电流目标值 (通常设为0，实现零直轴电流控制)
 *    - iq* : q轴电流目标值 (控制转矩)
 * 
 * 2. Clarke变换 (三相 → 两相静止坐标系)
 *    iα = ia
 *    iβ = (ia + 2*ib) / √3
 *    将三相电流 (ia, ib, ic) 转换为两相静止坐标系 (iα, iβ)
 * 
 * 3. Park变换 (静止坐标系 → 旋转坐标系)
 *    id = iα*cos(θ) + iβ*sin(θ)
 *    iq = -iα*sin(θ) + iβ*cos(θ)
 *    将静止坐标系电流转换为旋转坐标系 (id, iq)，θ为转子电角度
 * 
 * 4. PID控制
 *    ud = PID_d(id* - id)  // d轴电流环PI控制
 *    uq = PID_q(iq* - iq)  // q轴电流环PI控制
 *    计算d轴和q轴的控制电压
 * 
 * 5. Inverse Park变换 (旋转坐标系 → 静止坐标系)
 *    uα = ud*cos(θ) - uq*sin(θ)
 *    uβ = ud*sin(θ) + uq*cos(θ)
 *    将dq轴控制电压转换回静止坐标系
 * 
 * 6. SVPWM调制 (空间矢量脉宽调制)
 *    根据uα, uβ计算三相PWM占空比
 *    实现高效的三相逆变器开关控制
 * 
 * 7. 三相输出
 *    生成PWM信号驱动三相逆变器
 *    控制电机三相绕组电流
 * 
 * 反馈环路:
 *    - 电流反馈: ADC采样三相电流 → Clarke变换 → Park变换 → 与目标值比较
 *    - 位置反馈: 编码器/传感器提供转子位置θ，用于坐标变换
 */
class Foc {
   public:
   private:
    Config  _config;
    PidNode _idqPid[2]{PidNode(_config.idPid), PidNode(_config.iqPid)};  // d轴和q轴电流环PID控制器
};

}  // namespace wibot::motor