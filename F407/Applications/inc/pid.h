#ifndef _PID_H
#define _PID_H

#include "system.h"

typedef struct {
    float target_val;             //目标值
    float Kp;                     //比例系数Proportional
    float Ki;                     //积分系数Integral
    float Kd;                     //微分系数Derivative
    float Ek;                     //当前误差
    float Ek1;                    //前一次误差 e(k-1)
    float Ek2;                    //再前一次误差 e(k-2)
    float OUT;                    //PID输出
}pid_value;

extern pid_value motor_speed_pid[4];
extern pid_value car_position_pid[3];

/**
 * @brief 设置 PID 控制器的比例、积分和微分增益。
 * @param pid 要设置的 PID 控制器。
 * @param Kp 比例增益。
 * @param Ki 积分增益。
 * @param Kd 微分增益。
 * @note 只更新增益，不清除误差历史或当前输出。
 */
void PidSet(pid_value *pid, float Kp, float Ki, float Kd);

/**
 * @brief 更新增量式 PID 控制器。
 * @param target 目标值。
 * @param current 当前测量值。
 * @param[in,out] pid 控制器参数及累计状态。
 * @return 更新后的控制器输出。
 */
float PidIncCtl(float target, float current, pid_value *pid);
/**
 * @brief 计算位置式 PID 控制器的输出。
 * @param target 目标值。
 * @param current 当前测量值。
 * @param[in,out] pid 控制器参数及历史误差。
 * @return 更新后的控制器输出。
 */
float PidPosCtl(float target, float current, pid_value *pid);
/**
 * @brief 初始化底盘速度环和位置环的 PID 参数及限幅值。
 */
void PidCtl_Init(void);
/**
 * @brief 清零 PID 目标值、历史误差和输出，不修改增益参数。
 */
void PidCtl_Clear(void);

#endif
