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

//电机PID闭环速度控制函数
float PidIncCtl(float target, float current, pid_value *pid);
float PidPosCtl(float target, float current, pid_value *pid);
void PidCtl_Init(void);
void PidCtl_Clear(void);

#endif
