#include "pid.h"

pid_value motor_speed_pid[4];  //电机速度环PID参数结构体
pid_value car_position_pid[3];  //车位置环PID参数结构体

float PidIncCtl(float target, float current, pid_value *pid)
{
    pid->Ek = target - current;

    // 增量式 PID
    pid->OUT +=
          pid->Kp * (pid->Ek - pid->Ek1)
        + pid->Ki * pid->Ek
        + pid->Kd * (pid->Ek - 2.0f * pid->Ek1 + pid->Ek2);

    pid->Ek2 = pid->Ek1;
    pid->Ek1 = pid->Ek;

    return pid->OUT;
}

float PidPosCtl(float target, float current, pid_value *pid)
{

    pid->Ek = target - current;

    // 位置式PID
    pid->OUT =
          pid->Kp * pid->Ek
        + pid->Ki * (pid->Ek + pid->Ek1 + pid->Ek2)
        + pid->Kd * (pid->Ek - pid->Ek1);

    pid->Ek2 = pid->Ek1;
    pid->Ek1 = pid->Ek;

    return pid->OUT;
}


void PidCtl_Init(void)
{
  //电机速度环PID参数初始化
  motor_speed_pid[0].Kp = 0.51;   //电机A速度环比例系数
  motor_speed_pid[0].Ki = 0.09;  //电机A速度环微分系数
  motor_speed_pid[0].Kd = 0.11;
  
  motor_speed_pid[1].Kp = 0.51;   //电机B速度环比例系数
  motor_speed_pid[1].Ki = 0.09;  //电机B速度环微分系数
  motor_speed_pid[1].Kd = 0.11;
  
  motor_speed_pid[2].Kp = 0.51;   //电机C速度环比例系数
  motor_speed_pid[2].Ki = 0.09;  //电机C速度环微分系数
  motor_speed_pid[2].Kd = 0.11;
  
  motor_speed_pid[3].Kp = 0.51;   //电机D速度环比例系数
  motor_speed_pid[3].Ki = 0.09;  //电机D速度环微分系数
  motor_speed_pid[3].Kd = 0.11;

  //电机位置环PID参数初始化
  car_position_pid[0].Kp = 1.0;   //x位置环比例系数
  car_position_pid[0].Ki = 0.1;   //x位置环微分系数

  car_position_pid[1].Kp = 1.0;   //y位置环比例系数
  car_position_pid[1].Ki = 0.1;   //y位置环微分系数

  car_position_pid[2].Kp = 1.0;   //yaw位置环比例系数
  car_position_pid[2].Ki = 0.1;   //yaw位置环微分系数

  R_Vel.X_lim = 1.0;
  R_Vel.Y_lim = 1.0;
  R_Vel.W_lim = 1.0;
}

void PidCtl_Clear(void)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        motor_speed_pid[i].target_val = 0.0f;
        motor_speed_pid[i].Ek = 0.0f;
        motor_speed_pid[i].Ek1 = 0.0f;
        motor_speed_pid[i].Ek2 = 0.0f;
        motor_speed_pid[i].OUT = 0.0f;
    }

    for (i = 0U; i < 3U; i++) {
        car_position_pid[i].target_val = 0.0f;
        car_position_pid[i].Ek = 0.0f;
        car_position_pid[i].Ek1 = 0.0f;
        car_position_pid[i].Ek2 = 0.0f;
        car_position_pid[i].OUT = 0.0f;
    }
}
