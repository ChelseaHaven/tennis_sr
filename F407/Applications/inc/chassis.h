#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "system.h"

#define MEC_WHEEL_BASE  420.0f  /* Front-to-rear wheel center distance, mm */
#define MEC_ACLE_BASE   475.0f  /* Left-to-right wheel center distance, mm */
#define PERIOD_20MS     0.02f
#define DEG_TO_RAD(x) ((x) * 3.1415926f / 180.0f)
#define RAD_TO_DEG(x) ((x) * 180.0f / 3.1415926f)

#define R_VX_LIMIT  2000   //X轴速度限值 mm/s
#define R_VY_LIMIT  2000   //Y轴速度限值 mm/s
#define R_VW_LIMIT  3140   //W旋转角速度限值rad/s*1000

typedef struct  
{
	float  V_real;       //车轮实时速度，单位mm/s
	float  V_target;     //车轮目标速度，单位mm/s
	float  D_real;       //车轮实时距离，单位mm
	float  D_target;     //车轮目标距离，单位mm
	float  PWM;      	 //车轮PWM控制速度
}ROBOT_Wheel;

typedef struct  
{
	float  X;     //实时X轴速度
	float  Y;     //实时Y轴速度
	float  W;     //实时Yaw旋转轴速度
	
	float  X_target;     //目标X轴速度
	float  Y_target;     //目标Y轴速度
	float  W_target;     //目标Yaw旋转轴速度

	float  X_lim;     //X轴速度限制
	float  Y_lim;     //Y轴速度限制
	float  W_lim;     //Yaw轴速度限制
	
}ROBOT_Velocity;

typedef struct  
{
	float  X_body;     			//自身坐标系，实时X轴位置
	float  Y_body;     			//实时Y轴位置
	float  W_body;     			//实时Yaw旋转轴位置
	
	float  X_body_error;    	//自身坐标系，目标X轴位置误差
	float  Y_body_error;    	//目标Y轴位置误差
	float  W_body_error;     	//目标Yaw旋转轴位置误差

	float  X_global;	 		//全局坐标系，全局X轴位置
	float  Y_global;	 		//全局Y轴位置
	float  W_global;	 		//全局Yaw旋转轴位置

	float  X_global_target;	 	//全局坐标系，目标全局X轴位置
	float  Y_global_target;		//目标全局Y轴位置
	float  W_global_target;	 	//目标全局Yaw旋转轴位置
}ROBOT_Position;

extern ROBOT_Wheel  R_Wheel_A,R_Wheel_B,R_Wheel_C,R_Wheel_D;
extern ROBOT_Velocity  R_Vel;
extern ROBOT_Position  R_Pos;

/**
 * @brief 结合 IMU 航向角，将车轮运动量积分到全局位置。
 */
void Global_PositionUpdate(void);
/**
 * @brief 将全局坐标系下的目标位置误差转换到车体坐标系。
 */
void Global__PositionTransform(void);
/**
 * @brief 更新底盘各车轮目标速度并发送电机速度命令。
 * @param run_enable 非零表示运行，零表示执行底盘急停。
 */
void chassis_control_process(uint8_t run_enable);

#endif
