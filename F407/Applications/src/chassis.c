#include "chassis.h"

ROBOT_Wheel  R_Wheel_A,R_Wheel_B,R_Wheel_C,R_Wheel_D;
ROBOT_Velocity  R_Vel;
ROBOT_Position  R_Pos;

/**
 * @brief 处理底盘急停，并在每次进入急停时清零运动状态。
 * @param run_enable 非零表示运行底盘，零表示急停。
 * @return 1 表示底盘保持停止，0 表示可以运行。
 */
static uint8_t chassis_handle_emergency_stop(uint8_t run_enable)
{
  static uint8_t emergency_stop_handled = 0U;
  uint8_t i;

  if (run_enable != 0U)
  {
    emergency_stop_handled = 0U;
    return 0U;
  }

  if (emergency_stop_handled == 0U)
  {
    ROBOT_Wheel *wheels[] = {&R_Wheel_A, &R_Wheel_B, &R_Wheel_C, &R_Wheel_D};

    R_Vel.X_target = 0.0f;
    R_Vel.Y_target = 0.0f;
    R_Vel.W_target = 0.0f;

    R_Pos.X_body_error = 0.0f;
    R_Pos.Y_body_error = 0.0f;
    R_Pos.W_body_error = 0.0f;
    R_Pos.X_global_target = 0.0f;
    R_Pos.Y_global_target = 0.0f;
    R_Pos.W_global_target = 0.0f;
    wireless_test[0] = 0.0f;
    wireless_test[1] = 0.0f;
    wireless_test[2] = 0.0f;

    for (i = 0U; i < 4U; i++)
    {
      wheels[i]->V_target = 0.0f;
      wheels[i]->D_target = 0.0f;
      wheels[i]->PWM = 0.0f;
    }

    PidCtl_Clear();
    (void)motor_set_zero_current(MOTOR_ID_FRONT_RIGHT);
    (void)motor_set_zero_current(MOTOR_ID_REAR_LEFT);
    (void)motor_set_zero_current(MOTOR_ID_REAR_RIGHT);
    (void)motor_set_zero_current(MOTOR_ID_FRONT_LEFT);
    emergency_stop_handled = 1U;
  }

  return 1U;
}

static float clamp_float(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }
  if (value < min_value)
  {
    return min_value;
  }
  return value;
}

void Global_PositionUpdate(void)
{
  float dx_body;
  float dy_body;
  float dx_world;
  float dy_world;
  const float wheel_lever_arm = MEC_WHEEL_BASE / 2.0f + MEC_ACLE_BASE / 2.0f;

  R_Vel.X = (R_Wheel_A.V_real + R_Wheel_B.V_real + R_Wheel_C.V_real + R_Wheel_D.V_real) / 4.0f;
  R_Vel.Y = (-R_Wheel_A.V_real + R_Wheel_B.V_real + R_Wheel_C.V_real - R_Wheel_D.V_real) / 4.0f;
  R_Vel.W = (-R_Wheel_A.V_real + R_Wheel_B.V_real - R_Wheel_C.V_real + R_Wheel_D.V_real) / (4.0f * wheel_lever_arm);

  R_Pos.W_body += R_Vel.W * PERIOD_20MS;
  if (R_Pos.W_body > (float)PI) R_Pos.W_body -= 2.0f * (float)PI;
  if (R_Pos.W_body < -(float)PI) R_Pos.W_body += 2.0f * (float)PI;

  dx_body = R_Vel.X * PERIOD_20MS;
  dy_body = R_Vel.Y * PERIOD_20MS;

  // R_Pos.W_global = R_Pos.W_body;
  R_Pos.W_global = DEG_TO_RAD(imu.yaw_total);
  
  dx_world = dx_body * cosf(R_Pos.W_global) - dy_body * sinf(R_Pos.W_global);
  dy_world = dx_body * sinf(R_Pos.W_global) + dy_body * cosf(R_Pos.W_global);

  // 更新全局位置
  R_Pos.X_global += dx_world;
  R_Pos.Y_global += dy_world;
//printf("%f, %f, %f, %f, %f, %f\r\n",R_Pos.X_global,R_Pos.X_global_target,R_Pos.Y_global,R_Pos.Y_global_target,R_Pos.W_global,R_Pos.W_global_target);
}

//全局位置转换为车体坐标系下的位置误差
void Global__PositionTransform(void)
{
	float dx, dy;
  // 计算全局坐标系下的位置误差
  dx = R_Pos.X_global_target - R_Pos.X_global;
  dy = R_Pos.Y_global_target - R_Pos.Y_global;
  //printf("R_Pos.X_global_target=%f, R_Pos.X_global=%f, R_Pos.Y_global_target=%f, R_Pos.Y_global=%f\r\n",R_Pos.X_global_target,R_Pos.X_global,R_Pos.Y_global_target,R_Pos.Y_global);

  // 转换到车体系（得到车体系下的位置误差）
  R_Pos.X_body_error = dx * cosf(R_Pos.W_global) + dy * sinf(R_Pos.W_global);      // 车体系X方向误差
  R_Pos.Y_body_error = -dx * sinf(R_Pos.W_global) + dy * cosf(R_Pos.W_global);     // 车体系Y方向误差
	R_Pos.W_body_error = R_Pos.W_global_target - R_Pos.W_global;      // 航向角误差
	if(R_Pos.W_body_error > (float)PI) R_Pos.W_body_error -= 2.0f * (float)PI;
	if(R_Pos.W_body_error < -(float)PI) R_Pos.W_body_error += 2.0f * (float)PI;
	//printf("R_Pos.X_body_error=%f, R_Pos.Y_body_error=%f, R_Pos.W_body_error=%f\r\n",R_Pos.X_body_error,R_Pos.Y_body_error,R_Pos.W_body_error);
}

void chassis_control_process(uint8_t run_enable)
{
  // R_Wheel_A.V_real = (float)motor_encoder.front_left;
  // R_Wheel_B.V_real = (float)motor_encoder.front_right;
  // R_Wheel_C.V_real = (float)motor_encoder.rear_left;
  // R_Wheel_D.V_real = (float)motor_encoder.rear_right;

  R_Wheel_A.V_real = (float)motor_encoder.front_left;
  R_Wheel_B.V_real = (float)motor_encoder.front_right;
  R_Wheel_C.V_real = (float)motor_encoder.rear_left;
  R_Wheel_D.V_real = (float)motor_encoder.rear_right;

  // Global_PositionUpdate();
  // Global__PositionTransform();

  if (chassis_handle_emergency_stop(run_enable) != 0U)
  {
    return;
  }

  /* Position loop: convert body-frame position errors into velocity targets. */
  // R_Vel.X_target = PidPosCtl(R_Pos.X_body_error, 0.0f, &car_position_pid[0]);
  // R_Vel.Y_target = PidPosCtl(R_Pos.Y_body_error, 0.0f, &car_position_pid[1]);
  // R_Vel.W_target = PidPosCtl(R_Pos.W_body_error, 0.0f, &car_position_pid[2]);

  R_Vel.X_target = clamp_float(R_Vel.X_target, -R_Vel.X_lim * R_VX_LIMIT, R_Vel.X_lim * R_VX_LIMIT);
  R_Vel.Y_target = clamp_float(R_Vel.Y_target, -R_Vel.Y_lim * R_VY_LIMIT, R_Vel.Y_lim * R_VY_LIMIT);
  R_Vel.W_target = clamp_float(R_Vel.W_target, -R_Vel.W_lim * (R_VW_LIMIT / 1000.0f), R_Vel.W_lim * (R_VW_LIMIT / 1000.0f));

    //运动学解析，由机器人目标速度计算电机轮子速度（mm/s)
  R_Wheel_A.V_target = R_Vel.X_target - R_Vel.Y_target - R_Vel.W_target*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
  R_Wheel_B.V_target = R_Vel.X_target + R_Vel.Y_target + R_Vel.W_target*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
  R_Wheel_C.V_target = R_Vel.X_target + R_Vel.Y_target - R_Vel.W_target*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
  R_Wheel_D.V_target = R_Vel.X_target - R_Vel.Y_target + R_Vel.W_target*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);

  R_Wheel_A.PWM = PidIncCtl(R_Wheel_A.V_target, R_Wheel_A.V_real, &motor_speed_pid[0]);
  R_Wheel_B.PWM = PidIncCtl(R_Wheel_B.V_target, R_Wheel_B.V_real, &motor_speed_pid[1]);
  R_Wheel_C.PWM = PidIncCtl(R_Wheel_C.V_target, R_Wheel_C.V_real, &motor_speed_pid[2]);
  R_Wheel_D.PWM = PidIncCtl(R_Wheel_D.V_target, R_Wheel_D.V_real, &motor_speed_pid[3]);

  R_Wheel_A.PWM = clamp_float(R_Wheel_A.PWM, -R_VX_LIMIT, R_VX_LIMIT);
  R_Wheel_B.PWM = clamp_float(R_Wheel_B.PWM, -R_VX_LIMIT, R_VX_LIMIT);
  R_Wheel_C.PWM = clamp_float(R_Wheel_C.PWM, -R_VX_LIMIT, R_VX_LIMIT);
  R_Wheel_D.PWM = clamp_float(R_Wheel_D.PWM, -R_VX_LIMIT, R_VX_LIMIT);

  motor_set_chassis_speed_mm_s(MOTOR_ID_FRONT_LEFT,  (R_Wheel_A.PWM >= -MOTOR_PWM_ZERO_DEADBAND && R_Wheel_A.PWM <= MOTOR_PWM_ZERO_DEADBAND) ? 0 : (int32_t)R_Wheel_A.PWM);
  motor_set_chassis_speed_mm_s(MOTOR_ID_FRONT_RIGHT, (R_Wheel_B.PWM >= -MOTOR_PWM_ZERO_DEADBAND && R_Wheel_B.PWM <= MOTOR_PWM_ZERO_DEADBAND) ? 0 : (int32_t)R_Wheel_B.PWM);
  motor_set_chassis_speed_mm_s(MOTOR_ID_REAR_LEFT,   (R_Wheel_C.PWM >= -MOTOR_PWM_ZERO_DEADBAND && R_Wheel_C.PWM <= MOTOR_PWM_ZERO_DEADBAND) ? 0 : (int32_t)R_Wheel_C.PWM);
  motor_set_chassis_speed_mm_s(MOTOR_ID_REAR_RIGHT,  (R_Wheel_D.PWM >= -MOTOR_PWM_ZERO_DEADBAND && R_Wheel_D.PWM <= MOTOR_PWM_ZERO_DEADBAND) ? 0 : (int32_t)R_Wheel_D.PWM);

          // printf("%f,%f,%f,%f\r\n", R_Vel.X_target, R_Vel.Y_target, R_Vel.W_target, R_Wheel_D.V_target);

}
