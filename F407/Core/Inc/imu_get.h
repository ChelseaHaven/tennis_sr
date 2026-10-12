#ifndef _IMU_GET_H_
#define _IMU_GET_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "usart.h"

#define REGSIZE 0x90

#define WIT_HAL_OK      (0)     /**< There is no error */
#define WIT_HAL_BUSY    (-1)    /**< Busy */
#define WIT_HAL_TIMEOUT (-2)    /**< Timed out */
#define WIT_HAL_ERROR   (-3)    /**< A generic error happens */
#define WIT_HAL_NOMEM   (-4)    /**< No memory */
#define WIT_HAL_EMPTY   (-5)    /**< The resource is empty */
#define WIT_HAL_INVAL   (-6)    /**< Invalid argument */

#define WIT_DATA_BUFF_SIZE  256

#define WIT_PROTOCOL_NORMAL 0
#define WIT_PROTOCOL_MODBUS 1
#define WIT_PROTOCOL_CAN    2
#define WIT_PROTOCOL_I2C    3


/* serial function */
typedef void (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);
/**
 * @brief 注册 WIT 驱动使用的串口发送回调。
 * @param write_func 串口发送函数。
 * @return 注册成功返回 WIT_HAL_OK，回调为空时返回 WIT_HAL_INVAL。
 */
int32_t WitSerialWriteRegister(SerialWrite write_func);
/**
 * @brief 将串口收到的一个字节送入 WIT 协议解析器。
 * @param ucData 串口接收字节。
 */
void WitSerialDataIn(uint8_t ucData);

/* iic function */

/*
    i2c write function example

    int32_t WitI2cWrite(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
    {
        i2c_start();
        i2c_send(ucAddr);
        if(i2c_wait_ask() != SUCCESS)return 0;
        i2c_send(ucReg);
        if(i2c_wait_ask() != SUCCESS)return 0;
        for(uint32_t i = 0; i < uiLen; i++)
        {
            i2c_send(*p_ucVal++); 
            if(i2c_wait_ask() != SUCCESS)return 0;
        }
        i2c_stop();
        return 1;
    }
*/
typedef int32_t (*WitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
/*
    i2c read function example

    int32_t WitI2cRead(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen)
    {
        i2c_start();
        i2c_send(ucAddr);
        if(i2c_wait_ask() != SUCCESS)return 0;
        i2c_send(ucReg);
        if(i2c_wait_ask() != SUCCESS)return 0;
        
        i2c_start();
        i2c_send(ucAddr+1);
        for(uint32_t i = 0; i < uiLen; i++)
        {
            if(i+1 == uiLen)*p_ucVal++ = i2c_read(0);  //last byte no ask
            else *p_ucVal++ = i2c_read(1);  //  ask
        }
        i2c_stop();
        return 1;
    }
*/
typedef int32_t (*WitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
/**
 * @brief 注册 WIT 驱动使用的 I2C 读写回调。
 * @param write_func I2C 写函数。
 * @param read_func I2C 读函数。
 * @return 注册成功返回 WIT_HAL_OK；任一回调为空时返回 WIT_HAL_INVAL。
 */
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func);

/* can function */
typedef void (*CanWrite)(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);
/**
 * @brief 注册 WIT 驱动使用的 CAN 发送回调。
 * @param write_func CAN 发送函数。
 * @return 注册成功返回 WIT_HAL_OK，回调为空时返回 WIT_HAL_INVAL。
 */
int32_t WitCanWriteRegister(CanWrite write_func);

/* Delayms function */
typedef void (*DelaymsCb)(uint16_t ucMs);
/**
 * @brief 注册 WIT 驱动使用的毫秒延时回调。
 * @param delayms_func 延时函数。
 * @return 注册成功返回 WIT_HAL_OK，回调为空时返回 WIT_HAL_INVAL。
 */
int32_t WitDelayMsRegister(DelaymsCb delayms_func);


/**
 * @brief 将一帧 CAN 数据送入 WIT CAN 协议解析器。
 * @param ucData CAN 数据缓冲区，至少包含 8 字节。
 * @param ucLen CAN 数据长度。
 */
void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen);


typedef void (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);
/**
 * @brief 注册寄存器数据更新回调。
 * @param update_func 数据更新时调用的回调函数。
 * @return 注册成功返回 WIT_HAL_OK，回调为空时返回 WIT_HAL_INVAL。
 */
int32_t WitRegisterCallBack(RegUpdateCb update_func);
/**
 * @brief 通过当前配置的通信接口写入一个传感器寄存器。
 * @param uiReg 寄存器地址。
 * @param usData 要写入的 16 位数值。
 * @return WIT_HAL_OK 表示成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData);
/**
 * @brief 通过当前配置的通信接口请求连续寄存器数据。
 * @param uiReg 起始寄存器地址。
 * @param uiReadNum 连续读取的寄存器数量。
 * @return WIT_HAL_OK 表示请求成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);
/**
 * @brief 初始化 WIT 驱动通信协议和设备地址。
 * @param uiProtocol 通信协议，使用 WIT_PROTOCOL_* 定义。
 * @param ucAddr 传感器通信地址。
 * @return 初始化成功返回 WIT_HAL_OK，协议无效时返回 WIT_HAL_INVAL。
 */
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);
/**
 * @brief 清除 WIT 驱动中已注册的回调和通信配置。
 */
void WitDeInit(void);



/**
  ******************************************************************************
  * @file    wit_c_sdk.h
  * @author  Wit
  * @version V1.0
  * @date    05-May-2022
  * @brief   This file provides all Configure sensor function.
  ******************************************************************************
  * @attention
  *
  *        http://wit-motion.cn/
  *
  ******************************************************************************
  */
/**
 * @brief 启动加速度计校准。
 * @return WIT_HAL_OK 表示命令发送成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitStartAccCali(void);
/**
 * @brief 结束加速度计校准并保存校准结果。
 * @return WIT_HAL_OK 表示命令发送成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitStopAccCali(void);
/**
 * @brief 启动磁场校准。
 * @return WIT_HAL_OK 表示命令发送成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitStartMagCali(void);
/**
 * @brief 结束磁场校准。
 * @return WIT_HAL_OK 表示命令发送成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitStopMagCali(void);
/**
 * @brief 设置传感器 UART 波特率。
 * @param uiBaudIndex WIT_BAUD_* 波特率枚举值。
 * @return WIT_HAL_OK 表示命令发送成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitSetUartBaud(int32_t uiBaudIndex);
/**
 * @brief 设置传感器带宽。
 * @param uiBaudWidth BANDWIDTH_* 带宽枚举值。
 * @return WIT_HAL_OK 表示命令发送成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitSetBandwidth(int32_t uiBaudWidth);
/**
 * @brief 设置传感器输出数据频率。
 * @param uiRate RRATE_* 输出频率枚举值。
 * @return WIT_HAL_OK 表示命令发送成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitSetOutputRate(int32_t uiRate);
/**
 * @brief 设置传感器输出内容。
 * @param uiRsw RSW_* 输出内容配置值。
 * @return WIT_HAL_OK 表示命令发送成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitSetContent(int32_t uiRsw);
/**
 * @brief 设置传感器 CAN 波特率。
 * @param uiBaudIndex CAN_BAUD_* 波特率枚举值。
 * @return WIT_HAL_OK 表示命令发送成功；否则返回 WIT_HAL_* 错误码。
 */
int32_t WitSetCanBaud(int32_t uiBaudIndex);

/**
 * @brief 检查数值是否位于闭区间范围内。
 * @param sTemp 待检查数值。
 * @param sMin 区间下限。
 * @param sMax 区间上限。
 * @return 位于范围内返回 1，否则返回 0。
 */
char CheckRange(short sTemp,short sMin,short sMax);

extern int16_t sReg[REGSIZE];

#ifdef __cplusplus
}
#endif

#endif
