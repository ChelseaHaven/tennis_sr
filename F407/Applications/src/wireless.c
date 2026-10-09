#include "wireless.h"

uint8_t wireless_data[51];
float wireless_test[10]={0};
State_flag state;

void wireless_read(void){
	if(state.wireless){
		char command_flag1=0,command_flag2=0;
		float num1=0,num2=0,num3=0;
		int parsed;
		state.wireless=0;
		parsed = sscanf((const char*)wireless_data, "%c,%c,%f,%f,%f",
		                &command_flag1, &command_flag2, &num1, &num2, &num3);
		if (parsed >= 3)
		{
			switch(command_flag1){
				case 'A':
					switch(command_flag2){
						case 'B': wireless_test[0]=num1; wireless_test[1]=num2; wireless_test[2]=num3; break;	//速度位置之类的
						case 'C': wireless_test[3]=num1; break;	//切断电机电流急停
						case 'D': wireless_test[4]=num1; wireless_test[5]=num2; break;
						case 'E': wireless_test[6]=num1; wireless_test[7]=num2; wireless_test[8]=num3; break;	//pid速度环
						case 'F': wireless_test[6]=num1; wireless_test[7]=num2; wireless_test[8]=num3; break;	//pid位置环
					}
					break;
				default:
					break;
			}
		}
		(void)HAL_UARTEx_ReceiveToIdle_DMA(&huart3, wireless_data,
		                                   (uint16_t)(sizeof(wireless_data) - 1U));
		__HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
	}
}
