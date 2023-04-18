#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "adc.h"
#include "ads1115.h"
#include "esp8266.h"
#include "onenet.h"
#include "mqttkit.h"

int heart_rate;				 // 心率
float temperature;			 // 体温
u16 adcx2;					 // 烟雾浓度原始值
float smoke;				 // 烟雾浓度
float pitch, roll, yaw;		 // 欧拉角
short accx, accy, accz;		 // 加速度原始数据
short gyrox, gyroy, gyroz;	 // 陀螺仪原始数据
u8 mpu_errflag;				 // mpu6050错误标志
u8 fall_flag = 0;			 // 人体跌倒标志位 标志位1、2任意一个异常，该标志位为1。【1：跌倒；0：正常】
_Bool mpu_angle_errflag = 0; // 标志位1：mpu6050角度异常标志位【1：异常；0：正常】
_Bool mpu_acc_errflag = 0;	 // 标志位2：mpu6050加速度异常标志位【1：异常；0：正常】
int SVM;					 // 人体加速度向量幅值
char display[20];			 // 显示用。用来存放待显示的心率、体温、跌倒标志、烟雾浓度
u8 i = 3, t = 0;

int main(void)
{
	// char PUB_BUF[256];//上传数据的buf
	// const char *topics[] = {"mysmart/sub"};
	unsigned short timeCount = 0; // 发送间隔变量
	unsigned char *dataPtr = NULL;
	u16 adcx1;	// 体温原始值
	float temp; // 暂时存放体温

	delay_init();		  // 延时函数初始化
	NVIC_Configuration(); // 设置NVIC中断分组4:4位抢占优先级，0位响应优先级
	Usart1_Init(115200);  // 串口初始化为9600
	Usart2_Init(115200);  // 8266通讯串口
	ads1115_Init();		  // ADS1115初始化
	Adc_Init();			  // ADC初始化

	ESP8266_Init();			 // 8266初始化
	while (OneNet_DevLink()) // 接入onenet
		delay_ms(500);

	// OneNet_Subscribe(topics, 1);

	while (1)
	{
		// 采集体温
		adcx1 = GetAds1115Values(); // 采集ADC数据
		temp = (float)adcx1 * 0.125f;
		temperature = (-0.0000084515f) * temp * temp + (-0.176928f) * temp + 203.393f;

		if (++timeCount >= 40) // 发送间隔1s，也就是5s上传一次数据，1000/25=40
		{
			UsartPrintf(USART_DEBUG, "OneNet_SendData\r\n");
			OneNet_SendData();

			timeCount = 0;
			ESP8266_Clear();
		}

		dataPtr = ESP8266_GetIPD(3); // 完成需要15个毫秒，三次循环，一次5个毫秒
		if (dataPtr != NULL)
			OneNet_RevPro(dataPtr);
		delay_ms(10);
	}
}
