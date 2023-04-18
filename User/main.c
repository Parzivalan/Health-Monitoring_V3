#include "delay.h"
#include "sys.h"
#include "math.h"
// #include "string.h"
#include "stdio.h"
#include "SEGGER_RTT.h"
#include "usart.h"
#include "timer.h"
#include "adc.h"
#include "ads1115.h"
#include "oled.h"
#include "led.h"
#include "beep.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
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

void fall()
{

	MPU_Get_Accelerometer(&accx, &accy, &accz); // 得到加速度传感器数据
	SVM = sqrt(pow(accx, 2) + pow(accy, 2) + pow(accz, 2));
	SEGGER_RTT_printf(0, "pitch:%0.1f   roll:%0.1f   yaw:%0.1f   SVM:%u\r\n",fabs(pitch),fabs(roll),fabs(yaw),SVM);

	// 分析x、y、z角度的异常判断
	if (fabs(pitch) > 40 || fabs(roll) > 40 || fabs(yaw) > 40) // 倾斜角度的“绝对值”大于70°，SVM大于设定的阈值时，即认为摔倒
		mpu_angle_errflag = 1;
	else
		mpu_angle_errflag = 0;

	// 分析加速度SVM的异常判断
	if (SVM > 22000 || SVM < 12000)
	{
		i = 0;
	}
	i++;

	if (i <= 3)
	{
		mpu_acc_errflag = 1;
	}
	else
	{
		i = 3;
		mpu_acc_errflag = 0;
	}

	// 综合欧拉角、SVM异常判断异常
	if (mpu_acc_errflag || mpu_angle_errflag)
	{
		fall_flag = 1;
	}
	else
	{
		fall_flag = 0;
	}

	BEEP = 0;
	delay_ms(300); // 延时300ms
	if (fall_flag == 1)
	{
		BEEP = 1;
		/* LED_GREEN = 0;
		LED_RED = 1; */
		delay_ms(300); // 延时300ms
	}
	BEEP = 0;
	delay_ms(300);
	/* LED_RED = 0;
	LED_GREEN = 1; */
}

int main(void)
{
	// char PUB_BUF[256];//上传数据的buf
	// const char *topics[] = {"mysmart/sub"};
	unsigned short timeCount = 0; // 发送间隔变量
	unsigned char *dataPtr = NULL;
	u16 adcx1;	// 体温原始值
	float temp; // 暂时存放体温
	u8 j = 0;	// Welcome显示计数用

	delay_init();		  // 延时函数初始化
	NVIC_Configuration(); // 设置NVIC中断分组4:4位抢占优先级，0位响应优先级
	Usart1_Init(115200);  // 串口初始化为9600
	Usart2_Init(115200);  // 8266通讯串口
	OLED_Init();		  // OLED初始化
	BEEP_Init();		  // 蜂鸣器初始化
	LED_Init();			  // LED初始化
	MPU_Init();		  // MPU6050初始化
	ads1115_Init();		  // ADS1115初始化
	Adc_Init();			  // ADC初始化
	Time4_Int_Init();	  // 定时器4初始化

	// OneNet_Subscribe(topics, 1);

	/***********************Welcome!居家健康监测系统********************************/
	OLED_Clear();
	OLED_ShowString(30, 1, (u8 *)"Welcome!", 8);
	for (j = 0; j < 8; j++)
	{
		OLED_Show_Welcome(j * 16, 3, j);
	}
	OLED_ShowString(6, 7, (u8 *)"Based on OneNET", 14);
	delay_ms(1200);
	OLED_Clear();

	/***********************网络初始化********************************/
	OLED_ShowString(0, 1, (u8 *)"The network is connecting, please wait...", 50);
	ESP8266_Init();			 // 8266初始化
	while (OneNet_DevLink()) // 接入onenet
		delay_ms(500);

	while (mpu_dmp_init())
	{
		static u8 cnt = 20;

		OLED_Clear();
		OLED_ShowString(30, 3, (u8 *)"MPU ERROR!", 12);
		delay_ms(200);
		OLED_ShowString(30, 0, (u8 *)"           ", 12);
		delay_ms(200);
		if (!(cnt--))
		{
			mpu_errflag = 1;
			break;
		}
	}

	OLED_Clear();

	while (1)
	{
		// 采集体温
		adcx1 = GetAds1115Values(); // 采集ADC数据
		temp = (float)adcx1 * 0.125f;
		temperature = (-0.0000084515f) * temp * temp + (-0.176928f) * temp + 203.393f;

		// 显示体温、心率、跌倒标志、烟雾浓度
		OLED_ShowString(0, 7, (u8 *)display, 12); 

		// 检测跌倒
		fall(); 

		//数据上传OneNET
		if (++timeCount >= 40) // 发送间隔1s，也就是5s上传一次数据，1000/25=40
		{
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

void TIM4_IRQHandler(void) // TIM4中断
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) // 检查TIM4更新中断发生与否
	{

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		// unsigned char *dataPtr = NULL;
		/* adcx2 = Get_Adc_Average(ADC_Channel_1, 10);

		temp2 = (float)adcx2 * (3.3 / 4096);
		sprintf(display, "%d %.1f %.2lf %d", rx_data.Heart_rate, tem + 0.05f, temp2, mpu_flag);
		// 采集烟雾数值
		if (temp2 >= 0.75)
		{
			BEEP = 1;
		} */

		sprintf(display, "%.1f", temperature + 0.05f); // sprintf()在stdio.h中定义，用于格式化输出

		/* OneNet_SendData();
		ESP8266_Clear();
		dataPtr = ESP8266_GetIPD(3);
		if (dataPtr != NULL)
			OneNet_RevPro(dataPtr); */
	}
}
