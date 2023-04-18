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

int heart_rate;				 // ����
float temperature;			 // ����
u16 adcx2;					 // ����Ũ��ԭʼֵ
float smoke;				 // ����Ũ��
float pitch, roll, yaw;		 // ŷ����
short accx, accy, accz;		 // ���ٶ�ԭʼ����
short gyrox, gyroy, gyroz;	 // ������ԭʼ����
u8 mpu_errflag;				 // mpu6050�����־
u8 fall_flag = 0;			 // ���������־λ ��־λ1��2����һ���쳣���ñ�־λΪ1����1��������0��������
_Bool mpu_angle_errflag = 0; // ��־λ1��mpu6050�Ƕ��쳣��־λ��1���쳣��0��������
_Bool mpu_acc_errflag = 0;	 // ��־λ2��mpu6050���ٶ��쳣��־λ��1���쳣��0��������
int SVM;					 // ������ٶ�������ֵ
char display[20];			 // ��ʾ�á�������Ŵ���ʾ�����ʡ����¡�������־������Ũ��
u8 i = 3, t = 0;

void fall()
{

	MPU_Get_Accelerometer(&accx, &accy, &accz); // �õ����ٶȴ���������
	SVM = sqrt(pow(accx, 2) + pow(accy, 2) + pow(accz, 2));
	SEGGER_RTT_printf(0, "pitch:%0.1f   roll:%0.1f   yaw:%0.1f   SVM:%u\r\n",fabs(pitch),fabs(roll),fabs(yaw),SVM);

	// ����x��y��z�Ƕȵ��쳣�ж�
	if (fabs(pitch) > 40 || fabs(roll) > 40 || fabs(yaw) > 40) // ��б�Ƕȵġ�����ֵ������70�㣬SVM�����趨����ֵʱ������Ϊˤ��
		mpu_angle_errflag = 1;
	else
		mpu_angle_errflag = 0;

	// �������ٶ�SVM���쳣�ж�
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

	// �ۺ�ŷ���ǡ�SVM�쳣�ж��쳣
	if (mpu_acc_errflag || mpu_angle_errflag)
	{
		fall_flag = 1;
	}
	else
	{
		fall_flag = 0;
	}

	BEEP = 0;
	delay_ms(300); // ��ʱ300ms
	if (fall_flag == 1)
	{
		BEEP = 1;
		/* LED_GREEN = 0;
		LED_RED = 1; */
		delay_ms(300); // ��ʱ300ms
	}
	BEEP = 0;
	delay_ms(300);
	/* LED_RED = 0;
	LED_GREEN = 1; */
}

int main(void)
{
	// char PUB_BUF[256];//�ϴ����ݵ�buf
	// const char *topics[] = {"mysmart/sub"};
	unsigned short timeCount = 0; // ���ͼ������
	unsigned char *dataPtr = NULL;
	u16 adcx1;	// ����ԭʼֵ
	float temp; // ��ʱ�������
	u8 j = 0;	// Welcome��ʾ������

	delay_init();		  // ��ʱ������ʼ��
	NVIC_Configuration(); // ����NVIC�жϷ���4:4λ��ռ���ȼ���0λ��Ӧ���ȼ�
	Usart1_Init(115200);  // ���ڳ�ʼ��Ϊ9600
	Usart2_Init(115200);  // 8266ͨѶ����
	OLED_Init();		  // OLED��ʼ��
	BEEP_Init();		  // ��������ʼ��
	LED_Init();			  // LED��ʼ��
	MPU_Init();		  // MPU6050��ʼ��
	ads1115_Init();		  // ADS1115��ʼ��
	Adc_Init();			  // ADC��ʼ��
	Time4_Int_Init();	  // ��ʱ��4��ʼ��

	// OneNet_Subscribe(topics, 1);

	/***********************Welcome!�Ӽҽ������ϵͳ********************************/
	OLED_Clear();
	OLED_ShowString(30, 1, (u8 *)"Welcome!", 8);
	for (j = 0; j < 8; j++)
	{
		OLED_Show_Welcome(j * 16, 3, j);
	}
	OLED_ShowString(6, 7, (u8 *)"Based on OneNET", 14);
	delay_ms(1200);
	OLED_Clear();

	/***********************�����ʼ��********************************/
	OLED_ShowString(0, 1, (u8 *)"The network is connecting, please wait...", 50);
	ESP8266_Init();			 // 8266��ʼ��
	while (OneNet_DevLink()) // ����onenet
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
		// �ɼ�����
		adcx1 = GetAds1115Values(); // �ɼ�ADC����
		temp = (float)adcx1 * 0.125f;
		temperature = (-0.0000084515f) * temp * temp + (-0.176928f) * temp + 203.393f;

		// ��ʾ���¡����ʡ�������־������Ũ��
		OLED_ShowString(0, 7, (u8 *)display, 12); 

		// ������
		fall(); 

		//�����ϴ�OneNET
		if (++timeCount >= 40) // ���ͼ��1s��Ҳ����5s�ϴ�һ�����ݣ�1000/25=40
		{
			OneNet_SendData();

			timeCount = 0;
			ESP8266_Clear();
		}

		dataPtr = ESP8266_GetIPD(3); // �����Ҫ15�����룬����ѭ����һ��5������
		if (dataPtr != NULL)
			OneNet_RevPro(dataPtr);
		delay_ms(10);
	}
}

void TIM4_IRQHandler(void) // TIM4�ж�
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) // ���TIM4�����жϷ������
	{

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		// unsigned char *dataPtr = NULL;
		/* adcx2 = Get_Adc_Average(ADC_Channel_1, 10);

		temp2 = (float)adcx2 * (3.3 / 4096);
		sprintf(display, "%d %.1f %.2lf %d", rx_data.Heart_rate, tem + 0.05f, temp2, mpu_flag);
		// �ɼ�������ֵ
		if (temp2 >= 0.75)
		{
			BEEP = 1;
		} */

		sprintf(display, "%.1f", temperature + 0.05f); // sprintf()��stdio.h�ж��壬���ڸ�ʽ�����

		/* OneNet_SendData();
		ESP8266_Clear();
		dataPtr = ESP8266_GetIPD(3);
		if (dataPtr != NULL)
			OneNet_RevPro(dataPtr); */
	}
}
