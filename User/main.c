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
#include "algorithm.h"
#include "max30102.h"
#include "myiic.h"
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

uint32_t aun_ir_buffer[500];  // IR LED sensor data
int32_t n_ir_buffer_length;	  // data length
uint32_t aun_red_buffer[500]; // Red LED sensor data
int32_t n_sp02;				  // SPO2 value
int8_t ch_spo2_valid;		  // indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;		  // heart rate value
int8_t ch_hr_valid;			  // indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
u8 dis_hr = 0, dis_spo2 = 0;

#define MAX_BRIGHTNESS 255

void dis_DrawCurve(u32 *data, u8 x);

void fall()
{

	MPU_Get_Accelerometer(&accx, &accy, &accz); // �õ����ٶȴ���������
	SVM = sqrt(pow(accx, 2) + pow(accy, 2) + pow(accz, 2));
	SEGGER_RTT_printf(0, "pitch:%0.1f   roll:%0.1f   yaw:%0.1f   SVM:%u\r\n", fabs(pitch), fabs(roll), fabs(yaw), SVM);

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
	u16 adcx1;	 // ����ԭʼֵ
	float tem_t; // ��ʱ�������
	u8 j = 0;	 // Welcome��ʾ������

	// variables to calculate the on-board LED brightness that reflects the heartbeats
	uint32_t un_min, un_max, un_prev_data;
	int i;
	int32_t n_brightness;
	float f_temp;
	u8 temp_num = 0;
	u8 temp[6];
	u8 str[100];

	delay_init();		  // ��ʱ������ʼ��
	NVIC_Configuration(); // ����NVIC�жϷ���4:4λ��ռ���ȼ���0λ��Ӧ���ȼ�
	Usart1_Init(115200);  // ���ڳ�ʼ��Ϊ9600
	Usart2_Init(115200);  // 8266ͨѶ����
	OLED_Init();		  // OLED��ʼ��
	BEEP_Init();		  // ��������ʼ��
	LED_Init();			  // LED��ʼ��
	MPU_Init();			  // MPU6050��ʼ��
	max30102_init();	  // MAX30102��ʼ��
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

	un_min = 0x3FFFF;
	un_max = 0;

	n_ir_buffer_length = 500; // buffer length of 100 stores 5 seconds of samples running at 100sps
	// read the first 500 samples, and determine the signal range
	for (i = 0; i < n_ir_buffer_length; i++)
	{
		while (MAX30102_INT == 1)
			; // wait until the interrupt pin asserts

		max30102_FIFO_ReadBytes(REG_FIFO_DATA, temp);
		aun_red_buffer[i] = (long)((long)((long)temp[0] & 0x03) << 16) | (long)temp[1] << 8 | (long)temp[2]; // Combine values to get the actual number
		aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03) << 16) | (long)temp[4] << 8 | (long)temp[5];	 // Combine values to get the actual number

		if (un_min > aun_red_buffer[i])
			un_min = aun_red_buffer[i]; // update signal min
		if (un_max < aun_red_buffer[i])
			un_max = aun_red_buffer[i]; // update signal max
	}
	un_prev_data = aun_red_buffer[i];
	// calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);

	while (1)
	{
		// �ɼ�����
		adcx1 = GetAds1115Values(); // �ɼ�ADC����
		tem_t = (float)adcx1 * 0.125f;
		temperature = (-0.0000084515f) * tem_t * tem_t + (-0.176928f) * tem_t + 203.393f;

		i = 0;
		un_min = 0x3FFFF;
		un_max = 0;

		// OLED_showFrame(); // ��ʾ����
		OLED_ShowString(0, 7, (u8 *)display, 12);

		// dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
		for (i = 100; i < 500; i++)
		{
			aun_red_buffer[i - 100] = aun_red_buffer[i];
			aun_ir_buffer[i - 100] = aun_ir_buffer[i];

			// update the signal min and max
			if (un_min > aun_red_buffer[i])
				un_min = aun_red_buffer[i];
			if (un_max < aun_red_buffer[i])
				un_max = aun_red_buffer[i];
		}
		// take 100 sets of samples before calculating the heart rate.
		for (i = 400; i < 500; i++)
		{
			un_prev_data = aun_red_buffer[i - 1];
			while (MAX30102_INT == 1)
				;
			max30102_FIFO_ReadBytes(REG_FIFO_DATA, temp);
			aun_red_buffer[i] = (long)((long)((long)temp[0] & 0x03) << 16) | (long)temp[1] << 8 | (long)temp[2]; // Combine values to get the actual number
			aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03) << 16) | (long)temp[4] << 8 | (long)temp[5];	 // Combine values to get the actual number

			if (aun_red_buffer[i] > un_prev_data)
			{
				f_temp = aun_red_buffer[i] - un_prev_data;
				f_temp /= (un_max - un_min);
				f_temp *= MAX_BRIGHTNESS;
				n_brightness -= (int)f_temp;
				if (n_brightness < 0)
					n_brightness = 0;
			}
			else
			{
				f_temp = un_prev_data - aun_red_buffer[i];
				f_temp /= (un_max - un_min);
				f_temp *= MAX_BRIGHTNESS;
				n_brightness += (int)f_temp;
				if (n_brightness > MAX_BRIGHTNESS)
					n_brightness = MAX_BRIGHTNESS;
			}
			// send samples and calculation result to terminal program through UART
			if (ch_hr_valid == 1 && n_heart_rate < 120) //**/ ch_hr_valid == 1 && ch_spo2_valid ==1 && n_heart_rate<120 && n_sp02<101
			{
				dis_hr = n_heart_rate;
				dis_spo2 = n_sp02;
			}
			else
			{
				dis_hr = 0;
				dis_spo2 = 0;
			}
		}

		OLED_Fill(0, 23, 127, 63, 0);
		// ������ϣ���������
		dis_DrawCurve(aun_red_buffer, 20);
		dis_DrawCurve(aun_ir_buffer, 0);
		OLED_Refresh_Gram(); // ������ʾ��OLED

		// ������
		fall();

		// �����ϴ�OneNET
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

void dis_DrawCurve(u32 *data, u8 x)
{
	u16 i;
	u32 max = 0, min = 262144;
	u32 temp;
	u32 compress;

	for (i = 0; i < 128 * 2; i++)
	{
		if (data[i] > max)
		{
			max = data[i];
		}
		if (data[i] < min)
		{
			min = data[i];
		}
	}

	compress = (max - min) / 20;

	for (i = 0; i < 128; i++)
	{
		temp = data[i * 2] + data[i * 2 + 1];
		temp /= 2;
		temp -= min;
		temp /= compress;
		if (temp > 20)
			temp = 20;
		OLED_DrawPoint(i, 63 - x - temp, 1);
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

		adcx2 = Get_Adc_Average(ADC_Channel_1, 10);								// ADCģ���ź�ֵ
		smoke = adcx2 * (3.3 / 4096);											// ת��Ϊ��ѹֵ
		sprintf(display, "%.1f %d %.2lf ", temperature + 0.05f, dis_hr, smoke); // sprintf()��stdio.h�ж��壬���ڸ�ʽ�����

		/* OneNet_SendData();
		ESP8266_Clear();
		dataPtr = ESP8266_GetIPD(3);
		if (dataPtr != NULL)
			OneNet_RevPro(dataPtr); */
	}
}
