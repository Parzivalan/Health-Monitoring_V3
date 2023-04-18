#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "adc.h"
#include "ads1115.h"
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

int main(void)
{
	// char PUB_BUF[256];//�ϴ����ݵ�buf
	// const char *topics[] = {"mysmart/sub"};
	unsigned short timeCount = 0; // ���ͼ������
	unsigned char *dataPtr = NULL;
	u16 adcx1;	// ����ԭʼֵ
	float temp; // ��ʱ�������

	delay_init();		  // ��ʱ������ʼ��
	NVIC_Configuration(); // ����NVIC�жϷ���4:4λ��ռ���ȼ���0λ��Ӧ���ȼ�
	Usart1_Init(115200);  // ���ڳ�ʼ��Ϊ9600
	Usart2_Init(115200);  // 8266ͨѶ����
	ads1115_Init();		  // ADS1115��ʼ��
	Adc_Init();			  // ADC��ʼ��

	ESP8266_Init();			 // 8266��ʼ��
	while (OneNet_DevLink()) // ����onenet
		delay_ms(500);

	// OneNet_Subscribe(topics, 1);

	while (1)
	{
		// �ɼ�����
		adcx1 = GetAds1115Values(); // �ɼ�ADC����
		temp = (float)adcx1 * 0.125f;
		temperature = (-0.0000084515f) * temp * temp + (-0.176928f) * temp + 203.393f;

		if (++timeCount >= 40) // ���ͼ��1s��Ҳ����5s�ϴ�һ�����ݣ�1000/25=40
		{
			UsartPrintf(USART_DEBUG, "OneNet_SendData\r\n");
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
