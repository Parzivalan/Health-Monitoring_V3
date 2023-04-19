#include "oled.h"
#include "oledfont.h"
#include "string.h"
#include "delay.h"

#define OLED_NUM_A 0x1
#define OLED_NUM_B 0x2
#define OLED_NUM_C 0x4
#define OLED_NUM_D 0x8
#define OLED_NUM_E 0x10
#define OLED_NUM_F 0x20
#define OLED_NUM_G 0x40

//初始化硬件IIC引脚
void I2C_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    /* I2C1外设GPIO时钟使能 */
    RCC_APB2PeriphClockCmd(OLED_I2C_SCL_GPIO_CLK | OLED_I2C_SDA_GPIO_CLK, ENABLE);
    /* I2C1外设时钟使能 */
    RCC_APB1PeriphClockCmd(OLED_I2C_CLK, ENABLE);

    /* I2C_SCL、I2C_SDA */
    GPIO_InitStructure.GPIO_Pin = OLED_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; //复用开漏输出
    GPIO_Init(OLED_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = OLED_I2C_SDA_PIN;
    GPIO_Init(OLED_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

    /* I2C1配置 */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; /* 高电平数据稳定，低电平数据变化SCL时钟线的占空比 */
    I2C_InitStructure.I2C_OwnAddress1 = OLED_ADDRERSS; //主机的I2C地址
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; //I2C的寻址模式为7位地址
    I2C_InitStructure.I2C_ClockSpeed = I2C_Speed; //I2C通信速率为100KHz

    I2C_Init(OLED_I2C, &I2C_InitStructure); //初始化I2C1
    I2C_Cmd(OLED_I2C, ENABLE);             //使能I2C1
}

//IIC向OLED寄存器写入一个字节
void I2C_WriteByte(uint8_t addr, uint8_t data)
{
    while(I2C_GetFlagStatus(OLED_I2C, I2C_FLAG_BUSY)); //等待I2C1空闲
    
    I2C_GenerateSTART(OLED_I2C, ENABLE); //发送I2C1开始信号
    while (!I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_MODE_SELECT))
        ; //等待I2C1 EV5事件，检测起始信号是否发送成功

    I2C_Send7bitAddress(OLED_I2C, OLED_ADDRERSS, I2C_Direction_Transmitter); //发送I2C1地址+写信号
    while (!I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
        ; //等待I2C1 EV6事件，检测从机地址是否发送成功

    I2C_SendData(OLED_I2C, addr); //发送寄存器地址
    while (!I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ; //等待I2C1 EV8事件，检测数据是否发送成功

    I2C_SendData(OLED_I2C, data); //发送数据
    while (!I2C_CheckEvent(OLED_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        ; //等待I2C1 EV8事件，检测数据是否发送成功

    I2C_GenerateSTOP(OLED_I2C, ENABLE); //发送I2C1停止信号
}

//data:要写入的数据/命令
//cmd: 数据/命令标志。0,写命令;1,写数据;
void OLED_WriteByte(uint8_t data, uint8_t cmd)
{
    switch(cmd)
    {
        case OLED_DATA:
            I2C_WriteByte(0x40, data);
            break;
        case OLED_CMD:
            I2C_WriteByte(0x00, data);
            break;
    }
}

//写数据
void WriteData(unsigned char I2C_Data)
{
    I2C_WriteByte(0x40, I2C_Data);
}

const uint8_t heart[] = {
    0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC,
    0xF8, 0xF0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF,
    0xFF, 0xFF, 0xFE, 0xFC, 0xF8,
    0x01, 0x07, 0x1F, 0x3F, 0x7F, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0x7F, 0x3F, 0x1F, 0x07, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03,
    0x07, 0x0F, 0x0F, 0x0F, 0x07, 0x03, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00};
void OLED_showHeart(uint8_t showOrNot)
{
    uint8_t i;
    // 画心形
    OLED_Set_Pos(1, 0);
    if (showOrNot)
    {
        for (i = 0; i < 21; i++)
        {
            OLED_WriteByte(heart[i], OLED_DATA);
        }
        OLED_Set_Pos(1, 1);
        for (i = 0; i < 21; i++)
        {
            OLED_WriteByte(heart[21 + i], OLED_DATA);
        }
        OLED_Set_Pos(1, 2);
        for (i = 0; i < 21; i++)
        {
            OLED_WriteByte(heart[42 + i], OLED_DATA);
        }
    }
    else
    {
        for (i = 0; i < 21; i++)
        {
            OLED_WriteByte(0, OLED_DATA);
        }
        OLED_Set_Pos(1, 3);
        for (i = 0; i < 21; i++)
        {
            OLED_WriteByte(0, OLED_DATA);
        }
        OLED_Set_Pos(1, 4);
        for (i = 0; i < 21; i++)
        {
            OLED_WriteByte(0, OLED_DATA);
        }
    }
}

//初始化OLED
void OLED_Init(void)
{
    I2C_Configuration(); //初始化IIC

    delay_ms(500);
    OLED_WriteByte(0xAE, OLED_CMD); //关闭显示
    OLED_WriteByte(0x02, OLED_CMD); //设置低列地址
    OLED_WriteByte(0x10, OLED_CMD); //设置高列地址
    OLED_WriteByte(0x40, OLED_CMD); //设置起始行地址
    OLED_WriteByte(0xB0, OLED_CMD); //设置页地址
    OLED_WriteByte(0x81, OLED_CMD); //对比度设置
    OLED_WriteByte(0xFF, OLED_CMD); //亮度调节 0x00~0xFF
    OLED_WriteByte(0xA1, OLED_CMD); //段重定义设置,bit0:0,0->0;1,0->127;
    OLED_WriteByte(0xA6, OLED_CMD); //设置显示方式;bit0:1,反相显示;0,正常显示
    OLED_WriteByte(0xA8, OLED_CMD); //设置多路复用率
    OLED_WriteByte(0x3F, OLED_CMD); //1/64 duty
    OLED_WriteByte(0xC8, OLED_CMD); //Com scan direction
    OLED_WriteByte(0xD3, OLED_CMD); //设置显示偏移
    OLED_WriteByte(0x00, OLED_CMD); //无偏移

    OLED_WriteByte(0xD5, OLED_CMD); //设置显示时钟分频因子,震荡频率
    OLED_WriteByte(0x80, OLED_CMD); //设置为默认分频

    OLED_WriteByte(0xD8, OLED_CMD); //设置显示前驱电流
    OLED_WriteByte(0x05, OLED_CMD); //设置为默认

    OLED_WriteByte(0xD9, OLED_CMD); //设置显示后驱电流
    OLED_WriteByte(0xF1, OLED_CMD); //设置为默认

    OLED_WriteByte(0xDA, OLED_CMD); //设置com硬件引脚配置
    OLED_WriteByte(0x12, OLED_CMD);

    OLED_WriteByte(0xDB, OLED_CMD); //设置vcomh
    // OLED_WriteByte(0x20, OLED_CMD); //0.77xVcc
    OLED_WriteByte(0x30, OLED_CMD); //0.77xVcc

    OLED_WriteByte(0x8D, OLED_CMD); //电荷泵设置
    OLED_WriteByte(0x14, OLED_CMD); //bit2，开启/关闭

    OLED_WriteByte(0xAF, OLED_CMD); //开启显示

    OLED_Clear();
}

void OLED_showFrame(void)
{
    uint8_t i;
    // 画心形
    OLED_showHeart(1);
    //画bpm
    const uint8_t bpm[] = {
        0x3E, 0x28, 0x38, 0x00, 0x3E, 0x0A, 0x0E, 0x00, 0x3E, 0x02, 0x1C, 0x02, 0x3E};
    OLED_Set_Pos(58, 2);
    for (i = 0; i < 13; i++)
    {
        OLED_WriteByte(bpm[i], OLED_DATA);
    }
    //画%
    const uint8_t percent[] = {
        0x63, 0x33, 0x18, 0x0C, 0x66, 0x63};
    OLED_Set_Pos(121, 2);
    for (i = 0; i < 6; i++)
    {
        OLED_WriteByte(percent[i], OLED_DATA);
    }
    //画spo2
    const uint8_t spo2[] = {
        0x38, 0x44, 0x44, 0x44, 0x88, 0x00, 0xFC, 0x44, 0x44, 0x44, 0x38,
        0xE1, 0x12, 0x12, 0x12, 0xE1, 0x00, 0x23, 0x10, 0x10, 0x90, 0x60,
        0x07, 0x08, 0x08, 0x08, 0x07, 0x00, 0x0C, 0x0A, 0x09, 0x08, 0x08};
    OLED_Set_Pos(73, 0);
    for (i = 0; i < 11; i++)
    {
        OLED_WriteByte(spo2[i], OLED_DATA);
    }
    OLED_Set_Pos(73, 1);
    for (i = 0; i < 11; i++)
    {
        OLED_WriteByte(spo2[11 + i], OLED_DATA);
    }
    OLED_Set_Pos(73, 2);
    for (i = 0; i < 11; i++)
    {
        OLED_WriteByte(spo2[22 + i], OLED_DATA);
    }
}
    
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!
void OLED_Clear(void)
{
    uint8_t i, n;
    for (i = 0; i < 8; i++)
    {
        OLED_WriteByte(0xb0 + i, OLED_CMD); //设置页地址（0~7）
        // OLED_WriteByte(0x00, OLED_CMD);     //设置显示位置—列低地址
        OLED_WriteByte(0x02, OLED_CMD);     //设置显示位置—列低地址
        OLED_WriteByte(0x10, OLED_CMD);     //设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WriteByte(0, OLED_DATA);
    } //更新显示
}

//显示一个字符串
//x:0~127
//y:0~63
//size:选择字体 16/12
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t size)
{
    unsigned char j = 0;
    while (chr[j] != '\0')
    {
        OLED_ShowChar(x, y, chr[j], size);
        x += 8;
        if (x > 120)
        {
            x = 0;
            y += 2;
        }
        j++;
    }
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63
//size:选择字体 16/12
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size)
{
    unsigned char c = 0, i = 0;
    c = chr - ' '; //得到偏移后的值
    if (x > 128 - 1)
    {
        x = 0;
        y = y + 2;
    }
    if (size == 16)
    {
        OLED_Set_Pos(x, y);
        for (i = 0; i < 8; i++)
            OLED_WriteByte(F8X16[c * 16 + i], OLED_DATA);
        OLED_Set_Pos(x, y + 1);
        for (i = 0; i < 8; i++)
            OLED_WriteByte(F8X16[c * 16 + i + 8], OLED_DATA);
    }
    else
    {
        // OLED_Set_Pos(x, y + 1);
        OLED_Set_Pos(x, y);
        for (i = 0; i < 6; i++)
            OLED_WriteByte(F6x8[c][i], OLED_DATA);
    }
}

//坐标设置
void OLED_Set_Pos(unsigned char x, unsigned char y)
{
    OLED_WriteByte(0xb0 + y, OLED_CMD);
    OLED_WriteByte(((x & 0xf0) >> 4) | 0x10, OLED_CMD);
    // OLED_WriteByte((x & 0x0f) | 0x01, OLED_CMD);
    OLED_WriteByte((x & 0x0f), OLED_CMD);
}

//开启OLED显示
void OLED_Display_On(void)
{
    OLED_WriteByte(0X8D, OLED_CMD); //SET DCDC命令
    OLED_WriteByte(0X14, OLED_CMD); //DCDC ON
    OLED_WriteByte(0XAF, OLED_CMD); //DISPLAY ON
}

//关闭OLED显示
void OLED_Display_Off(void)
{
    OLED_WriteByte(0X8D, OLED_CMD); //SET DCDC命令
    OLED_WriteByte(0X10, OLED_CMD); //DCDC OFF
    OLED_WriteByte(0XAE, OLED_CMD); //DISPLAY OFF
}

//定义上面7行用于曲线显示
// uint8_t OLED_GRAM[128][8];
uint8_t OLED_GRAM[128][7];

//更新显存到LCD
void OLED_Refresh_Gram(void)
{
    uint8_t i, n;
    for (i = 0; i < 7; i++)
    {
        OLED_WriteByte(0xb0 + i, OLED_CMD); //设置页地址（0~7）
        // OLED_WriteByte(0x00, OLED_CMD);     //设置显示位置—列低地址
        OLED_WriteByte(0x02, OLED_CMD);     //设置显示位置—列低地址
        OLED_WriteByte(0x10, OLED_CMD);     //设置显示位置—列高地址
        for (n = 0; n < 128; n++)
            OLED_WriteByte(OLED_GRAM[n][i], OLED_DATA);
    } //更新显示
}

//画点
//x:0~127
//y:0~63
//t:1 填充 0,清空
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t pos, bx, temp = 0;
    // if (x > 127 || y > 63)
    if (x > 127 || y > 55)
        return;
    pos = 6 - y / 8;
    // pos = 7 - y / 8;
    bx = y % 8;
    temp = 1 << (7 - bx);
    if (t)
        OLED_GRAM[x][pos] |= temp;
    else
        OLED_GRAM[x][pos] &= ~temp;
}

//画线
//x1,y1:起点坐标
//x2,y2:终点坐标
//mode:0,清空;1,填充
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
    // uint8_t t;
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1; //计算坐标增量
    delta_y = y2 - y1;
    uRow = x1;
    uCol = y1;
    if (delta_x > 0)
        incx = 1; //设置单步方向
    else if (delta_x == 0)
        incx = 0; //垂直线
    else
    {
        incx = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; //水平线
    else
    {
        incy = -1;
        delta_y = -delta_y;
    }
    if (delta_x > delta_y)
        distance = delta_x; //选取基本增量坐标轴
    else
        distance = delta_y;
    for (t = 0; t <= distance + 1; t++) //画线输出
    {
        OLED_DrawPoint(uRow, uCol, mode); //画点
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance)
        {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance)
        {
            yerr -= distance;
            uCol += incy;
        }
    }
}

//绘制心率曲线
void OLED_DrawHeartRate(s32 *data)
{
    u8 i = 0;
    s32 max = -5000000;
    s32 min = 5000000;
    static float k;

    for(; i<128; i++)
    {
        max = max < data[i] ? data[i] : max;
        min = min > data[i] ? data[i] : min;
    }

    k = (float)56/(max-min);

    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
    for(i=0; i<127; i++)
    {
        OLED_DrawLine(i, (data[i]-min)*k, i+1, (data[i+1]-min)*k, 1);
    }
    OLED_Refresh_Gram();
}

void OLED_Show_Welcome(unsigned char x, unsigned char y, unsigned char N)
{
    unsigned char wm = 0;
    unsigned int adder = 32 * N;
    OLED_Set_Pos(x, y);
    for(wm = 0; wm < 16; wm++)
    {
        WriteData(System[adder]);
        adder += 1;
    }
    OLED_Set_Pos(x, y + 1);
    for(wm = 0; wm < 16; wm++)
    {
        WriteData(System[adder]);
        adder += 1;
    }
}

//x1,y1,x2,y2 填充区域的对角坐标
//确保x1<=x2;y1<=y2 0<=x1<=127 0<=y1<=63	 	 
//dot:0,清空;1,填充	  
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot)  
{  
	u8 x,y;  
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)OLED_DrawPoint(x,y,dot);
	}													    
	OLED_Refresh_Gram();//更新显示
}

//显示数字，心率888，spo2 888
const uint8_t oled_nums[] = {0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f, 0x00};
const uint8_t oled_nums_pos[] = {24, 36, 47, 86, 98, 109};
void OLED_showNum(uint8_t which, uint8_t num)
{
    uint8_t dat = oled_nums[num];
    uint8_t start = oled_nums_pos[which];
    uint8_t i, temp;
    OLED_Set_Pos(start, 0);
    for (i = 0; i < 10; i++)
    {
        temp = 0;
        if (i <= 1)
        {
            if (dat & OLED_NUM_F)
                temp |= 0xf8;
        }
        else if (i >= 2 && i <= 7)
        {
            if (dat & OLED_NUM_A)
                temp |= 0x06;
        }
        else
        {
            if (dat & OLED_NUM_B)
                temp |= 0xf8;
        }
        OLED_WriteByte(temp, OLED_DATA);
    }
    OLED_Set_Pos(start, 1);
    for (i = 0; i < 10; i++)
    {
        temp = 0;
        if (i <= 1)
        {
            if (dat & OLED_NUM_F)
                temp |= 0x03;
            if (dat & OLED_NUM_E)
                temp |= 0xf0;
        }
        else if (i >= 2 && i <= 7)
        {
            if (dat & OLED_NUM_G)
                temp |= 0x0c;
        }
        else
        {
            if (dat & OLED_NUM_B)
                temp |= 0x03;
            if (dat & OLED_NUM_C)
                temp |= 0xf0;
        }
        OLED_WriteByte(temp, OLED_DATA);
    }
    OLED_Set_Pos(start, 2);
    for (i = 0; i < 10; i++)
    {
        temp = 0;
        if (i <= 1)
        {
            if (dat & OLED_NUM_E)
                temp |= 0x07;
        }
        else if (i >= 2 && i <= 7)
        {
            if (dat & OLED_NUM_D)
                temp |= 0x18;
        }
        else
        {
            if (dat & OLED_NUM_C)
                temp |= 0x07;
        }
        OLED_WriteByte(temp, OLED_DATA);
    }
}

//画图,128*24
uint8_t pos_x_this = 0;
char pos_y_old = 0;
char max(char a, char b) { return a > b ? a : b; }
char min(char a, char b) { return a < b ? a : b; }
void OLED_drawChart(float value)
{
    uint8_t dat[] = {0xff, 0xff, 0xff};
    char pos_y = (char)(value * 0.12) + 10;
    if (pos_y > 23)
        pos_y = 23;
    if (pos_y <= 0)
        pos_y = 0;
    char i, ii, blank;
    char y_max = max(pos_y, pos_y_old);
    char y_min = min(pos_y, pos_y_old);

    if (y_max == y_min)
        y_max = y_min + 1;
    for (i = 0; i < 3; i++)
    {
        if ((y_min - i * 8) >= 0)
            dat[i] &= (uint8_t)(dat[i] >> (y_min - i * 8));
        if (((i + 1) * 8 - y_max) >= 0)
            dat[i] &= (uint8_t)(dat[i] << ((i + 1) * 8 - y_max));
    }
    for (i = 0; i < 3; i++)
    {
        OLED_Set_Pos(pos_x_this, 7 - i);
        OLED_WriteByte(dat[i], OLED_DATA);
        blank = 127 - pos_x_this;
        if (blank > 3)
            blank = 3;
        for (ii = 0; ii < blank; ii++)
            OLED_WriteByte(0, OLED_DATA);
    }
    pos_y_old = pos_y;
    pos_x_this++;
    if (pos_x_this > 127)
        pos_x_this = 0;
}

