/***********************************************************************************************************************************
 **【购买链接】  魔女科技    https://demoboard.taobao.com
 **【更新分享】  Q群文件夹
 ***********************************************************************************************************************************
 **【文件名称】  bsp_DHT11.c
 **【文件功能】
 **【适用平台】  STM32F103 + 标准库v3.5 + keil5
 **
 **
 **【更新记录】  2021-12-23  修改初始化函数，使文件移植更方便: DHT11_Init(GPIOx, GPIO_Pin_x);
 **              2021-10-26  修改c和h文件文件格式
 **              2021-10-26  修改DHT11_GetData()函数
 **              2021-05-20  创建文件
 **
************************************************************************************************************************************/
#include "bsp_DHT11.h"


#define DHT11_BUS_HIGH    (xDHT11.GPIOx->BSRR = (uint32_t)xDHT11.Pinx)
#define DHT11_BUS_LOW     (xDHT11.GPIOx->BSRR = ((uint32_t)xDHT11.Pinx) << 16)
#define DHT11_BUS_READ   ((xDHT11.GPIOx->IDR & xDHT11.Pinx) ? 1: 0)

// 声明本地变量
//typedef struct
//{
//    GPIO_TypeDef*  GPIOx;            // 所用的GPIO端口
//    uint32_t       Pinx;             // 所用的Pin引脚
//    uint8_t        flagInit;         // 初始化状态   0:正常, 1:失败；
//    float          temperature;      // 温度值：在调用DHT11_GetTemp()函数后，获取到的温度值；
//    float          humidity;         // 湿度值：在调用DHT11_GetTemp()函数后，获取到的温度值；
//}xDHT11_TypeDef;
xDHT11_TypeDef  xDHT11;       // 声明全局结构体, 用于记录信息


// 声明本地函数
static void     delayUS(u32 times);                             // 本地US粗略延时函数，减少移植时对外部文件依赖；
static void     delayMS(u32 ms);                                // ms延时函数，减少移植时对外部文件依赖；
static uint8_t  readByte(void);                                 // 读出一个字节
static uint8_t  readBit(void);                                  // 读出一个位
static uint8_t  resetAndCheck(void);                            // 检测是否存在DHT11
//static uint8_t  DHT11_Init(GPIO_TypeDef* GPIOx, uint32_t PINx); // 初始化DHT11, 返回值：0-失败，1-正常
//static uint8_t  DHT11_GetData(void);                            // 读取DHT11数据, 温度值保存到xDHT11.temperature, 湿度值保存到xDHT11.humidity




// 本地US粗略延时函数，减少移植时对外部文件依赖；
static void delayUS(u32 times)
{
    times = times * 7;  //  10us内用7;
    while (--times)
        __nop();
}

// 本地MS粗略延时函数，减少移植时对外部文件依赖；
static void delayMS(u32 ms)
{
    ms = ms * 6500;
    for (u32 i = 0; i < ms; i++); // 72MHz系统时钟下，多少个空循环约耗时1ms
}




// 复位、检测从机回应信号
// 返回: 0:检测失败; 1:检测正常
static uint8_t resetAndCheck(void)
{
    uint8_t retry = 0;

    // 1:主机发出复位信号(开始)
    DHT11_BUS_LOW;                          // 总线 低电平
    delayMS(25);                            // 保持至少18~至大30ms
    DHT11_BUS_HIGH ;                        // 总线 高电平, 即释放总线
    delayUS(20);                            // 没要求,但也稍作延时20~40us, 好让从机准备

    //while(1);
    // 2:检测从机回应信号中的低电平
    while (DHT11_BUS_READ && retry < 100)   // DHT11回应时,低电平部分会持续约83us
    {
        retry++;
        delayUS(1);
    };
    if (retry >= 100)    return 0;          // 返回异常,未检测到从机回应; 正常
    else              retry = 1;            // 超时计数清0, 准备下一个高电平测时
    // 检测从机回应信号中的高电平
    while (!DHT11_BUS_READ && retry < 100)  // DHT11回应时,低电平后的高电平部分会持续约87us
    {
        retry++;
        delayUS(1);
    };
    if (retry >= 100)    return 0;          // 返回异常,未检测到从机回应

    return 1;                               // 正常
}

// 从DHT11读取一个位
// 返回值：1/0
uint8_t readBit(void)
{
    uint8_t retry = 0;

    while (DHT11_BUS_READ && retry < 100) // 等待变为低电平
    {
        retry++;
        delayUS(1);
    }

    retry = 0;
    while (!DHT11_BUS_READ && retry < 100) // 等待变高电平
    {
        retry++;
        delayUS(1);
    }

    delayUS(40);//等待40us
    if (DHT11_BUS_READ)  return 1;
    else return 0;
}

// 从DHT11读取一个字节
// 返回：读到的字节数据
uint8_t readByte(void)
{
    uint8_t i, dat;
    dat = 0;
    for (i = 0; i < 8; i++)
    {
        dat <<= 1;
        dat |= readBit();
    }
    return dat;
}



/******************************************************************************
 * 函  数： DHT11_GetData
 * 功  能： 从DHT11中读取数据
 * 参  数： 1:
 * 说  明： 调用后，获取到的数据，保存到结构体xDHT11中
 *          温度值：xDHT11.Temperature (有效范围:0~50℃)
 *          湿度值: xDHT11.Humidity    (有效范围:20%~90%)
 * 返回值： 0-失败; 1-正常
 ******************************************************************************/
uint8_t DHT11_GetData(void)
{
    uint8_t buf[5];
    uint8_t i;

    if (resetAndCheck() == 1)
    {
        for (i = 0; i < 5; i++) //读取40bit数据
        {
            buf[i] = readByte();
        }
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
            xDHT11.Humidity = buf[0] + buf[1] / 10;
            xDHT11.Temperature = buf[2] + buf[3] / 10;
        }
        return 1;   // 正常, 返回1
    }
    else
        return 0;   // 异常, 返回0
}





/******************************************************************************
 * 函  数： DHT11_Init
 * 功  能： 初始化DHT11所用引脚
 *          重要说明: 引脚模式为开漏,适合已作上拉的模块
 * 参  数： GPIOx：GPIO端口，有效值范围：GPIOA ~ GPIOI
 *          PINx ：Pin引脚， 有效值范围：GPIO_Pin_0 ~ GPIO_Pin_15
 * 返回值： 无
 ******************************************************************************/
void DHT11_Init(GPIO_TypeDef *GPIOx, uint32_t PINx)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    xDHT11.GPIOx    = GPIOx;   // 保存所用GPIO端口，后面要用到
    xDHT11.Pinx     = PINx;    // 保存所用引脚编号，后面要用到

    // 时钟使能：引脚端口;用判断端口的方式使能时钟线, 减少移植时的工作
    if (GPIOx == GPIOA)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    if (GPIOx == GPIOB)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    if (GPIOx == GPIOC)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    if (GPIOx == GPIOD)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    if (GPIOx == GPIOE)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    if (GPIOx == GPIOF)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    if (GPIOx == GPIOG)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

    // 配置：引脚工作模式
    GPIO_InitStructure.GPIO_Pin   = PINx;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;   // 开漏模式！总线！
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    DHT11_BUS_HIGH ;                                    // 释放总线：拉高！
}



/******************************************************************************
 * 函  数： DHT11_GetTemperature
 * 功  能： 从DHT11中读取温度值
 * 参  数： GPIO_TypeDef *GPIOx: GPIO端口号，取值范围：GPIOA ~ GPIOG
 *          uint32_t     PINx  : 引脚编号，  取值范围：GPIO_Pin_0 ~ GPIO_Pin_15
 * 说  明： 温度值有效范围:0~50℃; 精度±2°C; 小数部份无效
 * 返回值： 0-失败，非0值-湿度值
 ******************************************************************************/
float DHT11_GetTemperature(GPIO_TypeDef *GPIOx, uint32_t PINx)
{
    DHT11_Init(GPIOx, PINx);
    DHT11_GetData();
    return xDHT11.Temperature;
}


/******************************************************************************
 * 函  数： DHT11_GetHumidity
 * 功  能： 从DHT11中读取湿度值
 * 参  数： GPIO_TypeDef *GPIOx: GPIO端口号，取值范围：GPIOA ~ GPIOG
 *          uint32_t     PINx  : 引脚编号，  取值范围：GPIO_Pin_0 ~ GPIO_Pin_15
 * 说  明： 湿度值有效范围:20%~90%; 精度±5%; 小数部分无效
 * 返回值： 0-失败，非0值-湿度值
 ******************************************************************************/
float DHT11_GetHumidity(GPIO_TypeDef *GPIOx, uint32_t PINx)
{
    DHT11_Init(GPIOx, PINx);
    DHT11_GetData();
    return xDHT11.Humidity;
}
