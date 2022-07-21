/***********************************************************************************************************************************
 **���������ӡ�  ħŮ�Ƽ�    https://demoboard.taobao.com
 **�����·���  QȺ�ļ���
 ***********************************************************************************************************************************
 **���ļ����ơ�  bsp_DHT11.c
 **���ļ����ܡ�
 **������ƽ̨��  STM32F103 + ��׼��v3.5 + keil5
 **
 **
 **�����¼�¼��  2021-12-23  �޸ĳ�ʼ��������ʹ�ļ���ֲ������: DHT11_Init(GPIOx, GPIO_Pin_x);
 **              2021-10-26  �޸�c��h�ļ��ļ���ʽ
 **              2021-10-26  �޸�DHT11_GetData()����
 **              2021-05-20  �����ļ�
 **
************************************************************************************************************************************/
#include "bsp_DHT11.h"


#define DHT11_BUS_HIGH    (xDHT11.GPIOx->BSRR = (uint32_t)xDHT11.Pinx)
#define DHT11_BUS_LOW     (xDHT11.GPIOx->BSRR = ((uint32_t)xDHT11.Pinx) << 16)
#define DHT11_BUS_READ   ((xDHT11.GPIOx->IDR & xDHT11.Pinx) ? 1: 0)

// �������ر���
//typedef struct
//{
//    GPIO_TypeDef*  GPIOx;            // ���õ�GPIO�˿�
//    uint32_t       Pinx;             // ���õ�Pin����
//    uint8_t        flagInit;         // ��ʼ��״̬   0:����, 1:ʧ�ܣ�
//    float          temperature;      // �¶�ֵ���ڵ���DHT11_GetTemp()�����󣬻�ȡ�����¶�ֵ��
//    float          humidity;         // ʪ��ֵ���ڵ���DHT11_GetTemp()�����󣬻�ȡ�����¶�ֵ��
//}xDHT11_TypeDef;
xDHT11_TypeDef  xDHT11;       // ����ȫ�ֽṹ��, ���ڼ�¼��Ϣ


// �������غ���
static void     delayUS(u32 times);                             // ����US������ʱ������������ֲʱ���ⲿ�ļ�������
static void     delayMS(u32 ms);                                // ms��ʱ������������ֲʱ���ⲿ�ļ�������
static uint8_t  readByte(void);                                 // ����һ���ֽ�
static uint8_t  readBit(void);                                  // ����һ��λ
static uint8_t  resetAndCheck(void);                            // ����Ƿ����DHT11
//static uint8_t  DHT11_Init(GPIO_TypeDef* GPIOx, uint32_t PINx); // ��ʼ��DHT11, ����ֵ��0-ʧ�ܣ�1-����
//static uint8_t  DHT11_GetData(void);                            // ��ȡDHT11����, �¶�ֵ���浽xDHT11.temperature, ʪ��ֵ���浽xDHT11.humidity




// ����US������ʱ������������ֲʱ���ⲿ�ļ�������
static void delayUS(u32 times)
{
    times = times * 7;  //  10us����7;
    while (--times)
        __nop();
}

// ����MS������ʱ������������ֲʱ���ⲿ�ļ�������
static void delayMS(u32 ms)
{
    ms = ms * 6500;
    for (u32 i = 0; i < ms; i++); // 72MHzϵͳʱ���£����ٸ���ѭ��Լ��ʱ1ms
}




// ��λ�����ӻ���Ӧ�ź�
// ����: 0:���ʧ��; 1:�������
static uint8_t resetAndCheck(void)
{
    uint8_t retry = 0;

    // 1:����������λ�ź�(��ʼ)
    DHT11_BUS_LOW;                          // ���� �͵�ƽ
    delayMS(25);                            // ��������18~����30ms
    DHT11_BUS_HIGH ;                        // ���� �ߵ�ƽ, ���ͷ�����
    delayUS(20);                            // ûҪ��,��Ҳ������ʱ20~40us, ���ôӻ�׼��

    //while(1);
    // 2:���ӻ���Ӧ�ź��еĵ͵�ƽ
    while (DHT11_BUS_READ && retry < 100)   // DHT11��Ӧʱ,�͵�ƽ���ֻ����Լ83us
    {
        retry++;
        delayUS(1);
    };
    if (retry >= 100)    return 0;          // �����쳣,δ��⵽�ӻ���Ӧ; ����
    else              retry = 1;            // ��ʱ������0, ׼����һ���ߵ�ƽ��ʱ
    // ���ӻ���Ӧ�ź��еĸߵ�ƽ
    while (!DHT11_BUS_READ && retry < 100)  // DHT11��Ӧʱ,�͵�ƽ��ĸߵ�ƽ���ֻ����Լ87us
    {
        retry++;
        delayUS(1);
    };
    if (retry >= 100)    return 0;          // �����쳣,δ��⵽�ӻ���Ӧ

    return 1;                               // ����
}

// ��DHT11��ȡһ��λ
// ����ֵ��1/0
uint8_t readBit(void)
{
    uint8_t retry = 0;

    while (DHT11_BUS_READ && retry < 100) // �ȴ���Ϊ�͵�ƽ
    {
        retry++;
        delayUS(1);
    }

    retry = 0;
    while (!DHT11_BUS_READ && retry < 100) // �ȴ���ߵ�ƽ
    {
        retry++;
        delayUS(1);
    }

    delayUS(40);//�ȴ�40us
    if (DHT11_BUS_READ)  return 1;
    else return 0;
}

// ��DHT11��ȡһ���ֽ�
// ���أ��������ֽ�����
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
 * ��  ���� DHT11_GetData
 * ��  �ܣ� ��DHT11�ж�ȡ����
 * ��  ���� 1:
 * ˵  ���� ���ú󣬻�ȡ�������ݣ����浽�ṹ��xDHT11��
 *          �¶�ֵ��xDHT11.Temperature (��Ч��Χ:0~50��)
 *          ʪ��ֵ: xDHT11.Humidity    (��Ч��Χ:20%~90%)
 * ����ֵ�� 0-ʧ��; 1-����
 ******************************************************************************/
uint8_t DHT11_GetData(void)
{
    uint8_t buf[5];
    uint8_t i;

    if (resetAndCheck() == 1)
    {
        for (i = 0; i < 5; i++) //��ȡ40bit����
        {
            buf[i] = readByte();
        }
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
            xDHT11.Humidity = buf[0] + buf[1] / 10;
            xDHT11.Temperature = buf[2] + buf[3] / 10;
        }
        return 1;   // ����, ����1
    }
    else
        return 0;   // �쳣, ����0
}





/******************************************************************************
 * ��  ���� DHT11_Init
 * ��  �ܣ� ��ʼ��DHT11��������
 *          ��Ҫ˵��: ����ģʽΪ��©,�ʺ�����������ģ��
 * ��  ���� GPIOx��GPIO�˿ڣ���Чֵ��Χ��GPIOA ~ GPIOI
 *          PINx ��Pin���ţ� ��Чֵ��Χ��GPIO_Pin_0 ~ GPIO_Pin_15
 * ����ֵ�� ��
 ******************************************************************************/
void DHT11_Init(GPIO_TypeDef *GPIOx, uint32_t PINx)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    xDHT11.GPIOx    = GPIOx;   // ��������GPIO�˿ڣ�����Ҫ�õ�
    xDHT11.Pinx     = PINx;    // �����������ű�ţ�����Ҫ�õ�

    // ʱ��ʹ�ܣ����Ŷ˿�;���ж϶˿ڵķ�ʽʹ��ʱ����, ������ֲʱ�Ĺ���
    if (GPIOx == GPIOA)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    if (GPIOx == GPIOB)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    if (GPIOx == GPIOC)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    if (GPIOx == GPIOD)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    if (GPIOx == GPIOE)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    if (GPIOx == GPIOF)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    if (GPIOx == GPIOG)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

    // ���ã����Ź���ģʽ
    GPIO_InitStructure.GPIO_Pin   = PINx;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;   // ��©ģʽ�����ߣ�
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStructure);

    DHT11_BUS_HIGH ;                                    // �ͷ����ߣ����ߣ�
}



/******************************************************************************
 * ��  ���� DHT11_GetTemperature
 * ��  �ܣ� ��DHT11�ж�ȡ�¶�ֵ
 * ��  ���� GPIO_TypeDef *GPIOx: GPIO�˿ںţ�ȡֵ��Χ��GPIOA ~ GPIOG
 *          uint32_t     PINx  : ���ű�ţ�  ȡֵ��Χ��GPIO_Pin_0 ~ GPIO_Pin_15
 * ˵  ���� �¶�ֵ��Ч��Χ:0~50��; ���ȡ�2��C; С��������Ч
 * ����ֵ�� 0-ʧ�ܣ���0ֵ-ʪ��ֵ
 ******************************************************************************/
float DHT11_GetTemperature(GPIO_TypeDef *GPIOx, uint32_t PINx)
{
    DHT11_Init(GPIOx, PINx);
    DHT11_GetData();
    return xDHT11.Temperature;
}


/******************************************************************************
 * ��  ���� DHT11_GetHumidity
 * ��  �ܣ� ��DHT11�ж�ȡʪ��ֵ
 * ��  ���� GPIO_TypeDef *GPIOx: GPIO�˿ںţ�ȡֵ��Χ��GPIOA ~ GPIOG
 *          uint32_t     PINx  : ���ű�ţ�  ȡֵ��Χ��GPIO_Pin_0 ~ GPIO_Pin_15
 * ˵  ���� ʪ��ֵ��Ч��Χ:20%~90%; ���ȡ�5%; С��������Ч
 * ����ֵ�� 0-ʧ�ܣ���0ֵ-ʪ��ֵ
 ******************************************************************************/
float DHT11_GetHumidity(GPIO_TypeDef *GPIOx, uint32_t PINx)
{
    DHT11_Init(GPIOx, PINx);
    DHT11_GetData();
    return xDHT11.Humidity;
}
