/***********************************************************************************************************************************
  *���ļ����ơ�  w25qxx.h
  *����д��Ա��  ħŮ�������Ŷ�
  *�����·���  ���и��£�������QȺ�ļ���  262901124
  *����    ����  ħŮ������  https://demoboard.taobao.com
  **********************************************************************************************************************************
  *���������á�  ����STM32F103xx����֤����
  *
  *                 
  *����ֲ˵����  ��ʾ���У�Ϊ�򻯴��룬ʹ����manage_f103.c��h�ļ��е�һЩ��������ֲʱ�ر�ע��
  *              
  *              
  *�����¼�¼��  2020-04-15  ��������ɵ���
  *              2021-01-13  �����ļ��������ṹ������ע��
  *               
  *
  *
***********************************************************************************************************************************/
#include "ws2812.h"



/*****************************************************************************
 ** ���ر���
*****************************************************************************/
#define  LEDRGB_1  WS2812_GPIO->BSRR |= WS2812_PIN ;
#define  LEDRGB_0  WS2812_GPIO->BSRR |= WS2812_PIN << 16;



/*****************************************************************************
 ** ���غ�������
 **
 ** ˵��1���ܶ��˲�����ws2812��ͨ��Э�顢0�롢1�룬�ɲο������������
    https://blog.csdn.net/qq_38190041/article/details/91437493
 **
 ** ˵��2��Ҳ�кܶ��˲�������ο��ơ����__nop()��ʱ����������ο����淽��
    https://blog.csdn.net/ybhuangfugui/article/details/105190292?utm_medium=distribute.pc_relevant.none-task-blog-baidujs_utm_term-7&spm=1001.2101.3001.4242
    https://blog.csdn.net/qq_25144391/article/details/103552526?utm_medium=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.control&depth_1-utm_source=distribute.pc_relevant_t0.none-task-blog-BlogCommendFromMachineLearnPai2-1.control
*****************************************************************************/
// 0�룻 ע�⣺ws2812��0�룬�ǵ�ָ�͵�ƽ���Ǹߵ͵�ƽ�����ʱ�䣡
void write0(void)
{    
    u8 cnt0=3;
    
    LEDRGB_1;        // 0��ʱ��ǰ�θߵ�ƽʱ��: 220~380ns
    __nop();
    
    LEDRGB_0;        // 0��ʱ����ε͵�ƽʱ��: 580~1000ns   
    while(cnt0--)    
        __nop();  
}

// 1��;  ע�⣺ws2812��1�룬�ǵ�ָ�ߵ�ƽ���Ǹߵ͵�ƽ�����ʱ�䣡
void write1(void)
{    
    u8  cnt0=3;
    LEDRGB_1;        // 1��ʱ��ǰ�θߵ�ƽʱ��: 580~1000us  
    while(cnt0--)     
        __nop();    
    
    LEDRGB_0;        // 1��ʱ��ǰ�θߵ�ƽʱ��: 580~1000us   
    while(cnt0--)   
        __nop();     
}

// д1�ֽ�ֵ
void writeByte(u8 data)
{
    for(u8 i=0; i<8; i++)
    {
        if((data<<i) & 0x80)   write1();     // �������ţ����1��ĵ�ƽʱ�����
        else                   write0();     // �������ţ����0��ĵ�ƽʱ�����
    }
}



/*****************************************************************************
 ** ��  ����W2812_Init
 ** ��  �ܣ���ʼ��ws2812���õ�����
 ** ˵  ������ͨ�������
 ** ��  ������
 ** ����ֵ���� 
 ** ��  ע��
*****************************************************************************/
void Ws2812_Init(void)
{   
    GPIO_InitTypeDef G;                              /* ����һ��GPIO_InitTypeDef���͵Ľṹ�� */   

    // ʹ�ö˿��жϵķ���ʹ��ʱ��, ��ʹ������ֲ������
    // ʹ�����Ŷ˿�ʱ��
    if(WS2812_GPIO == GPIOA)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    if(WS2812_GPIO == GPIOB)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    if(WS2812_GPIO == GPIOC)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
    if(WS2812_GPIO == GPIOD)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
    if(WS2812_GPIO == GPIOE)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE , ENABLE);
    if(WS2812_GPIO == GPIOF)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF , ENABLE);
    if(WS2812_GPIO == GPIOG)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF , ENABLE);
    
    G.GPIO_Pin   = WS2812_PIN;                         /* ѡ��Ҫ���Ƶ�GPIO����       */    
    G.GPIO_Mode  = GPIO_Mode_Out_PP;                 /* ��������ģʽΪͨ��������� */     
    G.GPIO_Speed = GPIO_Speed_50MHz;                 /* ������������Ϊ50MHz        */      
    GPIO_Init(WS2812_GPIO, &G);                    /* ���ÿ⺯������ʼ��GPIO     */
      
    GPIO_ResetBits(WS2812_GPIO,  WS2812_PIN);        

    printf("LED ָʾ��            �������\r");     
    //GPIOSet (LED_RGB_GPIO , LED_RGB_PIN, GPIO_Mode_OUT ,GPIO_OType_PP , GPIO_Speed_50M , GPIO_PuPd_DOWN );   
    // ��ʼ�����ȸ�λһ�Σ������ƽ�Ŷ�����ɵ�ͨ�Ŵ��� 
    Ws2812_Reset();      
}
   


/*****************************************************************************
 ** ��  ����Ws2812_SetColor
 ** ��  �ܣ�������ɫֵ
 ** ˵  �����ֱ�����R\G\B��ֵ
 ** ��  ����
 ** ����ֵ���� 
 ** ��  ע��
*****************************************************************************/
void Ws2812_SetColor(u8 red, u8 green, u8 blue)
{
    writeByte(green);
    writeByte(red);
    writeByte(blue);
}  



/*****************************************************************************
 ** ��  ����Ws2812_SetRGB
 ** ��  �ܣ�������ɫֵ
 ** ˵  ������Ws2812_SetColor�����Ĺ��ܻ���һ���ģ�ֻ��ֱ������RGBֵ
 ** ��  ����u16 rgb: 16λRGB��ɫֵ; ����Ϊu32���ͣ�ֻΪ�˼����պ������չ
 ** ����ֵ���� 
 ** ��  ע��
*****************************************************************************/
void Ws2812_SetRGB(u32 rgb)
{
    u8 red   = (rgb>>16)&0xFF;
    u8 green = (rgb>>8) &0xFF;
    u8 blue  = rgb & 0xFF;
    
    Ws2812_SetColor(red, green, blue);
}



/*****************************************************************************
 ** ��  ����Ws2812_Off
 ** ��  �ܣ��رյƹ�
 ** ˵  ����
 ** ��  ����Ԫ
 ** ����ֵ���� 
 ** ��  ע��
*****************************************************************************/
void Ws2812_Off(void)
{
    Ws2812_SetColor(0, 0, 0);  
}



/*****************************************************************************
 ** ��  ����Ws2812_WriteReset
 ** ��  �ܣ���λ��
 ** ˵  ����һ������ĳʱ���ĵ͵�ƽ
 ** ��  ����Ԫ
 ** ����ֵ���� 
 ** ��  ע��
*****************************************************************************/
void Ws2812_Reset(void)
{
    LEDRGB_0;      // RES : >280us  
    __nop();
    __nop();    
    __nop();
    __nop();
}

