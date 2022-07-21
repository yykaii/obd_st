/**==================================================================================================================
 ** ���ļ����ơ�  main.c
 **
 ** ��ʵ�ֹ��ܡ�  USB���⴮��
 **==================================================================================================================
 **
 ** ������ƽ̨��  STM32F103 + ��׼��v3.5 + keil5
 **
 ** ����ֲ˵����  1- ���ƹ����ļ�����USB�ļ��У���Ŀ��λ��;
 **               2- ��Ŀ�깤���У������籾ʾ��: �û�����CONFIG�ļ���6��c�ļ����������޸ĵĺ����ļ�6��c�ļ�;
 **               3- ������λ�ã�#include "bsp_usb.h" �����ɵ���غ���
 **
 **
 ** ���� �� �㡿  1-WIN7 ��Ҫ��װ��������Ȼ�޷�ʶ��; WIN10 ���谲װ����;
 **               2-�ʺϵ�·��PA12��1.5K�����������3.3V�ĵ�·; ���ʺ�PA12�������辭�����ܿ��Ƶĵ�·;
 **               3-F103ϵ��оƬ�У�USB��CAN����ͬһоƬ���棬����ͬʱʹ�á�
 **               4-ͬ�ϣ�������ţ��Ƿ�������ñ�л���USB/CAN��
 **               5-����֮�أ�ÿ֡���ݣ������Ч���ȣ�64�ֽ�; ����������ȣ���Ҫ���б�д����ʵ�ְַ�����;
 **               6-USB���⴮�ڵ�ö�ٹ���: �ɶ˵�0�жϸ��������úõ����������Զ����
 **               7-USB���⴮�ڵķ��ʹ������ɶ˵�1�жϴ���;(ֻҪ�����ݺͳ��ȣ�д�뻺������Ȼ��ʹ�ܶ˵�1�����Զ�����)
 **               8-USB���⴮�ڵĽ��մ������ɶ˵�3�жϴ���;(�����ж�ʱ����ȡ��Ӧ�����������ɵõ������յ���Ч���ȣ�������)
 **
 **
 ** ������˵����  1-����֮�أ�ÿ֡���ݣ������Ч���ȣ�64�ֽ�; ����������ȣ���Ҫ���б�д����ʵ�ְַ�����;
 **
 **               2-��ʼ����  ���������ʺ�PA12��1.5K����������ĵ�·;
 **                           ---USB_Config();                                // ���ú󣬼�������������ù���
 **
 **               3-�������ݣ�ͨ���������������������ݴ��뷢�ͻ��棬�˵�1�жϻص���������ʵ�ֱ�������ȡ����;
 **                           ---USB_SendDatas(uint8_t *buf, uint16_t len);   // ����ָ�����ȵ�����
 **                           ---USB_SendString(char *fmt, ...);              // �����ַ���; ʹ�÷�ʽ��ͬprintf
 **
 **               4-�������ݣ����ݵĽ����ɶ˵�3�жϻص������Զ�ʵ�֣�����ֻ�账�����յ������ݼ���;
 **                           ---���ݳ��� xUSB.ReceivedNum > 0                // xUSB.ReceivedNum��Ϊ���һ֡���ݵĸ��س��ȣ��� EP3_OUT_Callback() ˢ��; ����ʹ�ú����ֶ���0
 **                           ---���ݻ��� xUSB.ReceivedBuffer                 // xUSB.ReceivedBuffer[], Ϊ���һ֡���ݵĸ��ػ��棬�� EP3_OUT_Callback() ˢ��; ע�⣬δ����64�ֽڵķְ�����
 **
 **
 ** �����¼�¼��  2022-06-02  �ο���ֲ������ԭ��ʾ��
 **
 **
====================================================================================================================*/
#include <stm32f10x.h>            // ͷ�ļ�����(��׼��); �ںˡ�оƬ����....��(stm32f10x.conf.h, �Ա�׼��ͷ�ļ����е���)     
#include "stm32f10x_conf.h"       // ͷ�ļ�����(��׼��); �ںˡ�оƬ����....��(stm32f10x.conf.h, �Ա�׼��ͷ�ļ����е���) 
#include "bsp_led.h"              // LEDָʾ��
#include "bsp_key.h"
#include "bsp_usart.h"            // USART1��2��3��UART4��5
#include "bsp_can.h"              // CAN



// ms��ʱ������������ֲʱ���ⲿ�ļ�������
static void delay_ms(uint32_t ms)
{
    ms = ms * 10240;
    for (uint32_t i = 0; i < ms; i++); // 72MHzϵͳʱ���£����ٸ���ѭ��Լ��ʱ1ms
}



// ������
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                // �жϷ��飬��2:��ռ��0~3,�����ȼ�0~3 ; ȫ��ֻ����һ�Σ������������۵ĵط�
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);           // ʹ��AFIO��������IOʱ��
    AFIO->MAPR |= 0x2 << 24;                                       // ����оƬ���Է�ʽ(SWD): 000_ȫ��, 010_ֻ��SWD, 100:ȫ��; ����:�ر�JTAGֻ����SWD, ���ͷ�PB3��PB4��PA15��ֻ��PA13��PA14

    USART1_Init(115200);                                           // ���ڳ�ʼ����USART1(115200-N-8-1), �ҹ����Ѱ�printf�ض�����USART1���; �ر�ע�⣺����printf()ǰ�������ȳ�ʼ���������USARTx,��Ȼ����Ῠ��

    Led_Init();                                                    // LED ��ʼ��; ����LED����Ҫ���ã���ϴ�����ԣ��ܷ���ض�λ����
    LED_RED_OFF;                                                   // ��ƣ�����������������ָʾ��
    LED_BLUE_OFF;                                                  // ���ƣ���������USB����ָʾ����-���ӳɹ�����-δ����

    Key_Init();                                                    // ������ʼ��

    CAN1_Config();                                                 // CAN����500Kbs��

    while (1)                                                      // while������ѭ����������main�������н�������������Ӳ������
    {
        delay_ms(500);                                             // ��ʱ
        LED_RED_TOGGLE;                                            // ��ɫLED�����ɵ���˸������۲�����Ƿ���������
        
        /******  ����CAN���� ******/
        CAN1_SendData((uint8_t *)"CAN_SEND", RECIVE_ID);           // ÿ��0.5����ⷢ��һ�α��ģ�����Է�����
        
        /******  ����CAN���� ******/                               // 
        if (xCAN.ReceivedNum >0 )                                  // ��� xCAN.ReceivedNum>0, ��Ϊ�յ��±���
        {
            printf("\r CAN���յ� %d �ֽ�����.", xCAN.ReceivedNum); // ��1�δ�ӡ: �����ֽ�����
            printf("\r 16���Ʒ�ʽ��ʾ��");
            for (uint8_t i = 0; i < xCAN.ReceivedNum; i++)         // ��2�δ�ӡ����: ��16���Ʒ�ʽ��ʾ����������۲���ʵ����
            {                
                printf(" 0x%X ", xCAN.ReceivedBuf[i]);     
            }
            printf("\r �ַ�����ʽ��ʾ��%s\r", xCAN.ReceivedBuf);   // ��3�δ�ӡ���ݣ����ַ�����ʽ��ʾ������۲��ַ�������; xCAN.ReceivedBuf[9]������9���ֽڣ�����Ϊ�˷������ַ�ʽ�������9�ֽ�Ϊ:'\0'
            xCAN.ReceivedNum = 0;                                  // ��Ҫ�������������ˣ��ѽ��������声           
        }             
    }
}

