#ifndef __ESP8266__H
#define __ESP8266__H



#include <stm32f10x.h>  
#include "stdio.h"
#include "bsp_usart.h"



/*****************************************************************************
 ** ��ֲ����
****************************************************************************/
#define ESP8266_AP_NAME           "CMCC-zml"
#define ESP8266_AP_PASSWORD       "55025865502586"



/*****************************************************************************
 ** ȫ�ֱ��� 
****************************************************************************/
#define ESP8266_RX_BUF_SIZE       1200                    // ���ݽ��ջ�������С���󲿷�����¶������޸�; USART�ж���Ļ����С

typedef struct 
{    
    uint8_t         Flag_FinishInit;                      // ��ʼ�����; 0=δ��ʼ��, 1=�ѳ�ʼ��
    //uint16_t        ReceivedBytesNum;                   // ���յ��µ�һ֡���ݣ�
    uint16_t        ReceivedNum;                          // ���յ����ٸ��ֽ�����; 0-�����ݣ���0_���յ����ֽ���
    uint8_t         ReceivedBuffer[ESP8266_RX_BUF_SIZE];  // ���յ����ݵĻ���; ESP8266��ATģʽ�£�ÿ֡�����Ϊ1056���ֽڣ�
    char*           APName;
    char*           APPassword;
    uint32_t        Baudrate;                             // ��¼���õĴ��ڲ�����
    USART_TypeDef*  USARTx;                               // ��¼���õĶ˿�
}xESP8266_TypeDef;
extern xESP8266_TypeDef  xESP8266;                        // ����Ϊȫ�ֱ���,�����¼��Ϣ��״̬
    



/*****************************************************************************
 ** ����ȫ�ֺ���
****************************************************************************/
// ����4�����������������AT�����¾��󲿷ֲ���
void      ESP8266_Init(USART_TypeDef* USARTx, uint32_t baudrate);  // ��ʼ��
void      ESP8266_SendString(char* str);                           // �������ⳤ���ַ���
uint16_t  ESP8266_CheckReceivedNum(void);                          // ����Ƿ��յ������ݣ����ؽ��յ����ֽڳ���
void      ESP8266_CleanReceivedFlag(void);                         // ����ESP8266�Ľ��ջ��棬�������ճ��ȱ��������ݴ�Ż���
// ����4����չ������Ϊ����ʹ�ö���д����������4������ʵ��
uint8_t   ESP8266_CMD (char* cmdString, char* answerString, uint32_t waitTimesMS); // ����AT������ȴ�����
uint8_t   ESP8266_JoinAP (char* SSID, char* passWord);                             // ����ĳ�ȵ㣬
uint8_t   ESP8266_SetAP(char* SSID, char* passWord);                               // ��ģ�����ó�APģʽ 
uint8_t   ESP8266_GetLinkStatus(void);                                             // ��ȡ����״̬
// ͸��

#endif


