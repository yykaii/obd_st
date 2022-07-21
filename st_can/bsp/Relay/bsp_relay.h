#ifndef __BSP_RELAY_H__
#define __BSP_RELAY_H__


#include <stm32f10x.h>



/*****************************************************************************
 ** ��ֲ�޸���
******************************************************************************/
// ��1·�̵���
#define  RELAY_1_GPIO       GPIOC         // ���Ŷ˿�
#define  RELAY_1_PIN        GPIO_Pin_0    // ���ű��
#define  RELAY_1_TRIGGER    1             // �̵���������ƽ��1���ߵ�ƽ������0-�͵�ƽ����
// ��2·�̵������������ã����ü���
#define  RELAY_2_GPIO       GPIOC         
#define  RELAY_2_PIN        GPIO_Pin_5  
#define  RELAY_2_TRIGGER    1             
// ��3·�̵������������ã����ü���
#define  RELAY_3_GPIO       GPIOC        
#define  RELAY_3_PIN        GPIO_Pin_5  
#define  RELAY_3_TRIGGER    1            
// ��4·�̵������������ã����ü���
#define  RELAY_4_GPIO       GPIOC        
#define  RELAY_4_PIN        GPIO_Pin_5  
#define  RELAY_4_TRIGGER    1            
//END ��ֲ ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



/*****************************************************************************
 ** ����ȫ�ֺ���
******************************************************************************/
// ��1·�̵���
void    Relay_1_Init(void);
void    Relay_1_ON(void);
void    Relay_1_OFF(void);
uint8_t Relay_1_GetState(void);
// ��2·�̵���
void    Relay_2_Init(void);
void    Relay_2_ON(void);
void    Relay_2_OFF(void);
uint8_t Relay_2_GetState(void);
// ��3·�̵���
void    Relay_3_Init(void);
void    Relay_3_ON(void);
void    Relay_3_OFF(void);
uint8_t Relay_3_GetState(void);
// ��4·�̵���
void    Relay_4_Init(void);
void    Relay_4_ON(void);
void    Relay_4_OFF(void);
uint8_t Relay_4_GetState(void);



#endif  


