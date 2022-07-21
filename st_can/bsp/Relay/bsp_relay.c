/**
  ******************************************************************************
  * �ļ�����: bsp_RELAY.c
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: �̵���ģ����������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  *
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "bsp_relay.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: �̵���IO���ų�ʼ��.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ����ʹ�ú궨�巽������������źţ����������ֲ��ֻҪ���޸�bsp_RELAY.h
  *           �ļ���غ궨��Ϳ��Է����޸����š�
  */
void Relay_1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;   /* ����IOӲ����ʼ���ṹ����� */

    // ʹ���������Ŷ˿�ʱ�ӣ�ʹ�ö˿��жϵķ���ʹ��ʱ��, ��ʹ������ֲ������
    if (RELAY_1_GPIO == GPIOA)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    if (RELAY_1_GPIO == GPIOB)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    if (RELAY_1_GPIO == GPIOC)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    if (RELAY_1_GPIO == GPIOD)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    if (RELAY_1_GPIO == GPIOE)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    if (RELAY_1_GPIO == GPIOF)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    if (RELAY_1_GPIO == GPIOG)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

    GPIO_InitStructure.GPIO_Pin = RELAY_1_PIN;          // �趨�̵�����Ӧ����IO���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // �趨�̵�����Ӧ����IO�������ٶ�
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // �趨�̵�����Ӧ����IOΪ���ģʽ
    GPIO_Init(RELAY_1_GPIO, &GPIO_InitStructure);       // ��ʼ���̵�����Ӧ����IO
    Relay_1_OFF();                                      // �رռ̵���
}


#if RELAY_1_TRIGGER      // ��Ҫ����1·�̵�����Ч������ƽ��1-�ߵ�ƽ��0-�͵�ƽ
void Relay_1_ON(void)
{
    RELAY_1_GPIO->BSRR = RELAY_1_PIN;
}

void Relay_1_OFF(void)
{
    RELAY_1_GPIO->BRR = RELAY_1_PIN;
}
#else
void Relay_1_ON(void)
{
    RELAY_1_GPIO->BRR = RELAY_1_PIN;
}

void Relay_1_OFF(void)
{
    RELAY_1_GPIO->BSRR = RELAY_1_PIN;
}
}
#endif

// ���أ�ʵ�ʿ���״̬
//
uint8_t Relay_1_GetState(void)
{
    uint8_t state = 0;

    (RELAY_1_GPIO->IDR & RELAY_1_PIN) ? (state = 1) : (state = 0);

    if (state == RELAY_1_TRIGGER)
        return 1;
    else
        return 0;
}

