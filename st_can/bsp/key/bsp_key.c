#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_relay.h"





void Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;    // ���ã��������Ź���ģʽ
    NVIC_InitTypeDef NVIC_InitStruct ;      // ���ã��������ȼ�
    EXTI_InitTypeDef EXTI_InitStruct ;      // ���ã����������жϷ�ʽ

    // ʹ��AFIOʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE) ;      // EXTI��ʱ��Ҫ����AFIO�Ĵ���

    // ʹ��GPIOʱ��, Ϊ���ٵ��Թ�������ʹ��ʱ�Ӷ����������GPIO�˿�ʱ�ӣ���ʹ���ˣ�
    RCC->APB2ENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF ;

    // KEY_1
    // �������Ź���ģʽ
    GPIO_InitStructure.GPIO_Pin  = KEY_1_PIN;                  // ����KEY_1, ��ʱ����(��Ҫ)�� ����ʱ���øߵ�ƽ
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD ;             // ���Ź���ģʽ; ��ʱ��ƽ״̬(ʹ��оƬ�ڲ�������е�ƽ������)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;         // �����ƽ��ת�ٶȣ�������״̬ʱ��Ч����GPIO_Init������Ҫ�õ���
    GPIO_Init(KEY_1_GPIO, &GPIO_InitStructure);                // ��ʼ�����������Ź���ģʽ���ú���
    // �����жϵ����ȼ�
    NVIC_InitStruct.NVIC_IRQChannel = KEY_1_IRQN ;             // �жϺ�
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1 ;    // ������ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0 ;           // ���������ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE ;              // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStruct) ;                              // ��ʼ�����������ȼ����ú���
    // �����жϵķ�ʽ
    GPIO_EXTILineConfig(KEY_1_GPIOSOURCE, KEY_1_PINSOURCE);    // ѡ����ΪEXTI�ߵ�GPIO����
    EXTI_InitStruct.EXTI_Line    = KEY_1_EXTI_LINE ;           // �����ж�or�¼���
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt ;       // ģʽ���жϣ�EXTI_Mode_Interrupt���¼���EXTI_Mode_Event
    EXTI_InitStruct.EXTI_Trigger = KEY_1_TRIM ;                // ���ش����� ������EXTI_Trigger_Rising ���½���EXTI_Trigger_Falling �����գ�EXTI_Trigger_Rising_Falling
    EXTI_InitStruct.EXTI_LineCmd = ENABLE ;                    // ʹ��EXTI��
    EXTI_Init(&EXTI_InitStruct) ;                              // ��ʼ���������ж������ú���

    // KEY_2
    // �������Ź���ģʽ
    GPIO_InitStructure.GPIO_Pin  = KEY_2_PIN;                  // ����KEY_1, ��ʱ����(��Ҫ)�� ����ʱ���øߵ�ƽ
    GPIO_InitStructure.GPIO_Mode = KEY_2_MODE ;                // ���Ź���ģʽ; ��ʱ��ƽ״̬(ʹ��оƬ�ڲ�������е�ƽ������)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;         // �����ƽ��ת�ٶȣ�������״̬ʱ��Ч����GPIO_Init������Ҫ�õ���
    GPIO_Init(KEY_2_GPIO, &GPIO_InitStructure);                // ��ʼ�����������Ź���ģʽ���ú���
    // �����жϵ����ȼ�
    NVIC_InitStruct.NVIC_IRQChannel = KEY_2_IRQN ;             // �жϺ�
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1 ;    // ������ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0 ;           // ���������ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE ;              // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStruct) ;                              // ��ʼ�����������ȼ����ú���
    // �����жϵķ�ʽ
    GPIO_EXTILineConfig(KEY_2_GPIOSOURCE, KEY_2_PINSOURCE);    // ѡ����ΪEXTI�ߵ�GPIO����
    EXTI_InitStruct.EXTI_Line    = KEY_2_EXTI_LINE ;           // �����ж�or�¼���
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt ;       // ģʽ���жϣ�EXTI_Mode_Interrupt���¼���EXTI_Mode_Event
    EXTI_InitStruct.EXTI_Trigger = KEY_2_TRIM ;                // ���ش����� ������EXTI_Trigger_Rising ���½���EXTI_Trigger_Falling �����գ�EXTI_Trigger_Rising_Falling
    EXTI_InitStruct.EXTI_LineCmd = ENABLE ;                    // ʹ��EXTI��
    EXTI_Init(&EXTI_InitStruct) ;                              // ��ʼ���������ж������ú���

    // KEY_3
    // �������Ź���ģʽ
    GPIO_InitStructure.GPIO_Pin  = KEY_3_PIN;                  // ����KEY_1, ��ʱ����(��Ҫ)�� ����ʱ���øߵ�ƽ
    GPIO_InitStructure.GPIO_Mode = KEY_3_MODE ;                // ���Ź���ģʽ; ��ʱ��ƽ״̬(ʹ��оƬ�ڲ�������е�ƽ������)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;         // �����ƽ��ת�ٶȣ�������״̬ʱ��Ч����GPIO_Init������Ҫ�õ���
    GPIO_Init(KEY_3_GPIO, &GPIO_InitStructure);                // ��ʼ�����������Ź���ģʽ���ú���
    // �����жϵ����ȼ�
    NVIC_InitStruct.NVIC_IRQChannel = KEY_3_IRQN ;             // �жϺ�
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1 ;    // ������ռ���ȼ�
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0 ;           // ���������ȼ�
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE ;              // ʹ���ж�ͨ��
    NVIC_Init(&NVIC_InitStruct) ;                              // ��ʼ�����������ȼ����ú���
    // �����жϵķ�ʽ
    GPIO_EXTILineConfig(KEY_3_GPIOSOURCE, KEY_3_PINSOURCE);    // ѡ����ΪEXTI�ߵ�GPIO����
    EXTI_InitStruct.EXTI_Line    = KEY_3_EXTI_LINE ;           // �����ж�or�¼���
    EXTI_InitStruct.EXTI_Mode    = EXTI_Mode_Interrupt ;       // ģʽ���жϣ�EXTI_Mode_Interrupt���¼���EXTI_Mode_Event
    EXTI_InitStruct.EXTI_Trigger = KEY_3_TRIM ;                // ���ش����� ������EXTI_Trigger_Rising ���½���EXTI_Trigger_Falling �����գ�EXTI_Trigger_Rising_Falling
    EXTI_InitStruct.EXTI_LineCmd = ENABLE ;                    // ʹ��EXTI��
    EXTI_Init(&EXTI_InitStruct) ;                              // ��ʼ���������ж������ú���

    printf("���� ��ʼ��           �������\r");
}



// KEY_1 �жϷ�����
void KEY_1_IRQHANDLER(void)                        // ��ʾ���������������h�ļ��еĺ궨�壬�ڱ�������У��ᱻ�滻�ɺ궨���ֵ
{
    if (EXTI->PR & KEY_1_PIN)                      // �����ϵİ�����ʹ�õ������򵥵�Ӳ������,������ʹ�������ʱ����
    {
        EXTI->PR |= KEY_1_PIN  ;                   // �����жϱ�ʾ
        printf("�� 1 ������������, ���Ʒ�ת\r");   // ��Ҫ��ʾ��printf�ǲ������뺯�����жϷ�������ʹ�ã����ܻ��������Ԥ��Ĵ�������ʹ��printf��ֻ�ô������ʹ�ã���
        LED_BLUE_TOGGLE;                           // ����ת�����Է������۹۲�Ч��
    }
}



// KEY_2 �жϷ�����
void KEY_2_IRQHANDLER(void)                        // ��ʾ���������������h�ļ��еĺ궨�壬�ڱ�������У��ᱻ�滻�ɺ궨���ֵ
{
    if (EXTI->PR & KEY_2_PIN)                      // �����ϵİ�����ʹ�õ������򵥵�Ӳ������,������ʹ�������ʱ����
    {
        EXTI->PR |= KEY_2_PIN  ;                   // �����жϱ�ʾ
        printf("�� 2 ������������, ���Ʒ�ת\r");   // ��Ҫ��ʾ��printf�ǲ������뺯�����жϷ�������ʹ�ã����ܻ��������Ԥ��Ĵ�������ʹ��printf��ֻ�ô������ʹ�ã���      // ħŮ������İ���ʹ�õ��ݽ���Ӳ������,������ʹ�������ʱ����
        LED_BLUE_TOGGLE;                           // ����ת�����Է������۹۲�Ч��
    }
}



// KEY_3 �жϷ�����
void KEY_3_IRQHANDLER(void)                        // ��ʾ���������������h�ļ��еĺ궨�壬�ڱ�������У��ᱻ�滻�ɺ궨���ֵ
{
    if (EXTI->PR & KEY_3_PIN)                      // �����ϵİ�����ʹ�õ������򵥵�Ӳ������,������ʹ�������ʱ����
    {
        EXTI->PR |= KEY_3_PIN  ;                   // �����жϱ�ʾ
        printf("�� 3 ������������, ���Ʒ�ת\r");   // ��Ҫ��ʾ��printf�ǲ������뺯�����жϷ�������ʹ�ã����ܻ��������Ԥ��Ĵ�������ʹ��printf��ֻ�ô������ʹ�ã���      // ħŮ������İ���ʹ�õ��ݽ���Ӳ������,������ʹ�������ʱ����
        LED_BLUE_TOGGLE;                           // ����ת�����Է������۹۲�Ч��
    }
}




