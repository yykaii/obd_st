/***********************************************************************************************************************************
 ** �������д��  ħŮ�������Ŷ�
 ** ��������¡�  QȺ�ļ���       1126717453
 ** ����    ����  ħŮ������      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 **���ļ����ơ�  bsp_usart.c
 **
 **���ļ����ܡ�  ��USART��GPIO���á�ͨ��Э�����á��ж����ã������ܺ���ʵ��
 **
 **������ƽ̨��  STM32F103 + ��׼��v3.5 + keil5
 **
 ** ������˵����  ���ļ����շ�����, ������޸�, �ѱȽ�����. 
 **               ��ʼ��: ֻ����ã�USARTx_Init(������), ���������������ż�ʱ�����ã� 
 **               �� �� : ��������, �ַ���: USARTx_SendString (char* stringTemp)�� ����: USARTx_SendData (uint8_t* buf, uint8_t cnt);
 **               �� �� : ��ʽ1��ͨ��ȫ�ֺ���USARTx_GetBuffer (uint8_t* buffer, uint8_t* cnt);����������������ʾ���˽��ջ��ƣ� 
 **                       ��ʽ2��ͨ���ж�xUSART.USARTxReceivedFla==1;
 **                              ����while�в�����ѯ�������κ�һ����Ҫ�ĵط��жϽ��ձ�־��ʾ����
 **                              while(1){
 **                                  if(xUSART1.USART1ReceivedFlag==1){
 **                                      printf((char*)xUSART.USART1ReceivedBuffer);          // ʾ��1: ���������ַ���
 **                                      uint16_t GPS_Value = xUSART.USART1ReceivedBuffer[1]; // ʾ��2: ��ζ�д����ĳ����Ա������
 **                                      xUSART1.USART1ReceivedFlag == 0;                     // ��Ҫ�����������ݺ�
 **                                  }  
 **                              }   
 **
 **�����¼�¼��  
 **              2021-12-16  ���ƽ��ջ��ƣ�ȡ�����ձ�־���жϽ����ֽ���>0��Ϊ���յ�������
 **              2021-12-15  ���ƽ��ջ��ƣ���֡���ݰ����ɸ��Ǿ�����
 **              2021-11-03  ���ƽ��պ�������ֵ����
 **              2021-08-31  �޸�: �����жϣ������ѽ��հ������жϣ�����ɰ�δ����������°������ݣ������°����Ǿɰ���
 **              2021-08-14  ����: �����жϣ�uint8_t cnt, �޸�Ϊuint16_t; �������ѽ����ֽ����жϣ����������յ�֡���ݴ��ڻ���ռ�ʱ���������ݸ�����ʼ����;
 **              2021-06-22  ����ע��˵��;
 **              2021-06-12  �����ļ��ṹ;
 **              2021-06-09  �����ļ���ʽ��ע�� 
 **              2021-06-08  USART1��2��3��UART4��5 ���շ����ƴ���
 **              2020-09-02  �ļ�������USART1�����жϡ������жϡ������жϡ�DMA�շ�
 **    
************************************************************************************************************************************/
#include "bsp_usart.h"



xUSATR_TypeDef  xUSART;         // ����Ϊȫ�ֱ���,�����¼��Ϣ��״̬




//////////////////////////////////////////////////////////////   USART-1   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * ��  ���� vUSART1_Init
 * ��  �ܣ� ��ʼ��USART1��GPIO��ͨ�Ų������á��ж����ȼ� 
 *          (8λ���ݡ���У�顢1��ֹͣλ)
 * ��  ���� uint32_t baudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/  
void USART1_Init(uint32_t baudrate)
{   
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;    
    
    // ʱ��ʹ��
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                           // ʹ��USART1ʱ��
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                             // ʹ��GPIOAʱ��

    // GPIO_TX��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                // TX���ţ�����Ϊ�������칤��ģʽ
    GPIO_Init (GPIOA, &GPIO_InitStructure);
    // GPIO_RX��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING  ;        // RX���ţ����ڰ�����Ϊһ����ӵ�·����ѡ���ÿ�©ģʽ
    GPIO_Init (GPIOA, &GPIO_InitStructure);    

    // �ж�����
    NVIC_InitStructure .NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority=2 ;       // ��ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 2;             // �����ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);                              
    
    //USART ��ʼ������
    USART_DeInit(USART1); 
    USART_InitStructure.USART_BaudRate   = baudrate;                // ���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;        // һ��ֹͣλ
    USART_InitStructure.USART_Parity     = USART_Parity_No;         // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // ʹ���ա���ģʽ
    USART_Init(USART1, &USART_InitStructure);                       // ��ʼ������
    
    USART_ITConfig(USART1, USART_IT_TXE , DISABLE );
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                  // ʹ�ܽ����ж�
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);                  // ʹ�ܿ����ж�
    
    USART_Cmd(USART1, ENABLE);                                      // ʹ�ܴ���, ��ʼ����  
    
    USART1->SR = ~(0x00F0);                                         // �����ж�
    
    xUSART.USART1InitFlag =1;                                       // ��ǳ�ʼ����־
    xUSART.USART1ReceivedNum =0;                                    // �����ֽ�������
   
    printf("\r\r\r=========== STM32F103 �������ʼ������ ===========\r");      
    printf("USART1��ʼ������      �����жϡ������ж�, �����ж�\r");    
}

/******************************************************************************
 * ��  ���� USART1_IRQHandler
 * ��  �ܣ� USART1�Ľ����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 *           
******************************************************************************/
static uint8_t U1TxBuffer[256] ;    // �����жϷ��ͣ����λ�������256���ֽ�
static uint8_t U1TxCounter = 0 ;    // �����жϷ��ͣ�����ѷ��͵��ֽ���(����)
static uint8_t U1TxCount   = 0 ;    // �����жϷ��ͣ���ǽ�Ҫ���͵��ֽ���(����)

void USART1_IRQHandler(void)           
{     
    static uint16_t cnt=0;                                           // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  RxTemp[U1_RX_BUF_SIZE];                          // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽ȫ�ֱ�����xUSART.USARTxReceivedBuffer[xx]�У�
    
    // �����ж�
    if(USART1->SR & (1<<5))                                          // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {
        if((cnt>=U1_RX_BUF_SIZE))//||(xUSART.USART1ReceivedFlag==1)) // �ж�1: ��ǰ֡�ѽ��յ���������������(������), Ϊ�������������������յ�������ֱ��������
        {                                                            // �ж�2: ���֮ǰ���պõ����ݰ���û�����ͷ��������ݣ�����������֡���ܸ��Ǿ�����֡��ֱ��������֡������ȱ�㣺���ݴ�������ڴ����ٶ�ʱ��������ô����������������ڵ���
            USART1->DR;                                              // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;
        } 
        RxTemp[cnt++] = USART1->DR ;                                 // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ��
    }
    
    // �����ж�, ������Ͻ����жϣ����ж�һ֡���ݵĽ������
    if(USART1->SR & (1<<4))                                          // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {         
        xUSART.USART1ReceivedNum  = 0;                               // �ѽ��յ��������ֽ�����0     
        memcpy(xUSART.USART1ReceivedBuffer, RxTemp, U1_RX_BUF_SIZE); // �ѱ�֡���յ������ݣ���ŵ�ȫ�ֱ���xUSART.USARTxReceivedBuffer��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ�������
        xUSART.USART1ReceivedNum  = cnt;                             // �ѽ��յ����ֽ�������ŵ�ȫ�ֱ���xUSART.USARTxReceivedNum�У�     
        cnt=0;                                                       // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(RxTemp ,0, U1_RX_BUF_SIZE);                           // �������ݻ������飬����; ׼����һ�εĽ���   
        USART1 ->SR;  USART1 ->DR;                                   // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!      
     }     

    // �����ж�
    if ((USART1->SR & 1<<7) && (USART1->CR1 & 1<<7))                 // ���TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                
        USART1->DR = U1TxBuffer[U1TxCounter++];                      // ��ȡ���ݼĴ���ֵ��ע�⣺��ȡDRʱ�Զ������ж�λ��        
        if(U1TxCounter == U1TxCount )
            USART1->CR1 &= ~(1<<7);                                  // �ѷ�����ɣ��رշ��ͻ����������ж� TXEIE
    }    
}  

/******************************************************************************
 * ��  ���� vUSART1_GetBuffer
 * ��  �ܣ� ��ȡUART�����յ�������
 * ��  ���� uint8_t* buffer   ���ݴ�Ż����ַ
 *          uint8_t* cnt      ���յ����ֽ��� 
 * ����ֵ�� 0_û�н��յ������ݣ� 1_���յ�������
 ******************************************************************************/  
uint8_t USART1_GetBuffer(uint8_t* buffer, uint8_t* cnt)
{    
    if(xUSART.USART1ReceivedNum)               // �ж��Ƿ���������
    {    
        memcpy(buffer, xUSART.USART1ReceivedBuffer, xUSART.USART1ReceivedNum );     // �������ݸ��Ƶ�ָ��λ��
        memset(xUSART.USART1ReceivedBuffer ,0, U1_RX_BUF_SIZE);                     // �������ݻ������飬����; ׼����һ�εĽ���   
        *cnt = xUSART.USART1ReceivedNum;       // �������ݵ��ֽ��������ָ������   
        xUSART.USART1ReceivedNum = 0;          // ���ձ����0
        return 1;                              // ����1, ��ʾ���յ�������
    }
    return 0;                                  // ����0, ��ʾû�н��յ�������
}

/******************************************************************************
 * ��  ���� vUSART1_SendDatas
 * ��  �ܣ� UARTͨ���жϷ�������,�ʺϸ�����������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע�⻷�λ���������256�ֽڣ��������Ƶ��̫�ߣ�ע�Ⲩ����
 * ��  ���� uint8_t* buffer   �跢�����ݵ��׵�ַ
 *          uint8_t  cnt      ���͵��ֽ��� �������жϷ��͵Ļ�������С�����ܴ���256���ֽ�
 * ����ֵ��
 ******************************************************************************/  
void USART1_SendDatas(uint8_t* buf, uint8_t cnt)
{
    for(uint8_t i=0; i<cnt; i++) 
        U1TxBuffer[U1TxCount++] = buf[i];
     
    if((USART1->CR1 & 1<<7) == 0 )         // ��鷢�ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        USART1->CR1 |= 1<<7;             
}

/******************************************************************************
 * ��  ���� vUSART1_SendString
 * ��  �ܣ� UARTͨ���жϷ�������ַ���,�����������ݳ���
 *         ���ʺϳ������ַ���������<=256�ֽ�
 *         ���� �� �ϡ�int,float����������
 * ��  ���� char* stringTemp   �跢�����ݵĻ����׵�ַ
 * ����ֵ�� Ԫ
 ******************************************************************************/  
#if 0
void USART1_SendString(char *stringTemp)
{
    u16 num=0;                                   // �ַ�������
    char* t=stringTemp ;                         // ������ϼ��㷢�͵�����    
    while(*t++ !=0)  num++;                      // ����Ҫ���͵���Ŀ���ⲽ�ȽϺ�ʱ�����Է���ÿ��6���ֽڣ�����1us����λ��8λ      
    USART1_SendDatas((u8*)stringTemp, num);      // ���ú�����ɷ��ͣ�num+1���ַ�����0��β����෢һ��:0 
}
#else
void USART1_SendString(char *fmt, ...)           // ʹ����printfһ���ĸ�ʽ; ע���޸�h�ļ��еĺ�������
{
    static uint8_t stringTemp[256];              // stringTemp ���ͻ�����
    va_list ap;
    va_start(ap, fmt);
    vsprintf((char *)stringTemp, fmt, ap);
    va_end(ap);
    USART1_SendDatas(stringTemp, strlen((const char *)stringTemp));    
}
#endif



/******************************************************************************
 * ��  ���� vUSART1_SendStringForDMA
 * ��  �ܣ� UARTͨ��DMA�������ݣ�ʡ��ռ���жϵ�ʱ��
 *         ���ʺϳ������ַ������ֽ����ǳ��࣬
 *         ���� �� �ϡ�1:ֻ�ʺϷ����ַ��������ʺϷ��Ϳ��ܺ�0����ֵ������; 2-ʱ����Ҫ�㹻
 * ��  ���� char strintTemp  Ҫ���͵��ַ����׵�ַ
 * ����ֵ�� ��
 ******************************************************************************/  
void USART1_SendStringForDMA(char* stringTemp) 
{
    static u8 Flag_DmaTxInit=0;                  // ���ڱ���Ƿ�������DMA����
    u32   num = 0;                               // ���͵�������ע�ⷢ�͵ĵ�λ���Ǳ���8λ��    
    char* t =stringTemp ;                        // ������ϼ��㷢�͵�����    
    
    while(*t++ !=0)  num++;                      // ����Ҫ���͵���Ŀ���ⲽ�ȽϺ�ʱ�����Է���ÿ��6���ֽڣ�����1us����λ��8λ           

    while(DMA1_Channel4->CNDTR > 0);             // ��Ҫ�����DMA���ڽ����ϴη��ͣ��͵ȴ�; �ý�����ж����־��F4������ô�鷳���������EN�Զ�����
    if( Flag_DmaTxInit == 0)                     // �Ƿ��ѽ��й�USAART_TX��DMA��������
    {   
        Flag_DmaTxInit  = 1;                     // ���ñ�ǣ��´ε��ñ������Ͳ��ٽ���������
        USART1 ->CR3   |= 1<<7;                  // ʹ��DMA����
        RCC->AHBENR    |= 1<<0;                  // ����DMA1ʱ��  [0]DMA1   [1]DMA2        
 
        DMA1_Channel4->CCR   = 0;                // ʧ�ܣ� ��0�����Ĵ���, DMA����ʧ�ܲ�������
        DMA1_Channel4->CNDTR = num;              // ����������   
        DMA1_Channel4->CMAR  = (u32)stringTemp;  // �洢����ַ 
        DMA1_Channel4->CPAR  = (u32)&USART1->DR; // �����ַ      

        DMA1_Channel4->CCR |= 1<<4;              // ���ݴ��䷽��   0:�������   1:�Ӵ洢����
        DMA1_Channel4->CCR |= 0<<5;              // ѭ��ģʽ       0:��ѭ��     1��ѭ��
        DMA1_Channel4->CCR |= 0<<6;              // �����ַ������ģʽ
        DMA1_Channel4->CCR |= 1<<7;              // �洢������ģʽ
        DMA1_Channel4->CCR |= 0<<8;              // �������ݿ��Ϊ8λ
        DMA1_Channel4->CCR |= 0<<10;             // �洢�����ݿ��8λ
        DMA1_Channel4->CCR |= 0<<12;             // �е����ȼ�
        DMA1_Channel4->CCR |= 0<<14;             // �Ǵ洢�����洢��ģʽ    
    }    
    DMA1_Channel4->CCR  &= ~((u32)(1<<0));       // ʧ�ܣ�DMA����ʧ�ܲ�������
    DMA1_Channel4->CNDTR = num;                  // ����������
    DMA1_Channel4->CMAR  = (u32)stringTemp;      // �洢����ַ      
    DMA1_Channel4->CCR  |= 1<<0;                 // ����DMA����   
} 






//////////////////////////////////////////////////////////////   USART-2   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * ��  ���� vUSART2_Init
 * ��  �ܣ� ��ʼ��USART��GPIO��ͨ�Ų������á��ж����ȼ� 
 *          (8λ���ݡ���У�顢1��ֹͣλ)
 * ��  ���� uint32_t baudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/  
void USART2_Init(uint32_t baudrate)
{   
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;    
    
    // ʱ��ʹ��
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;                           // ʹ��USART1ʱ��
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                             // ʹ��GPIOAʱ��

    // GPIO_TX��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                // TX���ţ�����Ϊ�������칤��ģʽ
    GPIO_Init (GPIOA, &GPIO_InitStructure);
    // GPIO_RX��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING ;         // RX���ţ�����Ϊ�������빤��ģʽ
    GPIO_Init (GPIOA, &GPIO_InitStructure);
    
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    // �ж�����
    NVIC_InitStructure .NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority=2 ;       // ��ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 2;             // �����ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);                              
    
    //USART ��ʼ������
    //USART_DeInit(USART2); 
    USART_InitStructure.USART_BaudRate   = baudrate;                // ���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;        // һ��ֹͣλ
    USART_InitStructure.USART_Parity     = USART_Parity_No;         // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // ʹ���ա���ģʽ
    USART_Init(USART2, &USART_InitStructure);                       // ��ʼ������
    
    USART_ITConfig(USART2, USART_IT_TXE , DISABLE );
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                  // ʹ�ܽ����ж�
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);                  // ʹ�ܿ����ж�
   
    USART_Cmd(USART2, ENABLE);                                      // ʹ�ܴ���, ��ʼ����  
    
    USART2->SR = ~(0x00F0);                                         // �����ж�
    
    xUSART.USART2InitFlag =1;                                       // ��ǳ�ʼ����־
    xUSART.USART2ReceivedNum =0;                                    // �����ֽ�������
      
    printf("USART2��ʼ������      �����жϡ������ж�, �����ж�\r");    
}

/******************************************************************************
 * ��  ���� USART2_IRQHandler
 * ��  �ܣ� USART2�Ľ����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
static uint8_t U2TxBuffer[256] ;    // �����жϷ��ͣ����λ�������256���ֽ�
static uint8_t U2TxCounter = 0 ;    // �����жϷ��ͣ�����ѷ��͵��ֽ���(����)
static uint8_t U2TxCount   = 0 ;    // �����жϷ��ͣ���ǽ�Ҫ���͵��ֽ���(����)

void USART2_IRQHandler(void)           
{     
    static uint16_t cnt=0;                                           // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  RxTemp[U2_RX_BUF_SIZE];                          // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽ȫ�ֱ�����xUSART.USARTxReceivedBuffer[xx]�У�
    
    // �����ж�
    if(USART2->SR & (1<<5))                                          // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {
        if((cnt>=U2_RX_BUF_SIZE))//||(xUSART.USART2ReceivedFlag==1)) // �ж�1: ��ǰ֡�ѽ��յ���������������(������), Ϊ�������������������յ�������ֱ��������
        {                                                            // �ж�2: ���֮ǰ���պõ����ݰ���û�����ͷ��������ݣ�����������֡���ܸ��Ǿ�����֡��ֱ��������֡������ȱ�㣺���ݴ�������ڴ����ٶ�ʱ��������ô����������������ڵ���
            USART2->DR;                                              // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;
        } 
        RxTemp[cnt++] = USART2->DR ;                                 // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ��
    }
    
    // �����ж�, ������Ͻ����жϣ����ж�һ֡���ݵĽ������
    if(USART2->SR & (1<<4))                                          // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {  
        xUSART.USART2ReceivedNum  = 0;                               // �ѽ��յ��������ֽ�����0     
        memcpy(xUSART.USART2ReceivedBuffer, RxTemp, U2_RX_BUF_SIZE); // �ѱ�֡���յ������ݣ���ŵ�ȫ�ֱ���xUSART.USARTxReceivedBuffer��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ�������
        xUSART.USART2ReceivedNum  = cnt;                             // �ѽ��յ����ֽ�������ŵ�ȫ�ֱ���xUSART.USARTxReceivedNum�У�     
        cnt=0;                                                       // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(RxTemp ,0, U2_RX_BUF_SIZE);                           // �������ݻ������飬����; ׼����һ�εĽ���   
        USART2 ->SR;  USART2 ->DR;                                   // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!      
     }     

    // �����ж�
    if ((USART2->SR & 1<<7) && (USART2->CR1 & 1<<7))                 // ���TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                
        USART2->DR = U2TxBuffer[U2TxCounter++];                      // ��ȡ���ݼĴ���ֵ��ע�⣺��ȡDRʱ�Զ������ж�λ��        
        if(U2TxCounter == U2TxCount )
            USART2->CR1 &= ~(1<<7);                                  // �ѷ�����ɣ��رշ��ͻ����������ж� TXEIE
    }    
}  

/******************************************************************************
 * ��  ���� vUSART2_GetBuffer
 * ��  �ܣ� ��ȡUART�����յ�������
 * ��  ���� uint8_t* buffer   ���ݴ�Ż����ַ
 *          uint8_t* cnt      ���յ����ֽ��� 
 * ����ֵ�� 0_û�н��յ������ݣ� 1_���յ�������
 ******************************************************************************/  
uint8_t USART2_GetBuffer(uint8_t* buffer, uint8_t* cnt)
{    
    if(xUSART.USART2ReceivedNum)                // �ж��Ƿ���������
    {    
        memcpy(buffer, xUSART.USART2ReceivedBuffer, xUSART.USART2ReceivedNum );  // �������ݸ��Ƶ�ָ��λ��
        memset(xUSART.USART2ReceivedBuffer ,0, U2_RX_BUF_SIZE);                    // �������ݻ������飬����; ׼����һ�εĽ���   
        *cnt = xUSART.USART2ReceivedNum;       // �������ݵ��ֽ��������ָ������   
        xUSART.USART2ReceivedNum =0;           // ���ձ����0
        return 1;                              // ����1, ��ʾ���յ�������
    }
    return 0;                                  // ����0, ��ʾû�н��յ�������
}

/******************************************************************************
 * ��  ���� vUSART2_SendData
 * ��  �ܣ� UARTͨ���жϷ�������,�ʺϸ�����������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע�⻷�λ���������256�ֽڣ��������Ƶ��̫�ߣ�ע�Ⲩ����
 * ��  ���� uint8_t* buffer   �跢�����ݵ��׵�ַ
 *          uint8_t  cnt      ���͵��ֽ��� �������жϷ��͵Ļ�������С�����ܴ���256���ֽ�
 * ����ֵ��
 ******************************************************************************/  
void USART2_SendData(uint8_t* buf, uint8_t cnt)
{
    for(uint8_t i=0; i<cnt; i++) 
        U2TxBuffer[U2TxCount++] = buf[i];
     
    if((USART2->CR1 & 1<<7) == 0 )         // ��鷢�ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        USART2->CR1 |= 1<<7;             
}

/******************************************************************************
 * ��  ���� vUSART2_SendString
 * ��  �ܣ� UARTͨ���жϷ�������ַ���,�����������ݳ���
 *         ���ʺϳ������ַ���������<=256�ֽ�
 *         ���� �� �ϡ�int,float����������
 * ��  ���� char* stringTemp   �跢�����ݵĻ����׵�ַ
 * ����ֵ�� Ԫ
 ******************************************************************************/  
void USART2_SendString(char* stringTemp)
{
    u16 num=0;                                   // �ַ�������
    char* t=stringTemp ;                         // ������ϼ��㷢�͵�����    
    while(*t++ !=0)  num++;                      // ����Ҫ���͵���Ŀ���ⲽ�ȽϺ�ʱ�����Է���ÿ��6���ֽڣ�����1us����λ��8λ      
    USART2_SendData((u8*)stringTemp, num+1);     // ���ú�����ɷ��ͣ�num+1���ַ�����0��β����෢һ��:0   
}





//////////////////////////////////////////////////////////////   USART-3   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * ��  ���� vUSART3_Init
 * ��  �ܣ� ��ʼ��USART��GPIO��ͨ�Ų������á��ж����ȼ� 
 *          (8λ���ݡ���У�顢1��ֹͣλ)
 * ��  ���� uint32_t baudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/  
void USART3_Init(uint32_t baudrate)
{   
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;    
    
    // ʱ��ʹ��
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;                           // ʹ��USART1ʱ��
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                             // ʹ��GPIOAʱ��

    // GPIO_TX��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                // TX���ţ�����Ϊ�������칤��ģʽ
    GPIO_Init (GPIOB, &GPIO_InitStructure);
    // GPIO_RX��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING ;         // RX���ţ�����Ϊ�������빤��ģʽ
    GPIO_Init (GPIOB, &GPIO_InitStructure);
    
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    // �ж�����
    NVIC_InitStructure .NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority=2 ;       // ��ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 2;             // �����ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);                              
    
    //USART ��ʼ������
    USART_DeInit(USART3); 
    USART_InitStructure.USART_BaudRate   = baudrate;                // ���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;        // һ��ֹͣλ
    USART_InitStructure.USART_Parity     = USART_Parity_No;         // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // ʹ���ա���ģʽ
    USART_Init(USART3, &USART_InitStructure);                       // ��ʼ������
    
    USART_ITConfig(USART3, USART_IT_TXE , DISABLE );
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                  // ʹ�ܽ����ж�
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);                  // ʹ�ܿ����ж�
    
    USART_Cmd(USART3, ENABLE);                                      // ʹ�ܴ���, ��ʼ���� 
    
    USART3->SR = ~(0x00F0);                                         // �����ж�

    xUSART.USART3InitFlag =1;                                       // ��ǳ�ʼ����־
    xUSART.USART3ReceivedNum =0;                                    // �����ֽ�������
      
    printf("USART3��ʼ������      �����жϡ������ж�, �����ж�\r");    
}

/******************************************************************************
 * ��  ���� USART3_IRQHandler
 * ��  �ܣ� USART�Ľ����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
static uint8_t U3TxBuffer[256] ;    // �����жϷ��ͣ����λ�������256���ֽ�
static uint8_t U3TxCounter = 0 ;    // �����жϷ��ͣ�����ѷ��͵��ֽ���(����)
static uint8_t U3TxCount   = 0 ;    // �����жϷ��ͣ���ǽ�Ҫ���͵��ֽ���(����)

void USART3_IRQHandler(void)           
{     
    static uint16_t cnt=0;                                           // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  RxTemp[U3_RX_BUF_SIZE];                          // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽ȫ�ֱ�����xUSART.USARTxReceivedBuffer[xx]�У�
    
    // �����ж�
    if(USART3->SR & (1<<5))                                          // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {
        if((cnt>=U3_RX_BUF_SIZE))//||(xUSART.USART3ReceivedFlag==1)) // �ж�1: ��ǰ֡�ѽ��յ���������������(������), Ϊ�������������������յ�������ֱ��������
        {                                                            // �ж�2: ���֮ǰ���պõ����ݰ���û�����ͷ��������ݣ�����������֡���ܸ��Ǿ�����֡��ֱ��������֡������ȱ�㣺���ݴ�������ڴ����ٶ�ʱ��������ô����������������ڵ���
            USART3->DR;                                              // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;
        } 
        RxTemp[cnt++] = USART3->DR ;                                 // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ��
    }
    
    // �����ж�, ������Ͻ����жϣ����ж�һ֡���ݵĽ������
    if(USART3->SR & (1<<4))                                          // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {        
        xUSART.USART3ReceivedNum  = 0;                               // �ѽ��յ��������ֽ�����0                
        memcpy(xUSART.USART3ReceivedBuffer, RxTemp, U3_RX_BUF_SIZE); // �ѱ�֡���յ������ݣ���ŵ�ȫ�ֱ���xUSART.USARTxReceivedBuffer��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ�������
        xUSART.USART3ReceivedNum  = cnt;                             // �ѽ��յ����ֽ�������ŵ�ȫ�ֱ���xUSART.USARTxReceivedNum�У�     
        cnt=0;                                                       // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(RxTemp ,0, U3_RX_BUF_SIZE);                           // �������ݻ������飬����; ׼����һ�εĽ���   
        USART3 ->SR;  USART3 ->DR;                                   // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!      
     }     

    // �����ж�
    if ((USART3->SR & 1<<7) && (USART3->CR1 & 1<<7))                 // ���TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                
        USART3->DR = U3TxBuffer[U3TxCounter++];                      // ��ȡ���ݼĴ���ֵ��ע�⣺��ȡDRʱ�Զ������ж�λ��        
        if(U3TxCounter == U3TxCount )
            USART3->CR1 &= ~(1<<7);                                  // �ѷ�����ɣ��رշ��ͻ����������ж� TXEIE
    }    
}    

/******************************************************************************
 * ��  ���� vUSART3_GetBuffer
 * ��  �ܣ� ��ȡUART�����յ�������
 * ��  ���� uint8_t* buffer   ���ݴ�Ż����ַ
 *          uint8_t* cnt      ���յ����ֽ��� 
 * ����ֵ�� 0_û�н��յ������ݣ� 1_���յ�������
 ******************************************************************************/  
uint8_t USART3_GetBuffer(uint8_t* buffer, uint8_t* cnt)
{    
    if(xUSART.USART3ReceivedNum)               // �ж��Ƿ���������
    {    
        memcpy(buffer, xUSART.USART3ReceivedBuffer, xUSART.USART3ReceivedNum );  // �������ݸ��Ƶ�ָ��λ��
        memset(xUSART.USART3ReceivedBuffer ,0, U3_RX_BUF_SIZE);                     // �������ݻ������飬����; ׼����һ�εĽ���     
        *cnt = xUSART.USART3ReceivedNum;       // �������ݵ��ֽ��������ָ������   
        xUSART.USART3ReceivedNum =0;           // ���ձ����0
        return 1;                              // ����1, ��ʾ���յ�������
    }
    return 0;                                  // ����0, ��ʾû�н��յ�������
}

/******************************************************************************
 * ��  ���� vUSART3_SendData
 * ��  �ܣ� UARTͨ���жϷ�������,�ʺϸ�����������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע�⻷�λ���������256�ֽڣ��������Ƶ��̫�ߣ�ע�Ⲩ����
 * ��  ���� uint8_t* buffer   �跢�����ݵ��׵�ַ
 *          uint8_t  cnt      ���͵��ֽ��� �������жϷ��͵Ļ�������С�����ܴ���256���ֽ�
 * ����ֵ��
 ******************************************************************************/  
void USART3_SendData(uint8_t* buf, uint8_t cnt)
{
    for(uint8_t i=0; i<cnt; i++) 
        U3TxBuffer[U3TxCount++] = buf[i];
     
    if((USART3->CR1 & 1<<7) == 0 )         // ��鷢�ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        USART3->CR1 |= 1<<7;             
}

/******************************************************************************
 * ��  ���� vUSART3_SendString
 * ��  �ܣ� UARTͨ���жϷ�������ַ���,�����������ݳ���
 *         ���ʺϳ������ַ���������<=256�ֽ�
 *         ���� �� �ϡ�int,float����������
 * ��  ���� char* stringTemp   �跢�����ݵĻ����׵�ַ
 * ����ֵ�� Ԫ
 ******************************************************************************/  
void USART3_SendString(char* stringTemp)
{
    u16 num=0;                                   // �ַ�������
    char* t=stringTemp ;                         // ������ϼ��㷢�͵�����    
    while(*t++ !=0)  num++;                      // ����Ҫ���͵���Ŀ���ⲽ�ȽϺ�ʱ�����Է���ÿ��6���ֽڣ�����1us����λ��8λ      
    USART3_SendData((u8*)stringTemp, num+1);     // ���ú�����ɷ��ͣ�num+1���ַ�����0��β����෢һ��:0   
}




#ifdef STM32F10X_HD  // STM32F103R�������ϣ�����UART4��UART5

//////////////////////////////////////////////////////////////   UART-4   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * ��  ���� vUART4_Init
 * ��  �ܣ� ��ʼ��USART��GPIO��ͨ�Ų������á��ж����ȼ� 
 *          (8λ���ݡ���У�顢1��ֹͣλ)
 * ��  ���� uint32_t baudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/  
void UART4_Init(uint32_t baudrate)
{   
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;    
    
    // ʱ��ʹ��
    RCC->APB1ENR |= RCC_APB1ENR_UART4EN;                            // ʹ��USARTʱ��
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                             // ʹ��GPIOAʱ��

    // GPIO_TX��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                // TX���ţ�����Ϊ�������칤��ģʽ
    GPIO_Init (GPIOC, &GPIO_InitStructure);
    // GPIO_RX��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING ;         // RX���ţ�����Ϊ�������빤��ģʽ
    GPIO_Init (GPIOC, &GPIO_InitStructure);
                       
    //USART ��ʼ������
    USART_DeInit(UART4); 
    USART_InitStructure.USART_BaudRate   = baudrate;                // ���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;        // һ��ֹͣλ
    USART_InitStructure.USART_Parity     = USART_Parity_No;         // ����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // ʹ���ա���ģʽ
    USART_Init(UART4, &USART_InitStructure);                        // ��ʼ������
    
    USART_ITConfig(UART4, USART_IT_TXE , DISABLE );
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);                   // ʹ�ܽ����ж�
    USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);                   // ʹ�ܿ����ж�
    
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    // �ж�����
    NVIC_InitStructure .NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority=2 ;       // ��ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 2;             // �����ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);  
    
    USART_Cmd(UART4, ENABLE);                                       // ʹ�ܴ���, ��ʼ���� 

    UART4->SR = ~(0x00F0);                                          // �����ж�
    
    xUSART.UART4InitFlag =1;                                        // ��ǳ�ʼ����־
    xUSART.UART4ReceivedNum =0;                                     // �����ֽ�������
      
    printf("UART4 ��ʼ������      �����жϡ������ж�, �����ж�\r");    
}

/******************************************************************************
 * ��  ���� UART4_IRQHandler
 * ��  �ܣ� USART2�Ľ����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
static uint8_t U4TxBuffer[256] ;    // �����жϷ��ͣ����λ�������256���ֽ�
static uint8_t U4TxCounter = 0 ;    // �����жϷ��ͣ�����ѷ��͵��ֽ���(����)
static uint8_t U4TxCount   = 0 ;    // �����жϷ��ͣ���ǽ�Ҫ���͵��ֽ���(����)

void UART4_IRQHandler(void)           
{     
    static uint16_t cnt=0;                                          // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  RxTemp[U4_RX_BUF_SIZE];                         // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽ȫ�ֱ�����xUSART.USARTxReceivedBuffer[xx]�У�
    
    // �����ж�
    if(UART4->SR & (1<<5))                                          // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {
        if((cnt>=U4_RX_BUF_SIZE))//||(xUSART.UART5ReceivedFlag==1)) // �ж�1: ��ǰ֡�ѽ��յ���������������(������), Ϊ�������������������յ�������ֱ��������
        {                                                           // �ж�2: ���֮ǰ���պõ����ݰ���û�����ͷ��������ݣ�����������֡���ܸ��Ǿ�����֡��ֱ��������֡������ȱ�㣺���ݴ�������ڴ����ٶ�ʱ��������ô����������������ڵ���
            UART4->DR;                                              // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;
        } 
        RxTemp[cnt++] = UART4->DR ;                                 // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ��
    }
    
    // �����ж�, ������Ͻ����жϣ����ж�һ֡���ݵĽ������
    if(UART4->SR & (1<<4))                                          // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {         
        xUSART.UART4ReceivedNum  = 0;                               // �ѽ��յ��������ֽ�����0    
        memcpy(xUSART.UART4ReceivedBuffer, RxTemp, U4_RX_BUF_SIZE); // �ѱ�֡���յ������ݣ���ŵ�ȫ�ֱ���xUSART.USARTxReceivedBuffer��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ�������
        xUSART.UART4ReceivedNum  = cnt;                             // �ѽ��յ����ֽ�������ŵ�ȫ�ֱ���xUSART.USARTxReceivedNum�У�     
        cnt=0;                                                      // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(RxTemp ,0, U4_RX_BUF_SIZE);                          // �������ݻ������飬����; ׼����һ�εĽ���   
        UART4 ->SR;  UART4 ->DR;                                    // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!      
     }     

    // �����ж�
    if ((UART4->SR & 1<<7) && (UART4->CR1 & 1<<7))                  // ���TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                
        UART4->DR = U4TxBuffer[U4TxCounter++];                      // ��ȡ���ݼĴ���ֵ��ע�⣺��ȡDRʱ�Զ������ж�λ��        
        if(U4TxCounter == U4TxCount )
            UART4->CR1 &= ~(1<<7);                                  // �ѷ�����ɣ��رշ��ͻ����������ж� TXEIE
    }    
}  

/******************************************************************************
 * ��  ���� vUART4_GetBuffer
 * ��  �ܣ� ��ȡUART�����յ�������
 * ��  ���� uint8_t* buffer   ���ݴ�Ż����ַ
 *          uint8_t* cnt      ���յ����ֽ��� 
 * ����ֵ�� 0_û�н��յ������ݣ� 1_���յ�������
 ******************************************************************************/  
uint8_t UART4_GetBuffer(uint8_t* buffer, uint8_t* cnt)
{    
    if(xUSART.UART4ReceivedNum)               // �ж��Ƿ���������
    {    
        memcpy(buffer, xUSART.UART4ReceivedBuffer, xUSART.UART4ReceivedNum );     // �������ݸ��Ƶ�ָ��λ��
        memset(xUSART.UART4ReceivedBuffer ,0, U4_RX_BUF_SIZE);                    // �������ݻ������飬����; ׼����һ�εĽ���   
        *cnt = xUSART.UART4ReceivedNum;       // �������ݵ��ֽ��������ָ������   
        xUSART.UART4ReceivedNum =0;           // ���ձ����0
        return 1;                             // ����1, ��ʾ���յ�������
    }
    return 0;                                 // ����0, ��ʾû�н��յ�������
}

/******************************************************************************
 * ��  ���� vUART4_SendData
 * ��  �ܣ� UARTͨ���жϷ�������,�ʺϸ�����������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע�⻷�λ���������256�ֽڣ��������Ƶ��̫�ߣ�ע�Ⲩ����
 * ��  ���� uint8_t* buffer   �跢�����ݵ��׵�ַ
 *          uint8_t  cnt      ���͵��ֽ��� �������жϷ��͵Ļ�������С�����ܴ���256���ֽ�
 * ����ֵ��
 ******************************************************************************/  
void UART4_SendData(uint8_t* buf, uint8_t cnt)
{
    for(uint8_t i=0; i<cnt; i++) 
        U4TxBuffer[U4TxCount++] = buf[i];
     
    if((UART4->CR1 & 1<<7) == 0 )         // ��鷢�ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        UART4->CR1 |= 1<<7;             
}

/******************************************************************************
 * ��  ���� vUART4_SendString
 * ��  �ܣ� UARTͨ���жϷ�������ַ���,�����������ݳ���
 *         ���ʺϳ������ַ���������<=256�ֽ�
 *         ���� �� �ϡ�int,float����������
 * ��  ���� char* stringTemp   �跢�����ݵĻ����׵�ַ
 * ����ֵ�� Ԫ
 ******************************************************************************/  
void UART4_SendString(char* stringTemp)
{
    u16 num=0;                                   // �ַ�������
    char* t = stringTemp ;                       // ������ϼ��㷢�͵�����    
    while(*t++ !=0)  num++;                      // ����Ҫ���͵���Ŀ���ⲽ�ȽϺ�ʱ�����Է���ÿ��6���ֽڣ�����1us����λ��8λ      
    UART4_SendData((u8*)stringTemp, num+1);      // ���ú�����ɷ��ͣ�num+1���ַ�����0��β����෢һ��:0   
}




//////////////////////////////////////////////////////////////   UART-4   //////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * ��  ���� vUART5_Init
 * ��  �ܣ� ��ʼ��USART��GPIO��ͨ�Ų������á��ж����ȼ� 
 *          (8λ���ݡ���У�顢1��ֹͣλ)
 * ��  ���� uint32_t baudrate  ͨ�Ų�����
 * ����ֵ�� ��
 ******************************************************************************/  
void UART5_Init(uint32_t baudrate)
{   
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    USART_InitTypeDef USART_InitStructure;    
    
    // ʱ��ʹ��
    RCC->APB1ENR |= RCC_APB1ENR_UART5EN;                            // ʹ��USART1ʱ��
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPCEN;        // ʹ��GPIOʱ��

    // GPIO_TX��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;                // TX���ţ�����Ϊ�������칤��ģʽ
    GPIO_Init (GPIOC, &GPIO_InitStructure);
    // GPIO_RX��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING ;         // RX���ţ�����Ϊ�������빤��ģʽ
    GPIO_Init (GPIOD, &GPIO_InitStructure);
       
    //USART ��ʼ������
    USART_DeInit(UART5); 
    USART_InitStructure.USART_BaudRate   = baudrate;                // ���ڲ�����
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;     // �ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;        // һ��ֹͣλ
    USART_InitStructure.USART_Parity     = USART_Parity_No;         // ����żУ��λ
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // ʹ���ա���ģʽ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART5, &USART_InitStructure);                        // ��ʼ������
    
    USART_ITConfig(UART5, USART_IT_TXE , DISABLE );
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);                   // ʹ�ܽ����ж�
    USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);                   // ʹ�ܿ����ж�
    
    // NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    // �ж�����
    NVIC_InitStructure .NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure .NVIC_IRQChannelPreemptionPriority=2 ;       // ��ռ���ȼ�
    NVIC_InitStructure .NVIC_IRQChannelSubPriority = 2;             // �����ȼ�
    NVIC_InitStructure .NVIC_IRQChannelCmd = ENABLE;                // IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);           
    
    USART_Cmd(UART5, ENABLE);                                       // ʹ�ܴ���, ��ʼ����  
    
    UART5->SR = ~(0x00F0);                                          // �����ж�
    
    xUSART.UART5InitFlag =1;                                        // ��ǳ�ʼ����־
    xUSART.UART5ReceivedNum =0;                                     // �����ֽ�������
      
    printf("UART5 ��ʼ������      �����жϡ������ж�, �����ж�\r");    
}

/******************************************************************************
 * ��  ���� UART5_IRQHandler
 * ��  �ܣ� USART2�Ľ����жϡ������жϡ������ж�
 * ��  ���� ��
 * ����ֵ�� ��
 ******************************************************************************/
static uint8_t U5TxBuffer[256] ;    // �����жϷ��ͣ����λ�������256���ֽ�
static uint8_t U5TxCounter = 0 ;    // �����жϷ��ͣ�����ѷ��͵��ֽ���(����)
static uint8_t U5TxCount   = 0 ;    // �����жϷ��ͣ���ǽ�Ҫ���͵��ֽ���(����)

void UART5_IRQHandler(void)           
{     
    static uint16_t cnt=0;                                          // �����ֽ����ۼƣ�ÿһ֡�����ѽ��յ����ֽ���
    static uint8_t  RxTemp[U5_RX_BUF_SIZE];                         // �������ݻ������飺ÿ�½��գ����ֽڣ���˳���ŵ������һ֡������(���������ж�), ��ת�浽ȫ�ֱ�����xUSART.USARTxReceivedBuffer[xx]�У�
    
    // �����ж�
    if(UART5->SR & (1<<5))                                          // ���RXNE(�����ݼĴ����ǿձ�־λ); RXNE�ж�����������DRʱ�Զ�����
    {
        if((cnt>=U5_RX_BUF_SIZE))//||(xUSART.UART5ReceivedFlag==1)) // �ж�1: ��ǰ֡�ѽ��յ���������������(������), Ϊ�������������������յ�������ֱ��������
        {                                                           // �ж�2: ���֮ǰ���պõ����ݰ���û�����ͷ��������ݣ�����������֡���ܸ��Ǿ�����֡��ֱ��������֡������ȱ�㣺���ݴ�������ڴ����ٶ�ʱ��������ô����������������ڵ���
            UART5->DR;                                              // ��ȡ���ݼĴ��������ݣ��������森��Ҫ���ã���DRʱ�Զ���������жϱ�־��
            return;
        } 
        RxTemp[cnt++] = UART5->DR ;                                 // �����յ����ֽ����ݣ�˳���ŵ�RXTemp�����У�ע�⣺��ȡDRʱ�Զ������ж�λ��
    }
    
    // �����ж�, ������Ͻ����жϣ����ж�һ֡���ݵĽ������
    if(UART5->SR & (1<<4))                                          // ���IDLE(�����жϱ�־λ); IDLE�жϱ�־���������������㣬USART1 ->SR;  USART1 ->DR;
    {   
        xUSART.UART5ReceivedNum  = 0;                               // �ѽ��յ��������ֽ�����0    
        memcpy(xUSART.UART5ReceivedBuffer, RxTemp, U5_RX_BUF_SIZE); // �ѱ�֡���յ������ݣ���ŵ�ȫ�ֱ���xUSART.USARTxReceivedBuffer��, �ȴ�����; ע�⣺���Ƶ����������飬����0ֵ���Է����ַ�������
        xUSART.UART5ReceivedNum  = cnt;                             // �ѽ��յ����ֽ�������ŵ�ȫ�ֱ���xUSART.USARTxReceivedNum�У�
        cnt=0;                                                      // �����ֽ����ۼ���������; ׼����һ�εĽ���
        memset(RxTemp ,0, U5_RX_BUF_SIZE);                          // �������ݻ������飬����; ׼����һ�εĽ���   
        UART5 ->SR;  UART5 ->DR;                                    // ����IDLE�жϱ�־λ!! �������㣬˳���ܴ�!!      
     }     

    // �����ж�
    if ((UART5->SR & 1<<7) && (UART5->CR1 & 1<<7))                  // ���TXE(�������ݼĴ�����)��TXEIE(���ͻ��������ж�ʹ��)
    {                
        UART5->DR = U5TxBuffer[U5TxCounter++];                      // ��ȡ���ݼĴ���ֵ��ע�⣺��ȡDRʱ�Զ������ж�λ��        
        if(U5TxCounter == U5TxCount )
            UART5->CR1 &= ~(1<<7);                                  // �ѷ�����ɣ��رշ��ͻ����������ж� TXEIE
    }    
}    

/******************************************************************************
 * ��  ���� vUART5_GetBuffer
 * ��  �ܣ� ��ȡUART�����յ�������
 * ��  ���� uint8_t* buffer   ���ݴ�Ż����ַ
 *          uint8_t* cnt      ���յ����ֽ��� 
 * ����ֵ�� 0_û�н��յ������ݣ� 1_���յ�������
 ******************************************************************************/  
uint8_t UART5_GetBuffer(uint8_t* buffer, uint8_t* cnt)
{    
    if(xUSART.UART5ReceivedNum)                // �ж��Ƿ���������
    {    
        memcpy(buffer, xUSART.UART5ReceivedBuffer, xUSART.UART5ReceivedNum );     // �������ݸ��Ƶ�ָ��λ��
        memset(xUSART.UART5ReceivedBuffer ,0, U5_RX_BUF_SIZE);                    // �������ݻ������飬����; ׼����һ�εĽ���   
        *cnt = xUSART.UART5ReceivedNum;        // �������ݵ��ֽ��������ָ������   
        xUSART.UART5ReceivedNum =0;            // ���ձ����0
        return 1;                              // ����1, ��ʾ���յ�������
    }
    return 0;                                  // ����0, ��ʾû�н��յ�������
}

/******************************************************************************
 * ��  ���� vUART5_SendData
 * ��  �ܣ� UARTͨ���жϷ�������,�ʺϸ�����������
 *         ���ʺϳ������������ɷ��͸������ݣ����������ַ�������int,char
 *         ���� �� �ϡ�ע�⻷�λ���������256�ֽڣ��������Ƶ��̫�ߣ�ע�Ⲩ����
 * ��  ���� uint8_t* buffer   �跢�����ݵ��׵�ַ
 *          uint8_t  cnt      ���͵��ֽ��� �������жϷ��͵Ļ�������С�����ܴ���256���ֽ�
 * ����ֵ��
 ******************************************************************************/  
void UART5_SendData(uint8_t* buf, uint8_t cnt)
{
    for(uint8_t i=0; i<cnt; i++) 
        U5TxBuffer[U5TxCount++] = buf[i];
     
    if((UART5->CR1 & 1<<7) == 0 )         // ��鷢�ͻ����������ж�(TXEIE)�Ƿ��Ѵ�
        UART5->CR1 |= 1<<7;             
}

/******************************************************************************
 * ��  ���� vUART5_SendString
 * ��  �ܣ� UARTͨ���жϷ�������ַ���,�����������ݳ���
 *         ���ʺϳ������ַ���������<=256�ֽ�
 *         ���� �� �ϡ�int,float����������
 * ��  ���� char* stringTemp   �跢�����ݵĻ����׵�ַ
 * ����ֵ�� Ԫ
 ******************************************************************************/  
void UART5_SendString(char* stringTemp)
{
    u16 num=0;                                   // �ַ�������
    char* t=stringTemp ;                         // ������ϼ��㷢�͵�����    
    while(*t++ !=0)  num++;                      // ����Ҫ���͵���Ŀ���ⲽ�ȽϺ�ʱ�����Է���ÿ��6���ֽڣ�����1us����λ��8λ      
    UART5_SendData((u8*)stringTemp, num+1);      // ���ú�����ɷ��ͣ�num+1���ַ�����0��β����෢һ��:0   
}

#endif





//////////////////////////////////////////////////////////////  printf   //////////////////////////////////////////////////////////////
/******************************************************************************
 * ��  �ܣ� printf����֧�ִ���
 *         ���ر�ע�⡿�������´���, ʹ��printf����ʱ, ������Ҫѡ��use MicroLIB     
 * ��  ���� 
 * ����ֵ��
 * ��  ע�� ħŮ�������Ŷ�  ���ϴ��QȺ��1126717453      ����޸�_2020��07��15��
 ******************************************************************************/  
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB     
#pragma import(__use_no_semihosting)                
struct __FILE       { int handle; };         // ��׼����Ҫ��֧�ֺ���
FILE __stdout;                               // FILE ��stdio.h�ļ�
void _sys_exit(int x) {    x = x; }          // ����_sys_exit()�Ա���ʹ�ð�����ģʽ

int fputc(int ch, FILE *f)                   // �ض���fputc������ʹprintf���������fputc�����UART,  ����ʹ�ô���1(USART1)
{ 
    #if 1                                    // ��ʽ1-ʹ�ó��õ�poll��ʽ�������ݣ��Ƚ�������⣬���ȴ���ʱ��  
        while((USARTx_DEBUG->SR & 0X40)==0); // �ȴ���һ�δ������ݷ������ 
        USARTx_DEBUG->DR = (u8) ch;          // дDR,����1���������� 
        return ch;
    #else                                    // ��ʽ2-ʹ��queue+�жϷ�ʽ��������; ������ʽ1�����ȴ���ʱ����Ҫ������д�õĺ��������λ���
        uint8_t c[1]={(uint8_t)ch};    
        if(USARTx_DEBUG == USART1)    vUSART1_SendData (c, 1);
        if(USARTx_DEBUG == USART2)    vUSART2_SendData (c, 1);
        if(USARTx_DEBUG == USART3)    vUSART3_SendData (c, 1);
        if(USARTx_DEBUG == UART4)     vUART4_SendData  (c, 1);
        if(USARTx_DEBUG == UART5)     vUART5_SendData  (c, 1);
        return ch;
    #endif    
}

