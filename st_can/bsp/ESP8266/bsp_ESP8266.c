#include "bsp_ESP8266.h"

xESP8266_TypeDef  xESP8266;  


// ����US������ʱ������������ֲʱ���ⲿ�ļ�������
#if 0
static void delayUS( __IO u32 times)
{
    times=times*7;      //  10us����7;
    while(--times)         
        __nop();  
}
#endif 

#if 1
// ms��ʱ������������ֲʱ���ⲿ�ļ�������
static void delayMS(u32 ms)
{
    ms=ms*6500;                  
    for(u32 i=0; i<ms; i++);      // 72MHzϵͳʱ���£����ٸ���ѭ��Լ��ʱ1ms
}
#endif 





uint8_t ESP8266_CMD(char* cmdString, char* answerString, uint32_t waitTimesMS)
{
    uint16_t CNT=0;                     // �ȴ���ʱ����
    ESP8266_CleanReceivedFlag();        // ���ձ�ʾ��0   
    ESP8266_SendString(cmdString);      // ����ATָ��        
    while(1)                            // �ȴ�ָ���ִ�����
    {
        if(ESP8266_CheckReceivedNum())  
        {
            if(strstr((char*)xESP8266.ReceivedBuffer, answerString)!=0)  
                return 1;
        }
        delayMS(1);
        if(++CNT > waitTimesMS)    return 0;  // ��ʱδ�յ���ȷ���ݣ����ش���ֵ��0  
    }
}






void ESP8266_Init(USART_TypeDef* USARTx, uint32_t baudrate)
{    
    printf("\r\nESP8266 ��ʼ���ü�����......\r\n");              
    delayMS(300);                                    // ��Ҫ���ϵ�󣬱�������ʱ�Եȴ�8266�ȶ����ɹ���
        
    if(USARTx == USART1)  { USART1_Init(baudrate); }   
    if(USARTx == USART2)  { USART2_Init(baudrate); }   
    if(USARTx == USART3)  { USART3_Init(baudrate); }   
    #ifdef STM32F10X_HD
    if(USARTx == UART4)   { UART4_Init(baudrate);  }   
    if(USARTx == UART5)   { UART5_Init(baudrate);  }   
    #endif   
    
    xESP8266.USARTx = USARTx;                        // ��¼���ô��ڶ˿�
    xESP8266.Baudrate = baudrate;                    // ��¼���õĲ�����
    xESP8266.Flag_FinishInit =1;                     // ��ǳ�ʼ����־
    xESP8266.APName = ESP8266_AP_NAME ;
    xESP8266.APPassword = ESP8266_AP_PASSWORD ;
    
    ESP8266_CMD("AT\r\n",            "OK", 1000)  ? printf("ģ�����Ӳ���: �ɹ�\r\n") : printf("ģ�����Ӳ���: ʧ��\r\n");   // ����
    
        
//   printf( "\r\n�������� ESP8266 ......\r\n" );


//   printf( "\r\n�������ù���ģʽ STA ......\r\n" );
//   while( ! ESP8266_Net_Mode_Choose ( STA ) );

//   printf( "\r\n�������� WiFi ......\r\n" );
//   while( ! ESP8266_JoinAP ( macUser_ESP8266_ApSsid, macUser_ESP8266_ApPwd ) );

//   printf( "\r\n��ֹ������ ......\r\n" );
//   while( ! ESP8266_Enable_MultipleId ( DISABLE ) );

//   printf( "\r\n�������� Server ......\r\n" );
//   while( !  ESP8266_Link_Server ( enumTCP, macUser_ESP8266_TcpServer_IP, macUser_ESP8266_TcpServer_Port, Single_ID_0 ) );

//   printf( "\r\n����͸������ģʽ ......\r\n" );
//   while( ! ESP8266_UnvarnishSend () );

//   printf( "\r\n���� ESP8266 ���\r\n" );
//   printf ( "\r\n��ʼ͸��......\r\n" );
//       
}


void ESP8266_SendString(char* str)
{
    if(xESP8266.USARTx== USART1) { USART1_SendString(str); }
    if(xESP8266.USARTx== USART2) { USART2_SendString(str); }
    if(xESP8266.USARTx== USART3) { USART3_SendString(str); }
    #ifdef STM32F10X_HD
    if(xESP8266.USARTx== UART4)  { UART4_SendString(str);  }
    if(xESP8266.USARTx== UART5)  { UART5_SendString(str);  }
    #endif          
}

// ����Ƿ��յ�ESP8266�����������ݣ���־
// ���ؽ��յ����ֽ�����
uint16_t ESP8266_CheckReceivedNum(void)
{
    if((xESP8266.USARTx== USART1) && (xUSART.USART1ReceivedNum))   {xESP8266.ReceivedNum = xUSART.USART1ReceivedNum;  memcpy(xESP8266.ReceivedBuffer, xUSART.USART1ReceivedBuffer, xESP8266.ReceivedNum);  xUSART.USART1ReceivedNum=0;}
    if((xESP8266.USARTx== USART2) && (xUSART.USART2ReceivedNum))   {xESP8266.ReceivedNum = xUSART.USART2ReceivedNum;  memcpy(xESP8266.ReceivedBuffer, xUSART.USART2ReceivedBuffer, xESP8266.ReceivedNum);  xUSART.USART2ReceivedNum=0;}
    if((xESP8266.USARTx== USART3) && (xUSART.USART3ReceivedNum))   {xESP8266.ReceivedNum = xUSART.USART3ReceivedNum;  memcpy(xESP8266.ReceivedBuffer, xUSART.USART3ReceivedBuffer, xESP8266.ReceivedNum);  xUSART.USART3ReceivedNum=0;}
    #ifdef STM32F10X_HD
    if((xESP8266.USARTx== UART4)  && (xUSART.UART4ReceivedNum))    {xESP8266.ReceivedNum = xUSART.UART4ReceivedNum;   memcpy(xESP8266.ReceivedBuffer, xUSART.UART4ReceivedBuffer, xESP8266.ReceivedNum);   xUSART.UART4ReceivedNum=0;}
    if((xESP8266.USARTx== UART5)  && (xUSART.UART5ReceivedNum))    {xESP8266.ReceivedNum = xUSART.UART5ReceivedNum;   memcpy(xESP8266.ReceivedBuffer, xUSART.UART5ReceivedBuffer, xESP8266.ReceivedNum);   xUSART.UART5ReceivedNum=0;}
    #endif   
        
    return xESP8266.ReceivedNum;        
}


// ����ESP8266�Ľ��ջ��棬�������ճ��ȱ��������ݴ�Ż���
void ESP8266_CleanReceivedFlag(void)
{
    if(xESP8266.USARTx== USART1)  { xUSART.USART1ReceivedNum =0;}
    if(xESP8266.USARTx== USART2)  { xUSART.USART2ReceivedNum =0;}
    if(xESP8266.USARTx== USART3)  { xUSART.USART3ReceivedNum =0;}
    #ifdef STM32F10X_HD
    if(xESP8266.USARTx== UART4)   { xUSART.UART4ReceivedNum =0; }
    if(xESP8266.USARTx== UART5)   { xUSART.UART5ReceivedNum =0; }
    #endif      
    
    xESP8266.ReceivedNum =0;                                          // ��0�����ճ���
    memset(xESP8266.ReceivedBuffer ,0, ESP8266_RX_BUF_SIZE);          // ���㣬���ջ���   
}


/******************************************************************************
 * ��  ���� ESP8266_GetLinkStatus
 * ��  �ܣ� ��ʼ��USART��GPIO��ͨ�Ų������á��ж����ȼ� 
 *          (8λ���ݡ���У�顢1��ֹͣλ)
 * ��  ���� uint32_t baudrate  ͨ�Ų�����
 * ����ֵ:  0_��ȡ״̬ʧ��
 *          2_���ip
 *          3_��������
 *          4_ʧȥ����
 ******************************************************************************/  
uint8_t ESP8266_GetLinkStatus(void)
{
    if (ESP8266_CMD("AT+CIPSTATUS\r\n", "OK", 10000))
    {
        if (strstr((char*)xESP8266.ReceivedBuffer , "STATUS:2"))   return 2;
        if (strstr((char*)xESP8266.ReceivedBuffer , "STATUS:3"))   return 3;
        if (strstr((char*)xESP8266.ReceivedBuffer , "STATUS:4"))   return 4; 
        if (strstr((char*)xESP8266.ReceivedBuffer , "STATUS:5"))   return 5;             
    }
    return 0;
}



/******************************************************************************
 * ��  ���� ESP8266_JoinAP
 * ��  �ܣ� ����AP
 * ��  ���� char* SSID       WiFi����
 *          char* passWord   WiFi����
 * �� �� ֵ: 0_����ʧ��
 *           1_���ӳɹ�
 ******************************************************************************/  
uint8_t ESP8266_JoinAP (char* SSID, char* passWord)
{
    char strTemp[60];
    uint8_t linkStatus=0;
    
    printf("׼������SSID��%s, %s\r\n", SSID, passWord ); 
    // ��ESP8266�������ó�SATģʽ    
    ESP8266_CMD("AT+RESTORE\r\n" ,"ready", 1000)  ? printf("�ָ���������: �ɹ�\r\n") : printf("�ָ���������: ʧ��\r\n");   // �ָ�ģ��ĳ�������
    ESP8266_CMD("AT+CWMODE=1\r\n",   "OK", 3000)  ? printf("���� STAģʽ: �ɹ�\r\n") : printf("���� STAģʽ: ʧ��\r\n");   // ����ģʽ��1_STA, 2_AP, 3_STA+AP
    ESP8266_CMD("AT+RST\r\n",     "ready", 3000)  ? printf("���� ESP8266: �ɹ�\r\n") : printf("���� ESP8266: ʧ��\r\n");   // ����ģ��: ���ù���ģʽ������������Ч
    // ������ָ��WiFi�ȵ�
    sprintf(strTemp, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, passWord );
    printf("��ʼ����AP... ");
    ESP8266_CMD(strTemp,        "OK\r\n", 10000)  ? printf("�ɹ�\r\n") : printf("ʧ��\r\n");
    // �������״̬
    printf("��ȡ����״̬��");
    linkStatus = ESP8266_GetLinkStatus();
    ESP8266_CleanReceivedFlag();  
        
    if(linkStatus==0)  {printf("ʧ�ܣ�ԭ�򣺻�ȡʧ�ܣ�\r\n");           return 0;}
    if(linkStatus==2)  {printf("�ɹ����ѻ��IP\r\n");                   return 1;}
    if(linkStatus==3)  {printf("ʧ�ܣ�ԭ�������ӣ���δ���IP��\r\n"); return 0;}
    if(linkStatus==4)  {printf("ʧ�ܣ�ԭ��ʧȥ���ӣ�\r\n");           return 0;}
    if(linkStatus==5)  {printf("ʧ�ܣ�ԭ��û������\r\n");             return 0;}
         
    return 0;
    
}

 // ��ģ�����ó�APģʽ 
uint8_t ESP8266_SetAP(char* SSID, char* passWord)
{
    char strTemp[60];
    uint8_t status=0;
    
    printf("׼������SSID��%s, %s\r\n", SSID, passWord ); 
    // ��ESP8266�������ó�APģʽ    
    ESP8266_CMD("AT+RESTORE\r\n" ,"ready", 1000)  ? printf("�ָ���������: �ɹ�\r\n") : printf("�ָ���������: ʧ��\r\n");   // �ָ�ģ��ĳ�������
    ESP8266_CMD("AT+CWMODE=2\r\n",   "OK", 3000)  ? printf("����ΪAPģʽ: �ɹ�\r\n") : printf("���� STAģʽ: ʧ��\r\n");   // ����ģʽ��1_STA, 2_AP, 3_STA+AP
    ESP8266_CMD("AT+RST\r\n",     "ready", 3000)  ? printf("���� ESP8266: �ɹ�\r\n") : printf("���� ESP8266: ʧ��\r\n");   // ����ģ��: ���ù���ģʽ������������Ч
    
    ESP8266_CMD("AT+CIPMUX=1\r\n",   "OK", 2000)  ? printf("����ģʽ-�� : �ɹ�\r\n") : printf("����ģʽ-�� : ʧ��\r\n");   // ����ģ��: ���ù���ģʽ������������Ч
    
    // ����WiFi�ȵ�
    sprintf(strTemp, "AT+CWSAP=\"%s\",\"%s\",11,0\r\n", SSID, passWord );
    ESP8266_CMD(strTemp,        "OK\r\n", 10000)        ? printf("���ڴ���AP... �ɹ�\r\n") : printf("���ڴ���AP... ʧ��\r\n");   // ����ģ��: ���ù���ģʽ������������Ч
    ESP8266_CMD("AT+CIPSERVER=1,3333\r\n", "OK", 10000) ? printf("�����˿ں� �ɹ�\r\n") : printf("���ڴ���AP... ʧ��\r\n");   // ����������ģʽ
} 




