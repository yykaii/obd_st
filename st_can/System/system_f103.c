/***********************************************************************************************************************************
 ** ���ļ����ơ�  system_f103.c
 ** ����д��Ա��  ħŮ�������Ŷ�
 ** �����·���  QȺ�ļ���       1126717453
 ** ����    ����  ħŮ������      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** �� ��  �� ��  �򻯳��õ�ϵͳ��������ʼ������
 **
 ** �� c �ļ� ��  1- USART1  ����
 **               2- SysTick ����
 **               3- ���ó�ʼ������ ����
 **               4- �������ܺ���   ����
 **                 
 ** ����ֲ˵����  
 **
 ** �����¼�¼�� 
 **               2021-11-25  ����System_MCO1Init()ע��
 **               2021-11-25  ȡ��System_SysTickInit()�����ڲ���SWD�������ô���
 **               2021-09-07  �����ڲ�FLASH���ݴ�ȡ����
 **               2021-06-12  �Ƴ�USART1���ܺ�����ʹ���Ϊ�����ļ�
 **               2021-04-23  SysTick���ú���������ʱ��ԴΪHCLK
 **
***********************************************************************************************************************************/  
#include "system_f103.h"



/*****************************************************************************
 ** ���ر�������
 *****************************************************************************/
u64 sysTickCnt = 0;                    // ����ʱ������λ��ms
_flag xFlag;                           // ȫ��״̬��־





/*****************************************************************************
 * ��  ���� SysTick_Init
 * ��  �ܣ� ����systick��ʱ���� 1ms�ж�һ�Σ� ���������������System_DelayMS()��System_DelayUS()
 * ��  ����
 * ����ֵ�� 
 * ��  Ҫ�� SysTick ��ʱ��Դ�� HCLK �� 8 ��Ƶ
*****************************************************************************/
void System_SysTickInit(void)
{       
    SystemCoreClock = 5201314;             // ���ڴ��ϵͳʱ��Ƶ�ʣ���������ֵ
    SystemCoreClockUpdate();               // ��ȡ��ǰʱ��Ƶ�ʣ� ����ȫ�ֱ��� SystemCoreClockֵ 
    //printf("ϵͳ����ʱ��          %d Hz\r", SystemCoreClock);  // ϵͳʱ��Ƶ����Ϣ , SystemCoreClock��system_stm32f4xx.c�ж���   
    
    u32 msTick= SystemCoreClock /1000;     // ��������ֵ��ȫ�ֱ���SystemCoreClock��ֵ �� ������system_stm32f10x.c    
    SysTick -> LOAD  = msTick -1;          // �Զ�����
    SysTick -> VAL   = 0;                  // ��ռ�����
    SysTick -> CTRL  = 0;                  // ��0
    SysTick -> CTRL |= 1<<2;               // 0: ʱ��=HCLK/8, 1:ʱ��=HCLK
    SysTick -> CTRL |= 1<<1;               // ʹ���ж�
    SysTick -> CTRL |= 1<<0;               // ʹ��SysTick    
    
    printf("SysTickʱ������       1ms�ж�1��\r");
} 

/*****************************************************************************
 * ��  ����SysTick_Handler
 * ��  �ܣ�SysTick�жϺ���������ע�͵�stm32f10x_it.c�е�SysTick_Handler()
 * ��  ����
 * ����ֵ��
*****************************************************************************/
void SysTick_Handler(void)
{
    sysTickCnt++;             // 1ms ��1��    
    
    
    #ifdef __SCHEDULER_H     // ��������������������scheduler.h�ļ����͵�������ĺ���
        Scheduler_TickCnt();      
    #endif
}

/*****************************************************************************
 * ��  ���� System_GetRunTimes
 * ��  �ܣ� ��ȡ��ǰ������ʱ�䣬��λ������
 * ��  ����
 * ����ֵ�� 
*****************************************************************************/
u64 System_GetTimeMs(void)
{    
    return sysTickCnt  ;
}

/*****************************************************************************
 * ��  ���� System_GetTimeUs
 * ��  �ܣ� ��ȡϵͳ�ϵ������ʱ������us
 * ��  ����
 * ����ֵ�� u32 us
*****************************************************************************/
u32 System_GetTimeUs(void)
{
    u32 ms;
    u32 us;
    do{
        ms = System_GetTimeMs() ;
        us = (float)ms *1000 + (SysTick ->LOAD - SysTick ->VAL )*1000/SysTick->LOAD ;
    }while(ms != System_GetTimeMs() );
    return us;        
}

/*****************************************************************************
 * ��  ���� System_DelayMS
 * ��  �ܣ� ������ʱ
 * ʹ  �ã� �ڵ���System_SysTickInit()�����󣬼���ʹ�� 
 * ��  ���� u32 ms : ��Ҫ��ʱ�ĺ�����
 * ����ֵ�� 
*****************************************************************************/
void System_DelayMS(u32 ms)
{    
    static u64 _startTime=0;
    
    _startTime = System_GetTimeMs() ;
    while( System_GetTimeMs() - _startTime < ms );            
} 

/*****************************************************************************
 * ��  ���� System_DelayUS
 * ��  �ܣ� ΢����ʱ; 
 * ʹ  �ã� �ڵ���System_SysTickInit()�����󣬼���ʹ�� 
 * ��  ���� u32 us : ��Ҫ��ʱ��΢����
 * ����ֵ��
*****************************************************************************/
void System_DelayUS(u32 us)
{
    u64 nowUs = System_GetTimeUs ();
    while(System_GetTimeUs() - nowUs < us);    
}

/*****************************************************************************
 * ��  ���� System_GetTimeInterval
 * ��  �ܣ� ��ȡʱ���������ڲ��Դ���Ƭ������ʱ��
 * ��  ���� 
 * ����ֵ�� ��һ�ε��÷���0, ֮��ÿ�ε��÷������ϴε��õļ��ʱ��(��λ:us)
 *****************************************************************************/ 
u32 System_GetTimeInterval(void)
{
    static u32  lastTime=0, nowTime=0;
    
    lastTime = nowTime ;        
    nowTime = System_GetTimeUs ();               
    
    if(lastTime !=0 )                      // ���ǵ�һ�ε��� 
        return (nowTime-lastTime) ;          
   
    return 0;                              // ��1�ε���   
}

/*****************************************************************************
 * ��  ���� System_TestRunTimes
 * ��  �ܣ� ����ʱʹ�ã���ȡ����ε�����ʱ��������������� 
 * ��  ���� 
 * ����ֵ��
 *****************************************************************************/ 
void System_TestRunTimes(void)
{
    static u8 CNT=0;
    
    u32 intervalTimes = System_GetTimeInterval ();
    if(intervalTimes == 0)
    {
        printf("������ʱ��-����ԭ����á�\r");
        CNT++;
        System_GetTimeInterval ();
        return;
    }
    
    printf("������ʱ��-����-%d:%9u us��\r", CNT++, intervalTimes);
    System_GetTimeInterval ();
} 



/******************************************************************************
 * ��  ���� GPIOSet
 * ��  �ܣ� ʹ����Ӧʱ�ӡ���������
 *          ����ʱ��ʹ�ܣ�������GPIO����ǰ��������������
 * ��  ����
 * ����ֵ�� 
******************************************************************************/
void System_GPIOSet(GPIO_TypeDef* GPIOx, u32 allPin, u8 mode, u8 speed)
{
    u32 reg = 0; 
    u32 nowPin=0;    
    
    if(GPIOx==GPIOA )   RCC->APB2ENR |= RCC_APB2ENR_GPIOAEN ;
    if(GPIOx==GPIOB )   RCC->APB2ENR |= RCC_APB2ENR_GPIOBEN ;
    if(GPIOx==GPIOC )   RCC->APB2ENR |= RCC_APB2ENR_GPIOCEN ;
    if(GPIOx==GPIOD )   RCC->APB2ENR |= RCC_APB2ENR_GPIODEN ;
    if(GPIOx==GPIOE )   RCC->APB2ENR |= RCC_APB2ENR_GPIOEEN ;    
     
// ���F4��׼����
//    // ��ͨ����    
//    if(mode == GPIO_Mode_IN) {   
//        if(pupd==0)  reg |= 0x01<<2;
//        else         reg |= 0x02<<2;
//    }
//    
//    if((ospeed &0x03)==0) ospeed= 0x03;       // ����ٶȣ�
//    // ��ͨ���
//    if(mode==GPIO_Mode_OUT){
//        reg = ospeed & 0x03;                  // �����ٶ�
//        reg |= (otype & 0x01)<<2;             // ��ͨ���졢��©
//    }    
//    // �������
//    if(mode ==GPIO_Mode_AF){
//        reg = ospeed & 0x03;                  // �����ٶ�
//        reg |= ((otype | 0x02) & 0x03) <<2;   // �������졢��©
//    }      


    speed = speed & 0x03;             // ��ʽ��
    if(speed == 0)                    // ���ģʽ�£�����Ϊ0
        speed = 0x03;    
    
    // 8������ģʽ,��ϱ�׼��ķ�ʽ
    switch(mode)
    {
        // 4������
        case GPIO_Mode_AIN:            // 1 ģ������ģʽ
            reg = 0x00;     
        break;   
        
        case GPIO_Mode_IN_FLOATING:    // 2 ��������ģʽ(��λ���״̬)
            reg = 0x04;  
        break;  
        
        case GPIO_Mode_IPD:            // 3 ��������ģʽ
            reg = 0x08;
        break;
        
        case GPIO_Mode_IPU:            // 4 ��������ģʽ
            reg = 0x08;
        break;
        
        // 4�����
        case GPIO_Mode_Out_OD:         // 5 ��©���ģʽ
            reg = 0x04 | speed;
        break;
        
        case GPIO_Mode_Out_PP:         // 6 �������ģʽ
            reg = 0x00 | speed;
        break;
        
        case GPIO_Mode_AF_OD:          // 7 ���ÿ�©���ģʽ
            reg = 0x0c | speed;
        break;
        
        case GPIO_Mode_AF_PP:          // 8 �����������ģʽ
            reg = 0x08 | speed;    
        break;        
   
        default:                       // �޷��жϣ�����Ϊģ������ģʽ
            reg = 0x00;
        break;
    }       
    
    // ���üĴ���CHL, ��PIN 0~7          
    for(u32 i=0; i<8; i++) 
    {      
        nowPin = (u32) 0x01 << i;         // ��ǰҪ�жϵ����ź�     
        if((allPin & nowPin) != 0)        // ��ǰ����Ҫ����
        {
           GPIOx->CRL &= ~(0x0F<<(i*4));  // ��0
           GPIOx->CRL |= reg<<(i*4);      // д��������     
        }          
    }          
    
    // ���üĴ���CRH, ��PIN 8~15         
    for(u32 i=0; i<8; i++)    
    {      
        nowPin = (u32) 0x01 << (i+8);     // ��ǰҪ�жϵ����ź�     
        if((allPin & nowPin) != 0)        // ��ǰ����Ҫ����
        {
           GPIOx->CRH &= ~(0x0F<<(i*4));  // ��0
           GPIOx->CRH |= reg<<(i*4);      // д��������                
        }          
    }             
    
    if(mode == GPIO_Mode_IPU )   GPIOx->BSRR |= allPin ;
    if(mode == GPIO_Mode_IPD)    GPIOx->BSRR |= allPin << 16;         
}



/******************************************************************************
 * ��  ���� NVICSet
 * ��  �ܣ� ���ȼ����ã�Ϊ�������ʹ��FreeRTOS��ͳһʹ��4λ��ռ��(16��),0λ�����ȼ�(0��)
 *         ֱ�ӵ��ü��ɣ�������ǰ����
 * ��  ���� 
 * ����ֵ�� 
******************************************************************************/
void System_NVICSet(u8 NVIC_Channel, u8 Preemption)
{    
    static u8 setGrouped=0;
    if(setGrouped ==0){
        // ȫ�ַּ�����,ͳһΪ��4, ֵ0b11,����NVIC->IPx�и�4λ:����4λ(16��), �Ӽ�0λ(0����  
        SCB->AIRCR = ((u32)0x05FA0000)|(0x03<<8);   // ���ȼ���������, �Ѳ�,��3�� F103��F429�Ĵ���ͨ��        
        setGrouped =1;
    }
    
    // ͨ���ж����ȼ�����
    NVIC->IP[NVIC_Channel] &= ~(0xF<<4);                     // ���          
    NVIC->IP[NVIC_Channel]  =  (Preemption&0xF)<<4;          // д����ռ��\���ȼ�
    // ͨ���ж�ʹ��
    NVIC->ISER[NVIC_Channel/32] |= 1 << (NVIC_Channel % 32); // ʹ���ж�ͨ��        
    //NVIC->ICER[];                                            // �ж�ʧ��, �����õ�       
}



/***************************************************************************** 
 * ��  ���� EXTISet
 * ��  �ܣ� �ⲿ�ж����ú���
 *         ��Ҫ: һ��ֻ������1��IO��,  2020-2-26
 *         ֻ���GPIOA~G;������PVD,RTC��USB����������
 *         �ú������Զ�������Ӧ�ж�,�Լ�������  
 *         
 * ��  ���� ��GPIOx��:GPIOA~G, ����GPIOA~G
 *          ��BITx��:GPIO_Pin_0~15, ��Ҫʹ�ܵ�λ;
 *          ��TRIM��:����ģʽ, EXTI_FTIR/1:�½���;  EXTI_RTIR/2:������; 3:�����ƽ����
 * ��  �أ� 
*****************************************************************************/
void System_EXTISet(GPIO_TypeDef* GPIOx, u16 PINx, u8 TRIM)
{
    u8 gpioNum = 0;
    u8 pinNum  = 0;
    
    // ת��GPIOxΪ����
    if(GPIOx==GPIOA )  gpioNum=0;
    if(GPIOx==GPIOB )  gpioNum=1;
    if(GPIOx==GPIOC )  gpioNum=2;
    if(GPIOx==GPIOD )  gpioNum=3; 
    if(GPIOx==GPIOE )  gpioNum=4; 
    if(GPIOx==GPIOF )  gpioNum=5; 
    if(GPIOx==GPIOG )  gpioNum=6; 
    
    // ת��PINxΪ����
    for(u8 i=0; i<16; i++){
        if( PINx== ((u32)1<<i)){
            pinNum=i;
            break;
        }          
    }    
    
    u8 offSet   = (pinNum%4)*4;                    // �Ĵ�����ƫ��
    RCC->APB2ENR |=0x01;                           // ʹ��io����ʱ��             
    AFIO->EXTICR[pinNum/4] &=~(0x000F << offSet);  // ��0
    AFIO->EXTICR[pinNum/4] |=  gpioNum << offSet;  // EXTI.BITxӳ�䵽GPIOx.BITx 
    // ʹ��line BITx�ϵ��ж�, 1:ʹ��  0:����
    EXTI->IMR |= PINx ;                            
    //EXTI->EMR|=1<<BITx;                          // ������line BITx�ϵ��¼� (������������,��Ӳ�����ǿ��Ե�,��������������ʱ���޷������ж�!)
    // ������
    if(TRIM & 0x01)  EXTI->FTSR |= PINx ;          // line BITx���¼��½��ش���
    if(TRIM & 0x02)  EXTI->RTSR |= PINx ;          // line BITx���¼��������ش���
}



/******************************************************************************
 * ��  ���� System_SwdMode
 * ��  �ܣ� ����оƬ���Է�ʽ(SWD)
 *          �ر�JTAG-DP,����SW-DP�����ͷ�����PB3��PB4��PA15��ֻ��PA13��PA14
 * ��  ����
 * ����ֵ��     
*****************************************************************************/
void  System_SwdMode(void)                            
{
    RCC->APB2ENR|=1<<0;           // ��������ʱ��       
    AFIO->MAPR &= 0XF8FFFFFF;     // ��0MAPR��[26:24]
    AFIO->MAPR |= 0x2<<24;        // ����ģʽ  000:ȫ��   010��ֻ��SWD   100:ȫ�� 
}



/*****************************************************************************
 * ��  ���� GetSystemClock
 * ��  �ܣ� ��ȡϵͳʱ��Ƶ�ʣ�
 * ��  ����
 * ����ֵ�� u32 ��ǰϵͳʱ��Ƶ��
*****************************************************************************/
uint32_t System_GetSystemClock(void) 
{    
    SystemCoreClockUpdate();      // ��ȡϵͳʱ��Ƶ�ʣ�������ȫ�ֱ��� SystemCoreClock��ֵ 
    return SystemCoreClock ;
}



/*****************************************************************************
 * ��  ���� System_Reset
 * ��  �ܣ� ϵͳ��λ(F103��F429ͨ��) 
 * ��  ����
 * ����ֵ��
*****************************************************************************/   
void System_Reset(void)
{   
    SCB->AIRCR =0X05FA0000|(u32)0x04;      
}     



// �������·���ʵ��ִ�л��ָ��WFI  
void WFI_SET(void)
{
    __ASM volatile("wfi");          
}

// �ر������ж�
void System_IntxDisable(void)
{          
    __ASM volatile("cpsid i");
}

// ���������ж�
void System_IntxEnable(void)
{
    __ASM volatile("cpsie i");          
} 

// �������ģʽ      
void System_Standby(void)
{
    SCB->SCR|=1<<2;          // ʹ��SLEEPDEEPλ (SYS->CTRL)       
    RCC->APB1ENR|=1<<28;     // ʹ�ܵ�Դʱ��        
    PWR->CSR|=1<<8;          // ����WKUP���ڻ���
    PWR->CR|=1<<2;           // ���Wake-up ��־
    PWR->CR|=1<<1;           // PDDS��λ          
    WFI_SET();               // ִ��WFIָ��         
}      


/******************************************************************************
 * ��  ���� System_MCO1Init
 * ��  �ܣ� ��ʱ���������, ����ʹ��ʾ�������ʱ���Ƿ���ȷ 
 *          ���ʱ���뱣֤���ʱ��Ƶ�ʲ�����50MHz,��Ϊ��ΪIO�����Ƶ��
 * ��  ���� 0x00-û��ʱ�����
 *          0x04-ϵͳʱ��SYSCLK���
 *          0x05-�ڲ�8MHz��RC����ʱ�����
 *          0x06-�ⲿ3~25MHz����ʱ�����
 *          0x07-PLLʱ��2��Ƶ�����
 *          0x08-PLL2ʱ�����
 *          0x09-PLLʱ��3��Ƶ�����
 *          0x0A-XT1�ⲿ3~25MHz����ʱ�����(Ϊ��̫��)
 *          0x0B-PLL3ʱ�����(Ϊ��̫��)
 * ����ֵ�� ��
 ******************************************************************************/  
void System_MCO1Init (uint32_t source)
{
    RCC->CFGR &= ~(0x0f << 24);
    RCC->CFGR |= (source & 0x0f)<<24;
}



// ���ã��ڲ�FLASHд��ʱ���ȴ����У�BSYλ��־:0��1æ
static uint8_t waitForFlashBSY(uint32_t timeOut)
{                                                        
    while((FLASH->SR & 0x01) && (timeOut-- != 0x00)) ; // �ȴ�BSY��־����
    if(timeOut ==0)   
        return 1;    // ʧ�ܣ�����1, �ȴ���ʱ����
    
    return 0;        // ���������أ�                
}

// �������������壬���ڴ�ȡ�ڲ�FLASH���ݣ�F103ϵ�ж������޸�
#ifdef   STM32F10X_HD
#define  STM32_FLASH_SECTOR_SIZE       2048                    // �ڲ�FLASHҳ��С, ��λ��bytes ��(ע��STM32F10xxϵ���£�С�������洢������Ϊ1K, �������洢������Ϊ2K��
#else
#define  STM32_FLASH_SECTOR_SIZE       1024                    // �ڲ�FLASHҳ��С, ��λ��bytes ��(ע��STM32F10xxϵ���£�С�������洢������Ϊ1K, �������洢������Ϊ2K��
#endif

#define  STM32_FLASH_ADDR_BASE   0x08000000                    // оƬ�ڲ�FLASH��ַ(������������޸ģ�
static   uint8_t sectorbufferTemp[STM32_FLASH_SECTOR_SIZE];    // ����һ���ڴ�ռ䣬���ã����ڲ�FLASHд��ʱ�����ݻ���

/******************************************************************************
 * ������:  System_WriteInteriorFlash
 * ��  �ܣ� ��оƬ���ڲ�FLASH�д��ָ����������
 * ��  ���� uint32_t  writeAddr        ��Ҫ��д���Ŀ���ַ������ż��!!!
 *          uint8_t  *writeToBuffer
 *          uint16_t  numToWrite       ��Ҫ: д�������, ������ż��!!!��Ϊ�ڲ�FLASHÿ����16λд��.���Ŀ���������ǵ���,��׷�ӻ���λ�õĺ�һ�ֽڲ�λ����.
 *
 * ����ֵ��0_�ɹ���
 *         1_ʧ�ܣ���ַ��Χ����ȷ
 *         2_ʧ�ܣ�FLASH->SR:BSYæ��ʱ
 *         3_ʧ�ܣ�������ʱ
 * ��  ע�� 
 ******************************************************************************/  
uint8_t System_WriteInteriorFlash(uint32_t writeAddr, uint8_t *writeToBuffer, uint16_t numToWrite)               
{
    uint16_t flashSize = *(uint16_t*)(0x1FFFF7E0);                // ��ȡоƬFLASH��С;���Ĵ���ֵΪоƬ����ǰд���FLASH��С��ֻ������λ��KByte
                    
    uint32_t addrOff    = writeAddr - STM32_FLASH_ADDR_BASE;      // ȥ��0x08000000���ʵ��ƫ�Ƶ�ַ
    uint32_t secPos     = addrOff / STM32_FLASH_SECTOR_SIZE;;     // ������ַ,����ʼ��ַ�ڵڼ�������
    uint16_t secOff     = addrOff%STM32_FLASH_SECTOR_SIZE ;       // ��ʼ��ʼƫ���ֽ���: �����������ĵڼ��ֽڴ��
    uint16_t secRemain  = STM32_FLASH_SECTOR_SIZE - secOff;       // ��������Ҫд����ֽ��� ,�����жϹ�����������µ�����
    
    // �жϵ�ַ��Ч��   
    if(writeAddr < STM32_FLASH_ADDR_BASE)    return 1;                     // ������ĵ�ַ��С��FLASH����С��ַ�����˳�������1_��ַʧ��
    if(writeAddr > (STM32_FLASH_ADDR_BASE+(flashSize*1024)))    return 1;  // ������ĵ�ַ������FLASH������ַ�����˳�, ����1_��ַʧ��
               
    // 0_����FLASH
    FLASH->KEYR = ((uint32_t)0x45670123);
    FLASH->KEYR = ((uint32_t)0xCDEF89AB);
    
    if(numToWrite <= secRemain)    secRemain=numToWrite;  
    while(1){    
        // 1_��ȡ��ǰҳ������
        if(waitForFlashBSY(0x00888888))   return 2;                         // ʧ�ܣ�����:2, ʧ��ԭ��FLASH->SR:BSYæ��ʱ                    
        System_ReadInteriorFlash ( secPos*STM32_FLASH_SECTOR_SIZE+STM32_FLASH_ADDR_BASE , sectorbufferTemp, STM32_FLASH_SECTOR_SIZE );   // ��ȡ�������ݵ�����
                  
        // 2_����ָ��ҳ(����)                           
        if(waitForFlashBSY(0x00888888))   return 2;                         // ʧ�ܣ�����:2, ʧ��ԭ��FLASH->SR:BSYæ��ʱ  
        FLASH->CR|= 1<<1;                                                   // PER:ѡ��ҳ������λ2MERΪȫ����
        FLASH->AR = STM32_FLASH_ADDR_BASE + secPos*STM32_FLASH_SECTOR_SIZE; // ��дҪ������ҳ��ַ
        FLASH->CR|= 0x40;                                                   // STRT:д1ʱ����һ�β���������
        if(waitForFlashBSY(0x00888888))   return 2;                         // ʧ�ܣ�����:��, ʧ��ԭ�򣺲�����ʱ
        FLASH->CR &= ((uint32_t)0x00001FFD);                                // �ر�ҳ��������   
                                          
        for(uint16_t i=0; i<secRemain ; i++)                                // ԭʼ����д�뻺��
            sectorbufferTemp[secOff+i] = writeToBuffer[i];
       
        for(uint16_t i=0; i<STM32_FLASH_SECTOR_SIZE/2 ; i++){               // ��������д��оƬFLASH                      
            if(waitForFlashBSY(0x00888888))   return 2;                     // ʧ�ܣ�����:2, ʧ��ԭ��FLASH->SR:BSYæ��ʱ
            FLASH->CR |= 0x01<<0;                                           // PG: ���                             
            *(uint16_t*)(STM32_FLASH_ADDR_BASE + secPos*STM32_FLASH_SECTOR_SIZE +i*2) = (sectorbufferTemp[i*2+1]<<8) | sectorbufferTemp[i*2] ; // ��������д���豸
                                                        
            if(waitForFlashBSY(0x00888888))   return 2;                     // ʧ�ܣ�����:2, ʧ��ԭ��FLASH->SR:BSYæ��ʱ
            FLASH->CR &= ((uint32_t)0x00001FFE) ;                           // �رձ��
        }
        
        if(secRemain == numToWrite){                          
            break;                                                          // ��ȫ��д��
        }                        
        else{                                                               // δд��                                                                          
            writeToBuffer += secRemain ;                                    // ԭʼ����ָ��ƫ��
            secPos ++;                                                      // ������
            secOff =0;                                                      // ��ƫ��λ,������������ʼ��ַ            
            numToWrite -= secRemain ;                                       // ʣ��δд�ֽ���            
            secRemain = (numToWrite>STM32_FLASH_SECTOR_SIZE)?(STM32_FLASH_SECTOR_SIZE):numToWrite;  // ����������д���ֽ���                  
        }              
    } 
    FLASH->CR |= 1<<7 ;                                                     // LOCK:��������    
 
    return 0;                
}

/******************************************************************************
 * ������:  System_ReadInteriorFlash
 * ��  �ܣ� ��оƬ���ڲ�FLASH���ȡָ����������
 * ��  ���� uint32_t  readAddr     ���ݵ�ַ
 *          uint8_t  *pBuffer      ��������λ��
 *          uint16_t  numToRead    ��ȡ���ֽ�����
 * ����ֵ�� 0_�ɹ�
 *          1_ʧ�ܣ���ַС��FLASH��ַ
 *          2_ʧ�ܣ���ַ����FLASH���ֵ
 * ��  ע�� 
 ******************************************************************************/  
uint8_t  System_ReadInteriorFlash (uint32_t readAddr, uint8_t *pBuffer, uint16_t numToRead) 
{    
    // ��ȡоƬFLASH��С            
    uint16_t flashSize = *(uint16_t*)(0x1FFFF7E0);                        // ��ȡоƬFLASH��С;���Ĵ���ֵΪоƬ����ǰд���FLASH��С��ֻ����λ��KByte
                
    // �жϵ�ַ��Ч��                
    if(readAddr < STM32_FLASH_ADDR_BASE)    return 1;                     // ������ĵ�ַ��С��FLASH����С��ַ�����˳�
    if(readAddr > (STM32_FLASH_ADDR_BASE+(flashSize*1024)))  return 2;    // ������ĵ�ַ������FLASH������ַ�����˳�
              
    // ��ʼ����                
    while(numToRead--)
    {
        *pBuffer = *(__IO uint8_t*)readAddr;
        pBuffer++;        // ָ�����һ�����ݳ���
        readAddr++;       // ƫ��һ������λ
    }      
    
    return 0;             // �ɹ�������0;    
}
