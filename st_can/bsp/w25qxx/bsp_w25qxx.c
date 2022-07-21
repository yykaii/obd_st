/***********************************************************************************************************************************
 ** ���ļ����ơ�  bsp_w25qxx.c
 ** ����д��Ա��  ħŮ�������Ŷ�
 ** �����·���  QȺ�ļ���        
 ** ����    ����  ħŮ������      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 ** ���ļ����ܡ�  ��ʼ��GPIO��SPI, �����ܺ���
 ** ������ƽ̨��  STM32F103 + ��׼��v3.5 + keil5
 **
 ** �����غ�����
 **
 ** �����¼�¼��  2019-05-11  ����
 **               2019-12-03  ����޸�write����, ʹ���������������ص㣺��ȷ���Ⱥ�дҳ������, дҳָ����󻺴��ֽ���. 
 **               2020-08-15  ���ƴ���ע�ͣ��ļ���ʽ 
 **               2021-02-24  ������������жϣ�W25Qxx��ʼ��ʧ��ʱ��������������ֹ����
 ** 
************************************************************************************************************************************/
#include "bsp_w25qxx.h" 


  
  
  
/*****************************************************************************
 ** ��������
 *****************************************************************************/
 // �豸״̬
xW25QXX_TypeDef   xW25Qxx;
//W25Qϵ��оƬ�ͺŷ���ֵ       
#define    W25Q80            0XEF13     
#define    W25Q16            0XEF14
#define    W25Q32            0XEF15
#define    W25Q64            0XEF16
#define    W25Q128           0XEF17
#define    W25Q256           0XEF18
//#define  W25Qxx    65519   // �ܶ�ʱ���������غ�����Ķ���65519
#define    W25QX_NSS_HIGH    (W25QXX_NSS_GPIO -> BSRR =  W25QXX_NSS_PIN)
#define    W25QX_NSS_LOW     (W25QXX_NSS_GPIO -> BRR  =  W25QXX_NSS_PIN)
  
/*****************************************************************************
 ** �ڲ���������
****************************************************************************/
// 5����������
static u8    sendByte(u8 d);                          // 5_1    �ֽڶ�д
static void  writeEnable(void) ;                      // 5_2    дʹ��
static void  WaitReady(void) ;                        // 5_3    �ȴ�����
static void  eraseSector(u32 addr);                   // 5_4    ������
static void  writeSector(u32 addr, u8* p, u16 num);   // 5_5    д����
// ����
static void  readID(void);
static void  spiInit(void);
static void  checkFlagGBKStorage(void);                   // ����ֿ�������ȷ��
      
      
      
// ����US������ʱ������������ֲʱ���ⲿ�ļ�������
#if 1
static void delay_us( __IO u32 times)
{
    times=times*7;      //  10us����7;
    while(--times)         
        __nop();  
}
#endif 

// ms��ʱ������������ֲʱ���ⲿ�ļ�������
#if 0
static void delay_ms(u32 ms)
{
    ms=ms*6500;                  
    for(u32 i=0; i<ms; i++);      // 72MHzϵͳʱ���£����ٸ���ѭ��Լ��ʱ1ms
}
#endif



      
/***************************************************************************** 
* @Fun    W25Qxx_Init
* @brief  ��ģ�洢�豸  
*/  
void W25qx_Init()
{
    xW25Qxx.FlagInit =0;            // ��λ��ʼ���ɹ���־
    xW25Qxx.FlagGBKStorage = 1;       // �����ֿ����, �ȿ����ֿ��ַ��д������־, ������������ֿ⸳��ʵֵ
    
    // ʱ��ʹ��;���ж϶˿ڵķ�ʽʹ��ʱ����, ������ֲʱ�Ĺ���
    // ʹ��SPIʱ��
    if(W25QXX_SPI == SPI1)        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    if(W25QXX_SPI == SPI2)        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    if(W25QXX_SPI == SPI3)        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
    // ʹ��NSS���Ŷ˿�ʱ��
    if(W25QXX_NSS_GPIO == GPIOA)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    if(W25QXX_NSS_GPIO == GPIOB)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    if(W25QXX_NSS_GPIO == GPIOC)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
    if(W25QXX_NSS_GPIO == GPIOD)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
    if(W25QXX_NSS_GPIO == GPIOE)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE , ENABLE);
    if(W25QXX_NSS_GPIO == GPIOF)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF , ENABLE);
    if(W25QXX_NSS_GPIO == GPIOG)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF , ENABLE);    
    // ʹ��SPIx���Ŷ˿�ʱ��
    if(W25QXX_CLK_GPIO == GPIOA)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    if(W25QXX_CLK_GPIO == GPIOB)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
    if(W25QXX_CLK_GPIO == GPIOC)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
    if(W25QXX_CLK_GPIO == GPIOD)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
    if(W25QXX_CLK_GPIO == GPIOE)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE , ENABLE);
    if(W25QXX_CLK_GPIO == GPIOF)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF , ENABLE);
    if(W25QXX_CLK_GPIO == GPIOG)  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG , ENABLE);
    
    // �������Ź���ģʽ
    GPIO_InitTypeDef  G;         
    G.GPIO_Pin   = W25QXX_NSS_PIN;       // Ƭѡ����
    G.GPIO_Mode  = GPIO_Mode_Out_PP ;
    G.GPIO_Speed = GPIO_Speed_50MHz ;  
    GPIO_Init ( W25QXX_NSS_GPIO, &G);     
    W25QX_NSS_HIGH ;                     // Ƭѡ������  
    
    G.GPIO_Pin   = W25QXX_CLK_PIN | W25QXX_MISO_PIN | W25QXX_MOSI_PIN;   
    G.GPIO_Mode  = GPIO_Mode_AF_PP;      // SPIͨ������
    GPIO_Init ( W25QXX_CLK_GPIO, &G); 
    
    // SPI_ͨ�Ų�������
    spiInit ();                          // spi��ʼ����������װ�����ڲ�ͬ�豸ʹ��ͬһspi
    
    // �����Լ��
    readID();                            // ��ȡоƬ�ͺ�,���ж�ͨѶ�Ƿ�����   
    checkFlagGBKStorage();                   // ����ֿ� 
}

                        
/***************************************************************************** 
  * spi��ʼ��
  * ֻҪ��w25qxxϵ�У�������
  * ע�⣬�豸��SPI��ʼ������������һ�������ĺ�������ΪSTM32���豸���л�ʱ��SPI�Ĳ���Ҳ�����µ���
  */   
static void  spiInit(void)
{   
    W25QXX_SPI -> CR1  = 0x1<<0;         // CPHA:ʱ����λ,0x1=�ڵ�2��ʱ�ӱ��ؽ������ݲ���
    W25QXX_SPI -> CR1 |= 0x1<<1;         // CPOL:ʱ�Ӽ���,0x1=����״̬ʱ��SCK���ָߵ�ƽ
    W25QXX_SPI -> CR1 |= 0x1<<2;         // ����ģʽ:         1 = ������
    W25QXX_SPI -> CR1 |= 0x0<<3;         // �����ʿ���[5:3]:  0 = fPCLK /2
    W25QXX_SPI -> CR1 |= 0x0<<7;         // ֡��ʽ:           0 = �ȷ���MSB
    W25QXX_SPI -> CR1 |= 0x1<<9;         // ������������� :  1 = ʹ���������������(���NSS)
    W25QXX_SPI -> CR1 |= 0x1<<8;         // �ڲ�������ѡ��,����9λ����(ʧ���ڲ�NSS)
    W25QXX_SPI -> CR1 |= 0x0<<11;        // ����֡��ʽ,       0 = 8λ
    W25QXX_SPI -> CR1 |= 0x1<<6;         // SPIʹ��           1 = ʹ������
         
    delay_us(10);                         // ������ʱ
}



// 5_1 ����1�ֽ�,����1�ֽ�
// SPIͨ��,ֻһ������:��DRд���������ֵ,ͬ����������!д�����,������ʱ��ͼ��. ��Ϊ����,��Ϊ�շ�ͬ��,�����շ����ж�Ҳ���ÿ�,δ��֤�����ж϶��乤����Ӱ��. 
u8  sendByte(u8 d)
{
    u8 retry=0;
    
    while((W25QXX_SPI ->SR & 2) == 0)       // �ȴ�������Ϊ��    
    {  
    retry++;
    if(retry>250)    return 0;
    }
    W25QXX_SPI ->DR =d;
    
    retry =0;    
    while((W25QXX_SPI->SR & 1) == 0)        // �ȴ�����������      
    {          
        retry++;
        if(retry>250)    return 0;
    }
    return W25QXX_SPI->DR ;     
} 



// 5_2 дʹ��
void writeEnable()
{
    W25QX_NSS_LOW ;

    sendByte (0x6);                          // ����: Write Enable : 06h
    W25QX_NSS_HIGH ;              
}



// 5_3 �ȴ�����
void WaitReady()
{    
    W25QX_NSS_LOW ;

    sendByte (0x05);                         // ����: Read Status Register : 05h
    while(sendByte(0xFF) & 1) {}             // ֻҪ���Ͷ�״̬�Ĵ���ָ�оƬ�ͻ�����������������µ�״̬�Ĵ������� ��ֱ���յ�ͨ�ŵ�ֹͣ�źš�
           
    W25QX_NSS_HIGH ;    
} 
         


// 5_4 ����һ������, ÿ����>150ms
void eraseSector(u32 addr)
{
   if(xW25Qxx .FlagInit ==0) return;         // ���W25Qxx��ʼ��ʧ�ܣ���������⣬��ֹ����

    addr=addr*4096;                          // �ӵڼ�������ʼ
    
    writeEnable();
    WaitReady();
    // ����
    W25QX_NSS_LOW ;
    sendByte (0x20);                         // ����: Sector Erase(4K) : 20h
    sendByte ((u8)(addr>>16));
    sendByte ((u8)(addr>>8));
    sendByte ((u8)addr);
    W25QX_NSS_HIGH ;    
    
    WaitReady();       
} 



// 5_5 д����. Ҫ��ҳд��
void writeSector(u32 addr, u8 *p, u16 num)
{
    if(xW25Qxx .FlagInit ==0) return;    // ���W25Qxx��ʼ��ʧ�ܣ���������⣬��ֹ����

    u16 pageRemain = 256;                // ��Ҫ����Ҫ����Ҫ��W25Qxxÿ��ҳ�������д���ֽ���:256�ֽ�;    
  
    // ����:4096bytes, ����ҳ:256bytes, д����Ҫ��16��ҳ����д��     
    for(char i=0; i<16; i++)
    {              
        writeEnable ();                  // дʹ��
        WaitReady ();                    // �ȴ�����
        
        W25QX_NSS_LOW ;                  // �͵�ƽ,��ʼ
        sendByte(0x02);                  // ����: page program : 02h , ÿ��дҳ������󻺴�256�ֽ�
        sendByte((u8)(addr>>16));        // ��ַ
        sendByte((u8)(addr>> 8));
        sendByte ((u8)addr); 
        for(u16 i=0;i<pageRemain; i++)   // ����д������� 
        sendByte( p[i] );                // �ߵ�ƽ, ����
        W25QX_NSS_HIGH ;     
        
        WaitReady ();                    // �ȴ�����    
      
        p = p + pageRemain;              // ����ָ������һҳ�ֽ��� 
        addr = addr + pageRemain ;       // д��ַ����һҳ�ֽ���
    }      
}
/***************************************************************************** 
  * @Fun    W25Qxx_readID
  * @brief  ��ȡоƬ�ͺ�,�����ж�ͨѶ״��         
  */        
static void readID()
{   
    u16 W25QxxType = 0 ;     
    // 1: ��ȡоƬ�ͺ�, �ж�����״��    
    W25QX_NSS_LOW; 
    sendByte(0x90);  // ���Ͷ�ȡID����,���������,��һ�ֽ�������,�����ֽ���0
    sendByte(0x00);
    sendByte(0x00);
    sendByte(0x00);  // �����ֽڱؽ����� 0h      
    W25QxxType  = (sendByte(0xFF))<<8;   // u16 W25QxxType  �ڱ��ļ�����,ȫ��
    W25QxxType |= sendByte(0xFF);    
    W25QX_NSS_HIGH;  
    
    xW25Qxx.FlagInit  =1;    
    switch (W25QxxType)    {                        
        case W25Q16:            
            sprintf((char*)xW25Qxx.type, "%s", "W25Q16");               
            break;        
        case W25Q32:
            sprintf((char*)xW25Qxx.type, "%s", "W25Q32");              
            break;
        case W25Q64:
            sprintf((char*)xW25Qxx.type, "%s", "W25Q64");              
            break;
        case W25Q128:
            sprintf((char*)xW25Qxx.type, "%s","W25Q128");              
            break;
        case W25Q256:
            sprintf((char*)xW25Qxx.type, "%s", "W25Q256");           // ע��:W25Q256�ĵ�ַ��4�ֽ�               
            break;        
        default:             
            sprintf((char*)xW25Qxx.type, "%s", "Flash�豸ʧ�� !!!");              
            xW25Qxx.FlagInit =0;
            printf("��ȡ���Ĵ����ͺ����ݣ�%d\r\n",W25QxxType);
            break;
    }        
   
    // 2:��ȡ�洢����, ��������������¼      
    if(xW25Qxx.FlagInit  == 1 ) 
    {   
        u32 Addr = 0x00;                  // ���ݵ�ַ,  W25Q128����ַ:0X0100 0000
        u8 d[4]={0};                      // ���ݻ��棬 0x0000:��־0xEE, 0x0001:��־0X00,     0x0002:���ݸ�λ, 0x0003:���ݵ�λ
        u16 startFlag  = 0;               // ��־
        u16 startNum   = 0;               // ��������
        
        W25qxx_ReadBuffer(  Addr, d, 4);  // ��ȡ4���ֽ�����
        startFlag = (d[0]<<8) | d[1];     // ��־
        startNum  = (d[2]<<8) | d[3];     // ��������        
        
        if(startFlag!=0xEE00)             // û�оɼ�¼    
        {         
            startNum=1;            
            d[2]=0;
            d[3]=1;                
        }
        else
        {
            startNum++;                   // �ɹ���ȡ���ݣ� ��������1                    
            d[2]=(u8)(startNum>>8);
            d[3]=(u8)startNum;   
            if(STARTUPTIMES_RESET==1)
            {                
                d[2]=(u8)0;              // �ظ��������� = 0, ȡ�������м��ɣ�
                d[3]=(u8)0;              // ������¼�󣬵�����ע�ͣ������ٴ���¼ 
            }                
        }
        d[0]=0xEE;
        d[1]=0x00;
        
        // ���غ�f=0xEE00ʱ�����������Ī����λ
        W25qxx_WriteBuffer( Addr, d, 4);  
        xW25Qxx.StartupTimes = startNum; 
        
        printf("Flash�洢 ���...     �ͺ�:%s ,��%d��ʹ��\r", xW25Qxx.type, startNum); 
    }
    else 
    {    // ���W25Qxxʧ��      
        printf("���ݴ洢��⣺        �ͺŶ�ȡ�����豸������!\r"); 
        printf("���Ը�λоƬ......\r"); 
        //System_Reset ();
    }          
}



/******************************************************************************
 * @Function        W25Qxx_Read  ȫ�� 4_3
 * @Description     ��ȡ����
 *                                 
 * @Input           u8   *p    ��������ֵ���λ��    
 *                  u32  addr  ��ȡ��ַ
 *                  u16  num   ������ȡ���ֽ��� 
**/
void W25qxx_ReadBuffer( u32 addr, u8 *p, u16 num)
{
    if(xW25Qxx .FlagInit ==0) return;   // ���W25Qxx��ʼ��ʧ�ܣ���������⣬��ֹ����

    spiInit ();                       // ÿ�ζ�дǰ������������SPI���������豸����һSPIʱ�����ò�ͬ
    
    W25QX_NSS_LOW ;
    sendByte ( 0x03);                 // ���Ͷ�ȡ���� 03h
    sendByte ((u8)(addr>>16));
    sendByte ((u8)(addr>>8));
    sendByte ((u8)addr);
            
    for(u32 i=0;i<num;i++)
    {
        p[i]=sendByte(0xFF);
    }
                
    W25QX_NSS_HIGH ;    
}


/******************************************************************************
 * �������� W25Qxx_Write
 * ��  �ܣ� ��addr���𣬶�ȡnum���ֽڣ���ŵ�����p
 * ��  ���� u8   *p    Ҫд������ݴ洢��  
 *          u32  addr  д���ַ         (W25Q128 ֻ��3�ֽ�, W25Q256��4�ֽ�)
 *          u16  num   ����д����ֽ���  
 * ��  �أ� ��
 * ��  ע�� ħŮ�������Ŷ�    ���ϸ���QȺ��     ����޸�_2020��12��15��
 ******************************************************************************/  
u8 W25QXX_buffer[4096];                       // ����һ���ڴ�ռ�

void W25qxx_WriteBuffer( u32 addr, u8* p, u16 num)
{
    if(xW25Qxx.FlagInit ==0) return ;           // ���w25qxx�豸��ʼ��ʧ�ܣ�����������������ֹ����

    // �ֿ��д����, ��ֹ�ֿⱻ����д���ĸ�
    if(((addr+num)>0x00A00000) && (xW25Qxx.FlagGBKStorage ==1 )) 
    {
        printf("Ҫд����������ֿ����ݴ洢���ڣ����������β���!!\r");
        return;
    }
    
    u32  secPos      = addr/4096;             // ������ַ,�ڼ�������
    u16  secOff      = addr%4096;             // ��ʼ��ʼƫ���ֽ���: �����������ĵڼ��ֽڴ��
    u16  secRemain   = 4096-secOff;           // ����ʣ��ռ��ֽ��� ,�����жϹ�����������µ�����
    u8*  buf = W25QXX_buffer;                 // ԭ�Ӹ����,Ϊʲô��ֱ��ʹ��������������. (�ؿ�ǰ�������, �Ӵ�C��15��, ԭ��û�¹�����) 
    
    spiInit ();                               // ÿ�ζ�дǰ������������SPI���������豸����һSPIʱ�����ò�ͬ
    if(num<=secRemain) secRemain=num;  
    while(1) 
    {
        W25qxx_ReadBuffer ( secPos*4096, buf, 4096);   // ��ȡ�������ݵ�����
        
        eraseSector(secPos );                 // ������
        for(u16 i=0;i<secRemain ;i++)         // ԭʼ����д�뻺��
            buf[secOff +i]=p[i];
        writeSector( secPos*4096, buf, 4096); // ��������д���豸
        
        if(secRemain == num)                  // ��ȫ��д��
            break;                                         
        else
        {                                     // δд��
            p=p+secRemain ;                   // ԭʼ����ָ��ƫ��
            secPos ++;                        // ������
            secOff =0;                        // ��ƫ��λ,������������ʼ��ַ            
            num=num-secRemain ;               // ʣ��δд�ֽ���            
            secRemain = (num>4096)?4096:num;  // ����������д���ֽ���                  
        }          
    }    
}



// ����ֿ���������ȷ��
void checkFlagGBKStorage(void)
{
    if(xW25Qxx .FlagInit ==0) return;                      // ���W25Qxx��ʼ��ʧ�ܣ���������⣬��ֹ����

    printf("GBK�ֿ� ����...       ");                   
    u8 sub = 0;
    u8 f=0 ;                                                            
                                                   
    for(u32 i=0; i< 6128640; i=i+1000000)                               
    {
        W25qxx_ReadBuffer(GBK_STORAGE_ADDR +i, &f , 1);               
        sub = sub + f;                                   // 80 , 0, 98, 79, 0, 1, 0
    }      
    xW25Qxx.FlagGBKStorage = (sub==146 ? 1 : 0);             // �ж��Ƿ����ֿ�,�򿪵�ַд����, ��ֹ�ֿⱻ����д���ĸ�
    
    if(xW25Qxx.FlagGBKStorage==1)    printf("�ֿ����\r");   // ����ֿ����
    else        printf(" �����ֿⲻ����!\r");        
}



/******************************************************************************
 * �������� W25qxx_ReadGBK
 * ��  �ܣ� ��w25qxx���ֿ��ж�ȡ����ģ����    
 *          (�ο���ԭ�ӡ�Ұ�����Ĵ���������������޸�)
 * ��  ���� u8* code   �ַ�ָ����ʼλ��GBK��
 *          u8  size   �����С 12/16/24/32
 *          u3* macr   ���ݴ�ŵ�ַ (size/8+((size%8)?1:0))*(size) bytes��С
 * ��  �أ� ��
 * ��  ע�� ħŮ�������Ŷ�    ���ϸ���QȺ��     ����޸�_2020��12��15��
 ******************************************************************************/  
void W25qxx_ReadGBK(u8* typeface, u8 size, u8* dataBuf)
{            
    u8 qh,ql;                          
    u32 foffset; 
    u8 csize=(size/8+((size%8)?1:0))*(size);        // ���㺺�ֵ����С����λ�ֽ���     
    
    qh=*typeface;
    ql=*(++typeface);    
    
    if(qh<0x81||ql<0x40||ql==0xff||qh==0xff)        // �ǳ��ú��֣����������ʾ����λ��
    {                 
        for(u8 i=0; i<csize; i++) *dataBuf++=0x00;  // �������
        return;                                     // ��������
    }     

    if(ql<0x7f) ql-=0x40;                           // ����Ҫ��ȡ�ĺ������ֿ��е�ƫ��λ��
    else        ql-=0x41;
    qh-=0x81;   
    foffset=((unsigned long)190*qh+ql)*csize;        // �õ��������ֿ��е�ƫ��λ��          
    
    switch(size)
    {                                                                                 // ������Ĳ�ͬ���ڲ�ͬ�ֿ��ȡ�������
        case 12:
        W25qxx_ReadBuffer( foffset + GBK_STORAGE_ADDR,            dataBuf, csize);    // 12������           
        break;  
        case 16:
        W25qxx_ReadBuffer( foffset + GBK_STORAGE_ADDR+0x0008c460, dataBuf, csize);    // 16������
        break;
        case 24:
        W25qxx_ReadBuffer( foffset + GBK_STORAGE_ADDR+0x001474E0, dataBuf, csize);    // 24������
        break;
        case 32:
        W25qxx_ReadBuffer( foffset + GBK_STORAGE_ADDR+0x002EC200, dataBuf, csize);    // 32������
        break;            
    }         
}  


