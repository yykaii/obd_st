/***********************************************************************************************************************************
    *   @file      lcd_ili9341.h
    *   @date      2020-12-13        
    *   @author    ħŮ������   ����
    *   @brief    
    *   @taobao    �Ա�Ӳ��     ttps://demoboard.taobao.com/
    ********************************************************************************************************************************
    * ��ʵ��ƽ̨�� ħŮ������_STM32F103VE + KEIL5.27 + 2.8����ʾ��_ILI9341
    *
    * ����ֲ˵���� 
    *    1��������ʹ����F103RC�ϣ�ʹ��IOģ��FSMCͨ�ţ�ע�����ŵ��޸�
    *    2�����ֵ���ʾ��ʹ�ÿ������ϵ��ⲿFLASH���ֿ�
    *     
    * �����¼�¼��
    *  2020-12-19  ����ԭ�ӡ�Ұ����룬��д
    *  2020-12-21  ���ƴ���ṹ��ʵ�ֺ������
    *
    
************************************************************************************************************************************/
#include "bsp_lcd_ili9341.h"
#include "font.h"    
#include "bsp_w25qxx.h"
#include "bsp_usart.h"



/*****************************************************************************
 ** ��������
 *****************************************************************************/
_LCD xLCD;




/*****************************************************************************
 ** ��������
 ****************************************************************************/
static void       wrtieCmd  (uint16_t usCmd);
static void       writeData (uint16_t usData);
static uint16_t   readData  (void);
static void       gpioConfig(void);
static void       fsmcConfig(void);
static void       ili9341RegConfig(void);



// ms��ʱ������������ֲʱ���ⲿ�ļ�������
#if 1
static void delay_ms(u32 ms)
{
    ms=ms*6500;                  
    for(u32 i=0; i<ms; i++);      // 72MHzϵͳʱ���£����ٸ���ѭ��Լ��ʱ1ms
}
#endif



void LCD_Init ( void )
{
    gpioConfig ();          // ���ų�ʼ��
    fsmcConfig ();          // FMSC��ʼ��
    ili9341RegConfig ();    // ili9341��������
}

// ��ʼ��ILI9341��IO����
static void gpioConfig ( void )
{
    GPIO_InitTypeDef G;

    /* ʹ��FSMC��Ӧ��Ӧ�ܽ�ʱ��*/
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN ;   // ʹ��PORTAʱ��
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN ;   // ʹ��PORTBʱ��, ħŮ��������û�õ���ֻ��Ϊ�˷�����ֲ������
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN ;   // ʹ��PORTCʱ��, ħŮ��������û�õ���ֻ��Ϊ�˷�����ֲ������
    RCC->APB2ENR |= RCC_APB2ENR_IOPDEN ;   // ʹ��PORTDʱ��
    RCC->APB2ENR |= RCC_APB2ENR_IOPEEN ;   // ʹ��PORTEʱ��
            
    /* ����FSMC���Ӧ��������,FSMC-D0~D15 */    
    G.GPIO_Speed = GPIO_Speed_50MHz;
    G.GPIO_Mode =  GPIO_Mode_AF_PP;    
    G.GPIO_Pin = ILI9341_D0_PIN;    GPIO_Init ( ILI9341_D0_PORT, & G );
    G.GPIO_Pin = ILI9341_D1_PIN;    GPIO_Init ( ILI9341_D1_PORT, & G );    
    G.GPIO_Pin = ILI9341_D2_PIN;    GPIO_Init ( ILI9341_D2_PORT, & G );    
    G.GPIO_Pin = ILI9341_D3_PIN;    GPIO_Init ( ILI9341_D3_PORT, & G );    
    G.GPIO_Pin = ILI9341_D4_PIN;    GPIO_Init ( ILI9341_D4_PORT, & G );    
    G.GPIO_Pin = ILI9341_D5_PIN;    GPIO_Init ( ILI9341_D5_PORT, & G );    
    G.GPIO_Pin = ILI9341_D6_PIN;    GPIO_Init ( ILI9341_D6_PORT, & G );    
    G.GPIO_Pin = ILI9341_D7_PIN;    GPIO_Init ( ILI9341_D7_PORT, & G );    
    G.GPIO_Pin = ILI9341_D8_PIN;    GPIO_Init ( ILI9341_D8_PORT, & G );    
    G.GPIO_Pin = ILI9341_D9_PIN;    GPIO_Init ( ILI9341_D9_PORT, & G );    
    G.GPIO_Pin = ILI9341_D10_PIN;    GPIO_Init ( ILI9341_D10_PORT, & G );    
    G.GPIO_Pin = ILI9341_D11_PIN;    GPIO_Init ( ILI9341_D11_PORT, & G );
    G.GPIO_Pin = ILI9341_D12_PIN;    GPIO_Init ( ILI9341_D12_PORT, & G );        
    G.GPIO_Pin = ILI9341_D13_PIN;    GPIO_Init ( ILI9341_D13_PORT, & G );    
    G.GPIO_Pin = ILI9341_D14_PIN;    GPIO_Init ( ILI9341_D14_PORT, & G );    
    G.GPIO_Pin = ILI9341_D15_PIN;    GPIO_Init ( ILI9341_D15_PORT, & G );        
    // ����FSMC���Ӧ�Ŀ�����    
    G.GPIO_Pin = ILI9341_RD_PIN;     GPIO_Init (ILI9341_RD_PORT, & G );           // FSMC_NOE   :LCD-RD
    G.GPIO_Pin = ILI9341_WR_PIN;     GPIO_Init (ILI9341_WR_PORT, & G );         // FSMC_NWE   :LCD-WR
    G.GPIO_Pin = ILI9341_CS_PIN;     GPIO_Init ( ILI9341_CS_PORT, & G );       // FSMC_NE1   :LCD-CS
    G.GPIO_Pin = ILI9341_DC_PIN;     GPIO_Init ( ILI9341_DC_PORT, & G );         // FSMC_A16   :LCD-DC
    // ����LCD��λRST���ƹܽ�*/
    G.GPIO_Mode= GPIO_Mode_Out_PP;    G.GPIO_Speed = GPIO_Speed_50MHz;    
//    G.GPIO_Pin = ILI9341_RST_PIN;     GPIO_Init ( ILI9341_RST_PORT, & G );        
    /* ����LCD������ƹܽ�BK*/    
    G.GPIO_Pin = ILI9341_BK_PIN;     GPIO_Init ( ILI9341_BK_PORT, & G );
}

//FSMC ģʽ���ã�ʹ��FSMCģ��8080�ӿ�
static void fsmcConfig ( void )
{
    #if 0
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  readWriteTiming;     

    //��ַ����ʱ�䣨ADDSET��Ϊ1��HCLK 2/72M=28ns
    readWriteTiming.FSMC_AddressSetupTime      = 0x01;     //��ַ����ʱ��
    //���ݱ���ʱ�䣨DATAST��+ 1��HCLK = 5/72M=70ns    
    readWriteTiming.FSMC_DataSetupTime         = 0x04;     //���ݽ���ʱ��
    //ѡ����Ƶ�ģʽ
    //ģʽB,�첽NOR FLASHģʽ����ILI9341��8080ʱ��ƥ��
    readWriteTiming.FSMC_AccessMode            = FSMC_AccessMode_B;    
    
    /*����������ģʽB�޹�*/
    //��ַ����ʱ�䣨ADDHLD��ģʽAδ�õ�
    readWriteTiming.FSMC_AddressHoldTime       = 0x00;     //��ַ����ʱ��
    //��������ת�����ڣ������ڸ���ģʽ��NOR����
    readWriteTiming.FSMC_BusTurnAroundDuration = 0x00;
    //����ʱ�ӷ�Ƶ��������ͬ�����͵Ĵ洢��
    readWriteTiming.FSMC_CLKDivision           = 0x00;
    //���ݱ���ʱ�䣬������ͬ���͵�NOR    
    readWriteTiming.FSMC_DataLatency           = 0x00;    
    
    FSMC_NORSRAMInitStructure.FSMC_Bank                  = FSMC_Bank1_NORSRAMx;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux        = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType            = FSMC_MemoryType_NOR;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth       = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode       = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity    = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode              = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive      = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation        = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal            = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode          = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst            = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &readWriteTiming;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct     = &readWriteTiming;  
    
    FSMC_NORSRAMInit ( & FSMC_NORSRAMInitStructure );     
    
    /* ʹ�� FSMC_Bank1_NORSRAM4 */
    FSMC_NORSRAMCmd ( FSMC_Bank1_NORSRAMx, ENABLE );      

    #else
    
    /* ʹ��FSMCʱ��*/
    RCC->AHBENR|=1<<8;              //ʹ��FSMCʱ��          
    //�Ĵ�������
    //bank1��NE1~4,ÿһ����һ��BCR+TCR�������ܹ��˸��Ĵ�����
    //��������ʹ��NE4 ��Ҳ�Ͷ�ӦBTCR[6],[7]��                    
    FSMC_Bank1->BTCR[0]=0X00000000;
    FSMC_Bank1->BTCR[1]=0X00000000;
    FSMC_Bank1E->BWTR[0]=0X00000000;
    //����BCR�Ĵ���    ʹ���첽ģʽ
    FSMC_Bank1->BTCR[0]|=1<<12;        //�洢��дʹ��
    FSMC_Bank1->BTCR[0]|=1<<14;        //��дʹ�ò�ͬ��ʱ��
    FSMC_Bank1->BTCR[0]|=1<<4;         //�洢�����ݿ��Ϊ16bit         
    //����BTR�Ĵ���    
    //��ʱ����ƼĴ���                                 
    FSMC_Bank1->BTCR[1]|=0<<28;        //ģʽA                                         
    FSMC_Bank1->BTCR[1]|=1<<0;         //��ַ����ʱ�䣨ADDSET��Ϊ2��HCLK 1/36M=27ns(ʵ��>200ns)          
    //��ΪҺ������IC�Ķ����ݵ�ʱ���ٶȲ���̫�죬�����1289���IC��
    FSMC_Bank1->BTCR[1]|=0XF<<8;      //���ݱ���ʱ��Ϊ16��HCLK          
    //дʱ����ƼĴ���  
    FSMC_Bank1E->BWTR[0]|=0<<28;     //ģʽA                                      
    FSMC_Bank1E->BWTR[0]|=0<<0;        //��ַ����ʱ�䣨ADDSET��Ϊ1��HCLK 
     //4��HCLK��HCLK=72M����ΪҺ������IC��д�ź���������Ҳ��50ns��72M/4=24M=55ns       
    FSMC_Bank1E->BWTR[0]|=3<<8;     //���ݱ���ʱ��Ϊ4��HCLK    
    //ʹ��BANK1,����4
    FSMC_Bank1->BTCR[0]|=1<<0;        //ʹ��BANK1������4    
    #endif
}

// ��ʼ��ILI9341�Ĵ���
static void ili9341RegConfig ( void )
{    
    //����9341 ID�Ķ�ȡ        
    wrtieCmd(0XD3);            // ָ���ID               
    readData();             // ��1��������dummy        
    readData();               // ��2��������IC�汾��
    xLCD.id=readData();     // ��3��������IC����(93)    
       xLCD.id<<=8;
    xLCD.id|=readData();    // ��4��������IC����(41)    
     printf("��ʾ�� ���...        %x\r\n",xLCD.id);  // ��ӡLCD ID  
    if(xLCD.id !=0X9341)    // 9341��ʼ��ʧ��
        return;                // ע�⣺���FSMC���ò���ȷ���п��ܿ���ʾ����ˢ���ٶ������Ҷ�ȡ������ȷ�ͺ�
    
    // Power control B (CFh) 
    wrtieCmd ( 0xCF  );
    writeData ( 0x00  );
    writeData ( 0xC1  ); //-81
    writeData ( 0x30  );    
    // Power on sequence control (EDh)
    wrtieCmd ( 0xED );
    writeData ( 0x64 );
    writeData ( 0x03 );
    writeData ( 0x12 );
    writeData ( 0x81 );    
    // Driver timing control A (E8h)
    wrtieCmd ( 0xE8 ); //
    writeData ( 0x85 );
    writeData ( 0x10 );
    writeData ( 0x7A ); //-78    
    // Power control A (CBh) 
    wrtieCmd ( 0xCB );
    writeData ( 0x39 );
    writeData ( 0x2C );
    writeData ( 0x00 );
    writeData ( 0x34 );
    writeData ( 0x02 );    
    // Pump ratio control (F7h)
    wrtieCmd ( 0xF7 );
    writeData ( 0x20 );        
    // Driver timing control B 
    wrtieCmd ( 0xEA );
    writeData ( 0x00 );
    writeData ( 0x00 );            
    // Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
    wrtieCmd ( 0xB1 );
    writeData ( 0x00 );
    writeData ( 0x1A );  //-1B    
    //  Display Function Control (B6h) */
    wrtieCmd ( 0xB6 );
    writeData ( 0x0A );
    writeData ( 0xA2 );    
    // Power Control 1 (C0h) 
    wrtieCmd ( 0xC0 );
    writeData ( 0x1B ); //-35    
    // Power Control 2 (C1h) 
    wrtieCmd ( 0xC1 );
    writeData ( 0x01 ); //-11    
    // VCOM ���� 1 (C5h) 
    wrtieCmd ( 0xC5 );
    writeData ( 0x30 );  // 45
    writeData ( 0x30 );  // 45    
    // VCOM ���� 2 (C7h) 
    wrtieCmd ( 0xC7 );
    writeData ( 0xB7 );  //-A2    
    // ʹ��3 ٤����� (F2h) 
    wrtieCmd ( 0xF2 );
    writeData ( 0x00 );    
    // ٤������ (26h)
    wrtieCmd ( 0x26 );
    writeData ( 0x01 );    
    // ����٤��У׼ (E0H)
    wrtieCmd ( 0xE0 ); 
    writeData ( 0x0F );
    writeData ( 0x2A ); // 26
    writeData ( 0x28 ); // 24
    writeData ( 0x08 ); //0B
    writeData ( 0x0E );
    writeData ( 0x08 ); // 09
    writeData ( 0x54 );
    writeData ( 0xA9 ); // A8
    writeData ( 0x43 ); // 46
    writeData ( 0x0A ); // 0c
    writeData ( 0x0F ); // 17
    writeData ( 0x00 ); // 00
    writeData ( 0x00 ); // 00
    writeData ( 0x00 ); // 00
    writeData ( 0x00 );     
    // ����٤��У׼ (E1h) 
    wrtieCmd ( 0XE1 );  
    writeData ( 0x00 );
    writeData ( 0x15 ); // 19
    writeData ( 0x17 ); // 1B
    writeData ( 0x07 ); // 04
    writeData ( 0x11 ); // 10
    writeData ( 0x06 ); // 07
    writeData ( 0x2B ); // 2A
    writeData ( 0x56 ); // 47
    writeData ( 0x3C ); // 39
    writeData ( 0x05 ); // 03
    writeData ( 0x10 ); // 06
    writeData ( 0x0F ); // 06
    writeData ( 0x3F ); // 30
    writeData ( 0x3F ); // 38
    writeData ( 0x0F ); //     
    // �е�ַ����
    wrtieCmd ( 0x2A );     
    writeData ( 0x00 );
    writeData ( 0x00 );
    writeData ( 0x00 );
    writeData ( 0xEF );    
    // ҳ��ַ����                 
    wrtieCmd ( 0x2B ); 
    writeData ( 0x00 );
    writeData ( 0x00 );
    writeData ( 0x01 );
    writeData ( 0x3F );    
    // ���ظ�ʽ���� (3Ah)
    wrtieCmd ( 0x3a );    // ���ظ�ʽ����
    writeData ( 0x55 );   // 16λ�ӿڣ�16λ����    
    // �˳�˯��ģʽ (11h) 
    wrtieCmd ( 0x11 );      // �ȴ�5ms����·�ȶ�����ִ������ָ��
    delay_ms(10);                
    // ����ʾ(29h) 
    wrtieCmd ( 0x29 );    // ����ı�֡�洢������        
    
    LCD_DisplayDir(LCD_DIR);    // ������ʾ����
    ILI9341_BK_PORT->BSRR |= ILI9341_BK_PIN;
    LCD_Fill(0, 0, xLCD.width-1, xLCD.height-1, BLACK);

    xLCD.FlagInit =1;    
}

// ��ILI9341д������ (��Ĵ�����ַ)
// FSMC_Bank1_NORSRAM����LCD��������ĵ�ַ
static void wrtieCmd (uint16_t cmdTemp)
{
    * (__IO uint16_t *)0x60000000 = cmdTemp;    
}

// ��ILI9341д������ (Ҫд�������)
// FSMC_Bank1_NORSRAM����LCD���ݲ����ĵ�ַ 
static void writeData (uint16_t dataTemp)
{
    * (__IO uint16_t *)0x60020000 = dataTemp;    
}

// ��ILI9341��ȡ����
static uint16_t readData (void)
{
    return (*(__IO uint16_t *)0x60020000);    
}

/*****************************************************************
 * ��  ����setCursor
 * ��  �ܣ�������ʾ�����ڴ�����д�������Զ�����
 * ��  ����x      �����������, 
 *         y      �����������,
 *         width  �����ȣ�     
 *         height ������������,
 * ����ֵ����
 *
 * ��  ע�� ħŮ�������Ŷӱ�д   �Ա� https://demoboard.taobao.com
 * ��  �� ����޸�_2020��09��014��
******************************************************************/
void setCursor(u16 x, u16 y, u16 width, u16 height)
{             
    wrtieCmd (0X2A);                 // ����ָ�����x����
    writeData(x>>8);
    writeData(x&0xFF);
    writeData((x+width-1)>>8);
    writeData((x+width-1)&0xFF);     
    
    wrtieCmd (0X2B); 
    writeData(y>>8);
    writeData(y&0xFF);
    writeData((y+height-1)>>8);
    writeData((y+height-1)&0xFF);     

    // ����дGRAMָ��
    wrtieCmd(0X2C);     
}     

/*****************************************************************
 * ��  ����LCD_DrawPoint
 * ��  �ܣ���һ����
 * ��  ����x���꣬y����, 16λ��ɫֵ
 * ����ֵ����
 * 
 * ��  ע�� ħŮ�������Ŷӱ�д   �Ա� https://demoboard.taobao.com
 * ��  �� ����޸�_2020��09��01��
******************************************************************/
void LCD_DrawPoint(u16 x, u16 y, u16 color)
{
    setCursor(x,y, 1, 1);        //���ù��λ�� 
    writeData(color); 
}    

/*****************************************************************
 * ��  ����LCD_GetPointPixel
 * ��  �ܣ���ȡĳһ����������ص���ɫֵ����
 * ��  ����x      x���� 
 *         y      y����
 * ����ֵ��16λ������ɫֵ
 * 
 * ��  ע�� ħŮ�������Ŷӱ�д   �Ա� https://demoboard.taobao.com
 * ��  �� ����޸�_2020��09��01��
******************************************************************/
uint16_t LCD_GetPoint ( uint16_t x, uint16_t y )
{ 
    uint16_t r=0, g=0, b=0 ;
    
    setCursor ( x, y, 1, 1 );    
    wrtieCmd ( 0x2E );  // ��GRAN
    
    r = readData ();     // ��1�����ص�����Чֵ 
    r = readData ();  
    b = readData ();  
    g = readData ();         
    
    return ( ( ( r >> 11 ) << 11 ) | ( ( g >> 10 ) << 5 ) | ( b >> 11 ) );
}

/*****************************************************************
 * ��  ����LCD_DisplayDir
 * ��  �ܣ�����LCD��ʾ����
 * ��  ����scanDir ��ʾ����(ɨ�跽��)��0-��������3-��������5-������, 6-������
 *    
 * ����ֵ��
 * 
 * ��  ע�� ħŮ�������Ŷӱ�д   �Ա� https://demoboard.taobao.com
 * ��  �� ����޸�_2020��09��01��
******************************************************************/
void LCD_DisplayDir(u8 scanDir)
{
    u16 regval=0;
    
    if(scanDir==0||scanDir==3)       // ����
    {
        xLCD.dir=0;          
        xLCD.width  = LCD_WIDTH;
        xLCD.height = LCD_HIGH ;
    }
    if(scanDir==5 || scanDir==6)   // ����
    {                      
        xLCD.dir=1;          
        xLCD.width  = LCD_HIGH;
        xLCD.height = LCD_WIDTH;
    }         
    
    if(scanDir==0) regval|=(0<<7)|(0<<6)|(0<<5); // ������,���ϵ���
    if(scanDir==3) regval|=(1<<7)|(1<<6)|(0<<5); // ���ҵ���,���µ���
    if(scanDir==5) regval|=(0<<7)|(1<<6)|(1<<5); // ���ϵ���,���ҵ���
    if(scanDir==6) regval|=(1<<7)|(0<<6)|(1<<5); // ���µ���,������           
    wrtieCmd (0X36);             // ��д������ɫģʽ
    writeData(regval|0x08);          // 
}     

/*****************************************************************
 * ��  ����LCD_DisplayOn
 * ��  �ܣ�������ʾ������
 * ��  ����
 *    
 * ����ֵ��
 * 
 * ��  ע�� ħŮ�������Ŷӱ�д   �Ա� https://demoboard.taobao.com
 * ��  �� ����޸�_2020��09��01��
******************************************************************/
void LCD_DisplayOn(void)
{                       
    wrtieCmd(0X29);    //������ʾ
    ILI9341_BK_PORT->BSRR |= ILI9341_BK_PIN;
}     

/*****************************************************************
 * ��  ����LCD_DisplayOff
 * ��  �ܣ��ر���ʾ, Ϩ��
 * ��  ����
 *    
 * ����ֵ��
 * 
 * ��  ע�� ħŮ�������Ŷӱ�д   �Ա� https://demoboard.taobao.com
 * ��  �� ����޸�_2020��09��01��
******************************************************************/
void LCD_DisplayOff(void)
{       
    wrtieCmd(0X28);    //�ر���ʾ
    ILI9341_BK_PORT->BSRR |= (u32)ILI9341_BK_PIN << 16;
}   

/*****************************************************************
 * ��  ����LCD_Fill
 * ��  �ܣ������������ĳһ��ɫ���ص�
 * ��  ����x      �����������, 
 *         y      �����������,
 *         width  �����ȣ�     
 *         height ������������,
 *         color  ��ɫֵ
 * ����ֵ����
 * 
 * ��  ע�� ħŮ�������Ŷӱ�д   �Ա� https://demoboard.taobao.com
 * ��  �� ����޸�_2020��09��01��
******************************************************************/
void LCD_Fill(u16 x, u16 y, u16 width, u16 height, u16 color)
{          
    u32 num = width * height;
    setCursor(x, y, width, height);    // ���õ��д���� 
    while(num--)
        writeData(color);
}  

/*****************************************************************
 * ��  ����LCD_Line
 * ��  �ܣ����߶� (�ο�Ұ�����)  
 * ��  ����xStart      ��ʼx���� 
 *         yStart      ��ʼy����
 *         xEnd        ����x����
 *         yEnd        ����y����
 *         color       ��ɫ
 * ����ֵ��
 * 
 * ��  ע�� ħŮ�������Ŷ�        �Ա� https://demoboard.taobao.com
 * ��  �� ����޸�_2020��09��01��
******************************************************************/
void LCD_Line ( uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd , uint16_t color)
{
    uint16_t us; 
    uint16_t usX_Current, usY_Current;
    
    int32_t lError_X = 0, lError_Y = 0, lDelta_X, lDelta_Y, lDistance; 
    int32_t lIncrease_X, lIncrease_Y;         
    
    lDelta_X = xEnd - xStart; //������������ 
    lDelta_Y = yEnd - yStart; 
    
    usX_Current = xStart; 
    usY_Current = yStart;     
    
    if ( lDelta_X > 0 ) 
        lIncrease_X = 1; //���õ�������     
    else if ( lDelta_X == 0 ) 
        lIncrease_X = 0;//��ֱ��     
    else 
    { 
        lIncrease_X = -1;
        lDelta_X = - lDelta_X;
    } 
    
    if ( lDelta_Y > 0 )
        lIncrease_Y = 1;     
    else if ( lDelta_Y == 0 )
        lIncrease_Y = 0;//ˮƽ��     
    else 
    {
        lIncrease_Y = -1;
        lDelta_Y = - lDelta_Y;
    } 
    
    if (  lDelta_X > lDelta_Y )
        lDistance = lDelta_X; //ѡȡ��������������     
    else 
        lDistance = lDelta_Y; 
    
    for ( us = 0; us <= lDistance + 1; us ++ )//������� 
    {  
        LCD_DrawPoint ( usX_Current, usY_Current , color);//���� 
        
        lError_X += lDelta_X ; 
        lError_Y += lDelta_Y ; 
        
        if ( lError_X > lDistance ) 
        { 
            lError_X -= lDistance; 
            usX_Current += lIncrease_X; 
        }  
        
        if ( lError_Y > lDistance ) 
        { 
            lError_Y -= lDistance; 
            usY_Current += lIncrease_Y; 
        }         
    }      
}   

/*****************************************************************
 * ��  ����LCD_Rectangle
 * ��  �ܣ���һ������ (�ο�Ұ�����)
 * ��  ����x       ��ʼx���� 
 *         y       ��ʼy����
 *         width   ���ε����ؿ��
 *         height  ���ε����ظ߶�
 *         color   ��ɫ
 *         filled  �Ƿ�ʵ�����
 * ����ֵ��
 * 
 * ��  ע�� ħŮ�������Ŷӱ�д   �Ա� https://demoboard.taobao.com
 * ��  �� ����޸�_2020��09��01��
******************************************************************/
void LCD_Rectangle ( uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t filled )
{
    if ( filled )
    {
        LCD_Fill (x, y, width, height, color);
    }
    else
    {
        LCD_Line ( x, y, x + width - 1, y , color);
        LCD_Line ( x, y + height - 1, x + width - 1, y + height - 1 , color);
        LCD_Line ( x, y, x, y + height - 1, color );
        LCD_Line ( x + width - 1, y, x + width - 1, y + height - 1 , color);        
    }
}

/*****************************************************************
 * ��  ����LCD_Circle
 * ��  �ܣ���һ��Բ��  (�ο�Ұ�����)
 * ��  ����x       ��ʼx���� 
 *         y       ��ʼy����
 *         radius  �뾶
 *         color   ��ɫ
 *         filled  �Ƿ�ʵ�����
 * ����ֵ��
 * 
 * ��  ע�� ħŮ�������Ŷӱ�д   �Ա� https://demoboard.taobao.com
 * ��  �� ����޸�_2020��09��01��
******************************************************************/
void LCD_Circle ( uint16_t x, uint16_t y, uint16_t radius, uint16_t color, uint8_t filled )
{
    int16_t sCurrentX, sCurrentY;
    int16_t sError;    
    
    sCurrentX = 0; sCurrentY = radius;        
    sError = 3 - ( radius << 1 );     //�ж��¸���λ�õı�־    
    
    while ( sCurrentX <= sCurrentY )
    {
        int16_t sCountY;        
        
        if ( filled )             
            for ( sCountY = sCurrentX; sCountY <= sCurrentY; sCountY ++ ) 
            {                      
                LCD_DrawPoint ( x + sCurrentX, y + sCountY, color );           //1���о����� 
                LCD_DrawPoint ( x - sCurrentX, y + sCountY , color);           //2       
                LCD_DrawPoint ( x - sCountY,   y + sCurrentX, color );           //3
                LCD_DrawPoint ( x - sCountY,   y - sCurrentX , color);           //4
                LCD_DrawPoint ( x - sCurrentX, y - sCountY , color);           //5    
                LCD_DrawPoint ( x + sCurrentX, y - sCountY , color);           //6
                LCD_DrawPoint ( x + sCountY,   y - sCurrentX , color);           //7     
                LCD_DrawPoint ( x + sCountY,   y + sCurrentX , color);           //0                
            }        
        else
        {          
            LCD_DrawPoint ( x + sCurrentX, y + sCurrentY , color);             //1���о�����
            LCD_DrawPoint ( x - sCurrentX, y + sCurrentY , color);             //2      
            LCD_DrawPoint ( x - sCurrentY, y + sCurrentX , color);             //3
            LCD_DrawPoint ( x - sCurrentY, y - sCurrentX , color);             //4
            LCD_DrawPoint ( x - sCurrentX, y - sCurrentY , color);             //5       
            LCD_DrawPoint ( x + sCurrentX, y - sCurrentY , color);             //6
            LCD_DrawPoint ( x + sCurrentY, y - sCurrentX , color);             //7 
            LCD_DrawPoint ( x + sCurrentY, y + sCurrentX , color);             //0
    }    
        
        sCurrentX ++;
        
        if ( sError < 0 ) 
            sError += 4 * sCurrentX + 6;            
        else
        {
            sError += 10 + 4 * ( sCurrentX - sCurrentY );   
            sCurrentY --;
        }             
    }    
}


// ����һ�Σ���Ұ�����Ĵ��룬���������պ�ο�
/***********************��������****************************/
#define ZOOMMAXBUFF 16384
uint8_t zoomBuff[ZOOMMAXBUFF] = {0};    //�������ŵĻ��棬���֧�ֵ�128*128
uint8_t zoomTempBuff[1024] = {0};

/**
 * @brief  ������ģ�����ź����ģ��1�����ص���8������λ����ʾ
                                        0x01��ʾ�ʼ���0x00��ʾ�հ���
 * @param  in_width ��ԭʼ�ַ����
 * @param  in_heig ��ԭʼ�ַ��߶�
 * @param  out_width �����ź���ַ����
 * @param  out_heig�����ź���ַ��߶�
 * @param  in_ptr ���ֿ�����ָ��    ע�⣺1pixel 1bit
 * @param  out_ptr �����ź���ַ����ָ�� ע��: 1pixel 8bit
 *        out_ptrʵ����û������������ĳ���ֱ�������ȫ��ָ��zoomBuff��
 * @param  en_cn ��0ΪӢ�ģ�1Ϊ����
 * @retval ��
 */
void ILI9341_zoomChar(uint16_t in_width,    //ԭʼ�ַ����
                                    uint16_t in_heig,        //ԭʼ�ַ��߶�
                                    uint16_t out_width,    //���ź���ַ����
                                    uint16_t out_heig,    //���ź���ַ��߶�
                                    uint8_t *in_ptr,    //�ֿ�����ָ��    ע�⣺1pixel 1bit
                                    uint8_t *out_ptr, //���ź���ַ����ָ�� ע��: 1pixel 8bit
                                    uint8_t en_cn)        //0ΪӢ�ģ�1Ϊ����    
{
    uint8_t *pts,*ots;
    //����Դ��ģ��Ŀ����ģ��С���趨����������ӣ�����16��Ϊ�˰Ѹ�������ת�ɶ�������
    unsigned int xrIntFloat_16=(in_width<<16)/out_width+1; 
  unsigned int yrIntFloat_16=(in_heig<<16)/out_heig+1;
    
    unsigned int srcy_16=0;
    unsigned int y,x;
    uint8_t *pSrcLine;
    
    uint16_t byteCount,bitCount;
    
    //�������Ƿ�Ϸ�
    if(in_width >= 32) return;                                                //�ֿⲻ������32����
    if(in_width * in_heig == 0) return;    
    if(in_width * in_heig >= 1024 ) return;                     //����������� 32*32
    
    if(out_width * out_heig == 0) return;    
    if(out_width * out_heig >= ZOOMMAXBUFF ) return; //����������� 128*128
    pts = (uint8_t*)&zoomTempBuff;
    
    //Ϊ�������㣬�ֿ��������1 pixel/1bit ӳ�䵽1pixel/8bit
    //0x01��ʾ�ʼ���0x00��ʾ�հ���
    if(en_cn == 0x00)//Ӣ��
    {
        //Ӣ�ĺ������ֿ����±߽粻�ԣ����ڴ˴���������Ҫע��tempBuff��ֹ���
            for(byteCount=0;byteCount<in_heig*in_width/8;byteCount++)    
            {
                for(bitCount=0;bitCount<8;bitCount++)
                    {                        
                        //��Դ��ģ������λӳ�䵽�ֽ�
                        //in_ptr��bitXΪ1����pts�������ֽ�ֵΪ1
                        //in_ptr��bitXΪ0����pts�������ֽ�ֵΪ0
                        *pts++ = (in_ptr[byteCount] & (0x80>>bitCount))?1:0; 
                    }
            }                
    }
    else //����
    {            
            for(byteCount=0;byteCount<in_heig*in_width/8;byteCount++)    
            {
                for(bitCount=0;bitCount<8;bitCount++)
                    {                        
                        //��Դ��ģ������λӳ�䵽�ֽ�
                        //in_ptr��bitXΪ1����pts�������ֽ�ֵΪ1
                        //in_ptr��bitXΪ0����pts�������ֽ�ֵΪ0
                        *pts++ = (in_ptr[byteCount] & (0x80>>bitCount))?1:0; 
                    }
            }        
    }

    //zoom����
    pts = (uint8_t*)&zoomTempBuff;    //ӳ����Դ����ָ��
    ots = (uint8_t*)&zoomBuff;    //������ݵ�ָ��
    for (y=0;y<out_heig;y++)    /*�б���*/
    {
                unsigned int srcx_16=0;
        pSrcLine=pts+in_width*(srcy_16>>16);                
        for (x=0;x<out_width;x++) /*�������ر���*/
        {
            ots[x]=pSrcLine[srcx_16>>16]; //��Դ��ģ���ݸ��Ƶ�Ŀ��ָ����
            srcx_16+=xrIntFloat_16;            //������ƫ��Դ���ص�
        }
        srcy_16+=yrIntFloat_16;                  //������ƫ��Դ���ص�
        ots+=out_width;                        
    }
    /*���������ź����ģ����ֱ�Ӵ洢��ȫ��ָ��zoomBuff����*/
    out_ptr = (uint8_t*)&zoomBuff;    //out_ptrû����ȷ�������������ֱ�Ӹĳ���ȫ�ֱ���ָ�룡
    
    /*ʵ�������ʹ��out_ptr����Ҫ������һ�䣡����
        ֻ����Ϊout_ptrû��ʹ�ã��ᵼ��warning��ǿ��֢*/
    out_ptr++; 
}    


/******************************************************************
 * �������� drawAscii
 * ��  �ܣ� ��ָ��λ����ʾһ���ַ�    
 * ��  ���� u16 x,y     ��ʼ����
 *          u8  num     Ҫ��ʾ���ַ�:" "--->"~"
 *          u8  size    �����С 12/16/24/32
 *          u32 fColor  ������ɫ
 *          u32 bColor  ������ɫ
 * ��  ע�� �ο�ԭ�Ӹ��Ұ�����Ĵ�����޸� 
 *****************************************************************/
void drawAscii(u16 x,u16 y,u8 num,u8 size,u32 fColor, u32 bColor)
{              
    //spiInit();                                      // ��ֹSPI�����������豸�޸���
     
    u8 temp;
    u16 y0=y;
    
    u8 csize=(size/8+((size%8)?1:0))*(size/2);           // �õ�����һ���ַ���Ӧ������ռ���ֽ���    
     num=num-' ';                                       // �õ�ƫ�ƺ��ֵ��ASCII�ֿ��Ǵӿո�ʼȡģ������-' '���Ƕ�Ӧ�ַ����ֿ⣩
    for(u8 t=0;t<csize;t++)
    {   
        if(size==12)         temp=asc2_1206[num][t];   // ����1206����
        else if(size==16)    temp=asc2_1608[num][t];   // ����1608����
        else if(size==24)    temp=asc2_2412[num][t];   // ����2412����
        else if(size==32)    temp=asc2_3216[num][t];   // ����3216����
        else return;                                   // û�е��ֿ�
        
        for(u8 t1=0; t1<8; t1++)
        {                
            if(temp&0x80)   LCD_DrawPoint (x, y, fColor);  // ���� ���� 
            else            LCD_DrawPoint (x, y, bColor);  // ���� ����
            temp<<=1;
            y++;
            if(y>=xLCD.height)    return;               // ������Ļ�߶�(��)
            if((y-y0)==size)
            {
                y=y0;
                x++;
                if(x>=xLCD.width) return;              // ������Ļ���(��)
                break;
            }
        }       
    }                     
} 

/******************************************************************
 * �������� drawGBK
 * ��  �ܣ� ��ָ��λ����ʾһ���ַ�    
 * ��  ���� u16 x,y     ��ʼ����
 *          u8  num     Ҫ��ʾ���ַ�:" "--->"~"
 *          u8  size    �����С 12/16/24/32
 *          u32 fColor  ������ɫ
 *          u32 bColor  ������ɫ
 * ��  ע�� �ο�ԭ�Ӹ��Ұ�����Ĵ�����޸�  
 *****************************************************************/
void drawGBK( u16 x, u16 y, u8* font, u8 size, u32 fColor, u32 bColor)
{    
    u8 temp;
    u16 y0=y;
    u8 GBK[128];   
    u8 csize=(size/8+((size%8)?1:0))*(size);    // �õ�����һ���ַ���Ӧ������ռ���ֽ���     
    W25qxx_ReadGBK(font, size, GBK);                    // �õ���Ӧ��С�ĵ�������
    
    //spiInit();                                  // ��ֹSPI�����������豸�޸���    
    for(u8 t=0; t<csize; t++)
    {                                                      
        temp = GBK[t];                            // �õ�GBK��������                          
        for(u8 t1=0; t1<8; t1++)
        {
            if(temp&0x80)   LCD_DrawPoint (x, y, fColor);    
            else            LCD_DrawPoint (x, y, bColor);            
            temp<<=1;
            y++;
            if((y-y0)==size)
            {
                y=y0;
                x++;
                break;
            }
        }       
    }      
}

/******************************************************************************
 * ��  ���� LCD_String
 * ��  �ܣ� ��LCD����ʾ�ַ���(֧��Ӣ�ġ�����)
 * ��  ���� Ӣ�ģ���ģ���ݱ�����font.h�������ʹ���һ�𱣴���оƬ�ڲ�Flash
 *          ���֣���ģ�������ⲿFlash�У��������ֿ���W25Q128��
 *                ħŮ��������W25Q128����¼����4���ֺŴ�С��ģ����
 * ��  ���� u16   x      �������Ͻ�X����
 *          u16   y      �������Ͻ�y����
 *          char* pFont  Ҫ��ʾ���ַ�������
 *          u8    size   �ֺŴ�С��12 16 24 32
 *          u32   fColor ������ɫ
 *          u32   bColor ������ɫ
 * ����ֵ:  ��
 * ��  ע�� ����޸�_2020��05��1����
 ******************************************************************************/  
void LCD_String(u16 x, u16 y, char* pFont, u8 size, u32 fColor, u32 bColor)
{
    if(xLCD .FlagInit ==0 ) return;
    
    u16 xStart = x;    

    if( size!=12 && size!=16 && size!=24 && size!=32 )       // �����С����
        size=24;    
    
    while(*pFont != 0)                // ������ȡ�ַ������ݣ�ֱ��'\0'ʱֹͣ
    {    
        if(x>(xLCD.width-size))       // ��λ���жϣ����������ĩ���Ͱѹ�껻��
        {
            x=xStart;
            y=y+size;
        }
        if(y>(xLCD.height - size))    // ��λ���жϣ����������ĩ���ͷ��أ��������
            return;        
        
        if(*pFont < 127)              // ASCII�ַ�
        {
            drawAscii (x, y, *pFont, size, fColor, bColor);            
            pFont++;
            x+=size/2;            
        }
        else                          // ������ʾ
        {     
            // ��Ҫ: ����õĲ���ħŮ��������ֿ�, ��Ҫ�޸Ļ�ע��������һ��, �����Ͳ�Ӱ��ASCIIӢ���ַ������            
            drawGBK(x, y, (u8*)pFont, size, fColor, bColor);     
            pFont = pFont+2;          // ��һ��Ҫ��ʾ���������ڴ��е�λ��              
            x=x+size;                 // ��һ��Ҫ��ʾ����������Ļ�ϵ�Xλ��    
        }
    }      
}



/******************************************************************
 * �������� LCD_Cross
 * ��  �ܣ� ��ָ�����ϻ���ʮ���ߣ�����У׼������
 * ��  ���� uint16_t x  ��   ʮ���ߵ����ĵ�����x
 *          uint16_t y  ��   ʮ���ߵ����ĵ�����x
 *          uint16_t len     ʮ���ߵ����س���
 *          uint32_t fColor  ��ɫ
 * �����أ� ��
 * ��  ע�� 
 *****************************************************************/
void LCD_Cross( uint16_t x, uint16_t y, uint16_t len, uint32_t fColor)
{
    uint16_t temp=len/2;                
                
    LCD_Line(x-temp, y, x+temp, y, fColor);   
    LCD_Line(x, y-temp, x, y+temp, fColor);       
}



/******************************************************************
 * �������� LCD_GUI
 * ��  �ܣ� ���԰����豸�����LCD��ʾ����
 * ��  ����           
 * �����أ� 
 * ��  ע�� 
 *****************************************************************/
void LCD_GUI(void)
{
    char strTemp[30];
    
    LCD_Fill(0, 0,  xLCD.width,  xLCD.height, BLACK);
    //LCD_Image (0,0, 60, 60, imageLoge);  // ͼƬ��ʾ����
    LCD_Line(  0, 49,  0,329,GREY);        // ����
    LCD_Line(119, 70,119,329,GREY);        // ����
    LCD_Line(239, 49,239,329,GREY);        // ����
    LCD_Fill(  0,287,239, 33,WHITE);       // �װ�
    LCD_Line(  0,303,239,303,BLACK);
    LCD_Line(119,287,119,329,BLACK);
    LCD_Fill(  0, 49,239, 21,WHITE);    
    LCD_Line(119, 49,119, 70,BLACK);       // ����
    LCD_Fill(119,168,120,21,WHITE);    
    
    LCD_String( 60,  0,"ħŮ������", 24, WHITE, BLACK);    
    LCD_String(  8, 28,"STM32F103VET6 - �豸�����", 16, GREY, BLACK);        
    LCD_String(  6, 52,"�����豸", 16,  BLACK , WHITE);
    LCD_String(125, 52,"NRF����ͨ��", 16, BLACK , WHITE);
    LCD_String(125,171,"CANͨ��" , 16, BLACK , WHITE);    
    LCD_String(  6,290,"�ڲ��¶�", 12, BLACK , WHITE);     // �ڲ��¶�
    LCD_String(  6,306,"��������", 12, BLACK , WHITE);     // ��������
    LCD_String(125,290,"��������", 12, BLACK , WHITE);     // ��������
    LCD_String(125,306,"����ʱ��", 12, BLACK , WHITE);     // ����ʱ��
    sprintf(strTemp, "��%d��", xW25Qxx .StartupTimes);
    LCD_String( 68,306,strTemp,12,BLUE,WHITE);
    
    u16 y=74;
    // UASRT1
    LCD_String( 6, y, "UART1����",  12, YELLOW ,BLACK);  
    if(xUSART.USART1InitFlag ==1)  { LCD_String(90,y,"���", 12, GREEN ,BLACK); }   else    { LCD_String(90,y,"ʧ��", 12, RED ,BLACK); }  y=y+15;
    // SystemClock
    LCD_String( 6, y, "ϵͳʱ��",   12, YELLOW ,BLACK);   
    sprintf(strTemp,"%d",SystemCoreClock/1000000);
    LCD_String(84, y,strTemp,       12, GREEN ,BLACK);  
    LCD_String(96, y, "MHz", 12, GREEN ,BLACK);   y=y+15;
    // LEDָʾ��
    LCD_String( 6, y, "LEDָʾ��",  12, YELLOW ,BLACK);   
    LCD_String(90,y,"���", 12, GREEN ,BLACK);    y=y+15;
    // �����ж�
    LCD_String( 6, y, "�����ж�",   12, YELLOW ,BLACK);   
    LCD_String(90,y,"���", 12, GREEN ,BLACK);    y=y+15;
    // FLASH�洢��
    LCD_String( 6, y, "FLASH�洢",  12, YELLOW ,BLACK);   
    if(xW25Qxx.FlagInit  ==1)  { LCD_String(71,y,xW25Qxx.type, 12, GREEN ,BLACK); }   else    { LCD_String(90,y,"ʧ��", 12, RED ,BLACK); }  y=y+15;
    // �����ֿ�
    LCD_String( 6, y, "�����ֿ�",   12, YELLOW ,BLACK);  
    if(xW25Qxx .FlagGBKStorage ==1)  { LCD_String(90,y,"����", 12, GREEN ,BLACK); }   else    { LCD_String(90,y,"ʧ��", 12, RED ,BLACK); }  y=y+15;
    // ��ʾ��
    LCD_String( 6, y, "��ʾ��оƬ", 12, YELLOW ,BLACK);  
    sprintf(strTemp,"%X",xLCD.id);        
    if(xLCD.FlagInit  ==1)  { LCD_String(90,y,strTemp, 12, GREEN ,BLACK); }   else    { LCD_String(90,y,"ʧ��", 12, RED ,BLACK); }  y=y+15;
           
}


