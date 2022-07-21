#ifndef      __BSP_ILI9341_LCD_H
#define         __BSP_ILI9341_LCD_H
/***********************************************************************************************************************************
    *   @file     lcd_ili9341.h
    *   @date     2020-12-13        
    *   @author   ħŮ������   ����
    *   @brief    
    *   @taobao   �Ա�Ӳ��     ttps://demoboard.taobao.com/
    ********************************************************************************************************************************
    *   ��ʵ��ƽ̨�� 
    *    ħŮ������_STM32F103VE + KEIL5.27 + 2.8����ʾ��_ILI9341
    *
    *   ����ֲ˵���� 
    *    1��������ʹ����F103VC�ϣ�ʹ��FSMCģ��8080�ӿڽ��У�ע�����ŵ��޸�
    *    2�����ֵ���ʾ��ʹ�ÿ������ϵ��ⲿFLASH���ֿ�
    *
    *   ��FSMC_Bank˵����
    *    ����4��Bank
    *    64MB:FSMC_Bank1_NORSRAM1:0X6000 0000 ~ 0X63FF FFFF
    *    64MB:FSMC_Bank1_NORSRAM2:0X6400 0000 ~ 0X67FF FFFF
    *    64MB:FSMC_Bank1_NORSRAM3:0X6800 0000 ~ 0X6BFF FFFF
    *    64MB:FSMC_Bank1_NORSRAM4:0X6C00 0000 ~ 0X6FFF FFFF
    *    ÿ�� BANK ��4*64MB = 256MB(ΪʲôBank��Ҫ��4���飺Ѱַ���� 2^26 =0X0400 0000 = 64MB)
    *    ѡ��BANK1-BORSRAM1 ���� TFT����ַ��ΧΪ0X6000 0000 ~ 0X63FF FFFF
    *    LCD��DC(�Ĵ���/����ѡ��)�Ž�FSMC_A16��ַ�ţ���ѡ��ͬ�ĵ�ַ��ʱ�����µ�ַҪ���¼���  
    *    �Ĵ�������ַ = 0X60000000
    *    RAM����ַ = 0X60000000 + 2^16*2 = 0X60000000 + 0X20000 = 0X60020000
    *     
    *   �����¼�¼��
    *    2020-12-19  ����ԭ�ӡ�Ұ����룬��д
    *    2020-12-21  ������ɫֵ�����ӵ��ñ�׼��FSMCʱ�Ŀ鶨��
    
************************************************************************************************************************************/
#include "stm32f10x.h"




/**************************** ��ֲ�޸� ILI9341 8080ͨѶ���Ŷ��� ***************************/
/******�����ź���******/
//Ƭѡ��ѡ��NOR/SRAM��
#define      ILI9341_CS_PORT               GPIOD
#define      ILI9341_CS_PIN                GPIO_Pin_7
//PD11ΪFSMC_A16
#define      ILI9341_DC_PORT               GPIOD        // DC���ţ�ʹ��FSMC�ĵ�ַ�źſ��ƣ������ž����˷���LCDʱʹ�õĵ�ַ
#define      ILI9341_DC_PIN                GPIO_Pin_11
//дʹ��
#define      ILI9341_WR_PORT               GPIOD
#define      ILI9341_WR_PIN                GPIO_Pin_5
//��ʹ��
#define      ILI9341_RD_PORT               GPIOD
#define      ILI9341_RD_PIN                GPIO_Pin_4
//��λ����
//#define      ILI9341_RST_PORT              GPIOE
//#define      ILI9341_RST_PIN               GPIO_Pin_1
//��������
#define      ILI9341_BK_PORT               GPIOA
#define      ILI9341_BK_PIN                GPIO_Pin_15
/******�����ź���*******/
#define      ILI9341_D0_PORT               GPIOD
#define      ILI9341_D0_PIN                GPIO_Pin_14

#define      ILI9341_D1_PORT               GPIOD
#define      ILI9341_D1_PIN                GPIO_Pin_15

#define      ILI9341_D2_PORT               GPIOD
#define      ILI9341_D2_PIN                GPIO_Pin_0

#define      ILI9341_D3_PORT               GPIOD
#define      ILI9341_D3_PIN                GPIO_Pin_1

#define      ILI9341_D4_PORT               GPIOE
#define      ILI9341_D4_PIN                GPIO_Pin_7

#define      ILI9341_D5_PORT               GPIOE
#define      ILI9341_D5_PIN                GPIO_Pin_8

#define      ILI9341_D6_PORT               GPIOE
#define      ILI9341_D6_PIN                GPIO_Pin_9

#define      ILI9341_D7_PORT               GPIOE
#define      ILI9341_D7_PIN                GPIO_Pin_10

#define      ILI9341_D8_PORT               GPIOE
#define      ILI9341_D8_PIN                GPIO_Pin_11

#define      ILI9341_D9_PORT               GPIOE
#define      ILI9341_D9_PIN                GPIO_Pin_12

#define      ILI9341_D10_PORT              GPIOE
#define      ILI9341_D10_PIN               GPIO_Pin_13

#define      ILI9341_D11_PORT              GPIOE
#define      ILI9341_D11_PIN               GPIO_Pin_14

#define      ILI9341_D12_PORT              GPIOE
#define      ILI9341_D12_PIN               GPIO_Pin_15

#define      ILI9341_D13_PORT              GPIOD
#define      ILI9341_D13_PIN               GPIO_Pin_8

#define      ILI9341_D14_PORT              GPIOD
#define      ILI9341_D14_PIN               GPIO_Pin_9

#define      ILI9341_D15_PORT              GPIOD
#define      ILI9341_D15_PIN               GPIO_Pin_10

/****** ��Ļ���� *******/
#define LCD_WIDTH         240            // ��Ļ������أ�ע�⣺0~239
#define LCD_HIGH          320            // ��Ļ�߶����أ�ע�⣺0~319
#define LCD_DIR           0              // Ĭ����ʾ����0-��������3-��������5-������, 6-������

//Ƭѡ���ž�����NOR/SRAM��
#define      FSMC_Bank1_NORSRAMx           FSMC_Bank1_NORSRAM1
// ���� ��ֲ�޸ģ��·����벻���޸� *********************************************************
    
    
    
/******************************* ���� LCD ��Ϣ�ṹ�� ***************************************/
typedef struct          // �ṹ�壺���ڱ�����Ϣ��״̬
{     
    u8  FlagInit;       // ��ʼ����ɱ�־
    u16 width;          // LCD ���
    u16 height;         // LCD �߶�
    u8  dir;            // ���������������ƣ�0��������1������
    u16 id;
}_LCD;
extern _LCD xLCD;


/******************************* ���� ILI934 ��ʾ��������ɫ ********************************/
#define      WHITE               0xFFFF    // ��ɫ
#define      BLACK               0x0000    // ��ɫ 
#define      GREY                0xF7DE    // ��ɫ 
#define      RED                 0xF800    // �� 
#define      MAGENTA             0xF81F    // ���ɫ 
#define      GRED                0xFFE0    // ���ɫ
#define      GREEN               0x07E0    // �� 
#define      CYAN                0x7FFF    // ��ɫ 
#define      YELLOW              0xFFE0    // ��ɫ 
#define      LIGHTGREEN          0X841F    // ǳ��ɫ 
#define      BLUE                0x001F    // �� 
#define      GBLUE               0x07FF    // ǳ�� 1
#define      LIGHTBLUE           0X7D7C    // ǳ�� 2
#define      BLUE2               0x051F    // ǳ�� 3
#define      GRAYBLUE            0X5458    // ���� 
#define      DARKBLUE            0X01CF    // ����

#define      LGRAY               0XC618    // ǳ��ɫ,���屳��ɫ
#define      LGRAYBLUE           0XA651    // ǳ����ɫ(�м����ɫ)
#define      LBBLUE              0X2B12    // ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)
                                                                      
        


/*****************************************************************************
 ** ����ȫ�ֺ���
****************************************************************************/
void LCD_Init (void);                                                          // ��ʼ��
void LCD_DisplayOn(void);                                                      // ����ʾ
void LCD_DisplayOff(void);                                                     // ����ʾ
void LCD_DisplayDir(u8 dir);                                                   // ��ʾ����, 0-��������3-��������5-������, 6-������
void LCD_DrawPoint(u16 x, u16 y, u16 color);                                   // ����
void LCD_Circle ( uint16_t usX_Center, uint16_t usY_Center, uint16_t usRadius, uint16_t color, uint8_t Filled );
void LCD_Line(u16 x1, u16 y1, u16 x2, u16 y2, u16 color);                      // ����
void LCD_Fill(u16 sx,u16 sy,u16 ex,u16 ey,u16 color);                          // ��䵥ɫ
void LCD_String(u16 x, u16 y, char* pFont, u8 size, u32 fColor, u32 bColor);   // ��ʾ��Ӣ���ַ���
void LCD_Image(u16 x, u16 y, u16 width, u16 height, const u8 *image) ;         // ��ʾͼ��
void LCD_Rectangle ( uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color, uint8_t filled );  // ������
void LCD_Image(u16 x, u16 y, u16 width, u16 height, const u8 *image) ;         // ��ʾͼ��
void LCD_Cross( uint16_t x, uint16_t y, uint16_t len, uint32_t fColor);        // ��ʮ����
void LCD_GUI(void);                                                            // ���Ƽ򵥽���
 
#endif /* __BSP_ILI9341_ILI9341_H */


