#ifndef  __BSP_XPT2046_H
#define  __BSP_XPT2046_H


#include "stm32f10x.h"
#include "bsp_lcd_ili9341.h"
#include "stdio.h"



/*****************************************************************************
 ** ���ų趨��
 ** ��ע��ʹ�õ���ģ��SPI, INT����ʹ��״̬��ѯ, ��ʹ���ж�;
****************************************************************************/
#define    XPT2046_IRQ_GPIO                 GPIOE        // �����ź�ָʾ����(��ʹ���ж�) 
#define    XPT2046_IRQ_PIN                  GPIO_Pin_4

#define    XPT2046_CS_GPIO                  GPIOD        // ģ��SPI_CS
#define    XPT2046_CS_PIN                   GPIO_Pin_13  

#define    XPT2046_CLK_GPIO                 GPIOE        // ģ��SPI_CLK
#define    XPT2046_CLK_PIN                  GPIO_Pin_0   

#define    XPT2046_MOSI_GPIO                GPIOE        // ģ��SPI_MOSI
#define    XPT2046_MOSI_PIN                 GPIO_Pin_2   

#define    XPT2046_MISO_GPIO                GPIOE        // ģ��SPI_MISO
#define    XPT2046_MISO_PIN                 GPIO_Pin_3   




/*****************************************************************************
 ** ��������������
 *****************************************************************************/  



                                            
/*****************************************************************************
 ** ������������
 *****************************************************************************/                     
typedef    struct        // ����������ṹ��          
{          
    uint8_t EN;          // ������⿪��, 0_�ر�, 1_��; �رպʹ�, ��ָ�����ϵļ�����, ����ʱ�ɹر�, ����оƬ��Դ����
       
    uint16_t lcdX;       // ��ǰ���µ�LCD����ֵ(
    uint16_t lcdY;                 
                
    uint16_t adcX;       // ��������ȡ�Ĵ�����X�����ADCֵ������ƽ��ֵ�˲�
    uint16_t adcY;                     
                
    uint16_t lcdWidth;   // ���ڼ�¼LCDʵ�ʵĿ������; ��XPT2046_Init�e�xֵ         
    uint16_t lcdHeight;  // ���ڼ�¼LCDʵ�ʵĸ߶�����; ��XPT2046_Init�e�xֵ            
                
    float xfac;          // ��������LCD���������ϵ��,  xfac=(float)(20-320)/(t1x-t2x);
    float yfac; 
    short xoff;          // ���ص�ƫ��ֵ, xoff=(320-xfac*(t1x+t2x))/2;
    short yoff;                 
                
    uint8_t  dir;        // ��ʾ����, 0-����, 1_����       
    uint32_t dataAddr;   // �����������ڲ�FLASH�еĴ�ŵ�ַ                
}xXPT2046_TypeDey;  


/*****************************************************************************
 ** ����ȫ�ֱ���
 *****************************************************************************/
extern xXPT2046_TypeDey xXPT2046;





/*****************************************************************************
 ** ����ȫ�ֺ�������
 ** ����ʹ��˵����
 **     ����������ǰ, ��У׼����Ļ����;
 **     ����XPT2046_Init()��ʼ����, 
 **     ͨ��XPT2046_Cmd()��XPT2046_TouchDown()��XPT2046_TouchDown()���������������,
 **     ��ʵ�ִ󲿷ݵ�����ʵ��;                                     
 *****************************************************************************/
void     XPT2046_Init (uint16_t lcdWidth, uint16_t lcdHeight, uint8_t dir ); // ��ʼ��(����, У׼���);  ����: ��ʾ������ʾ����, ȡֵ��Χ: 1,2,3,4
uint8_t  XPT2046_ReCalibration(void);                                        // ������������У׼
void     XPT2046_Cmd(uint8_t status);                                        // ��������
void     XPT2046_GetXY(uint16_t* X, uint16_t* Y);                            // ��ȡ����������ֵ, �������X��Y��; ������Ǳ�Ҫ��, ͨ��XPT2046_Cmd()��XPT2046_TouchDown()��XPT2046_TouchDown()                            
void     XPT2046_TouchHandler(void);                                         // �����жϴ�����        
void     XPT2046_TouchDown(void);                                            // �հ׺���, ���ð���ʱ�Ĵ���
void     XPT2046_TouchUp(void);                                              // �հ׺���, �����ͷ�ʱ�Ĵ���   




#endif 

