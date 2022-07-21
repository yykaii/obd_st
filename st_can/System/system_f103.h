#ifndef __SYSTEM_F103_H
#define __SYSTEM_F103_H    
/***********************************************************************************************************************************
 ** ���ļ����ơ�  system_f103.h
 ** ����д��Ա��  ħŮ�������Ŷ�
 ** �����·���  QȺ�ļ���       1126717453
 ** ����    ����  ħŮ������      https://demoboard.taobao.com
 ***********************************************************************************************************************************
 **�� ��  �� ��  �򻯳��õ�ϵͳ��������ʼ������
 **
 **�� h �ļ� ��  �������ݶ��塢��д��
 **              ----����������д
 **              ----λ��     
 ** 
 **              ���ú���������                
 **              ----��ʼ�� USART1 ��Ϊ������Ϣ��� 
 **              ----��ʼ�� SysTickʱ�ӣ� ʹSystem_DelayMS()��System_DelayUS()�������� 
 **              ----���� GPIOSet()��NVICSet()��EXTISet(), ���ٴ������빤�� 
 **                 
 **����ֲʹ�á�  ��ο���ԭ�ӡ�Ұ��ȴ���󣬽�Ϲ�����д����;
 **              ���ԣ���ֲʹ�ã����ܻ�������ԭ��ԭsys�ļ�����3���ļ�sys.c��delay.c��usart.c�������ϵĳ�ͻ��
 **              ���ļ��Ѱ������������������ļ��Ĺ��ܣ�ʹ��ʱ�����ȳ���ȡ�����������ļ������ã����ɽ����ͻ���⡣
 **
 **�����¼�¼��  2020-11-23  �ļ�����magic_f103��Ϊmanage_f103;
 **              2020-11-24  delay_ms(),delay_us() ����ע��;
 **              2020-12-15  ����print()��PrintString()������ע��;
 **              2021-04-03  ����GPIOSet()��������;
 **              2021-04-21  �ļ���manage_f103.h�� ��Ϊ��system_f103.h
 **              2021-06-12  �Ƴ�USART1�궨�壬ʹ���Ϊ�����ļ�
 **              2021-07-20  �޸�EXTI�����ء��½��غ궨������
 **              2021-09-07  �����ڲ�FLASH���ݴ�ȡ����
 **
***********************************************************************************************************************************/  
#include <stm32f10x.h>       // ����ʹ���û�Ŀ¼���ļ��������޸��Ż�������ļ��Ǳ���ģ� ���ֵ�ַ�Ͳ����ĺ궨��
#include "stdio.h"           // C��׼��ͷ�ļ�: �������������getchar()��putchar()��scanf()��printf()��gets()��puts()��sprintf()
#include "stdlib.h"          // C��׼��ͷ�ļ�: ͨ�ù��ߺ�����malloc()��calloc()��realloc()��free()��system()��atoi()��atol()��rand()��srand()��exit()
#include "stdbool.h"         // C��׼��ͷ�ļ�: ���岼������: bool��true��false
#include "string.h"          // C��׼��ͷ�ļ�: �ַ����鳣�ã�strcpy()��strncpy()��strcmp()��strlen()��strnset()

//#include "scheduler.h"



/*****************************************************************************
 ** ��ֲ�޸���
 ** ��ֲʱ�����ʹ��SPI1,ֻ��Ҫ�޸��������
****************************************************************************/



// ȫ��״̬��־�ṹ��,  0=ʧ��/û��, 1=�ɹ�/����
typedef struct 
{
    uint8_t  allOK;                     // ȫ��״̬����        0=ʧ��   1=�ɹ�  

    uint8_t  PrintfOK;                  // ���USART�Ƿ�����   0=ʧ��   1=�ɹ�   ��ֹ������ǰ����printf����
    uint8_t  usartRecived;              // �������õĴ��ڽ��յ����ݺ󣬵ȴ������־�� u8 ReceivedBuffer[255]
    
    uint8_t  RTCOK;  
    // LCD
    uint8_t  LcdOK;                     // lcd״̬ 
    uint8_t  LcdType;                   // �ͺ�
    uint8_t  LcdInfo[60];               // ״̬��Ϣ  
    
}_flag; 
extern _flag xFlag;



////////////////////////////////////////////////////////////////////////////////// 
// �� 1 �� �������Ͷ̹ؼ���, ��stdint.h�Ļ����Ͻ��� 
// �з���                               // stdint.h �ϵ����Ͷ���
typedef              int8_t    s8;      // typedef   signed          char int8_t; 
typedef             int16_t   s16;      // typedef   signed short     int int16_t;
typedef             int32_t   s32;      // typedef   signed           int int32_t;
typedef             int64_t   s64;      // typedef   signed       __INT64 int64_t;
// �޷���
typedef             uint8_t    u8;      // typedef unsigned          char uint8_t;
typedef            uint16_t   u16;      // typedef unsigned short     int uint16_t;
typedef            uint32_t   u32;      // typedef unsigned           int uint32_t;
typedef            uint64_t   u64;      // typedef unsigned       __INT64 uint64_t;
// 
typedef  volatile   uint8_t   vu8;      // volatile ÿһ�ζ�ȡ��Ҫ���ڴ��ȡ����ֹ��������Ż���
typedef  volatile  uint16_t  vu16;
typedef  volatile  uint32_t  vu32;  
//
#ifndef  bool
#define  bool      _Bool
#endif
//
#ifndef  true 
#define  true      1
#endif 
//
#ifndef  false
#define  false     0  
#endif
 
 
#ifndef USE_STDPERIPH_DRIVER
// GPIO���Ź���ģʽ   
#define GPIO_Mode_AIN              0x00
#define GPIO_Mode_IN_FLOATING      0x04
#define GPIO_Mode_IPD              0x28
#define GPIO_Mode_IPU              0x48
#define GPIO_Mode_Out_OD           0x14
#define GPIO_Mode_Out_PP           0x10
#define GPIO_Mode_AF_OD            0x1C
#define GPIO_Mode_AF_PP            0x18      
// GPIO���ű�Ŷ���
#define GPIO_Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /*!< Pin 1 selected */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /*!< Pin 2 selected */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /*!< Pin 5 selected */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /*!< Pin 6 selected */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /*!< Pin 7 selected */
#define GPIO_Pin_8                 ((uint16_t)0x0100)  /*!< Pin 8 selected */
#define GPIO_Pin_9                 ((uint16_t)0x0200)  /*!< Pin 9 selected */
#define GPIO_Pin_10                ((uint16_t)0x0400)  /*!< Pin 10 selected */
#define GPIO_Pin_11                ((uint16_t)0x0800)  /*!< Pin 11 selected */
#define GPIO_Pin_12                ((uint16_t)0x1000)  /*!< Pin 12 selected */
#define GPIO_Pin_13                ((uint16_t)0x2000)  /*!< Pin 13 selected */
#define GPIO_Pin_14                ((uint16_t)0x4000)  /*!< Pin 14 selected */
#define GPIO_Pin_15                ((uint16_t)0x8000)  /*!< Pin 15 selected */
#define GPIO_Pin_All               ((uint16_t)0xFFFF)  /*!< All pins selected */

// 
#define GPIO_Speed_10MHz           1        // GPIO�ٶ�2Mhz
#define GPIO_Speed_2MHz            2        // GPIO�ٶ�2Mhz
#define GPIO_Speed_50MHz           3        // GPIO�ٶ�50Mhz
#endif

   
   
// Ϊ����F4xx����д��� *************************************************************************
// ʱ��ʹ��
#define  RCC_APB2ENR_GPIOAEN      ((uint32_t)0x00000004)         /*!< I/O port A clock enable */
#define  RCC_APB2ENR_GPIOBEN      ((uint32_t)0x00000008)         /*!< I/O port B clock enable */
#define  RCC_APB2ENR_GPIOCEN      ((uint32_t)0x00000010)         /*!< I/O port C clock enable */
#define  RCC_APB2ENR_GPIODEN      ((uint32_t)0x00000020)         /*!< I/O port D clock enable */
#define  RCC_APB2ENR_GPIOEEN      ((uint32_t)0x00000040)         /*!< I/O port D clock enable */
// GPIOSet()ר�ò���        
//#define GPIO_Mode_AIN           0       // ģ������ģʽ
//#define GPIO_Mode_IN            1       // ��ͨ����ģʽ
//#define GPIO_Mode_OUT           2       // ��ͨ���ģʽ
//#define GPIO_Mode_AF            3       // AF����ģʽ
//
#define GPIO_OType_PP             0       // �������
#define GPIO_OType_OD             1       // ��©��� 
//
//#define GPIO_Speed_10MHz        1       // GPIO�ٶ�2Mhz
//#define GPIO_Speed_2MHz         2       // GPIO�ٶ�2Mhz
//#define GPIO_Speed_50MHz        3       // GPIO�ٶ�50Mhz
//
#define GPIO_PuPd_NOPULL          0       // ����������
#define GPIO_PuPd_UP              1       // ����
#define GPIO_PuPd_DOWN            2       // ����
// EXTISet()ר�ò���
#define EXTI_TRIGGER_FALLING      1       // �½��ش���
#define EXTI_TRIGGER_RISING       2       // �����ش���
// System_MCOxInit()ר�ò���
// MCO1�������ʱ��Ƶ��
#define RCC_MCO1Source_SYSCLK     4       // 0100
#define RCC_MCO1Source_HSI        5       // 0101
#define RCC_MCO1Source_HSE        6       // 0110
#define RCC_MCO1Source_NULL       0       // 0000���ر�

// ��������ƫ��ֵ�� This value must be a multiple of 0x200
#define VECT_TAB_OFFSET           0x0      

//λ������,ʵ��51���Ƶ�GPIO���ƹ���
//����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
//IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr    (GPIOA_BASE+12) //0x4001080C 
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C 
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C 
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C    
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C    

#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008 
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408 
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808 
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08 
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 
 
//IO�ڲ���,ֻ�Ե�һ��IO��!
//ȷ��n��ֵС��16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //��� 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //���� 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //��� 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //���� 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //��� 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //���� 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //��� 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //���� 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //��� 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //����

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //��� 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //����

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //��� 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //����




/*****************************************************************************
 ** ����  ȫ�ֺ���
 ** ����  
 ** ����  ���Ϸ���QȺ  1126717453
****************************************************************************/
// SysTick 
void  System_SysTickInit (void);                                             // ����SysTickʱ�ӣ����ú�System_DelayMS()��System_DelayUS()����ʹ��
void  System_DelayMS (u32);                                                  // ������ʱ  
void  System_DelayUS (u32);                                                  // ΢����ʱ
u64   System_GetTimeMs (void);                                               // ��ȡ SysTick ��ʱ��, ��λ:ms
u32   System_GetTimeInterval (void);                                         // �������ʱ��
void  System_TestRunTimes (void);                                            // printf��ӡ�������ʱ�� 
// �򻯳�ʼ�������3����
void  System_GPIOSet(GPIO_TypeDef* GPIOx, u32 allPin, u8 mode, u8 speed);    // GPIO��ʼ��
void  System_NVICSet (u8 NVIC_Channel, u8 Preemption);                       // NVIC���ȼ�����, �ѷֺ���, 4λ��ռ��, 16��, ���Ӽ�
void  System_EXTISet (GPIO_TypeDef* GPIOx, u16 PINx, u8 TRIM);               // �ⲿ�ж����ú���
// ��������
void  System_SwdMode (void);                                                 // SWD����ģʽ���ر�JTAGֻ����SWD�����ͷ�����PB3��PB4��PA15��ֻ��PA13��PA14
u32   System_GetSystemClock (void);                                          // ��ȡϵͳʱ��Ƶ��
void  System_Reset (void);                                                   // ϵͳ��λ  
void  System_MCO1Init (uint32_t);                                            // ��ѡ����,ǰ� RCC_MCO1Source_ +++ HSI , LSE , HSE , PLLCLK
void  System_MCO2Init (uint32_t);                                            // ��ѡ����,ǰ� RCC_MCO2Source_ +++ SYSCLK , PLLI2SCLK , HSE , PLLCLK
uint8_t  System_ReadInteriorFlash (uint32_t addr, uint8_t *buf, uint16_t num);  // ��оƬ���ڲ�FLASH���ȡָ����������
uint8_t  System_WriteInteriorFlash(uint32_t addr, uint8_t *buf, uint16_t num);  // ��оƬ���ڲ�FLASH�д��ָ����������
#endif



