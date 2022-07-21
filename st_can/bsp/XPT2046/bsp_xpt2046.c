#include "bsp_xpt2046.h"


/******************************* ������ص�ȫ�ֱ��� ***************************/
xXPT2046_TypeDey xXPT2046;    // ���ڴ��XPT2046�ĸ�����Ϣ��ȫ�ֱ�������h��extern


/******************************* ������صľ�̬���� ***************************/



#define DURIATION_TIME      2            // ����������ֵ
#define XPT2046_CHANNEL_X   0x90         // �����֣����ͨ��Y+��ѹֵ    
#define XPT2046_CHANNEL_Y   0xd0         // �����֣����ͨ��X+��ѹֵ

#define  IRQ_READ           GPIO_ReadInputDataBit ( XPT2046_IRQ_GPIO, XPT2046_IRQ_PIN )
#define  CS_HIGH            (XPT2046_CS_GPIO->BSRR = XPT2046_CS_PIN)
#define  CS_LOW             (XPT2046_CS_GPIO->BRR  = XPT2046_CS_PIN)
#define  CLK_HIGH           (XPT2046_CLK_GPIO->BSRR = XPT2046_CLK_PIN)
#define  CLK_LOW            (XPT2046_CLK_GPIO->BRR  = XPT2046_CLK_PIN)
#define  MOSI_1             (XPT2046_MOSI_GPIO->BSRR = XPT2046_MOSI_PIN)
#define  MOSI_0             (XPT2046_MOSI_GPIO->BRR  = XPT2046_MOSI_PIN)
#define  MISO               (((XPT2046_MISO_GPIO->IDR) & XPT2046_MISO_PIN) ? 1 : 0 )




/******************************* ���� XPT2046 ��صľ�̬���� ***************************/
static void      delay_us(__IO uint32_t ulCount);     // ���Ե���ʱ����, Ϊ������ֲ, ��ʹ���ⲿ��ʱ����
static void      sendCMD(uint8_t cmd);               // ����������
static uint16_t  receiveData(void);                  // ��ȡ����ֵ
static uint8_t   touchDetect(void);                  // �жϴ������Ƿ��Ѱ������ȶ���0_���ȶ��ذ��£�1_�ͷ�
static int16_t   readADC_X(void);                    // ��ȡX��ADCֵ
static int16_t   readADC_Y(void);                    // ��ȡY��ADCֵ
static uint8_t   readAdcXY(void);                    // ��ȡX��Y��ADCֵ���˲�����ŵ�ȫ�ֽṹ�����xXPT2046��



static uint8_t   waitForFlashBSY(uint32_t timeOut);  // ���ڰ�У׼���ݴ�ŵ��ڲ�FLASH��:�ȴ�FLASH->SR:BSY����;
static uint8_t   readInteriorFlash(uint32_t readAddr, uint8_t *pBuffer, uint16_t numToRead);          // ���ڰ�У׼���ݴ�ŵ��ڲ�FLASH��: ��ȡ�ڲ�FLASH����
static uint8_t   writeInteriorFlash(uint32_t writeAddr, uint8_t *writeToBuffer, uint16_t numToWrite); // ���ڰ�У׼���ݴ�ŵ��ڲ�FLASH��: д���ڲ�FLASH����



// У׼���ݴ�ŵ��ڲ�FLASH, ����������
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// �������������壬���ڴ�ȡ�ڲ�FLASH���ݣ�F103ϵ�ж������޸�
#ifdef   STM32F10X_HD
    #define  STM32_FLASH_SECTOR_SIZE       2048                  // �ڲ�FLASHҳ��С, ��λ��bytes ��(ע��STM32F10xxϵ���£�С�������洢������Ϊ1K, �������洢������Ϊ2K��
#else
    #define  STM32_FLASH_SECTOR_SIZE       1024             ���� // �ڲ�FLASHҳ��С, ��λ��bytes ��(ע��STM32F10xxϵ���£�С�������洢������Ϊ1K, �������洢������Ϊ2K��
#endif

#define  STM32_FLASH_ADDR_BASE   0x08000000                  // оƬ�ڲ�FLASH��ַ(������������޸ģ�
static   uint8_t sectorbufferTemp[STM32_FLASH_SECTOR_SIZE];  // ����һ���ڴ�ռ䣬���ã����ڲ�FLASHд��ʱ�����ݻ���


// ���ã��ڲ�FLASHд��ʱ���ȴ����У�BSYλ��־:0��1æ
static uint8_t waitForFlashBSY(uint32_t timeOut)
{
    while ((FLASH->SR & 0x01) && (timeOut-- != 0x00)) ; // �ȴ�BSY��־����
    if (timeOut == 0)    return 1;                     // ʧ�ܣ�����1, �ȴ���ʱ����

    return 0;                                          // ���������أ�
}

/******************************************************************************
 * ������:  readInteriorFlash
 * ��  �ܣ� ��оƬ���ڲ�FLASH���ȡָ����������
 * ��  ���� uint32_t  readAddr     ���ݵ�ַ
 *          uint8_t  *pBuffer      ��������λ��
 *          uint16_t  numToRead    ��ȡ���ֽ�����
 * ����ֵ�� 0_�ɹ�
 *          1_ʧ�ܣ���ַС��FLASH��ַ
 *          2_ʧ�ܣ���ַ����FLASH���ֵ
 * ��  ע��
 ******************************************************************************/
static uint8_t  readInteriorFlash(uint32_t readAddr, uint8_t *pBuffer, uint16_t numToRead)
{
    uint16_t flashSize = *(uint16_t *)(0x1FFFF7E0);                       // ��ȡоƬFLASH��С;���Ĵ���ֵΪоƬ����ǰд���FLASH��С��ֻ����λ��KByte

    // �жϵ�ַ��Ч��
    if (readAddr < STM32_FLASH_ADDR_BASE)    return 1;                    // ������ĵ�ַ��С��FLASH����С��ַ�����˳�
    if (readAddr > (STM32_FLASH_ADDR_BASE + (flashSize * 1024)))  return 2; // ������ĵ�ַ������FLASH������ַ�����˳�

    // ��ʼ����
    while (numToRead--)
    {
        *pBuffer = *(__IO uint8_t *)readAddr;
        pBuffer++;        // ָ�����һ�����ݳ���
        readAddr++;       // ƫ��һ������λ
    }

    return 0;             // �ɹ�������0;
}


/******************************************************************************
 * ������:  writeInteriorFlash
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
static uint8_t writeInteriorFlash(uint32_t writeAddr, uint8_t *writeToBuffer, uint16_t numToWrite)
{
    uint16_t flashSize = *(uint16_t *)(0x1FFFF7E0);               // ��ȡоƬFLASH��С;���Ĵ���ֵΪоƬ����ǰд���FLASH��С��ֻ������λ��KByte

    uint32_t addrOff    = writeAddr - STM32_FLASH_ADDR_BASE;      // ȥ��0x08000000���ʵ��ƫ�Ƶ�ַ
    uint32_t secPos     = addrOff / STM32_FLASH_SECTOR_SIZE;;     // ������ַ,����ʼ��ַ�ڵڼ�������
    uint16_t secOff     = addrOff % STM32_FLASH_SECTOR_SIZE ;     // ��ʼ��ʼƫ���ֽ���: �����������ĵڼ��ֽڴ��
    uint16_t secRemain  = STM32_FLASH_SECTOR_SIZE - secOff;       // ��������Ҫд����ֽ��� ,�����жϹ�����������µ�����

    // �жϵ�ַ��Ч��
    if (writeAddr < STM32_FLASH_ADDR_BASE)    return 1;                    // ������ĵ�ַ��С��FLASH����С��ַ�����˳�������1_��ַʧ��
    if (writeAddr > (STM32_FLASH_ADDR_BASE + (flashSize * 1024)))    return 1; // ������ĵ�ַ������FLASH������ַ�����˳�, ����1_��ַʧ��

    // 0_����FLASH
    FLASH->KEYR = ((uint32_t)0x45670123);
    FLASH->KEYR = ((uint32_t)0xCDEF89AB);

    if (numToWrite <= secRemain)    secRemain = numToWrite;
    while (1)
    {
        // 1_��ȡ��ǰҳ������
        if (waitForFlashBSY(0x00888888))   return 2;                        // ʧ�ܣ�����:2, ʧ��ԭ��FLASH->SR:BSYæ��ʱ
        readInteriorFlash(secPos * STM32_FLASH_SECTOR_SIZE + STM32_FLASH_ADDR_BASE, sectorbufferTemp, STM32_FLASH_SECTOR_SIZE);   // ��ȡ�������ݵ�����

        // 2_����ָ��ҳ(����)
        if (waitForFlashBSY(0x00888888))   return 2;                        // ʧ�ܣ�����:2, ʧ��ԭ��FLASH->SR:BSYæ��ʱ
        FLASH->CR |= 1 << 1;                                                // PER:ѡ��ҳ������λ2MERΪȫ����
        FLASH->AR = STM32_FLASH_ADDR_BASE + secPos * STM32_FLASH_SECTOR_SIZE; // ��дҪ������ҳ��ַ
        FLASH->CR |= 0x40;                                                  // STRT:д1ʱ����һ�β���������
        if (waitForFlashBSY(0x00888888))   return 2;                        // ʧ�ܣ�����:��, ʧ��ԭ�򣺲�����ʱ
        FLASH->CR &= ((uint32_t)0x00001FFD);                                // �ر�ҳ��������

        for (uint16_t i = 0; i < secRemain ; i++)                           // ԭʼ����д�뻺��
            sectorbufferTemp[secOff + i] = writeToBuffer[i];

        for (uint16_t i = 0; i < STM32_FLASH_SECTOR_SIZE / 2 ; i++)         // ��������д��оƬFLASH
        {
            if (waitForFlashBSY(0x00888888))   return 2;                    // ʧ�ܣ�����:2, ʧ��ԭ��FLASH->SR:BSYæ��ʱ
            FLASH->CR |= 0x01 << 0;                                         // PG: ���
            *(uint16_t *)(STM32_FLASH_ADDR_BASE + secPos * STM32_FLASH_SECTOR_SIZE + i * 2) = (sectorbufferTemp[i * 2 + 1] << 8) | sectorbufferTemp[i * 2] ; // ��������д���豸

            if (waitForFlashBSY(0x00888888))   return 2;                    // ʧ�ܣ�����:2, ʧ��ԭ��FLASH->SR:BSYæ��ʱ
            FLASH->CR &= ((uint32_t)0x00001FFE) ;                           // �رձ��
        }

        if (secRemain == numToWrite)
        {
            break;                                                          // ��ȫ��д��
        }
        else                                                                // δд��
        {
            writeToBuffer += secRemain ;                                    // ԭʼ����ָ��ƫ��
            secPos ++;                                                      // ������
            secOff = 0;                                                     // ��ƫ��λ,������������ʼ��ַ
            numToWrite -= secRemain ;                                       // ʣ��δд�ֽ���
            secRemain = (numToWrite > STM32_FLASH_SECTOR_SIZE) ? (STM32_FLASH_SECTOR_SIZE) : numToWrite; // ����������д���ֽ���
        }
    }
    FLASH->CR |= 1 << 7 ;                                                   // LOCK:��������

    return 0;
}



// ����US������ʱ������������ֲʱ���ⲿ�ļ�������
static void delay_us(__IO uint32_t us)
{
    for (uint32_t i = 0; i < us; i++)
    {
        uint8_t uc = 12;     //����ֵΪ12����Լ��1΢��
        while (uc --);       //��1΢��
    }
}



// д��������
// Cmd ��0x90_ͨ��Y+��ѡ�������, 0xd0_ͨ��X+��ѡ�������
// XPT2046ʱ��Ҫ��CLK��ʱ�͵�ƽ�������ز������ݣ��½��ظı�����
// ע�⣺���������ֺ󣬷��ص����ݣ��������֣���ͬһ��������(CS�͡��ߵ�ƽ�ڼ�)
static void sendCMD(uint8_t cmd)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        ((cmd >> (7 - i)) & 0x01) ? MOSI_1 : MOSI_0; // ��λ����
        delay_us(1);
        CLK_HIGH  ;
        delay_us(1);
        CLK_LOW   ;
    }
}



// ��ȡ��������(����sendCMD��������);
// ���ض�ȡ��������
static uint16_t receiveData(void)
{
    uint16_t usBuf = 0;

    CLK_HIGH;              // ��һ��ʱ�ӣ����BUSY�����ʱ���Ǹ��ڷ��������ֺ���ģ�ADת����Ҫ��Լ6US
    delay_us(5);
    CLK_LOW ;
    delay_us(5);

    for (uint8_t i = 0; i < 12; i++)
    {
        usBuf = usBuf << 1;
        CLK_HIGH;
        delay_us(1);
        usBuf |= MISO ;    // ��λ����
        CLK_LOW;
        delay_us(1);
    }

    return usBuf;
}



// ѡ��һ��ģ��ͨ��������ADC��������ADC�������
// 0x90 :ͨ��Y+��ѡ�������
// 0xd0 :ͨ��X+��ѡ�������
static int16_t readADC_X(void)
{
    if (xXPT2046.dir == 0)    sendCMD(XPT2046_CHANNEL_Y);
    if (xXPT2046.dir == 1)    sendCMD(XPT2046_CHANNEL_X);
    return  receiveData();
}



static int16_t readADC_Y(void)
{
    if (xXPT2046.dir == 0)    sendCMD(XPT2046_CHANNEL_X);
    if (xXPT2046.dir == 1)    sendCMD(XPT2046_CHANNEL_Y);
    return  receiveData();
}



/******************************************************************************
 * �������� readAdcXY
 * ��  �ܣ� ��ȡ����������ʱX��Y��ADCֵ�����˲�
 * ��  ����
 * ��  �أ� 1  ��ȡ�ɹ����Ѵ�ŵ�����Ľṹ����
 *          0  ʧ��
 ******************************************************************************/
static uint8_t readAdcXY()
{
    uint8_t cnt = 0, i;
    uint16_t xSum = 0, ySum = 0;
    int16_t xyArray [2] [10] = {{0}, {0}};    // ��ʱ��ά���飬���ڴ������X��Y��10�β���
    int32_t xMin, xMax, yMin, yMax;           // �洢�����е���Сֵ�����ֵ;��������κ�ȥͷȥβ��ƽ��ֵ

    while ((IRQ_READ == 0) && (cnt < 10))     // ѭ������10��; ������TP_INT_IN�ź�Ϊ��(��Ļ������), �� cnt<10
    {
        xyArray[0] [cnt] = readADC_X();
        xyArray[1] [cnt] = readADC_Y();
        cnt ++;
    }

    // ��ʼ��ƽ��ֵ
    if (cnt == 10)
    {
        // ɸѡ�Ȼ�Ҫȥ������Сֵ�����ֵ
        xMax = xMin = xyArray [0] [0];
        yMax = yMin = xyArray [1] [0];
        for (i = 1; i < 10; i++)
        {
            if (xyArray[0] [i] < xMin)    xMin = xyArray [0] [i];   // ��x��10�β�����СADCֵ
            if (xyArray[0] [i] > xMax)    xMax = xyArray [0] [i];   // ��x��10�β������ADCֵ

            if (xyArray[1] [i] < yMin)    yMin = xyArray [1] [i];   // ��y��10�β�����СADCֵ
            if (xyArray[1] [i] > yMax)    yMax = xyArray [1] [i];   // ��y��10�β�����СADCֵ
        }
        // ȥ����Сֵ�����ֵ֮����ƽ��ֵ
        for (i = 0; i < 10; i++)
        {
            xSum = xSum + xyArray[0][i];
            ySum = ySum + xyArray[1][i];
        }

        xXPT2046.adcX = (xSum - xMin - xMax) >> 3;
        xXPT2046.adcY = (ySum - yMin - yMax) >> 3;

        return 1;
    }

    return 0;
}



// �ѵ�ѹֵ����ɶ�Ӧ��LCD����ֵ
static void adcXYToLcdXY(void)
{
    static int16_t lcdX = 0;
    static int16_t lcdY = 0;
    // �������ϵ��
    lcdX = xXPT2046.adcX * xXPT2046.xfac + xXPT2046.xoff ;
    lcdY = xXPT2046.adcY * xXPT2046.yfac + xXPT2046.yoff ;
    // ��������ֵ��Χ
    // if(lcdX<0)  lcdX=0;
    if (lcdX > xXPT2046.lcdWidth)  lcdX = xXPT2046.lcdWidth;
    if (lcdY < 0)  lcdY = 0;
    if (lcdY > xXPT2046.lcdHeight)  lcdY = xXPT2046.lcdHeight;
    // ������, ����ֵ�������ֵ, ת�浽�ṹ��, ��ʱ�ɵ���
    xXPT2046.lcdX = lcdX;
    xXPT2046.lcdY = lcdY;
}


/******����״̬�����******/
typedef enum
{
    _STATUS_0_FREE  = 0,                   // �����ͷ�
    _STATUS_2_WAITING,                     // �ȴ��ȶ�
    _STATUS_3_PRESSED,                     // ��������
} eflagTouchStatus;

/******************************************************************************
 * �������� touchDetect
 * ��  �ܣ� ���������״̬��
 * ��  ����
 * ��  �أ� 1   �����ͷ���
 *          0   ����������
 ******************************************************************************/
static uint8_t touchDetect(void)
{
    static eflagTouchStatus  eTouchStatus = _STATUS_0_FREE;  //  �����ͷ� XPT2046_STATE_FREE=0�� �ȴ��ȶ� XPT2046_STATE_WAITING=1�� �������� XPT2046_STATE_PRESSED=2
    static uint32_t timeCNT = 1;

    switch (eTouchStatus)
    {
    case _STATUS_0_FREE:
        if (IRQ_READ  == 0)                             // ��һ�γ��ִ����͵�ƽ�ź�
            eTouchStatus = _STATUS_2_WAITING;           //
        else
            eTouchStatus = _STATUS_0_FREE;              // �޴���
        break;

    case _STATUS_2_WAITING:
        if (IRQ_READ  == 0)
        {
            timeCNT++;
            //�ȴ�ʱ�������ֵ����Ϊ����������
            //����ʱ�� = DURIATION_TIME * �����������õ�ʱ����
            //���ڶ�ʱ���е��ã�ÿ10ms����һ�Σ�������ʱ��Ϊ��DURIATION_TIME*10ms
            if (timeCNT > DURIATION_TIME)
            {
                timeCNT = 1;
                eTouchStatus = _STATUS_3_PRESSED;
            }
            else                                        // �ȴ�ʱ���ۼ�
                eTouchStatus = _STATUS_2_WAITING;
        }
        else                                            // �ȴ�ʱ��ֵδ�ﵽ��ֵ��Ϊ��Ч��ƽ�����ɶ�������
        {
            timeCNT = 1;
            eTouchStatus = _STATUS_0_FREE;
        }
        break;

    case _STATUS_3_PRESSED:
        if (IRQ_READ  == 0)                             // ������������
            eTouchStatus = _STATUS_3_PRESSED;
        else                                            // �����ͷ�
            eTouchStatus = _STATUS_0_FREE;
        break;

    default:
        eTouchStatus = _STATUS_0_FREE;
        break;
    }

    // �����������²��ȶ��󣬷���0, ���򷵻�1
    return (eTouchStatus == _STATUS_3_PRESSED) ? 0 : 1 ;
}





// ������Ǳ��غ���
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// �������ȫ�ֺ���





/******************************************************************************
 * �������� XPT2046_Init
 * ��  �ܣ� ��ʼ��
 * ��  ���� uint16_t lcdWidth     LCD�������
 *          uint16_t lcdHeight    LCD�������
 *          uint8_t dir           ��ʾ����    0-��������3-��������5-������, 6-������
 * ��  �أ�
 ******************************************************************************/
void XPT2046_Init(uint16_t lcdWidth, uint16_t lcdHeight, uint8_t dir)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    uint8_t flashDATA[50] = {0};
    uint32_t flashSize = *(uint16_t *)(0x1FFFF7E0);           // ��ȡоƬFLASH��С;���Ĵ���ֵΪоƬ����ǰд���FLASH��С��ֻ����λ��KByte
    xXPT2046.dataAddr  = 0x08000000 + (flashSize - 1) * 1024; // У׼���ݵĴ��λ��: �ڲ�FLASH�����1K�Ŀ�ͷ
    xXPT2046.lcdWidth  = lcdWidth;
    xXPT2046.lcdHeight = lcdHeight;
    xXPT2046.dir = dir;

    // ����GPIOʱ��
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPDEN | RCC_APB2ENR_IOPEEN | RCC_APB2ENR_IOPFEN ;

    // ģ��SPI GPIO��ʼ��
    GPIO_InitStructure.GPIO_Pin   = XPT2046_CS_PIN;      // CS
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(XPT2046_CS_GPIO, &GPIO_InitStructure);

    CS_HIGH;                                             // ����Ƭѡ����ֹ�����

    GPIO_InitStructure.GPIO_Pin   = XPT2046_CLK_PIN;     // CLK
    GPIO_Init(XPT2046_CLK_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = XPT2046_MOSI_PIN;    // MOSI
    GPIO_Init(XPT2046_MOSI_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = XPT2046_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;       // MISO, ��������
    GPIO_Init(XPT2046_MISO_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = XPT2046_IRQ_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        // �����������ź�ָʾ���ţ���ʹ���ж�
    GPIO_Init(XPT2046_IRQ_GPIO, &GPIO_InitStructure);

    CLK_LOW ;                                            // XPT2046ʱ��Ҫ��CLK ��ʱ�͵�ƽ�������ز������ݣ��½��ظı�����
    MOSI_0;                                              // XPT2046ʱ��Ҫ��MOSI��ʱ�͵�ƽ
    CS_LOW;                                              // ����Ƭѡ��ʹXTP2046��ʼͨ��

    // ͨ����ȡ������, ����Ƿ���У׼
    readInteriorFlash(xXPT2046.dataAddr, flashDATA, 20);
    if ((flashDATA[0] == 'O') && (flashDATA[1] == 'K'))  // ������У׼������
    {
        xXPT2046.xfac = *(float *)(flashDATA + 2);
        xXPT2046.xoff = *(short *)(flashDATA + 6);
        xXPT2046.yfac = *(float *)(flashDATA + 10);
        xXPT2046.yoff = *(short *)(flashDATA + 14);
    }
    else                                                 // û��У׼����
    {
        XPT2046_ReCalibration();                         // ����У׼
    }
}


/******************************************************************************
 * �������� XPT2046_ReCalibration
 * ��  �ܣ� ����У׼������,
 *          ����У׼�����ݴ����ڲ�FLASH, �Է����´ε���
 * ��  ���� ȡ��ԭ���Ĳ�������, ֱ����ȫ�ֽṹ���л��:xXPT2046
 *          uint16_t lcdWidth     LCD�������
 *          uint16_t lcdHeight    LCD�������
 *          uint8_t dir           ��ʾ����    0-��������3-��������5-������, 6-������
 * ��  �أ� 0_У׼�ɹ�
 *          1_У׼ʧ��
 ******************************************************************************/
uint8_t  XPT2046_ReCalibration(void)
{
    uint16_t pixelOff = 30;   // ƫ������,������ʮ��
    uint16_t adcX1, adcX2, adcX3, adcX4, adcY1, adcY2, adcY3, adcY4; // ��¼У׼�����е�����ֵ
    float xfac = 0;
    float yfac = 0;
    short xoff = 0;
    short yoff = 0;
    uint16_t crossX = 0;      // ���ڻ�ʮ����
    uint16_t crossY = 0;      // ���ڻ�ʮ����
    char strTemp[30];
    uint16_t lcdWidth  = xXPT2046.lcdWidth;
    uint16_t lcdHeight = xXPT2046.lcdHeight;

    printf("\r\n������У׼��....\r\n");
    printf("lcdWidth=%d  , lcdHeigh=%d\r\n", lcdWidth, lcdHeight);
    LCD_Fill(0, 0, lcdWidth, lcdHeight, BLACK);
    LCD_String(45, 110, "ReCalibration......", 16, WHITE, BLACK);
    LCD_String(52, 130, "Click the Cross!!", 16, WHITE, BLACK);


    // ���Ͻ�
    crossX = pixelOff;
    crossY = pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // �򿪴������(����)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // ��ʮ��
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // �ȴ�����
    XPT2046_Cmd(DISABLE);                                             // �رմ������(����)
    adcX1 = xXPT2046 .adcX;                                           // ��¼��ȡ�õ�adcֵ
    adcY1 = xXPT2046.adcY;                                            // ��¼��ȡ�õ�adcֵ
    LCD_Cross(crossX, crossY, 20, BLACK);                             // Ĩȥʮ��
    sprintf(strTemp, "X:%d", adcX1);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // ��ʾ
    sprintf(strTemp, "Y:%d", adcY1);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // ��ʾ
    delay_us(400000);

    // ���Ͻ�
    crossX = lcdWidth - pixelOff;
    crossY = pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // �򿪴������(����)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // ��ʮ��
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // �ȴ�����
    XPT2046_Cmd(DISABLE);                                             // �رմ������(����)
    adcX2 = xXPT2046 .adcX;                                           // ��¼��ȡ�õ�adcֵ
    adcY2 = xXPT2046.adcY;                                            // ��¼��ȡ�õ�adcֵ
    LCD_Cross(crossX, crossY, 20, BLACK);                             // Ĩȥʮ��
    sprintf(strTemp, "X:%d", adcX2);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // ��ʾ
    sprintf(strTemp, "Y:%d", adcY2);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // ��ʾ
    delay_us(400000);

    // ���½�
    crossX = pixelOff;
    crossY = lcdHeight - pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // �򿪴������(����)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // ��ʮ��
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // �ȴ�����
    XPT2046_Cmd(DISABLE);                                             // �رմ������(����)
    adcX3 = xXPT2046 .adcX;                                           // ��¼��ȡ�õ�adcֵ
    adcY3 = xXPT2046.adcY;                                            // ��¼��ȡ�õ�adcֵ
    LCD_Cross(crossX, crossY, 20, BLACK);                             // Ĩȥʮ��
    sprintf(strTemp, "X:%d", adcX3);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // ��ʾ
    sprintf(strTemp, "Y:%d", adcY3);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // ��ʾ
    delay_us(400000);

    // ���½�
    crossX = lcdWidth - pixelOff;
    crossY = lcdHeight - pixelOff;
    xXPT2046.adcX = 0;
    xXPT2046.adcY = 0;
    XPT2046_Cmd(ENABLE);                                              // �򿪴������(����)
    LCD_Cross(crossX, crossY, 20, YELLOW);                            // ��ʮ��
    while ((readAdcXY() == 0) || (xXPT2046.adcX == 0));               // �ȴ�����
    XPT2046_Cmd(DISABLE);                                             // �رմ������(����)
    adcX4 = xXPT2046 .adcX;                                           // ��¼��ȡ�õ�adcֵ
    adcY4 = xXPT2046.adcY;                                            // ��¼��ȡ�õ�adcֵ
    LCD_Cross(crossX, crossY, 20, BLACK);                             // Ĩȥʮ��
    sprintf(strTemp, "X:%d", adcX4);
    LCD_String(crossX - 12, crossY - 16, strTemp, 12, YELLOW, BLACK); // ��ʾ
    sprintf(strTemp, "Y:%d", adcY4);
    LCD_String(crossX - 12, crossY, strTemp, 12, YELLOW, BLACK);      // ��ʾ
    delay_us(400000);

    //timeCNT=0;
    // ȡadcX��adcY��ƽ��ֵ; �����ȡƽ��ֵ, �ڶԽǻ�����ʮ���߼���
    adcX1 = (adcX1 + adcX3) / 2;
    adcX2 = (adcX2 + adcX4) / 2;

    adcY1 = (adcY1 + adcY2) / 2;
    adcY2 = (adcY3 + adcY4) / 2;

    xfac = (float)(pixelOff - (lcdWidth - pixelOff)) / (adcX1 - adcX2); // ��������LCD���������ϵ��,  xfac=(float)(20-320)/(t1x-t2x);
    yfac = (float)(pixelOff - (lcdHeight - pixelOff)) / (adcY1 - adcY2);
    xoff = (lcdWidth - xfac * (adcX1 + adcX2)) / 2;   // ���ص�ƫ��ֵ, xoff=(320-xfac*(t1x+t2x))/2;
    yoff = (lcdHeight - yfac * (adcY1 + adcY2)) / 2;

    // �����ڲ�FLASH��
    uint8_t err = 0;
    uint16_t flag = 'O' | ('K' << 8);
    err |= writeInteriorFlash(xXPT2046.dataAddr, (uint8_t *)&flag, 2);
    err |= writeInteriorFlash(xXPT2046.dataAddr + 2, (uint8_t *)&xfac, 4);
    err |= writeInteriorFlash(xXPT2046.dataAddr + 6, (uint8_t *)&xoff, 4);
    err |= writeInteriorFlash(xXPT2046.dataAddr + 10, (uint8_t *)&yfac, 4);
    err |= writeInteriorFlash(xXPT2046.dataAddr + 14, (uint8_t *)&yoff, 4);

    if (err > 0)
    {
        printf(">>>У׼���, ���������ʱʧ��!\r\n");
        return err;
    }

    xXPT2046.xfac = xfac;
    xXPT2046.xoff = xoff;
    xXPT2046.yfac = yfac;
    xXPT2046.yoff = yoff;

    printf(">>>У׼���! ����ϵ����ƫ��ֵ�Ѵ����ڲ�FLASH, ��ַ:0x%X\r\n", xXPT2046.dataAddr);

    SCB->AIRCR = 0X05FA0000 | (u32)0x04; // �����ϵ�
    return 0;
}



/******************************************************************************
 * �������� XPT2046_Cmd
 * ��  �ܣ� ������⿪��
 *          ���ڼ��ȽϺ�ʱ, �ڲ�ʹ�ô�����״̬��, ���Թرռ���Խ�ʡоƬ��Դ
 *          �˿���״̬, ֻ������XPT2046_TouchHandler();
 * ��  ���� 0_�رմ������ļ��,�Խ�ʡ��Դ
 *          1_�򿪴������
 * ��  �أ�
 ******************************************************************************/
void XPT2046_Cmd(uint8_t status)
{
    if (status != 0)
    {
        xXPT2046 .EN = 1;
    }
    else
    {
        xXPT2046.EN = 0;
    }
}



/******************************************************************************
 * �������� XPT2046_GetXY
 * ��  �ܣ� ��ȡ��ǰ����λ�õ�����ֵ
 * ��  ���� uint16_t* X   x�������
 *          uint16_t* Y   y�������
 * ��  �أ� ��
 ******************************************************************************/
void  XPT2046_GetXY(uint16_t *X, uint16_t *Y)
{
    *X = xXPT2046.lcdX;
    *Y = xXPT2046.lcdY;
}



/******************************************************************************
 * �������� XPT2046_TouchHandler
 * ��  �ܣ� �����жϴ�����
 *          ��������Ҫ��whileѭ���ﱻ���ã�Ҳ��ʹ�ö�ʱ����ʱ����, ���ü��1ms�����
 * ��  ���� ��
 * ��  �أ� ��
 ******************************************************************************/
void XPT2046_TouchHandler(void)
{
    // ������������(0_�ر�, 1_��), �Ҵ������Ѱ���(0_���������ȶ���1_δ����)
    if ((xXPT2046.EN == 1) && (touchDetect() == 0))
    {
        // ��ȡ����ʱ��λ�õ�ѹֵ
        if (readAdcXY() == 0)
            return ;
        adcXYToLcdXY();

        // ���������Ļ���µĺ���(�û��Լ��Ĺ��ܶ���)
        XPT2046_TouchDown();
    }
    else
    {
        // ���������Ļû�а��µĺ���(�û��Լ��Ĺ��ܶ���)
        XPT2046_TouchUp();
    }
}



/******************************************************************************
 * �������� XPT2046_TouchDown
 * ��  �ܣ� ��������ʱ�Ĵ���
 *          �հ׺���, �û����б�д����
 * ��  ����
 *
 * ��  �أ�
 ******************************************************************************/
void XPT2046_TouchDown(void)
{
    // ʾ����λ�ȡ����ʱ��LCD����
    //sprintf(strTemp, "%4d,%4d", xXPT2046.lcdX, xXPT2046.lcdY);
    //LCD_String(180,290,strTemp, 12, BLUE , WHITE);              // ��LCD����ʾ��������

    // ʾ���ڴ����㻭һ����ɫ��
    LCD_DrawPoint(xXPT2046.lcdX, xXPT2046.lcdY, YELLOW);
}



/******************************************************************************
 * �������� XPT2046_TouchUp
 * ��  �ܣ� ��������ʱ�Ĵ���
 *          �հ׺���, �û����б�д����
 * ��  ����
 *
 * ��  �أ�
 ******************************************************************************/
void XPT2046_TouchUp(void)
{


}




