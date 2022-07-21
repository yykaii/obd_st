#include "bsp_can.h"
#include "string.h"


xCAN_InfoDef  xCAN;         // ����Ϊȫ�ֱ���,�����¼��Ϣ��״̬




void CAN1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef  CAN_InitStructure;

    // ʱ��ʹ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    // ��ʼ��GPIO
    GPIO_InitStructure.GPIO_Pin   = CAN1_TX_PIN ;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CAN1_TX_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin   = CAN1_RX_PIN ;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;     // CAN_RX����ģʽ�������������������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CAN1_RX_GPIO, &GPIO_InitStructure);

    // ��ʼ��CAN���衢�����ʡ�λ�����
    CAN_InitStructure.CAN_ABOM = ENABLE;               // �Զ����߹���
    CAN_InitStructure.CAN_AWUM = ENABLE;               // �Զ�����
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal ;     // ����ģʽ����Normal=����, LoopBack=�ػ�, Silent=��Ĭ, Silent_LoopBack=��Ĭ�ػ�
    CAN_InitStructure.CAN_NART = ENABLE;               // �����ش�
    CAN_InitStructure.CAN_RFLM = DISABLE;              // FIFO�������µĲ��ܸ��Ǿɵ�
    CAN_InitStructure.CAN_TTCM = DISABLE;              // ʱ�䴥����DISABLE-��ʱ�䴥��ͨ��ģʽ
    CAN_InitStructure.CAN_TXFP = DISABLE;              // �����Ĵ���������Ⱥ�˳���ͣ�����Ĭ��ID���ȼ�����
    CAN_InitStructure.CAN_Prescaler =12;               // ��Ƶϵ��, ֱ����дҪ�ķ�Ƶֵ���������Զ���1
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;           // ��Ӱ��λʱ��Ӱ��ͬ��Ч��
    CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
    CAN_Init(CAN1, &CAN_InitStructure);                // ��ʼ��CAN����ģʽ
    // CAN������=1Mbps=1tq*(1+BS1+BS2)=(BRR*Tpclk)*(1+BS1+BS2)
    // ���ò���������ֵ(F103, 72MHz, APB1=36MHz)��
    //  250K:  9, CAN_SJW_1tq, CAN_BS1_13tq, CAN_BS2_2tq;
    //  500K: 12, CAN_SJW_1tq, CAN_BS1_4tq,  CAN_BS2_1tq
    //  500K:  9, CAN_SJW_1tq, CAN_BS1_6tq,  CAN_BS2_1tq
    //  500K:  6, CAN_SJW_1tq, CAN_BS1_10tq, CAN_BS2_1tq
    // 1000K:  6, CAN_SJW_1tq, CAN_BS1_4tq,  CAN_BS2_1tq

    // ����ɸѡ��������ʾ��ֻ����һ�������������յ������ݴ�ŵ�FIFO0
    CAN_FilterInitTypeDef CAN_FilterInitTypeStruct;
    CAN_FilterInitTypeStruct.CAN_FilterNumber = 0;                         // ʹ���ĸ�������
    CAN_FilterInitTypeStruct.CAN_FilterMode   = CAN_FilterMode_IdMask ;    // ����ģʽ��IDMask=0=����λģʽ��IdList=1=�б�ģʽ
    CAN_FilterInitTypeStruct.CAN_FilterScale  = CAN_FilterScale_32bit ;    // λ��
    CAN_FilterInitTypeStruct.CAN_FilterIdHigh = ((RECIVE_ID << 3) & 0xFFFF0000) >> 16;
    CAN_FilterInitTypeStruct.CAN_FilterIdLow  = ((RECIVE_ID << 3 | CAN_Id_Extended | CAN_RTR_Data) & 0xFFFF); //ֻ������չģʽ������
    CAN_FilterInitTypeStruct.CAN_FilterMaskIdHigh = 0x0000;                //��������16λÿλ����ƥ��
    CAN_FilterInitTypeStruct.CAN_FilterMaskIdLow  = 0x0000;                //��������16λÿλ����ƥ��
    CAN_FilterInitTypeStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;  // FIFO���������յ������ݣ���ŵ�FIFO0���������������
    CAN_FilterInitTypeStruct.CAN_FilterActivation = ENABLE;                // ʹ�ܹ�����
    CAN_FilterInit(&CAN_FilterInitTypeStruct);

    // ʹ���ж�
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    // �ж����ȼ�����
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn ;  // ʹ������0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    printf("CAN ͨ������          �������\r\n");
}



void USB_LP_CAN1_RX0_IRQHandler(void)
{
    // �������жϱ�־��ʹ��CAN_Receive()���Զ���
    CAN_Receive(CAN1, CAN_FIFO0, &xCAN.RxData);                   // ��FIFO_0���յ����ݣ�����CanRxMsg�ṹ���У�
    xCAN.ReceivedNum = xCAN.RxData.DLC;                           // ���ⲿ�ж� xCAN.ReceivedNum > 0����Ϊ���յ�������
    memset(xCAN.ReceivedBuf ,0, 9);                               // CANһ֡�����Ч����8�ֽ�; ����ʱ������9���ֽڣ���Ϊ�������λ��Ϊ���һ�ֽڸ�'\0', ����������ַ���
    memcpy(xCAN.ReceivedBuf, xCAN.RxData.Data, xCAN.ReceivedNum); // �����ݴ�ŵ� xCAN.ReceivedBuf
    memset(xCAN.RxData.Data ,0, 8);                               
}


// ��������
// ���أ����ͱ��ĵ������
uint8_t CAN1_SendData(uint8_t *data, uint32_t targetID)    // ��������; Ϊ�򻯲������̶�Ϊÿ�η���8���ֽ�
{

    xCAN.TxData.RTR    = CAN_RTR_Data;       // ����֡
    xCAN.TxData.IDE    = CAN_Id_Extended;    // ��չ֡
    xCAN.TxData.StdId  = 0;                  // ��׼֡ID
    xCAN.TxData.ExtId  = targetID;           // ��չ֡ID
    xCAN.TxData.DLC    = 8 ;                 // ���ݳ���(�ֽ��������8���ֽ�)��Ϊ�򻯺����������趨Ϊ8���ֽ�

    xCAN.TxData.Data[0] = data[0];           // ��Ҫ���͵�����
    xCAN.TxData.Data[1] = data[1];
    xCAN.TxData.Data[2] = data[2];
    xCAN.TxData.Data[3] = data[3];
    xCAN.TxData.Data[4] = data[4];
    xCAN.TxData.Data[5] = data[5];
    xCAN.TxData.Data[6] = data[6];
    xCAN.TxData.Data[7] = data[7];

    return  CAN_Transmit(CAN1, &xCAN.TxData);   // ����, ���ط������õ������

}



//// CAN���Ľ���
//void CAN1_ReceiveData(uint8_t *data)
//{
//    printf("\r\n2222222 ");

//    if (xCAN.RxFlag == 1)                      // ͨ�����ձ�־xCan.RxFlag���ж��Ƿ��յ��±���
//    {
//        printf("\r\n<<CAN���յ�����:>>  ");
//        for (uint8_t i = 0; i < 8; i++)
//            printf("%c", xCAN.RxData.Data[i]);
//        printf("\r\n");

//        xCAN.RxFlag = 0;                       // �����������ˣ��ѽ��ձ�־�声
//    }
//}






