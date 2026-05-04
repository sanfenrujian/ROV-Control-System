#include "my_function.h"
#include "IIC.h"
#include "thread_manager.h"
#include "gpio.h"
#include "stdio.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MS5837.h"
#include "usart.h"
#include "my_adc.h"
#include "motion_control.h"
#include "rtthread.h"

/* 全局变量 (从 my_function.h 和 main.c extern) */
extern rt_sem_t sem;
extern uint16_t IIC_SCL_PIN;
extern uint16_t IIC_SDA_PIN;

float pitch_1, roll_1, yaw_1;     /* 姿态角 (MPU6050) */
short aacx, aacy, aacz;           /* 加速度原始值 */
short gyrox, gyroy, gyroz;        /* 陀螺仪原始值 */
float temp;                       /* 温度 */

JY901B_DataType jy901b;           /* JY901B 姿态数据 */
JY901B_Temp     jy901b_t;         /* JY901B 温度数据 */
MPU6050DATATYPE mpu6050;          /* MPU6050 统一数据结构 */
ps2DATATYPE     PS_2;             /* 遥控器数据 */
soundDATATYPE   Ultrasound;       /* 超声波数据 */
phDATATYPE      ph;               /* pH数据 */
MS5837_DATATYPE ms5837;           /* 水压传感器 */
shendu_DATATYPE shendu;           /* 深度传感器原始数据 */

/* 辅助函数 */
static void my_limit(float *p)
{
    if (*p > 2400.0f)
        *p = 2400.0f;
    else if (*p < 600.0f)
        *p = 600.0f;
}

/* LED 指示灯线程 - 用于心跳/状态指示 */
void led1_thread_entry(void *parameter)
{
    while (1)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);  /* LED ON */
        rt_thread_delay(500);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);    /* LED OFF */
        rt_thread_delay(500);
    }
}




//**************************************�߳�2********************************/
MPU6050DATATYPE mpu6050;


void mpu6050_thread_entry(void *parameter)
{

    rt_mq_t queue = rt_mq_create("my_queue", sizeof(float), 10, RT_IPC_FLAG_FIFO);
    if (queue == RT_NULL)
    {
        printf("Failed to create message queue!\n");
        rt_thread_delay(500);   /* ��ʱ500��tick */

    }
    IIC_SCL_PIN =     SCL_1_Pin;
    IIC_SDA_PIN =     SDA_1_Pin;
		while(MPU_Init()+mpu_dmp_init()!=0)
//    uint8_t a = MPU_Init();         //MPU6050��ʼ��
//    int b = mpu_dmp_init();     //dmp��ʼ��
//printf("b=%d",b);
    rt_thread_delay(500);   /* ��ʱ500��tick */

    while (1)
    {


//        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
        while (mpu_dmp_get_data(&roll_1, &pitch_1, &yaw_1)); //����Ҫ��while�ȴ������ܶ�ȡ�ɹ�
        mpu6050.st_data.pitch =  pitch_1;
        mpu6050.st_data.roll = roll_1;
        mpu6050.st_data.yaw = yaw_1;
//				mpu6050.st_data.pitch =  ((jy901b.byte[1]<<8)|jy901b.byte[0])/32768*180;
//        mpu6050.st_data.roll = ((jy901b.byte[3]<<8)|jy901b.byte[2])/32768*180;
//        mpu6050.st_data.yaw = ((jy901b.byte[5]<<8)|jy901b.byte[4])/32768*180;
        mpu6050.st_data.Temp = temp / 100;
        MPU_Get_Accelerometer(&aacx, &aacy, &aacz);     //�õ����ٶȴ���������
        MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);      //�õ�����������
        temp = MPU_Get_Temperature();                   //�õ��¶���Ϣ
//        printf("mpu1��X:%.1f��  Y:%.1f��  Z:%.1f��  %.2f��C\r\n",roll_1,pitch_1,yaw_1,temp/100);//����1����ɼ���Ϣ
//   rt_thread_delay(50);   /* ��ʱ500��tick */
//        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
//   rt_thread_delay(50);   /* ��ʱ500��tick */
        rt_thread_delay(50);   /* ��ʱ500��tick */




    }
}

\

/* MS5837 水压/深度传感器线程 */
void MS5837_thread_entry(void *parameter)
{
    /* 初始化阶段 */
    while (1)
    {
        IIC_SCL_PIN = MS5837_SCL_Pin;  /* 切换I2C引脚 (注意：建议使用互斥锁保护) */
        IIC_SDA_PIN = MS5837_SDA_Pin;

        MS5837_30BA_ReSet();
        rt_thread_mdelay(20);
        MS5837_30BA_PROM();
        rt_thread_mdelay(20);

        if (MS5837_30BA_Crc4())
        {
            printf("MS5837 initialized successfully\r\n");
            break;
        }
        else
        {
            printf("MS5837 CRC check failed, retrying...\r\n");
            rt_thread_mdelay(500);
        }
    }

    /* 主循环 - 数据采集 */
    while (1)
    {
        MS5837_30BA_GetData();
        rt_thread_mdelay(200);  /* 5Hz 更新率 */

        /* 建议将 printf 移到低优先级调试线程，或使用 rt_kprintf */
        /* printf("Depth: %.2f mBar, Temp: %.2f C\r\n", Pressure, Temperature); */
    }
}


/* 数据接收线程 - 只负责启动UART中断接收，实际解析在 Callback 中进行 */
uint8_t rx_buf[100];     /* 保留用于旧协议兼容 */
uint8_t data = 0;
uint8_t uart_temp[1];
uint8_t uart_temp1[1];

void data_receive_thread_entry(void *parameter)
{
    /* 启动所有相关UART的中断接收（只调用一次） */
    HAL_UART_Receive_IT(&huart1, &data, 1);
    HAL_UART_Receive_IT(&huart4, &data, 1);   /* 注意：data 被多个UART共用，生产环境建议每个UART用独立buffer */
    HAL_UART_Receive_IT(&huart7, &data, 1);

    while (1)
    {
        rt_thread_mdelay(1000);  /* 此线程主要用于初始化，之后可休眠或用于其他低优先级任务 */
    }
}


//**************************************�߳�5********************************/
void SendData_thread_entry(void *parameter)
{
    while (1)
    {

        Send_Data_Task();
        rt_thread_mdelay(5);

    }
}

//**************************************�߳�6��������********************************/
uint8_t enable = 1;

uint8_t high_data, low_data;
uint8_t sound_rx_buf[8];
uint8_t sound_sum = 0;
int distance;
soundDATATYPE Ultrasound;

unsigned char uart_buff[4];
unsigned char uart_temp[1];
unsigned int uart_rx_cnt = 0;
unsigned char uart_temp1[1];


void Ultrasound_thread_entry(void *parameter)
{
    while (1)
    {
  //   HAL_UART_Receive_IT(&huart5,(uint8_t *)uart_temp, 1);
//HAL_UART_Receive_IT(&huart4,(uint8_t *)uart_temp, 1);
		
        //HAL_UART_Receive_IT(&huart4, &sound_buf, 1);
		//	HAL_UART_Receive_IT(&huart4, &data, 1);

//        HAL_UART_Transmit(&huart4, &enable, sizeof(enable), 0xff);
    //HAL_UART_Transmit(&huart4, (uint8_t *)&enable, 1, 0xFFFF);


//        for (int i = 0; i < 8; i++)
//        {
//            if (sound_rx_buf[i] == 0xff)
//            {
////printf("%d %d %d %d\t\t",sound_rx_buf[i],sound_rx_buf[i+1],sound_rx_buf[i+2],sound_rx_buf[i+3]);

//                high_data = sound_rx_buf[i + 1];
//                low_data = sound_rx_buf[i + 2];

//                sound_sum = 0xff + high_data + low_data;
//                if (sound_sum == sound_rx_buf[i + 3])
//                {
//                    distance = high_data * 256 + low_data;
//                    Ultrasound.st_data.distance = (float)distance;

////                    printf("distance=%d\n",distance);
//                    break;
//                }
//                else
//                {
//                    printf("error");

//                }

//            }


//        }


        rt_thread_mdelay(50);

    }
}
//**************************************�߳�7:PH��********************************/

extern ADC_HandleTypeDef hadc1;
extern float volot[4];
phDATATYPE ph;

extern float v;
extern int adc_num;

uint8_t i;
float adcBuf[2];//���ADC

void PH_thread_entry(void *parameter)
{
    while (1)
    {


        i = 0;
        while (i < 2)
        {
            HAL_ADC_Start(&hadc1);//����ADC
            HAL_ADC_PollForConversion(&hadc1, 10); //��ʾ�ȴ�ת����ɣ��ڶ���������ʾ��ʱʱ�䣬��λms.
            //HAL_ADC_GetState(&hadc1)Ϊ��ȡADC״̬��HAL_ADC_STATE_REG_EOC��ʾת����ɱ�־λ��ת�����ݿ��á�
            if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC)) //�����ж�ת����ɱ�־λ�Ƿ�����,HAL_ADC_STATE_REG_EOC��ʾת����ɱ�־λ��ת�����ݿ���
            {
                //��ȡADCת�����ݣ�����Ϊ12λ���鿴�����ֲ��֪���Ĵ���Ϊ16λ�洢ת�����ݣ������Ҷ��룬��ת�������ݷ�ΧΪ0~2^12-1,��0~4095.
                adcBuf[i] = HAL_ADC_GetValue(&hadc1) * 3.3 / 4096;
//   printf("\nadc%d=%4.0d,��ѹ=%1.4f",i,adcBuf[i],adcBuf[i]*3.3f/65536);
                i++;
            }
        }

        HAL_ADC_Stop(&hadc1);

//        ph.st_data.ph = adcBuf[1]*-5.7541+16.654;
				ph.st_data.ph = 6.5-adcBuf[1];
        ph.st_data.quality = adcBuf[0];
				ph.st_data.quality = ph.st_data.quality*ph.st_data.quality*ph.st_data.quality*66.71-127.93*ph.st_data.quality*ph.st_data.quality+428.7*ph.st_data.quality;

//				ph.st_data.quality=jj++;
				
				//printf("v1=%f,v2=%f\n",adcBuf[0],adcBuf[1]);


        rt_thread_mdelay(50);


    }
}



//**************************************�߳�8��ˮ�ʴ�����********************************/

void water_quality_thread_entry(void *parameter)
{
    while (1)
    {


        rt_thread_mdelay(50);

    }
}


//**************************************�߳�9���˶�����********************************/
uint8_t lock=1;
float jiaodu[10000];
int ii=0;

			float angle_1,angle_2,angle_3,angle_4;
void motion_control_thread_entry(void *parameter)
{
				
	
	
    while (1)
    {
			


# if old
			control_motion(2-PS_2.st_data.ch4,PS_2.st_data.ch3,2-PS_2.st_data.ch2);//����ǰһֱ�õ�
# else
		control_motion(PS_2.st_data.ch1,PS_2.st_data.ch2,PS_2.st_data.ch3,PS_2.st_data.ch9,PS_2.st_data.ch10);//���Ժ�Ľ����㷨,���ĸ��������ٶȿ���

			
			if((lock==1)&&(PS_2.st_data.ch8>0.8))
			{
			
				lock=0;
				
							
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);			
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);		
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);							
					
			}			


				        rt_thread_mdelay(10);

# endif
    }
}
/* ====================== UART 中断回调（已迁移至 main_v2.c） ====================== */
#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t received_byte = data;

    if (huart->Instance == USART1)
    {
        parse_usart1(received_byte);
        HAL_UART_Receive_IT(&huart1, &data, 1);
    }
    else if (huart->Instance == UART4)
    {
        parse_uart4(received_byte);
        HAL_UART_Receive_IT(&huart4, &data, 1);
    }
    else if (huart->Instance == UART7)
    {
        parse_uart7(received_byte);
        HAL_UART_Receive_IT(&huart7, &data, 1);
    }
}
#endif  /* #if 0 - HAL_UART_RxCpltCallback migrated to main_v2.c */


   
//    if (huart->Instance == UART5) // 
//    {


//	static u8 _data_len = 0, _cnt = 0;
//	static u8 rx_state = 0;u8 check=1;
//	static u8 DT_RxBuffer[256];


//	if (rx_state == 0 && data == 0xAB)
//	{
//		rx_state = 1;
//		head = data;

//	}
//	//�ж�id
//	else if (rx_state == 1 && (data==0x01))
//	{
//		rx_state = 2;
//		 id = data;

//	}
//	else if (rx_state == 2 )
//	{
//		
//		 ms5837.byte[_cnt++]= data;
//if(_cnt>=8)
//{
//_cnt=0;
//rx_state=0;
//}
//	}
//	else
//	{
//	rx_state=0;
//	}

//HAL_UART_Receive_IT(&huart5, &data, 1);

//    }
		
		
		
		
		
////////////////////////////////////////////////////
//    if (huart->Instance == UART4) // ???????????? USART1
//    {

//        sound_rx_buf[sound_i++] = sound_buf;
//        if (sound_i > 8)
//        {
//            sound_i = 0;
//        }
//        HAL_UART_Receive_IT(&huart4, &sound_buf, 1);
//    }
		
/* ====================== 干净的UART协议解析器 (新重构版本) ====================== */
/* 这些函数将逐步替换旧的内联状态机。当前回调仍保留兼容，未来可完全切换。 */

static void parse_usart1(uint8_t byte);
static void parse_uart4(uint8_t byte);
static void parse_uart7(uint8_t byte);

/* USART1 - 遥控器数据 (AA BB + 40字节数据 + checksum) */
static void parse_usart1(uint8_t byte)
{
    static uint8_t rx_state = 0;
    static uint8_t data_cnt = 0;
    static uint8_t rx_buffer[48];
    static uint16_t checksum = 0;

    switch (rx_state)
    {
        case 0:
            if (byte == 0xAA)
            {
                rx_state = 1;
                checksum = byte;
                data_cnt = 0;
            }
            break;
        case 1:
            if (byte == 0xBB)
            {
                rx_state = 2;
                checksum += byte;
            }
            else rx_state = 0;
            break;
        case 2:
            rx_buffer[data_cnt++] = byte;
            checksum += byte;
            if (data_cnt >= 40)
                rx_state = 3;
            break;
        case 3:  /* 校验低字节 */
            rx_buffer[40] = byte;
            rx_state = 4;
            break;
        case 4:  /* 校验高字节 */
        {
            uint16_t received = ((uint16_t)byte << 8) | rx_buffer[40];
            if (received == (checksum & 0xFFFF))
            {
                for (int i = 0; i < 40; i++)
                    PS_2.byte[i] = rx_buffer[i];
                if (sem != RT_NULL)
                    rt_sem_release(sem);   /* 通知 motion_control_thread 有新数据 */
            }
            rx_state = 0;
            checksum = 0;
            data_cnt = 0;
            break;
        }
        default:
            rx_state = 0;
            break;
    }
}

/* UART4 - JY901B 姿态传感器 */
static void parse_uart4(uint8_t byte)
{
    static uint8_t rx_state = 0;
    static uint8_t cnt = 0;
    static uint8_t type = 0;

    if (rx_state == 0 && byte == 0x55)
        rx_state = 1;
    else if (rx_state == 1)
    {
        if (byte == 0x53 || byte == 0x51)
        {
            type = byte;
            rx_state = 2;
            cnt = 0;
        }
        else rx_state = 0;
    }
    else if (rx_state == 2)
    {
        if (type == 0x53 && cnt < 6)
            jy901b.byte[cnt] = byte;
        else if (type == 0x51 && cnt < 2)
            jy901b_t.byte[cnt] = byte;
        cnt++;
        if ((type == 0x53 && cnt >= 6) || (type == 0x51 && cnt >= 9))
            rx_state = 0;
    }
    else rx_state = 0;
}

/* UART7 - 深度传感器 */
static void parse_uart7(uint8_t byte)
{
    static uint8_t rx_state = 0;
    static uint8_t cnt = 0;

    if (rx_state < 6)
    {
        const char prefix[] = "Depth:";
        if (byte == (uint8_t)prefix[rx_state])
            rx_state++;
        else
            rx_state = (byte == 'D') ? 1 : 0;
    }
    else if (rx_state >= 6 && cnt < 4)
    {
        shendu.byte[cnt++] = byte;
        if (cnt >= 4)
        {
            cnt = 0;
            rx_state = 0;
            /* 可在这里转换深度值到 true_depth */
        }
    }
    else
    {
        rx_state = 0;
        cnt = 0;
    }
}
	

