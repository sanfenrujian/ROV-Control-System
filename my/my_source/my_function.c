# include "stdio.h"
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "my_function.h"
#include "mpu6050.h"
#include "math.h"
#include <stdlib.h>



extern MPU6050DATATYPE mpu6050;
extern soundDATATYPE Ultrasound;
extern phDATATYPE ph;
extern MS5837_DATATYPE ms5837;
extern JY901B_DataType jy901b;
extern ps2DATATYPE PS_2;
extern shendu_DATATYPE shendu;
extern JY901B_Temp  jy901b_t;
extern servo_angle_Datatype send_angle;
extern trueshendu_DATATYPE true_depth;

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}




float my_abs(float a)
{
    return a > 0 ? a : -a;
}















//�ٶ�̬����,��������������ͬ����֡�Ĵ�С
//�Ժ���Ҫ�����µ�ͨ��֡��ֻ��Ҫ
//1.���������������������
//2.��add�������������������ݼ���
//3. Send_Data_Task()����������Ӧ��send��������
u8 DT_TX_Buffer_SIZE[] =
{
    0,    //0x00
    16 + 5 //0x01 
    , 4+5 //0x02
    , 8+5  //0x03
	,8+5//0x04
	,6+5//0x05
	,5+40//0x06
	,5+8//0x07
	,5+2//0x08
	,5+32//0x09
};

u8 *p[sizeof(DT_TX_Buffer_SIZE)];
void DT_TX_P_Init()
{
    for (int i=0; i < sizeof(DT_TX_Buffer_SIZE); i++)
    {
        p[i] = (u8 *) malloc((int)DT_TX_Buffer_SIZE[i]); //����һ��������Ϊ����֡
    }
}



//ͨ�ų���
static u8 DT_RxBuffer[256], DT_data_cnt = 0;
void ANO_DT_LX_Data_Receive_Prepare(u8 data)
{
    static u8 _data_len = 0, _data_cnt = 0;
    static u8 rxstate = 0;

    //�ж�֡ͷ�Ƿ���������Э���0xAA
    if (rxstate == 0 && data == 0xAA)
    {
        rxstate = 1;
        DT_RxBuffer[0] = data;
    }


    //����֡CMD�ֽ�
    else if (rxstate == 1)
    {
        rxstate = 2;
        DT_RxBuffer[2] = data;
    }
    //�������ݳ����ֽ�
    else if (rxstate == 2 && data < 250)
    {
        rxstate = 3;
        DT_RxBuffer[3] = data;
        _data_len = data;
        _data_cnt = 0;
    }
    //����������
    else if (rxstate == 3 && _data_len > 0)
    {
        _data_len--;
        DT_RxBuffer[3 + _data_cnt++] = data;
        if (_data_len == 0)
            rxstate = 4;
    }

    //����У���ֽڣ���ʾһ֡���ݽ�����ϣ��������ݽ�������
    else if (rxstate == 4)
    {
        rxstate = 5;
        DT_RxBuffer[3 + _data_cnt] = data;
        DT_data_cnt = _data_cnt + 5;
        //ano_dt_data_ok = 1;
        ANO_DT_LX_Data_Receive_Anl(DT_RxBuffer, DT_data_cnt);
    }
    else
    {
        rxstate = 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl������Э�����ݽ������������������Ƿ���Э���ʽ��һ������֡���ú��������ȶ�Э�����ݽ���У��
//У��ͨ��������ݽ��н�����ʵ����Ӧ����
//�˺������Բ����û����е��ã��ɺ���ANO_Data_Receive_Prepare�Զ�����
static void ANO_DT_LX_Data_Receive_Anl(u8 *data, u8 len)
{
    u8 check_sum1 = 0;
    //�ж����ݳ����Ƿ���ȷ
    if (*(data + 3) != (len - 6))
        return;
    //�����յ������ݼ���У���ֽ�1��2
    for (u8 i = 0; i < len - 2; i++)
    {
        check_sum1 += *(data + i);

    }
    //�������У���ֽں��յ���У���ֽ����Աȣ���ȫһ�´�����֡���ݺϷ�����һ����������������
    if ((check_sum1 != *(data + len - 2)))  //�ж�sumУ��
        return;


    //=============================================================================
    //����֡��CMD��Ҳ���ǵ�3�ֽڣ����ж�Ӧ���ݵĽ���
    //PWM����

    switch (*(data + 1))
    {
    case 0x20:
        ;
    }


    /*
    if (*(data + 2) == 0X20)
    {
        pwm_to_esc.pwm_m1 = *((u16 *)(data + 4));
        pwm_to_esc.pwm_m2 = *((u16 *)(data + 6));
        pwm_to_esc.pwm_m3 = *((u16 *)(data + 8));
        pwm_to_esc.pwm_m4 = *((u16 *)(data + 10));
        pwm_to_esc.pwm_m5 = *((u16 *)(data + 12));
        pwm_to_esc.pwm_m6 = *((u16 *)(data + 14));
        pwm_to_esc.pwm_m7 = *((u16 *)(data + 16));
        pwm_to_esc.pwm_m8 = *((u16 *)(data + 18));
    }
    //����IMU������RGB�ƹ�����
    else if (*(data + 2) == 0X0f)
    {
        led.brightness[0] = *(data + 4);
        led.brightness[1] = *(data + 5);
        led.brightness[2] = *(data + 6);
        led.brightness[3] = *(data + 7);
    }
    //�����ɿص�ǰ������״̬
    else if (*(data + 2) == 0X06)
    {
        fc_sta.fc_mode_sta = *(data + 4);
        fc_sta.unlock_sta = *(data + 5);
        fc_sta.cmd_fun.CID = *(data + 6);
        fc_sta.cmd_fun.CMD_0 = *(data + 7);
        fc_sta.cmd_fun.CMD_1 = *(data + 8);
    }
    //�����ٶ�
    else if (*(data + 2) == 0X07)
    {
        for(u8 i=0;i<6;i++)
        {
            fc_vel.byte_data[i] = *(data + 4 + i);
        }
    }
    //��̬�ǣ���Ҫ����λ������IMU��������������ܣ�
    else if (*(data + 2) == 0X03)
    {
        for(u8 i=0;i<7;i++)
        {
            fc_att.byte_data[i] = *(data + 4 + i);
        }
    }
    //��̬��Ԫ��
    else if (*(data + 2) == 0X03)
    {
        for(u8 i=0;i<9;i++)
        {
            fc_att_qua.byte_data[i] = *(data + 4 + i);
        }
    }
    //����������
    else if (*(data + 2) == 0X01)
    {

        acc_x = *((s16 *)(data + 4));
        acc_y = *((s16 *)(data + 6));
        acc_z = *((s16 *)(data + 8));
        gyr_x = *((s16 *)(data + 10));
        gyr_y = *((s16 *)(data + 12));
        gyr_z = *((s16 *)(data + 14));
        state = *(data + 16);

    }
    //����E0�����������ʽ�����ܣ��μ�����ͨ��Э��V7��
    else if (*(data + 2) == 0XE0)
    {
        //��������ID��(*(data + 4)) ����ִ�в�ͬ������
        switch (*(data + 4))
        {
        case 0x01:
        {
        }
        break;
        case 0x02:
        {
        }
        break;
        case 0x10:
        {
        }
        break;
        case 0x11:
        {
        }
        break;
        default:
            break;
        }
        //�յ��������Ҫ���ض�Ӧ��Ӧ����Ϣ��Ҳ����CK_Back����
        dt.ck_send.ID = *(data + 4);
        dt.ck_send.SC = check_sum1;
        dt.ck_send.AC = check_sum2;
        CK_Back(SWJ_ADDR, &dt.ck_send);
    }
    //�յ�����ck����
    else if (*(data + 2) == 0X00)
    {
        //�ж��յ���CK��Ϣ�ͷ��͵�CK��Ϣ�Ƿ����
        if ((dt.ck_back.ID == *(data + 4)) && (dt.ck_back.SC == *(data + 5)) && (dt.ck_back.AC == *(data + 6)))
        {
            //У��ɹ�
            dt.wait_ck = 0;
        }
    }
    //��ȡ����
    else if (*(data + 2) == 0XE1)
    {
        //��ȡ��Ҫ��ȡ�Ĳ�����id
        u16 _par = *(data + 4) + *(data + 5) * 256;
        dt.par_data.par_id = _par;
        dt.par_data.par_val = 0;
        //���͸ò���
        PAR_Back(0xff, &dt.par_data);
    }
    //д�����
    else if (*(data + 2) == 0xE2)
    {
        //Ŀǰ������ԴMCU���漰������д�룬�Ƽ����ֱ��ʹ��Դ�뷽ʽ�����Լ�����Ĳ������ʴ˴�ֻ���ض�Ӧ��CKУ����Ϣ
        //      u16 _par = *(data+4)+*(data+5)*256;
        //      u32 _val = (s32)(((*(data+6))) + ((*(data+7))<<8) + ((*(data+8))<<16) + ((*(data+9))<<24));
        //
        dt.ck_send.ID = *(data + 4);
        dt.ck_send.SC = check_sum1;
        dt.ck_send.AC = check_sum2;
        CK_Back(0xff, &dt.ck_send);
        //��ֵ����
        //Parameter_Set(_par,_val);
    }
    */
}

//===================================================================
//ͨ��֡��亯��
//===================================================================
static void Add_Send_Data(u8 ID, u8 Tx_buffer[])
{
    s16 temp_data;
    s32 temp_data_32;

    u8 len, num = 0; //num����ָʾλ��
    //������Ҫ���͵�֡ID��Ҳ����frame_num����������ݣ���䵽send_buffer������
		//����ֻ��Ҫ������Ⱥ���̬�����Ķ�ɾ��
    uint16_t  sum = 0;
    Tx_buffer[0] = 0xAA;
		Tx_buffer[1]=0xBB;
		Tx_buffer[2]=ID;

    sum += Tx_buffer[num++];
    sum += Tx_buffer[num++];
		sum+=Tx_buffer[num++];
    switch (ID)
    {
    case 0x01:
        for (len = 0; len < 16; len++)
        {
            Tx_buffer[num++] = len;
            sum+=len;
//printf("sum=%d",sum);
        }
        break;
		case 0x04:
        for (len = 0; len < 8; len++)
        {
            Tx_buffer[num++] = ms5837.byte[len];
						sum+=ms5837.byte[len];
        }
        break;
		case 0x05:
        for (len = 0; len < 6; len++)
        {
            Tx_buffer[num++] = jy901b.byte[len];
            sum+=jy901b.byte[len];
        }
        break;
		case 0x06:
			for (len = 0; len < 40; len++)
        {
            Tx_buffer[num++] = PS_2.byte[len];
            sum+=PS_2.byte[len];
        }
				break;
		case 0x07:
				 {
					 for(len=0;len<8;len++)
					{
						Tx_buffer[num++]= true_depth.byte[len];
						sum+=true_depth.byte[len];
					}
				}
				 break;
		case 0x08:
		{
			for(len=0;len<2;len++)
			{
				Tx_buffer[num++]=jy901b_t.byte[len];
				sum+=jy901b_t.byte[len];
			}
		}
		break;
		
		case 0x09:
		{
			for(len=0;len<32;len++)
			{
				Tx_buffer[num++]=send_angle.byte[len];
				sum+=send_angle.byte[len];
			}
		}
		break;
	}
		unsigned char lowByte = sum & 0xff; // ��ȡ��8���ֽ�
    unsigned char highByte = (sum >> 8) & 0xff; // ��ȡ��8���ֽ�
		//�Ͱ�λ��ǰ
		Tx_buffer[num++]=lowByte;
		Tx_buffer[num]=highByte;
			
		
}

//===================================================================
//����ͨ��֡���ͺ���
//===================================================================
void Send_Data(u8 ID)
{
//int i;
//for(i=0;i<256;i++)


    Add_Send_Data(ID, p[ID]);
		int a= HAL_UART_Transmit(&huart1, p[ID], DT_TX_Buffer_SIZE[ID], 100);
		printf("1");


}
//ͨ��֡��������
void Send_Data_Task()
{

	//Send_Data(0x01);//0x01��mpu6050�������ĸ�float �ֱ��� pitch yaw roll �� temp
	Send_Data(0x05);
	//Send_Data(0x06);//test 
	Send_Data(0x07);//0x07����ȼƵ��������ݣ��ֱ�����Ⱥ��¶�
	Send_Data(0x08);
	Send_Data(0x09);//�ĸ����ת���Ƕ� �� �� ��������� ���������

}





