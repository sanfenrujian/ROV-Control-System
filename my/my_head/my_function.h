#ifndef _MY_FUN
#define _MY_FUN

#include "main.h"

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;


typedef  struct 
{
/*以下为基础的pid*/
float kp;
float ki;
float kd;
float err;
float measure;
float target;
float integral;
float out;

/*以下为进阶型pid*/
float err_last;
float err_next;
float err_max;
float integral_max;
uint8_t integral_flag;
float out_max;
} MYPID;

typedef struct{
double  leftangle;
double 	rightangle;
double  back_leftangle;
double  back_upangle;
}__attribute__((__packed__)) servo_angle;

typedef union{
uint8_t byte[8*4];
servo_angle st_data;
}servo_angle_Datatype;

typedef struct{
    float pitch;
    float roll;
    float yaw;
    float Temp;
} __attribute__((__packed__)) mpu6050_DataType;

typedef union
{
uint8_t byte[4*4];
mpu6050_DataType st_data;
}MPU6050DATATYPE;

typedef struct{
    
    uint16_t pitch;
    uint16_t roll;
    uint16_t yaw;
} __attribute__((__packed__)) jy901b_DataType;

typedef union
{
	uint8_t byte[2*3];
 jy901b_DataType st_data;
}JY901B_DataType;

typedef struct
{
  uint16_t temp;
}__attribute__((__packed__)) jy901b_Temp;

typedef union
{
	uint8_t byte[2];
  jy901b_Temp st_data;
}JY901B_Temp;

typedef struct
{
// 
//    float ch1;
//    float ch2;
//    float ch3;
//    float ch4;
//    float ch5;
//    float ch6;
//    float ch7;
//    float ch8;
//	
//	  float pitch_1;
//	  float roll_1;
//	  float yaw_1;
//	
//	  float pitch_2;
//	  float roll_2;
//	  float yaw_2;
//		
//	  float pitch_3;
//	  float roll_3;
//		float yaw_3;
//	  float ch5;
    float ch1; //前后速度期望 范围：0到1 0为静止 1为向前
    float ch2; //左右角速度期望 范围：-1到1 -1为向左 1为向右
    float ch3; //深度的微分期望 范围：-1到1 -1为向下 1为向上
    float ch4;
    float ch5;
    float ch6;
    float ch7;
    float ch8;
		uint8_t ch9; //A
		uint8_t ch10; //B
		uint8_t ch11; //X
		uint8_t ch12; //Y
		uint8_t ch13;
		uint8_t ch14;
		uint8_t ch15;
		uint8_t ch16;
} __attribute__((__packed__)) ps2_DataType;

typedef union
{
uint8_t byte[sizeof(ps2_DataType)];
ps2_DataType st_data;
}ps2DATATYPE;



typedef struct{

    float ph;
    float quality;

} __attribute__((__packed__)) ph_DataType;

typedef union
{
uint8_t byte[4*sizeof(ph_DataType)];
ph_DataType st_data;
}phDATATYPE;

typedef struct{

    float distance;


} __attribute__((__packed__)) sound_DataType;

typedef union
{
uint8_t byte[4*sizeof(sound_DataType)];
sound_DataType st_data;
}soundDATATYPE;


typedef struct{
    float pressure;
    float water_temp;
    
} __attribute__((__packed__)) ms5837_DataType;

typedef union
{
uint8_t byte[sizeof(ms5837_DataType)];
ms5837_DataType st_data;
}MS5837_DATATYPE;

typedef struct{
	uint8_t depth[4];
	uint8_t temp[4];
}__attribute__((__packed__)) shendu_DataType;

typedef union
{
uint8_t byte[sizeof(shendu_DataType)];
shendu_DataType st_data;
}shendu_DATATYPE;

typedef struct{
	double true_depth;
}__attribute__((__packed__)) trueshendu_DataType;

typedef union
{
uint8_t byte[sizeof(trueshendu_DataType)];
trueshendu_DataType st_data;
}trueshendu_DATATYPE;


float encoder_speed_calculate(uint32_t encoder_count, float d);
float my_abs(float a);

void PID_1_Init(MYPID mypid);
void PID_2_Init(MYPID mypid);
float PID_Calculate(MYPID MyPID, float measure, float target);

static void ANO_DT_LX_Data_Receive_Anl(u8 *data, u8 len);
void ANO_DT_LX_Data_Receive_Prepare(u8 data);
void Send_Data(u8 ID);
static void Add_Send_Data(u8 ID, u8 Tx_buffer[256]);
void Send_Data_Task();
void DT_TX_P_Init();
#endif