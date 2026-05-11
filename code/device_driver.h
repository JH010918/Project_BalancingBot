#include "stm32f4xx.h"
#include "option.h"
#include "macro.h"
#include "malloc.h"
#include <math.h>

#define MPU6050

extern volatile uint8_t bt_rx_data;  //uart 블루투스 통신
extern volatile uint8_t bt_rx_flag;  //uart 블루투스 통신

// Uart.c
extern void Uart2_Init(int baud);
extern void Uart2_Send_Byte(char data);
extern void Uart2_RX_Interrupt_Enable(int en);

extern void Uart1_Init(int baud);
extern void Uart1_Send_Byte(char data);
extern void Uart1_Send_String(char *pt);
extern void Uart1_Printf(char *fmt,...);
extern char Uart1_Get_Char(void);
extern char Uart1_Get_Pressed(void);

// SysTick.c
extern void SysTick_Run(unsigned int msec);
extern int SysTick_Check_Timeout(void);
extern unsigned int SysTick_Get_Time(void);
extern unsigned int SysTick_Get_Load_Time(void);
extern void SysTick_Stop(void);

// Clock.c
extern void Clock_Init(void);

// Timer.c
extern void TIM2_Delay(int time);
extern void TIM2_Stopwatch_Start(void);
extern unsigned int TIM2_Stopwatch_Stop(void);
extern void TIM4_Repeat(int time);
extern int TIM4_Check_Timeout(void);
extern void TIM4_Stop(void);
extern void TIM4_Change_Value(int time);
extern void TIM4_Repeat_Interrupt_Enable(int en, int time);
void TIM1_PWM_Init(void);
void TIM4_PWM_Init(void);

// i2c.c
#define SC16IS752_IODIR				0x0A
#define SC16IS752_IOSTATE			0x0B

extern void I2C1_SC16IS752_Init(unsigned int freq);
extern void I2C1_SC16IS752_Write_Reg(unsigned int addr, unsigned int data);
extern void I2C1_SC16IS752_Config_GPIO(unsigned int config);
extern void I2C1_SC16IS752_Write_GPIO(unsigned int data);

/* UART Functions */
char Uart2_Get_Pressed(void); // 추가된 Non-blocking 수신 함수

// I2C
void I2C1_Init(void); // I2C 설정함수 
void I2C1_Start(void);  // I2C 시작 신호
void I2C1_WriteAddress(uint8_t address);  // // I2C 주소 전송
void I2C1_WriteData(uint8_t data);  // I2C 데이터 전송
uint8_t MPU6050_ReadReg(uint8_t reg);  // 특정 레지스터에서 1바이트 읽어오기
void I2C1_Stop(void);  // I2C 종료 신호
void MPU6050_WriteReg(uint8_t reg, uint8_t data); // 특정 레지스터에 1바이트 쓰기
void MPU6050_Init(void);  // 센서 초기화 (수면 해제 및 필터 설정)

typedef struct {
    int16_t Accel_X;
    int16_t Accel_Y;
    int16_t Accel_Z;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
} MPU6050_Data;

// MPU6050 IMU 설정
void MPU6050_Get_RawData(MPU6050_Data *sensor);

float MPU6050_Get_Angle(MPU6050_Data *sensor, float dt);

void Bluetooth_Init(void);
void USART1_IRQHandler(void);

void I2C1_WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
void I2C1_ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int my_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data);
int my_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);

void my_get_ms(unsigned long *count);

void MPU6050_INT_Init(void);

void Encoder_Init(void);