#include "device_driver.h"

void I2C1_Init(void){
	unsigned int r;
    volatile int i;

    // 클록 활성화 (GPIOB, I2C1)
    Macro_Set_Bit(RCC->AHB1ENR, 1);
    Macro_Set_Bit(RCC->APB1ENR, 21);

	//하드웨어 리셋
    Macro_Clear_Bit(RCC->APB1RSTR, 21);
    Macro_Set_Bit(RCC->APB1RSTR, 21);
    for(i = 0; i < 1000; i++);
    Macro_Clear_Bit(RCC->APB1RSTR, 21);

	Macro_Write_Block(GPIOB->MODER, 0xf, 0xa, 16);      // PB[9:8] => ALT (10)
    Macro_Write_Block(GPIOB->AFR[1], 0xff, 0x44, 0);    // PB[9:8] => AF04 연결
    Macro_Write_Block(GPIOB->OTYPER, 0x3, 0x3, 8);      // PB[9:8] => Open Drain
    Macro_Write_Block(GPIOB->OSPEEDR, 0xf, 0xa, 16);    // PB[9:8] => Fast Speed
    Macro_Write_Block(GPIOB->PUPDR, 0xf, 0x5, 16);      // PB[9:8] => Internal Pull-up

    // I2C 하드웨어 설정 (PCLK1 기반 400kHz 고속 세팅)
    Macro_Write_Block(I2C1->CR2, 0x3f, PCLK1 / 1000000, 0);
    Macro_Clear_Bit(I2C1->CR1, 0);

    I2C1->TRISE = ((PCLK1 / 1000000) * 300 / 1000) + 1;
    I2C1->CCR = 0x8000 | (PCLK1 / 1200000);   

	Macro_Clear_Bit(I2C1->CR1, 1);
    Macro_Set_Bit(I2C1->CR1, 0); 
	Macro_Set_Bit(I2C1->CR1, 10);
}

void I2C1_Start(void) {
    I2C1->CR1 |= (1 << 8);
    while (!(I2C1->SR1 & (1 << 0)));
}

// I2C 주소 전송
void I2C1_WriteAddress(uint8_t address) {
    I2C1->DR = address;
    while (!(I2C1->SR1 & (1 << 1))); 
    volatile int temp = I2C1->SR1; 
    temp = I2C1->SR2;
}

// I2C 데이터 전송
void I2C1_WriteData(uint8_t data) {
    while (!(I2C1->SR1 & (1 << 7)));
    I2C1->DR = data;
    while (!(I2C1->SR1 & (1 << 2)));
}

// 특정 레지스터에서 1바이트 읽어오기
uint8_t MPU6050_ReadReg(uint8_t reg) {
    uint8_t data;

    I2C1_Start();
    I2C1_WriteAddress(0xD0); 
    I2C1_WriteData(reg); 

    I2C1_Start();       
    I2C1_WriteAddress(0xD1); 

    I2C1->CR1 &= ~(1 << 10); 
    I2C1->CR1 |= (1 << 9);  

    while (!(I2C1->SR1 & (1 << 6))); 
    data = I2C1->DR;        

	Macro_Set_Bit(I2C1->CR1, 10); 

    return data;
}

// I2C 종료 신호
void I2C1_Stop(void) {
    I2C1->CR1 |= (1 << 9); 
}

// 특정 레지스터에 1바이트 쓰기
void MPU6050_WriteReg(uint8_t reg, uint8_t data) {
    I2C1_Start();            
    I2C1_WriteAddress(0xD0); 
    I2C1_WriteData(reg);     
    I2C1_WriteData(data);    
    I2C1_Stop();           
}

// MPU6050 센서 초기화
void MPU6050_Init(void) {
    MPU6050_WriteReg(0x6B, 0x00); 

    // 진동이 많은 로봇을 위해 노이즈를 살짝 깎음.
    MPU6050_WriteReg(0x1A, 0x03);
}

// MPU6050 관련 설정
void MPU6050_Get_RawData(MPU6050_Data *sensor) {
    sensor->Accel_X = (MPU6050_ReadReg(0x3B) << 8) | MPU6050_ReadReg(0x3C);
    sensor->Accel_Y = (MPU6050_ReadReg(0x3D) << 8) | MPU6050_ReadReg(0x3E);
    sensor->Accel_Z = (MPU6050_ReadReg(0x3F) << 8) | MPU6050_ReadReg(0x40);

    sensor->Gyro_X = (MPU6050_ReadReg(0x43) << 8) | MPU6050_ReadReg(0x44);
    sensor->Gyro_Y = (MPU6050_ReadReg(0x45) << 8) | MPU6050_ReadReg(0x46);
    sensor->Gyro_Z = (MPU6050_ReadReg(0x47) << 8) | MPU6050_ReadReg(0x48);
}

// 다중 바이트 연속 쓰기 함수
void I2C1_WriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
    I2C1_Start();
    I2C1_WriteAddress(devAddr << 1); 
    I2C1_WriteData(regAddr);         
    for (uint8_t i = 0; i < length; i++) {
        I2C1_WriteData(data[i]);
    }
    I2C1_Stop();
}

// 다중 바이트 연속 읽기 함수
void I2C1_ReadBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
    I2C1_Start();
    I2C1_WriteAddress(devAddr << 1); 
    I2C1_WriteData(regAddr);         
    I2C1_Start(); 
    I2C1_WriteAddress((devAddr << 1) | 1); 
    for (uint8_t i = 0; i < length; i++) {
        if (i == length - 1) {
            I2C1->CR1 &= ~(1 << 10); // NACK
            I2C1->CR1 |= (1 << 9);   // STOP 예약
        } else {
            I2C1->CR1 |= (1 << 10);  // ACK
        }
        while (!(I2C1->SR1 & (1 << 6))); 
        data[i] = I2C1->DR; 
    }
    Macro_Set_Bit(I2C1->CR1, 10); 
}

// 라이브러리 연결용 래퍼 함수 
int my_i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
    I2C1_WriteBytes(slave_addr, reg_addr, length, (uint8_t*)data);
    return 0; 
}
int my_i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    I2C1_ReadBytes(slave_addr, reg_addr, length, data);
    return 0; 
}

// 라이브러리용 시간 중계 함수
void my_get_ms(unsigned long *count) {
    *count = SysTick_Get_Time(); 
}

// MPU6050 인터럽트 핀(PB12) 초기화 함수
void MPU6050_INT_Init(void) {
    RCC->AHB1ENR |= (1 << 1);  
    RCC->APB2ENR |= (1 << 14); 

    GPIOB->MODER &= ~(0x3 << 24); 
    GPIOB->PUPDR &= ~(0x3 << 24);
    GPIOB->PUPDR |= (0x2 << 24); 

    SYSCFG->EXTICR[3] &= ~(0xF << 0);
    SYSCFG->EXTICR[3] |= (0x1 << 0);

    EXTI->IMR  |= (1 << 12);
    EXTI->RTSR |= (1 << 12);   
    EXTI->FTSR &= ~(1 << 12);   

    NVIC_EnableIRQ(EXTI15_10_IRQn);
}