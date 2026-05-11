#include "max7219_8x32.h"
#include "device_driver.h"
/*
    ============================================
    16x32 MAX7219 Dot Matrix
    - 위쪽 8x32  : DIN=PA4, CS=PA5, CLK=PA8
    - 아래쪽 8x32: DIN=PA4, CS=PB0, CLK=PA8
    - DIN, CLK는 공통
    - CS만 분리
    ============================================
*/

#define MAX_DIN_PORT        GPIOA
#define MAX_CLK_PORT        GPIOA
#define MAX_TOP_CS_PORT     GPIOA
#define MAX_BOTTOM_CS_PORT  GPIOB

#define MAX_DIN_PIN         4
#define MAX_CLK_PIN         8
#define MAX_TOP_CS_PIN      5
#define MAX_BOTTOM_CS_PIN   0

#define DIN_HIGH()          Macro_Set_Bit(MAX_DIN_PORT->ODR, MAX_DIN_PIN)
#define DIN_LOW()           Macro_Clear_Bit(MAX_DIN_PORT->ODR, MAX_DIN_PIN)

#define CLK_HIGH()          Macro_Set_Bit(MAX_CLK_PORT->ODR, MAX_CLK_PIN)
#define CLK_LOW()           Macro_Clear_Bit(MAX_CLK_PORT->ODR, MAX_CLK_PIN)

#define TOP_CS_HIGH()       Macro_Set_Bit(MAX_TOP_CS_PORT->ODR, MAX_TOP_CS_PIN)
#define TOP_CS_LOW()        Macro_Clear_Bit(MAX_TOP_CS_PORT->ODR, MAX_TOP_CS_PIN)

#define BOTTOM_CS_HIGH()    Macro_Set_Bit(MAX_BOTTOM_CS_PORT->ODR, MAX_BOTTOM_CS_PIN)
#define BOTTOM_CS_LOW()     Macro_Clear_Bit(MAX_BOTTOM_CS_PORT->ODR, MAX_BOTTOM_CS_PIN)

static void MAX7219_Delay(void);
static void MAX7219_GPIO_Init(void);
static void MAX7219_Send16(unsigned char address, unsigned char data);
static void MAX7219_Write_All_Top(unsigned char address, unsigned char data);
static void MAX7219_Write_All_Bottom(unsigned char address, unsigned char data);
static unsigned char Reverse_Bits(unsigned char data);
static void MAX7219_Write_Row_Top(unsigned char row,
                                  unsigned char data0,
                                  unsigned char data1,
                                  unsigned char data2,
                                  unsigned char data3);
static void MAX7219_Write_Row_Bottom(unsigned char row,
                                     unsigned char data0,
                                     unsigned char data1,
                                     unsigned char data2,
                                     unsigned char data3);
static void MAX7219_16x32_Display(const unsigned char pattern[16][4]);

static void MAX7219_Delay(void){
    volatile int i;
    for(i = 0; i < 50; i++);
}

// GPIO 초기화
static void MAX7219_GPIO_Init(void){
    RCC->AHB1ENR |= (1 << 0);   // GPIOA
    RCC->AHB1ENR |= (1 << 1);   // GPIOB

    // PA4 = DIN output
    MAX_DIN_PORT->MODER &= ~(0x3 << (MAX_DIN_PIN * 2));
    MAX_DIN_PORT->MODER |=  (0x1 << (MAX_DIN_PIN * 2));

    // PA8 = CLK output
    MAX_CLK_PORT->MODER &= ~(0x3 << (MAX_CLK_PIN * 2));
    MAX_CLK_PORT->MODER |=  (0x1 << (MAX_CLK_PIN * 2));

    // PA5 = TOP CS output
    MAX_TOP_CS_PORT->MODER &= ~(0x3 << (MAX_TOP_CS_PIN * 2));
    MAX_TOP_CS_PORT->MODER |=  (0x1 << (MAX_TOP_CS_PIN * 2));

    // PB0 = BOTTOM CS output
    MAX_BOTTOM_CS_PORT->MODER &= ~(0x3 << (MAX_BOTTOM_CS_PIN * 2));
    MAX_BOTTOM_CS_PORT->MODER |=  (0x1 << (MAX_BOTTOM_CS_PIN * 2));

    // push-pull
    MAX_DIN_PORT->OTYPER &= ~(1 << MAX_DIN_PIN);
    MAX_CLK_PORT->OTYPER &= ~(1 << MAX_CLK_PIN);
    MAX_TOP_CS_PORT->OTYPER &= ~(1 << MAX_TOP_CS_PIN);
    MAX_BOTTOM_CS_PORT->OTYPER &= ~(1 << MAX_BOTTOM_CS_PIN);

    // speed
    MAX_DIN_PORT->OSPEEDR |= (0x2 << (MAX_DIN_PIN * 2));
    MAX_CLK_PORT->OSPEEDR |= (0x2 << (MAX_CLK_PIN * 2));
    MAX_TOP_CS_PORT->OSPEEDR |= (0x2 << (MAX_TOP_CS_PIN * 2));
    MAX_BOTTOM_CS_PORT->OSPEEDR |= (0x2 << (MAX_BOTTOM_CS_PIN * 2));

    // no pull
    MAX_DIN_PORT->PUPDR &= ~(0x3 << (MAX_DIN_PIN * 2));
    MAX_CLK_PORT->PUPDR &= ~(0x3 << (MAX_CLK_PIN * 2));
    MAX_TOP_CS_PORT->PUPDR &= ~(0x3 << (MAX_TOP_CS_PIN * 2));
    MAX_BOTTOM_CS_PORT->PUPDR &= ~(0x3 << (MAX_BOTTOM_CS_PIN * 2));

    TOP_CS_HIGH();
    BOTTOM_CS_HIGH();
    CLK_LOW();
    DIN_LOW();
}

// 16비트 전송
static void MAX7219_Send16(unsigned char address, unsigned char data){
    unsigned short packet;
    int i;

    packet = ((unsigned short)address << 8) | data;

    for(i = 0; i < 16; i++){
        CLK_LOW();

        if(packet & 0x8000) DIN_HIGH();
        else                DIN_LOW();

        MAX7219_Delay();
        CLK_HIGH();
        MAX7219_Delay();

        packet <<= 1;
    }
}

// 위쪽 8x32 전체 설정
static void MAX7219_Write_All_Top(unsigned char address, unsigned char data){
    int i;

    TOP_CS_LOW();
    for(i = 0; i < 4; i++){
        MAX7219_Send16(address, data);
    }
    TOP_CS_HIGH();
}

// 아래쪽 8x32 전체 설정
static void MAX7219_Write_All_Bottom(unsigned char address, unsigned char data){
    int i;

    BOTTOM_CS_LOW();
    for(i = 0; i < 4; i++){
        MAX7219_Send16(address, data);
    }
    BOTTOM_CS_HIGH();
}

// 위쪽 row 출력
static void MAX7219_Write_Row_Top(unsigned char row,
                                  unsigned char data0,
                                  unsigned char data1,
                                  unsigned char data2,
                                  unsigned char data3){
    TOP_CS_LOW();

    MAX7219_Send16(row, Reverse_Bits(data3));
    MAX7219_Send16(row, Reverse_Bits(data2));
    MAX7219_Send16(row, Reverse_Bits(data1));
    MAX7219_Send16(row, Reverse_Bits(data0));

    TOP_CS_HIGH();
}

// 아래쪽 row 출력
static void MAX7219_Write_Row_Bottom(unsigned char row,
                                     unsigned char data0,
                                     unsigned char data1,
                                     unsigned char data2,
                                     unsigned char data3){
    BOTTOM_CS_LOW();

    MAX7219_Send16(row, Reverse_Bits(data3));
    MAX7219_Send16(row, Reverse_Bits(data2));
    MAX7219_Send16(row, Reverse_Bits(data1));
    MAX7219_Send16(row, Reverse_Bits(data0));

    BOTTOM_CS_HIGH();
}

// 초기화
void MAX7219_16x32_Init(void){
    MAX7219_GPIO_Init();

    // decode mode off
    MAX7219_Write_All_Top(0x09, 0x00);
    MAX7219_Write_All_Bottom(0x09, 0x00);

    // intensity
    MAX7219_Write_All_Top(0x0A, 0x03);
    MAX7219_Write_All_Bottom(0x0A, 0x03);

    // scan limit = 8 rows
    MAX7219_Write_All_Top(0x0B, 0x07);
    MAX7219_Write_All_Bottom(0x0B, 0x07);

    // normal operation
    MAX7219_Write_All_Top(0x0C, 0x01);
    MAX7219_Write_All_Bottom(0x0C, 0x01);

    // display test off
    MAX7219_Write_All_Top(0x0F, 0x00);
    MAX7219_Write_All_Bottom(0x0F, 0x00);

    MAX7219_16x32_Clear();
}

// 전체 끄기
void MAX7219_16x32_Clear(void){
    int row;

    for(row = 1; row <= 8; row++){
        MAX7219_Write_Row_Top(row, 0x00, 0x00, 0x00, 0x00);
        MAX7219_Write_Row_Bottom(row, 0x00, 0x00, 0x00, 0x00);
    }
}

// 전체 켜기
void MAX7219_16x32_All_On(void){
    int row;

    for(row = 1; row <= 8; row++){
        MAX7219_Write_Row_Top(row, 0xFF, 0xFF, 0xFF, 0xFF);
        MAX7219_Write_Row_Bottom(row, 0xFF, 0xFF, 0xFF, 0xFF);
    }
}

// 밝기 조절
void MAX7219_16x32_Set_Brightness(unsigned char intensity){
    if(intensity > 0x0F) intensity = 0x0F;

    MAX7219_Write_All_Top(0x0A, intensity);
    MAX7219_Write_All_Bottom(0x0A, intensity);
}

// 16x32 출력
// pattern[0~7]   = 위쪽 8x32
// pattern[8~15]  = 아래쪽 8x32
static void MAX7219_16x32_Display(const unsigned char pattern[16][4]){
    int row;

    for(row = 0; row < 8; row++){
        MAX7219_Write_Row_Top(row+1,
                              pattern[row][0],
                              pattern[row][1],
                              pattern[row][2],
                              pattern[row][3]);

        MAX7219_Write_Row_Bottom(row+1,
                                 pattern[row + 8][0],
                                 pattern[row + 8][1],
                                 pattern[row + 8][2],
                                 pattern[row + 8][3]);
    }
}

static unsigned char Reverse_Bits(unsigned char data){
    unsigned char r = 0;
    int i;

    for(i = 0; i < 8; i++){
        r <<= 1;
        r |= (data & 0x01);
        data >>= 1;
    }
    return r;
}

// 표정 패턴

static const unsigned char face_happy[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000110, 0b00000000, 0b00000000, 0b01100000},
    {0b00000011, 0b00000000, 0b00000000, 0b11000000},
    {0b00000001, 0b10000000, 0b00000001, 0b10000000},
    {0b00000000, 0b11000000, 0b00000011, 0b00000000},
    {0b00000001, 0b10000000, 0b00000001, 0b10000000},
    {0b00000011, 0b00000000, 0b00000000, 0b11000000},
    {0b00000110, 0b00000000, 0b00000000, 0b01100000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b01000000, 0b00000010, 0b00000000},
    {0b00000000, 0b00100000, 0b00000100, 0b00000000},
    {0b00000000, 0b00011111, 0b11111000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
};

static const unsigned char face_smile[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00011111, 0b10000000, 0b00000001, 0b11111000},
    {0b00100000, 0b01000000, 0b00000010, 0b00000100},
    {0b01000000, 0b00100000, 0b00000100, 0b00000010},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b01000000, 0b00000010, 0b00000000},
    {0b00000000, 0b00100000, 0b00000100, 0b00000000},
    {0b00000000, 0b00011111, 0b11111000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
};

static const unsigned char face_sad[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b01111111, 0b11100000, 0b00000111, 0b11111110},
    {0b01111111, 0b11100000, 0b00000111, 0b11111110},
    {0b00011001, 0b10000000, 0b00000001, 0b10011000},
    {0b00011001, 0b10000000, 0b00000001, 0b10011000},
    {0b00011001, 0b10000000, 0b00000001, 0b10011000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00111111, 0b11111100, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000}
};

static const unsigned char face_angry[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000100, 0b00000000, 0b00000000, 0b00100000},
    {0b00001110, 0b00000000, 0b00000000, 0b01110000},
    {0b00001111, 0b00000000, 0b00000000, 0b11110000},
    {0b00001111, 0b10000000, 0b00000001, 0b11110000},
    {0b00000111, 0b11000000, 0b00000011, 0b11100000},
    {0b00000011, 0b10000000, 0b00000001, 0b11000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00001111, 0b11110000, 0b00000000},
    {0b00000000, 0b00010000, 0b00001000, 0b00000000},
    {0b00000000, 0b00100000, 0b00000100, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000}
};

static const unsigned char face_dead[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b01100000, 0b11000000, 0b00000011, 0b00000110},
    {0b00110001, 0b10000000, 0b00000001, 0b10001100},
    {0b00011011, 0b00000000, 0b00000000, 0b01011000},
    {0b00001110, 0b00000000, 0b00000000, 0b01110000},
    {0b00011011, 0b00000000, 0b00000000, 0b11011000},
    {0b00110001, 0b10000000, 0b00000001, 0b10001100},
    {0b01100000, 0b11000000, 0b00000011, 0b00000110},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00011111, 0b11111000, 0b00000000},
    {0b00000000, 0b00011111, 0b11111000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000}
};

static const unsigned char face_right1[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00011111, 0b11111000, 0b00011111, 0b11111000},
    {0b00000000, 0b11100000, 0b00000000, 0b11100000},
    {0b00000001, 0b11110000, 0b00000001, 0b11110000},
    {0b00000001, 0b11110000, 0b00000001, 0b11110000},
    {0b00000000, 0b11100000, 0b00000000, 0b11100000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b10000000, 0b00000000, 0b00010000},
    {0b00111111, 0b01000000, 0b00000111, 0b11101000},
    {0b00100000, 0b00100000, 0b00000100, 0b00000100},
    {0b00100000, 0b00100000, 0b00000100, 0b00000100},
    {0b00111111, 0b01000000, 0b00000111, 0b11101000},
    {0b00000000, 0b10000000, 0b00000000, 0b00010000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
};

static const unsigned char face_right2[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00011111, 0b11111000, 0b00011111, 0b11111000},
    {0b00000000, 0b11100000, 0b00000000, 0b11100000},
    {0b00000001, 0b11110000, 0b00000001, 0b11110000},
    {0b00000001, 0b11110000, 0b00000001, 0b11110000},
    {0b00000000, 0b11100000, 0b00000000, 0b11100000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000001, 0b00000000, 0b00000000},
    {0b01000000, 0b01111110, 0b10000000, 0b00111111},
    {0b00100000, 0b01000000, 0b01000000, 0b00100000},
    {0b00100000, 0b01000000, 0b01000000, 0b00100000},
    {0b01000000, 0b01111110, 0b10000000, 0b00111111},
    {0b00000000, 0b00000001, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
};

static const unsigned char face_right3[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00011111, 0b11111000, 0b00011111, 0b11111000},
    {0b00000000, 0b11100000, 0b00000000, 0b11100000},
    {0b00000001, 0b11110000, 0b00000001, 0b11110000},
    {0b00000001, 0b11110000, 0b00000001, 0b11110000},
    {0b00000000, 0b11100000, 0b00000000, 0b11100000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b10000000, 0b00000000, 0b00001000, 0b00000000},
    {0b01000000, 0b00000011, 0b11110100, 0b00000111},
    {0b00100000, 0b00000010, 0b00000010, 0b00000100},
    {0b00100000, 0b00000010, 0b00000010, 0b00000100},
    {0b01000000, 0b00000011, 0b11110100, 0b00000111},
    {0b10000000, 0b00000000, 0b00001000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
};

static const unsigned char face_left1[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00011111, 0b11111000, 0b00011111, 0b11111000},
    {0b00000111, 0b00000000, 0b00000111, 0b00000000},
    {0b00001111, 0b10000000, 0b00001111, 0b10000000},
    {0b00001111, 0b10000000, 0b00001111, 0b10000000},
    {0b00000111, 0b00000000, 0b00000111, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00001000, 0b00000000, 0b00000001, 0b00000000},
    {0b00010111, 0b11100000, 0b00000010, 0b11111100},
    {0b00100000, 0b00100000, 0b00000100, 0b00000100},
    {0b00100000, 0b00100000, 0b00000100, 0b00000100},
    {0b00010111, 0b11100000, 0b00000010, 0b11111100},
    {0b00001000, 0b00000000, 0b00000001, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
};

static const unsigned char face_left2[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00011111, 0b11111000, 0b00011111, 0b11111000},
    {0b00000111, 0b00000000, 0b00000111, 0b00000000},
    {0b00001111, 0b10000000, 0b00001111, 0b10000000},
    {0b00001111, 0b10000000, 0b00001111, 0b10000000},
    {0b00000111, 0b00000000, 0b00000111, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000001, 0b00000000, 0b00000100},
    {0b11111100, 0b00000010, 0b11111100, 0b00001011},
    {0b00000100, 0b00000100, 0b00000100, 0b00010000},
    {0b00000100, 0b00000100, 0b00000100, 0b00010000},
    {0b11111100, 0b00000010, 0b11111100, 0b00001011},
    {0b00000000, 0b00000001, 0b00000000, 0b00000100},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
};

static const unsigned char face_left3[16][4] =
{
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00011111, 0b11111000, 0b00011111, 0b11111000},
    {0b00000111, 0b00000000, 0b00000111, 0b00000000},
    {0b00001111, 0b10000000, 0b00001111, 0b10000000},
    {0b00001111, 0b10000000, 0b00001111, 0b10000000},
    {0b00000111, 0b00000000, 0b00000111, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b10000000, 0b00000000, 0b10000000},
    {0b11000001, 0b01111110, 0b00000001, 0b01111110},
    {0b01000010, 0b00000010, 0b00000010, 0b00000010},
    {0b01000010, 0b00000010, 0b00000010, 0b00000010},
    {0b11000001, 0b01111110, 0b00000001, 0b01111110},
    {0b00000000, 0b10000000, 0b00000000, 0b10000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
};

static const unsigned char face_back[16][4] =
{
    {0b00000000, 0b00000000, 0b00000111, 0b11100000},
    {0b00000011, 0b11000000, 0b00001000, 0b00010000},
    {0b00000100, 0b00100000, 0b00010000, 0b00001000},
    {0b00001000, 0b00010000, 0b00010000, 0b00001000},
    {0b00001000, 0b00010000, 0b00010000, 0b00001000},
    {0b00000100, 0b00100000, 0b00010000, 0b00001000},
    {0b00000011, 0b11000000, 0b00001000, 0b00010000},
    {0b00000000, 0b00000000, 0b00000111, 0b11100000},

    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00001111, 0b11110000, 0b00000000},
    {0b00000000, 0b00001000, 0b00010000, 0b00000000},
    {0b00000000, 0b00001000, 0b00010000, 0b00000000},
    {0b00000000, 0b00001111, 0b11110000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000},
    {0b00000000, 0b00000000, 0b00000000, 0b00000000}
};

// 표정 출력 함수
void MAX7219_16x32_Show_Happy(void){
    MAX7219_16x32_Display(face_happy);
}

void MAX7219_16x32_Show_Smile(void){
    MAX7219_16x32_Display(face_smile);
}

void MAX7219_16x32_Show_Sad(void){
    MAX7219_16x32_Display(face_sad);
}

void MAX7219_16x32_Show_Angry(void){
    MAX7219_16x32_Display(face_angry);
}

void MAX7219_16x32_Show_Dead(void){
    MAX7219_16x32_Display(face_dead);
}

void MAX7219_16x32_Show_right1(void){
    MAX7219_16x32_Display(face_right1);
}

void MAX7219_16x32_Show_right2(void){
    MAX7219_16x32_Display(face_right2);
}

void MAX7219_16x32_Show_right3(void){
    MAX7219_16x32_Display(face_right3);
}

void MAX7219_16x32_Show_left1(void){
    MAX7219_16x32_Display(face_left1);
}

void MAX7219_16x32_Show_left2(void){
    MAX7219_16x32_Display(face_left2);
}

void MAX7219_16x32_Show_left3(void){
    MAX7219_16x32_Display(face_left3);
}

void MAX7219_16x32_Show_back(void){
    MAX7219_16x32_Display(face_back);
}