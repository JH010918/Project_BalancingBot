#ifndef __MAX7219_8X32_H__
#define __MAX7219_8X32_H__

void MAX7219_16x32_Init(void);
void MAX7219_16x32_Clear(void);
void MAX7219_16x32_All_On(void);
void MAX7219_16x32_Set_Brightness(unsigned char intensity);

void MAX7219_16x32_Show_Happy(void);
void MAX7219_16x32_Show_Smile(void);
void MAX7219_16x32_Show_Sad(void);
void MAX7219_16x32_Show_Angry(void);
void MAX7219_16x32_Show_Dead(void);

void MAX7219_16x32_Show_right1(void);
void MAX7219_16x32_Show_right2(void);
void MAX7219_16x32_Show_right3(void);
void MAX7219_16x32_Show_left1(void);
void MAX7219_16x32_Show_left2(void);
void MAX7219_16x32_Show_left3(void);
void MAX7219_16x32_Show_back(void);