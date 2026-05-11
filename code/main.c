#include "device_driver.h"
#include <stdio.h>
#include <math.h>
#include "max7219_8x32.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#define MAX_PWM 1199

volatile uint8_t bt_rx_data = 0;
volatile uint8_t bt_rx_flag = 0;

volatile uint8_t mpu_data_ready = 0;

volatile float target_velocity = 0.0f;  // 속도 제어 변수, +값이면 전진
volatile float turn_speed = 0.0f;   // 회전 제어 변수, +값이면 우회전

volatile float target_velocity_control = 0.0f;      // 블루투스로 증감할 속도값 변수
volatile float turn_speed_control = 0.0f;      // 블루투스로 증감할 좌/우 회전 속도

volatile int robot_start = 0;  // 로봇 전원 변수
volatile int stop_face = 0;  // 로봇 표정 변수

volatile int face_turn = 0;  // 회전 시 표정변화 변수

// IMU값 읽어오기
static float prev_target_velocity = 0.0f;

// 모터 PWM 값 설정
void Set_Motor_Power_LR(float pid_L, float pid_R);
// DMP
void MPU6050_DMP_Init(void);
// MPU int핀 인터럽트
void EXTI15_10_IRQHandler(void);

void Find_Deadzone_Task(void);

void Encoder_Test_Task(void);

static void Sys_Init(int baud){
	SCB->CPACR |= (0x3 << 10*2)|(0x3 << 11*2); 
	Clock_Init();
	Uart2_Init(baud);
	setvbuf(stdout, NULL, _IONBF, 0);
	LED_Init();
}

#if 1
void Balancing_Task(void){
    short gyro[3], accel[3]; 
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    float q0, q1, q2, q3; 

    // --- 자세 제어 (Inner Loop) 변수 ---
    float current_angle = 0.0;

    float target_angle = 0.5; // 외부 루프에서 계속 갱신할 예정(주행 제어)
    
    float error = 0.0, prev_error = 0.0;
    float Ki = 180.0, Kd = 0.5;
    float Kp = 35.0f; 
    float Kp_boost = 15.0f;

    float P_term = 0.0, I_term = 0.0, D_term = 0.0;
    float PID_output = 0.0;

    // --- 속도 제어 (Outer Loop) 변수 ---
    int loop_counter = 0;
    int prev_enc_L = 0, prev_enc_R = 0;
    float current_velocity = 0.0;
    float filtered_velocity = 0.0; // 노이즈 제거용
    
    // 속도 제어용 PID 게인
    float vel_Kp = 0.02f;
    float vel_Ki = 0.001f;
    float vel_error = 0.0f, vel_prev_error = 0.0f;
    float vel_P = 0.0, vel_I = 0.0, vel_D = 0.0;
    float vel_PID_output = 0.0;

    float default_angle = 1.0f; // 무게중심 각도

    float goal_angle = 1.0f;       // 최종 목표 각도
    float active_velocity = 0.0f;  // 진짜 가속 페달 속도

    int fifo_status;

    static int current_face = -1; // 현재  표정
    int target_face = 0;          // 출력해야 할 표정

    mpu_reset_fifo(); 
    mpu_data_ready = 0; 
    
    // 엔코더 초기화 값 읽기
    prev_enc_L = (int)TIM5->CNT;
    prev_enc_R = -((short)TIM3->CNT);

    while(1){   

        // 1. 블루투스 수신 데이터 처리
        if (bt_rx_flag == 1){
            bt_rx_flag = 0;

            switch (bt_rx_data){
                case 1: // 로봇 동작 (ON)
                    robot_start = 1; 
                    break;
                case 2: // 동작 정지 (OFF)
                    robot_start = 0;
                    break;
                case 3: // 전후진 속도 증가
                    target_velocity_control += 5.0f;
                    if(target_velocity_control >= 80.0f) target_velocity_control = 80.0f;
                    break;
                case 4: // 전후진 속도 감소
                    target_velocity_control -= 5.0f; 
                    if(target_velocity_control <= -65.0f) target_velocity_control = -65.0f;
                    break;
                case 5: // 회전 속도 증가
                    turn_speed_control += 5.0f;
                    if(turn_speed_control >= 40.0f) turn_speed_control = 40.0f;
                    break;
                case 6: // 회전 속도 감소
                    turn_speed_control -= 5.0f;
                    if(turn_speed_control <= -25.0f) turn_speed_control = -25.0f;
                    break;
                case 7: // 속도 초기화
                    target_velocity_control = 0.0f;
                    turn_speed_control = 0.0f;
                    break;
                case 8: // 진입 금지 표정변화
                    stop_face = 1;
                    break;
                case 10: // 전진
                    target_velocity = 80.0f + target_velocity_control; 
                    turn_speed = 0.0f;
                    stop_face = 0;
                    break;
                case 11: // 후진
                    target_velocity = -80.0f - target_velocity_control; 
                    turn_speed = 0.0f;
                    stop_face = 0;
                    break;
                case 12: // 제자리 좌회전
                    target_velocity = 0.0f; 
                    turn_speed = -40.0f - turn_speed_control;
                    stop_face = 0;
                    break;
                case 13: // 제자리 우회전
                    target_velocity = 0.0f; 
                    turn_speed = 40.0f + turn_speed_control;
                    stop_face = 0;
                    break;
                case 14: // 전진하며 우회전
                    target_velocity = 80.0f;
                    turn_speed = 20.0f;
                    stop_face = 0;
                    break;
                case 15: // 전진하며 좌회전
                    target_velocity = 80.0f;
                    turn_speed = -20.0f;
                    stop_face = 0;
                    break;
                case 16: // 정지
                    target_velocity = 0.0f; 
                    turn_speed = 0.0f;
                    break;
                case 0:  // 정지
                    target_velocity = 0.0f; 
                    turn_speed = 0.0f;
                    vel_I = 0.0f;
                    break;
            }
        }

        // 2. 자세 유지 및 주행 제어 루프 (mpu_data_ready)
        if (mpu_data_ready == 1){
            mpu_data_ready = 0; 
            fifo_status = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
            
            if (fifo_status == 0){
                // 데이터가 밀려있다면(more > 0), 최신 데이터가 나올 때까지 
                // 계속 읽어서 덮어씌움. (DMP 기억 유지)
                while (more > 0){
                    dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
                }

                q0 = quat[0] / 1073741824.0f;
                q1 = quat[1] / 1073741824.0f;
                q2 = quat[2] / 1073741824.0f;
                q3 = quat[3] / 1073741824.0f;
                
                float gravity = -2.0f * (q1 * q3 - q0 * q2);
                
                if (gravity > 1.0f) gravity = 1.0f;
                else if (gravity < -1.0f) gravity = -1.0f;

                current_angle = asin(gravity) * 180.0f / 3.14159265f; 

                // [외부 루프] 50ms (20Hz) 주기로 속도 제어
                loop_counter++;
                if (loop_counter >= 5){
                    loop_counter = 0;
                    
                    // 1. 현재 엔코더 값 읽기 및 2,3,4 속도 변환
                    int curr_enc_L = (int)TIM5->CNT;
                    int curr_enc_R = -((short)TIM3->CNT); 
                    int delta_L = curr_enc_L - prev_enc_L;
                    int delta_R = curr_enc_R - prev_enc_R;
                    prev_enc_L = curr_enc_L;
                    prev_enc_R = curr_enc_R;
                    
                    float delta_avg = (delta_L + delta_R) / 2.0f;
                    current_velocity = delta_avg * 2.686f; 
                    filtered_velocity = (filtered_velocity * 0.8f) + (current_velocity * 0.2f);
                    
                    //초기 급가속 방지
                    if (target_velocity == 0.0f) {
                        active_velocity = 0.0f;
                    } 
                    else {
                        active_velocity = (active_velocity * 0.9f) + (target_velocity * 0.1f);
                    }

                    // 속도 PID 연산
                    vel_error = active_velocity - filtered_velocity; 
                    
                    // 뒤로 밀리는 스냅백 방지
                    if (target_velocity == 0.0f) {
                        if (prev_target_velocity != 0.0f) {
                            vel_I = 0.0f; 
                        }
                        if (fabs(vel_error) < 1.5f) {
                            vel_error = 0.0f; 
                        }

                        vel_P = (vel_Kp * 1.5f) * vel_error; 
                    }
                    else {
                        if (fabs(vel_error) < 5.0f) {
                            vel_error = 0.0f;
                        }
                        vel_P = vel_Kp * vel_error;
                    }

                    vel_I += vel_Ki * vel_error;
                    
                    // 적분 누적 방지 유지
                    if(vel_I > 3.0f) vel_I = 3.0f;
                    if(vel_I < -3.0f) vel_I = -3.0f;
                    if(fabs(current_angle) > 20.0f) vel_I = 0.0f; 

                    vel_PID_output = vel_P + vel_I; 
                    
                    // 임시 목표 기울기 계산
                    float raw_target_angle = default_angle + vel_PID_output;
                    
                    // 임시 목표 각도 안전 제한 
                    if (raw_target_angle > default_angle + 7.0f) raw_target_angle = default_angle + 7.0f;
                    if (raw_target_angle < default_angle - 7.0f) raw_target_angle = default_angle - 7.0f;

                   goal_angle = raw_target_angle;
                }  
                
                // [내부 루프] 10ms (100Hz) 주기로 자세 제어 (기존과 동일)
                // PID 제어
                target_angle = (target_angle * 0.6f) + (goal_angle * 0.4f);

                error = current_angle - target_angle; // 외부 루프가 준 목표 각도를 따라감

                // 비선형 제어 (제곱 곡선) 로직
                P_term = (Kp * error) + (Kp_boost * error * fabs(error));

                if (P_term > 900.0f) P_term = 900.0f;  // P_term 안전장치
                if (P_term < -900.0f) P_term = -900.0f;  // P_term 안전장치
                D_term = Kd * (error - prev_error) / 0.01;

                if (robot_start == 1) {
                    I_term += Ki * error * 0.01;
                    if (I_term > 300.0f) {
                        I_term = 300.0f;
                    } 
                    else if (I_term < -300.0f) {
                        I_term = -300.0f;
                    }
                    if (error * prev_error < 0.0f) { 
                        I_term = 0.0f; 
                    }

                    PID_output = P_term + I_term + D_term;
                    
                    float final_pid_L = PID_output + turn_speed;
                    float final_pid_R = PID_output - turn_speed;

                    Set_Motor_Power_LR(final_pid_L, final_pid_R);
                }
                else {
                    // 정지 상태일 때는 오차가 누적되지 않도록 I항을 0으로 초기화하고 모터 끄기
                    I_term = 0.0f;
                    vel_I = 0.0f;
                    PID_output = 0.0f;
                    
                    Set_Motor_Power_LR(0.0f, 0.0f); 
                }

                prev_error = error;

                if (fabs(current_angle) > 40.0f) {
                    target_face = 3; // 넘어짐 (Dead)
                }
                else if (stop_face == 1) {
                    target_face = 4; // 진입 금지 표정 
                }
                else if (target_velocity < -10.0f) {
                    target_face = 5; // 후진 중 
                }
                else if (turn_speed > 10.0f) {
                    target_face = 6; // 커브/제자리 우회전 중 
                }
                else if (turn_speed < -10.0f) {
                    target_face = 7; // 커브/제자리 좌회전 중 
                }
                else if (target_velocity > 10.0f) {
                    target_face = 0; // 전진 중       
                }
                else {
                    // 명령이 없는 대기(정지) 상태
                    if (fabs(error) > 8.0f) {
                        target_face = 2; // 크게 흔들림
                    } 
                    else if (fabs(error) > 3.0f) {
                        target_face = 1; // 중심 잡는 중
                    } 
                    else {
                        target_face = 0;
                    }
                }

                if (current_face != target_face) {
                    face_turn = 0;
                }

                int current_step = 0; // 이번 루프에서 출력해야 할 애니메이션 프레임 번호
                
                if(target_face == 6 || target_face == 7) {
                    face_turn++;  // 10ms마다 1씩 증가
                    if(face_turn >= 60) face_turn = 0; // 0~59 반복
                    
                    // 카운터 값에 따라 현재 프레임 번호(1, 2, 3) 결정
                    if(face_turn < 20) current_step = 1;
                    else if(face_turn < 40) current_step = 2;
                    else current_step = 3;
                }
                else {
                    face_turn = 0; 
                }

                //  표정 갱신 판단 로직
                static int prev_step = 0; 

                if (current_face != target_face || (current_face == target_face && prev_step != current_step)) {
                    current_face = target_face; // 상태 업데이트
                    prev_step = current_step;   // 프레임 업데이트

                    if (target_face == 0) MAX7219_16x32_Show_Happy();  // 완벽한 밸런스
                    else if (target_face == 1) MAX7219_16x32_Show_Smile();  // 중심 잡는 중 (균형잡기)
                    else if (target_face == 2) MAX7219_16x32_Show_Angry();  // 크게 흔들림
                    else if (target_face == 3) MAX7219_16x32_Show_Dead();  //밸런스 실패 또는 진입 금지
                    else if (target_face == 4) MAX7219_16x32_Show_Dead();  //밸런스 실패 또는 진입 금지
                    else if (target_face == 5) MAX7219_16x32_Show_back();  // 후진
                    else if (target_face == 6) {
                        if (current_step == 1) MAX7219_16x32_Show_right1();
                        else if (current_step == 2) MAX7219_16x32_Show_right2();
                        else if (current_step == 3) MAX7219_16x32_Show_right3();
                    }
                    else if (target_face == 7) {
                        if (current_step == 1) MAX7219_16x32_Show_left1();
                        else if (current_step == 2) MAX7219_16x32_Show_left2();
                        else if (current_step == 3) MAX7219_16x32_Show_left3();
                    }
                }
                
                static int print_count = 0;
                print_count++;
                
                if (print_count >= 10) { 
                    printf("A: %.1f | M: %.0f\n", current_angle, PID_output);
                    printf("T_Vel: %.1f | Cur_Vel: %.1f | Target_Ang: %.2f | Cur_Ang: %.2f\n", 
                            target_velocity, filtered_velocity, target_angle, current_angle);
                    print_count = 0; 
                }
            }
        }
    }       
}
#endif

// float 변수가 단 1개도 없는 껍데기 Main 함수
void Main(void)
{
    // 1. 여기서 FPU부터 안전하게 켭니다. (이제 뻗지 않습니다!)
    Sys_Init(115200);

    Uart2_Send_Byte('O');
    Uart2_Send_Byte('K');
    Uart2_Send_Byte('\n');

    printf("\n--- System Booting ---\n");
    
    I2C1_Init();
    printf(">> I2C Init OK!\n");

    TIM1_PWM_Init();
    TIM4_PWM_Init();
    Bluetooth_Init();
    // MPU6050_Init(); 

    uint8_t who_am_i = MPU6050_ReadReg(0x75);
    if (who_am_i == 0x68) {
        printf(">> [SUCCESS] MPU6050 Connection OK! (ID: 0x68)\n");
    } else {
        printf(">> [ERROR] MPU6050 Not Found. (ID: 0x%X)\n", who_am_i);
    }


    MPU6050_DMP_Init();
    MPU6050_INT_Init();  //DMP 인터럽트 활성화
    printf(">> MPU6050 Wake-up OK!\n");

    Encoder_Init();  //엔코더 활성화
    
    MAX7219_16x32_Init();
    MAX7219_16x32_Set_Brightness(0x08);
    
    MAX7219_16x32_Show_Happy();

    Balancing_Task();
}

// 좌/우 모터를 독립적으로 제어하는 함수
#if 1
void Set_Motor_Power_LR(float pid_L, float pid_R) {
    float abs_L = fabs(pid_L); 
    float abs_R = fabs(pid_R); 

    int MIN_PWM_L = 33; 
    int MIN_PWM_R = 30; 

    int pwm_L = 0, pwm_R = 0;

    if (abs_L > 0.5f) {
        pwm_L = (int)(abs_L + MIN_PWM_L); // 오차가 클 때는 온전한 마찰력 제공
    } else {
        // 오차가 0 ~ 0.5 사이일 때는 오차에 비례해서 마찰력을 서서히 올림
        pwm_L = (int)(abs_L + (MIN_PWM_L * (abs_L / 0.5f))); 
    }
    if (abs_R > 0.5f) {
        pwm_R = (int)(abs_R + MIN_PWM_R);
    } else {
        pwm_R = (int)(abs_R + (MIN_PWM_R * (abs_R / 0.5f))); 
    }

    // 최대 스피드 락
    if (pwm_L > MAX_PWM) pwm_L = MAX_PWM; 
    if (pwm_R > MAX_PWM) pwm_R = MAX_PWM; 

    // --- 왼쪽 모터 (TIM1) 방향 제어 ---
    if (pid_L > 0.1f) { 
        TIM1->CCR1 = 0; 
        TIM1->CCR2 = pwm_L; // 전진
    } else if (pid_L < -0.1f) { 
        TIM1->CCR1 = pwm_L; 
        TIM1->CCR2 = 0; // 후진
    } else {
        TIM1->CCR1 = 0; 
        TIM1->CCR2 = 0;
    }

    // --- 오른쪽 모터 (TIM4) 방향 제어 ---
    if (pid_R > 0.1f) { 
        TIM4->CCR1 = pwm_R; 
        TIM4->CCR2 = 0; // 전진
    } else if (pid_R < -0.1f) { 
        TIM4->CCR1 = 0; 
        TIM4->CCR2 = pwm_R; // 후진
    } else {
        TIM4->CCR1 = 0; 
        TIM4->CCR2 = 0;
    }
}
#endif
// MPU6050_DMP_Init(void) 가속도에 의존하는 값
#if 1
void MPU6050_DMP_Init(void) {
    printf(">> MPU6050 Init Start...\n");
    if (mpu_init(NULL) != 0) { printf(">> [ERROR] MPU Init Failed!\n"); return; }
    
    // DMP 내부 적분 주기와 하드웨어 주기를 200Hz로 일치
    mpu_set_sample_rate(200); 

    mpu_set_lpf(20);
    
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    
    if (dmp_load_motion_driver_firmware() != 0) { printf(">> [ERROR] DMP FW Failed!\n"); return; }
    dmp_set_orientation(136);  
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
    dmp_set_fifo_rate(100); // 제어 루프는 100Hz로 유지
    mpu_set_dmp_state(1);   // DMP 켜기
    
    printf(">> [SUCCESS] DMP Init OK!\n");
}
#endif

// PB10 ~ PB15 핀에 인터럽트가 걸리면 무조건 이 함수가 실행
void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & (1 << 12)) {
        // 인터럽트 발생 기록지우기 
        EXTI->PR = (1 << 12); 
        // 인터럽트 발생 변수 ON
        mpu_data_ready = 1;
    }
}

// 모터가 돌기 시작하는 최소 PWM(데드존) 값을 찾는 테스트 함수
void Find_Deadzone_Task(void){
    TIM2_Delay(3000); // 3초 대기 (안전)

    for (int test_pwm = 60; test_pwm <= 200; test_pwm += 2) {
        
        TIM1->CCR1 = 0; // 왼쪽 전진
        TIM1->CCR2 = test_pwm; 

        TIM4->CCR1 = test_pwm+20; // 오른쪽 전진
        TIM4->CCR2 = 0;
        
        // 현재 넣고 있는 파워를 터미널에 출력
        printf("Current Test PWM: %d\n", test_pwm);
        
        // 0.1초 대기 (천천히 파워가 올라가도록)
        TIM2_Delay(100); 
    }

    TIM4->CCR1 = 0; TIM4->CCR2 = 0;
    TIM1->CCR1 = 0; TIM1->CCR2 = 0;
    
    while(1); 
}


// 바퀴를 손으로 돌려보며 엔코더 값을 확인하는 테스트 태스크
void Encoder_Test_Task(void) {
    Encoder_Init();
    
    TIM5->CNT = 0;
    TIM3->CNT = 0;

    while(1) {
        int left_cnt = (int)TIM5->CNT;
        short right_cnt = -(short)TIM3->CNT;
        
        printf("Left(TIM5): %6d  |  Right(TIM3): %6d\n", left_cnt, right_cnt);
        
        TIM2_Delay(100); 
    }
}