#include "stm32f4xx.h"
#include <stdlib.h>
#include <stdio.h>

// --- KONFIGURASI PIN ---
// LCD (I2C Manual): SCL=PB8, SDA=PB9
// LED: PA0, PA1, PA2, PA3 (TIM2), PA8 (TIM1)
// Button: PB0, PB1, PB3, PB4, PB12 -> EXTI
// Buzzer: PB10

#define LCD_ADDR_FIXED 0x27 
#define NOTE_G4  392
#define NOTE_B4  494
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_G5  784
#define NOTE_B5  988
#define NOTE_D6  1175
#define NOTE_E6  1319

// --- GLOBAL VARIABLES ---
volatile int flag_button_pressed = -1; 
volatile int flag_timeout = 0;         
volatile uint32_t msTicks = 0;
volatile uint32_t last_debounce_time = 0;
volatile int input_allowed = 1; 

void SysTick_Handler(void) { msTicks++; }
uint32_t get_millis(void) { return msTicks; }

void delay_audio_math(uint32_t us) {
    volatile uint32_t count = us * (SystemCoreClock / 1000000) / 8;
    while(count--);
}

void delay_real_ms(uint32_t ms) {
    uint32_t start = get_millis();
    while ((get_millis() - start) < ms);
}

void delay_dumb(int count) {
    volatile int i;
    for (i = 0; i < count; i++);
}

void System_Init(void) {
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 1000); 
    
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SYSCFGEN;
}

void ADC_Init(void) {
    GPIOA->MODER |= (3 << (7 * 2));

    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON;

    delay_real_ms(1);

    ADC1->SQR3 = 7;
    ADC1->SMPR2 |= ADC_SMPR2_SMP7;
}

// --- I2C & LCD ---
void I2C_Init_Soft(void) {
    GPIOB->MODER &= ~((3<<(8*2)) | (3<<(9*2)));
    GPIOB->MODER |=  ((1<<(8*2)) | (1<<(9*2))); 
    GPIOB->OTYPER |= ((1<<8) | (1<<9)); 
    GPIOB->OSPEEDR |= ((3<<(8*2)) | (3<<(9*2)));
    GPIOB->PUPDR &= ~((3<<(8*2)) | (3<<(9*2)));
    GPIOB->PUPDR |=  ((1<<(8*2)) | (1<<(9*2)));
    GPIOB->BSRR = (1<<8) | (1<<9);
}

void SCL_H(void) { GPIOB->BSRR = (1<<8); }
void SCL_L(void) { GPIOB->BSRR = (1<<24); } 
void SDA_H(void) { GPIOB->BSRR = (1<<9); }
void SDA_L(void) { GPIOB->BSRR = (1<<25); } 

void I2C_Start(void) { SDA_H(); SCL_H(); delay_dumb(50); SDA_L(); delay_dumb(50); SCL_L(); delay_dumb(50); }
void I2C_Stop(void) { SDA_L(); delay_dumb(50); SCL_H(); delay_dumb(50); SDA_H(); delay_dumb(50); }
void I2C_Write(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        if (data & 0x80) SDA_H(); else SDA_L();
        data <<= 1; delay_dumb(50); SCL_H(); delay_dumb(50); SCL_L(); delay_dumb(50);
    }
    SDA_H(); delay_dumb(50); SCL_H(); delay_dumb(50); SCL_L(); delay_dumb(50);
}

uint8_t BL_STATE = 0x08;

void LCD_Nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble & 0xF0) | BL_STATE | (rs ? 1 : 0) | 4; 
    I2C_Start(); I2C_Write(LCD_ADDR_FIXED << 1); I2C_Write(data); I2C_Stop(); delay_dumb(500);
    data &= ~4; 
    I2C_Start(); I2C_Write(LCD_ADDR_FIXED << 1); I2C_Write(data); I2C_Stop(); delay_dumb(500);
}
void LCD_Command(uint8_t cmd) { LCD_Nibble(cmd & 0xF0, 0); LCD_Nibble((cmd << 4) & 0xF0, 0); }
void LCD_Data(uint8_t data) { LCD_Nibble(data & 0xF0, 1); LCD_Nibble((data << 4) & 0xF0, 1); }
void LCD_String(char *str) { while(*str) LCD_Data(*str++); }
void LCD_SetCursor(uint8_t row, uint8_t col) { uint8_t addr = (row == 0) ? 0x80 : 0xC0; LCD_Command(addr + col); }
void LCD_Clear(void) { LCD_Command(0x01); delay_real_ms(2); }
void LCD_Init(void) {
    I2C_Init_Soft(); delay_real_ms(50);
    LCD_Nibble(0x30, 0); delay_real_ms(5); LCD_Nibble(0x30, 0); delay_real_ms(2);
    LCD_Nibble(0x30, 0); delay_real_ms(2); LCD_Nibble(0x20, 0); delay_real_ms(2);
    LCD_Command(0x28); LCD_Command(0x08); LCD_Command(0x01); delay_real_ms(5); LCD_Command(0x0C);
}

// --- UART ---
void UART_Init(void) {
    GPIOA->MODER |= (2<<(9*2)); GPIOA->AFR[1] |= (7<<(1*4)); 
    USART1->BRR = SystemCoreClock / 9600; USART1->CR1 |= USART_CR1_TE | USART_CR1_UE; 
}
void UART_Print(char *str) { while (*str) { while (!(USART1->SR & USART_SR_TXE)); USART1->DR = *str++; } }
void UART_SendStats(int skor, int level, int reactionTime, int status) {
    char buf[64];
    UART_Print("--------------------------\r\n");
    if(status == 1) UART_Print("[RESULT] BENAR!\r\n");
    else UART_Print("[RESULT] SALAH/TIMEOUT\r\n");
    sprintf(buf, "Lvl: %d | Skor: %d | Time: %d ms\r\n", level, skor, reactionTime);
    UART_Print(buf);
}

// --- BUZZER ---
void Buzzer_Init(void) {
    GPIOB->MODER &= ~(3U << (10 * 2)); GPIOB->MODER |= (1U << (10 * 2));
    GPIOB->OTYPER &= ~(1U << 10); GPIOB->OSPEEDR |= (3U << (10 * 2)); GPIOB->PUPDR &= ~(3U << (10 * 2));
}
void play_tone(uint32_t frequency, uint32_t duration_ms) {
    if (frequency == 0) { GPIOB->ODR &= ~(1<<10); delay_real_ms(duration_ms); return; }
    uint32_t period_us = 1000000 / frequency; uint32_t half_period = period_us / 2;
    long cycles = (long)duration_ms * 1000 / period_us;
    for (long i = 0; i < cycles; i++) {
        GPIOB->ODR |= (1<<10);  delay_audio_math(half_period);
        GPIOB->ODR &= ~(1<<10); delay_audio_math(half_period);
    }
    GPIOB->ODR &= ~(1<<10);
}
void soundClick() { play_tone(NOTE_D5, 30); }

// --- LED PWM ---
void PWM_Init(void) {
    GPIOA->MODER &= ~((3<<(0*2)) | (3<<(1*2)) | (3<<(2*2)) | (3<<(3*2)) | (3<<(8*2)));
    GPIOA->MODER |=  ((2<<(0*2)) | (2<<(1*2)) | (2<<(2*2)) | (2<<(3*2)) | (2<<(8*2)));
    GPIOA->AFR[0] |= (1<<0) | (1<<4) | (1<<8) | (1<<12); GPIOA->AFR[1] |= (1<<0);                            
    TIM2->PSC = 83; TIM2->ARR = 255; TIM2->CCMR1 = (6<<4) | (6<<12); TIM2->CCMR2 = (6<<4) | (6<<12); 
    TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E; TIM2->CR1 |= TIM_CR1_CEN; 
    TIM1->PSC = 83; TIM1->ARR = 255; TIM1->CCMR1 = (6<<4); TIM1->CCER = TIM_CCER_CC1E;
    TIM1->BDTR |= TIM_BDTR_MOE; TIM1->CR1 |= TIM_CR1_CEN;
}
void set_led_pwm(int led_index, int value) {
    if (value > 255) value = 255; if (value < 0) value = 0;
    switch(led_index) {
        case 0: TIM2->CCR1 = value; break; case 1: TIM2->CCR2 = value; break;
        case 2: TIM2->CCR3 = value; break; case 3: TIM2->CCR4 = value; break;
        case 4: TIM1->CCR1 = value; break;
    }
}

// --- EXTI ---
void EXTI_Init_Custom(void) {
    GPIOB->MODER &= ~((3<<(0*2)) | (3<<(1*2)) | (3<<(3*2)) | (3<<(4*2)) | (3<<(12*2)));
    GPIOB->PUPDR |=  ((1<<(0*2)) | (1<<(1*2)) | (1<<(3*2)) | (1<<(4*2)) | (1<<(12*2)));
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB | SYSCFG_EXTICR1_EXTI1_PB | SYSCFG_EXTICR1_EXTI3_PB;
    SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB; SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI12_PB;
    EXTI->FTSR |= (1<<0)|(1<<1)|(1<<3)|(1<<4)|(1<<12);
    EXTI->IMR  |= (1<<0)|(1<<1)|(1<<3)|(1<<4)|(1<<12);
    NVIC_EnableIRQ(EXTI0_IRQn); NVIC_EnableIRQ(EXTI1_IRQn); NVIC_EnableIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI4_IRQn); NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Button_Handler(int idx) {
    if (input_allowed == 0) return; 

    uint32_t now = get_millis();
    if (now - last_debounce_time > 200) {
        flag_button_pressed = idx;
        last_debounce_time = now;
    }
}
void EXTI0_IRQHandler(void) { if(EXTI->PR & 1) { Button_Handler(0); EXTI->PR = 1; } }
void EXTI1_IRQHandler(void) { if(EXTI->PR & 2) { Button_Handler(1); EXTI->PR = 2; } }
void EXTI3_IRQHandler(void) { if(EXTI->PR & 8) { Button_Handler(3); EXTI->PR = 8; } }
void EXTI4_IRQHandler(void) { if(EXTI->PR & 16) { Button_Handler(4); EXTI->PR = 16; } }
void EXTI15_10_IRQHandler(void) { if(EXTI->PR & (1<<12)) { Button_Handler(2); EXTI->PR = (1<<12); } }

// --- TIMER 3 ---
void TIM3_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    TIM3->PSC = (SystemCoreClock / 1000) - 1; 
    TIM3->ARR = 2000;

    TIM3->EGR |= TIM_EGR_UG; 

    TIM3->SR = 0; 

    TIM3->DIER |= TIM_DIER_UIE; 
    NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_Start(int ms) {
    flag_timeout = 0;

    TIM3->CR1 = 0;                 
    TIM3->ARR = ms;                
    TIM3->CNT = ms;                
    
    TIM3->CR1 |= TIM_CR1_DIR;      
    
    TIM3->SR = 0;                 
    NVIC_ClearPendingIRQ(TIM3_IRQn);

    TIM3->CR1 |= TIM_CR1_CEN;      
}

void TIM3_Stop(void) { TIM3->CR1 = 0; }
void TIM3_IRQHandler(void) { if(TIM3->SR & 1) { TIM3->SR = 0; flag_timeout = 1; TIM3_Stop(); } }

// --- LOGIKA GAME ---
int score = 0;
int level = 1;
int timeLimit = 2000;
int isPlaying = 0;
int currentLed = 0;
int fadeValue = 0;
int fadingIn = 1;
uint32_t lastFadeTime = 0;
const int fadeSpeed = 10;
char lcdBuf[17];

void melodyStart() {
    int melody[] = {NOTE_G4, NOTE_B4, NOTE_D5, NOTE_G5};
    int durations[] = {100, 100, 100, 200};
    for (int i = 0; i < 4; i++) play_tone(melody[i], durations[i]);
}

void melodyCorrect() {
    play_tone(NOTE_E6, 100); delay_real_ms(20); play_tone(NOTE_G5, 100); 
    delay_real_ms(20); play_tone(NOTE_D6, 200); delay_real_ms(50);
}

void melodyGameOver() {
    for(int i=0; i<5; i++) set_led_pwm(i, 255); play_tone(NOTE_D6, 150);
    for(int i=0; i<5; i++) set_led_pwm(i, 0);   delay_real_ms(150);
    for(int i=0; i<5; i++) set_led_pwm(i, 255); play_tone(NOTE_B5, 150); 
    for(int i=0; i<5; i++) set_led_pwm(i, 0);   delay_real_ms(150);
    for(int i=0; i<5; i++) set_led_pwm(i, 255); play_tone(NOTE_G5, 400); 
    for(int i=0; i<5; i++) set_led_pwm(i, 0);   delay_real_ms(300);
}

void gameOverAnim() {
    LCD_Clear();
    LCD_SetCursor(0, 0); LCD_String("GAME OVER!");
    LCD_SetCursor(1, 0); sprintf(lcdBuf, "Skor: %d", score); LCD_String(lcdBuf);
    melodyGameOver();
    for(int i=0; i<5; i++) set_led_pwm(i, 0);
}

// --- MAIN FUNCTION ---
int main(void) {
    System_Init();
    ADC_Init();
    PWM_Init();
    Buzzer_Init();
    EXTI_Init_Custom();
    UART_Init();
    TIM3_Init();
    LCD_Init();

    LCD_SetCursor(0, 0); LCD_String("Tel-U Game");
    LCD_SetCursor(1, 0); LCD_String("Press any Btn...");
    UART_Print("=== SYSTEM READY ===\r\n");
    delay_real_ms(1000);

    flag_button_pressed = -1;
    input_allowed = 1; 

    while (1) {
        if (!isPlaying) {
            if (get_millis() - lastFadeTime >= fadeSpeed) {
                lastFadeTime = get_millis();
                for(int i=0; i<5; i++) if (i != currentLed) set_led_pwm(i, 0);
                if (fadingIn) {
                    fadeValue += 8; if (fadeValue >= 255) { fadeValue = 255; fadingIn = 0; }
                } else {
                    fadeValue -= 8;
                    if (fadeValue <= 0) {
                        fadeValue = 0; fadingIn = 1;
                        currentLed++; if (currentLed > 4) currentLed = 0;
                    }
                }
                set_led_pwm(currentLed, fadeValue);
            }

            if (flag_button_pressed != -1) {
                input_allowed = 0; 
                flag_button_pressed = -1; 

                soundClick();
                LCD_Clear();
                LCD_SetCursor(0, 0); LCD_String("Siap...");
                LCD_SetCursor(1, 0); LCD_String("Mulai!");
                melodyStart();
                delay_real_ms(800);

                score = 0; level = 1; timeLimit = 2000; isPlaying = 1;
                UART_Print("--- GAME START! ---\r\n");
                LCD_Clear();
                LCD_SetCursor(0, 0); LCD_String("Cari LED!");
                LCD_SetCursor(1, 0); LCD_String("Skor: 0");   
            }
        } 
        else {
            input_allowed = 0;
            
            for(int i=0; i<5; i++) set_led_pwm(i, 0);
            ADC1->CR2 |= ADC_CR2_SWSTART;
            while(!(ADC1->SR & ADC_SR_EOC));
            uint32_t adc_noise = ADC1->DR;

            uint32_t entropy = adc_noise ^ TIM3->CNT ^ msTicks;

            int target = entropy % 5;


            int levelBrightness = 100 + (level * 20);
            if(levelBrightness > 255) levelBrightness = 255;

            for (int b = 0; b <= levelBrightness; b += 20) {
                set_led_pwm(target, b); delay_real_ms(30); 
            }
            delay_real_ms(50);

            EXTI->PR = 0xFFFFFFFF; 
            flag_button_pressed = -1; 
            flag_timeout = 0;
            
            input_allowed = 1;
            TIM3_Start(timeLimit);

            int last_rem_display = -1;
            while (flag_timeout == 0 && flag_button_pressed == -1) {
                int remaining = TIM3->CNT;
                if (remaining < 0) remaining = 0;
                
                int display_val = remaining / 100; 
                
                if (display_val != last_rem_display) {
                    last_rem_display = display_val;
                    
                    int sec = remaining / 1000;
                    int ms_part = (remaining % 1000) / 100; 
                    
                    LCD_SetCursor(0, 11); 
                    sprintf(lcdBuf, "%d.%ds ", sec, ms_part);
                    LCD_String(lcdBuf);
                }
            }
            TIM3_Stop();
            input_allowed = 0;
            
            int reactionTime = timeLimit - TIM3->CNT;
            set_led_pwm(target, 0);

            if (flag_button_pressed != -1) {
                if (flag_button_pressed == target) {
                    // BENAR
                    score += 10;
                    if (score % 50 == 0) level++;
                    UART_SendStats(score, level, reactionTime, 1);
                    LCD_SetCursor(1, 0); sprintf(lcdBuf, "Skor: %d Lvl:%d", score, level); LCD_String(lcdBuf);
                    
                    for (int k=0; k<2; k++) {
                        set_led_pwm(target, 255); delay_real_ms(50); 
                        set_led_pwm(target, 0); delay_real_ms(50);
                    }
                    melodyCorrect();
                    if (timeLimit > 300) timeLimit -= 50;
                    delay_real_ms(300);
                } else {
                    // SALAH
                    UART_SendStats(score, level, reactionTime, 0);
                    gameOverAnim();
                    isPlaying = 0;
                }
            } else {
                // TIMEOUT
                UART_SendStats(score, level, timeLimit, 0);
                gameOverAnim();
                isPlaying = 0;
            }
            
            flag_button_pressed = -1;
            
            if(!isPlaying) {
                delay_real_ms(1000);
                LCD_Clear();
                LCD_SetCursor(0, 0); LCD_String("Tel-U Game");
                LCD_SetCursor(1, 0); LCD_String("Play Again?");
            }
            input_allowed = 1; 
        }
    }
}