//Working code, modification of HAL code
//Operation Successful
//MCU Clock speed set at 8MHz
//PSC = 200 - 1


#include <stdint.h>

// ========================
// STM32F103 Memory Map
// ========================

// RCC base
#define RCC_BASE        0x40021000UL
#define RCC_APB2ENR     (*(volatile unsigned int*)(RCC_BASE + 0x18))
#define RCC_APB1ENR     (*(volatile unsigned int*)(RCC_BASE + 0x1C))

// GPIOA base
#define GPIOA_BASE      0x40010800UL
#define GPIOA_CRL       (*(volatile unsigned int*)(GPIOA_BASE + 0x00))

// TIM2 base
#define TIM2_BASE       0x40000000UL
#define TIM2_CR1        (*(volatile unsigned int*)(TIM2_BASE + 0x00))
#define TIM2_CCMR1      (*(volatile unsigned int*)(TIM2_BASE + 0x18))
#define TIM2_CCER       (*(volatile unsigned int*)(TIM2_BASE + 0x20))
#define TIM2_PSC        (*(volatile unsigned int*)(TIM2_BASE + 0x28))
#define TIM2_ARR        (*(volatile unsigned int*)(TIM2_BASE + 0x2C))
#define TIM2_CCR1       (*(volatile unsigned int*)(TIM2_BASE + 0x34))
#define TIM2_EGR        (*(volatile unsigned int*)(TIM2_BASE + 0x14))

// SysTick base
#define SYSTICK_BASE    0xE000E010UL
#define SYSTICK_CTRL    (*(volatile unsigned int*)(SYSTICK_BASE + 0x00))
#define SYSTICK_LOAD    (*(volatile unsigned int*)(SYSTICK_BASE + 0x04))
#define SYSTICK_VAL     (*(volatile unsigned int*)(SYSTICK_BASE + 0x08))

// ========================
// Bit definitions
// ========================

// RCC
#define RCC_IOPAEN      (1 << 2)   // GPIOA clock enable
#define RCC_TIM2EN      (1 << 0)   // TIM2 clock enable

// TIM2
#define TIM_CR1_CEN     (1 << 0)
#define TIM_CR1_ARPE    (1 << 7)
#define TIM_EGR_UG      (1 << 0)
#define TIM_CCMR1_OC1M_PWM1 (6 << 4)
#define TIM_CCMR1_OC1PE (1 << 3)
#define TIM_CCER_CC1E   (1 << 0)

// SysTick
#define SYSTICK_CLKSRC  (1 << 2)
#define SYSTICK_ENABLE  (1 << 0)
#define SYSTICK_COUNTFLAG (1 << 16)

// ========================
// Global variables
// ========================
volatile uint32_t pwm_val = 0;
volatile unsigned char direction = 0;

#define RCC_CR      (*(volatile unsigned int*)(RCC_BASE + 0x00))
#define RCC_CFGR    (*(volatile unsigned int*)(RCC_BASE + 0x04))
#define FLASH_ACR   (*(volatile unsigned int*)0x40022000)

void clock_init(void) {
    // Enable HSE
    RCC_CR |= (1 << 16);
    while (!(RCC_CR & (1 << 17))); // wait for HSE ready

    // Configure Flash wait states
    FLASH_ACR |= (1 << 4);  // Prefetch enable
    FLASH_ACR &= ~0x7;
    FLASH_ACR |= 0x2;       // 2 wait states for 72MHz

    // Set PLL = HSE * 9
    RCC_CFGR |= (7 << 18);  // PLLMUL = x9
    RCC_CFGR |= (1 << 16);  // PLLSRC = HSE
    RCC_CR |= (1 << 24);    // PLLON
    while (!(RCC_CR & (1 << 25))); // wait for PLL ready

    // Select PLL as system clock
    RCC_CFGR &= ~(3 << 0);
    RCC_CFGR |= (2 << 0);
    while (((RCC_CFGR >> 2) & 0x3) != 0x2); // wait for switch
}


// ========================
// Delay using SysTick
// ========================
void delay_ms(unsigned int ms)
{
    SYSTICK_LOAD = 72000 - 1;     // 72MHz/1000 = 72k ticks/ms
    SYSTICK_VAL = 0;
    SYSTICK_CTRL = SYSTICK_CLKSRC | SYSTICK_ENABLE;

    for(unsigned int i = 0; i < ms; i++) {
        while(!(SYSTICK_CTRL & SYSTICK_COUNTFLAG));
    }
    SYSTICK_CTRL = 0;
}

void servo_angle(uint8_t angle)
{
	uint16_t pulse = 50 + ((50*angle)/120);

	TIM2_CCR1 = pulse;
}

void delay(uint32_t time)
{
	while(time--);
}

// ========================
// Main Program
// ========================
int main(void)
{
    //clock_init();

	// 1. Enable clocks
    RCC_APB2ENR |= RCC_IOPAEN;   // GPIOA clock enable
    RCC_APB1ENR |= RCC_TIM2EN;   // TIM2 clock enable

    // 2. Configure PA0 = Alternate Function Push-Pull (MODE=11, CNF=10)
    GPIOA_CRL &= ~(0xF << (0 * 4));
    GPIOA_CRL |=  (0xB << (0 * 4));

    // 3. Configure TIM2
    TIM2_PSC = 200-1;        // Prescaler
    TIM2_ARR = 800-1;     // Period
    TIM2_CCR1 = 75;        // Duty cycle start/ Reference position could be provided

    TIM2_CCMR1 &= ~(7 << 4);
    TIM2_CCMR1 |= TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE;
    TIM2_CCER |= TIM_CCER_CC1E;

    TIM2_CR1 |= TIM_CR1_ARPE;
    TIM2_EGR |= TIM_EGR_UG;
    TIM2_CR1 |= TIM_CR1_CEN;

    // 4. Main loop: PWM sweep
    while (1) {
        /*if (pwm_val < 1920) {
            direction = 0;
        } else if (pwm_val > 7680) {
            direction = 1;
        }

        if (!direction) {
            pwm_val += 5;
        } else {
            pwm_val -= 5;
        }*/

    	delay(9000000);

        //TIM2_CCR1 =100;
    	servo_angle(0);
        //delay_ms(1);
        delay(9000000);

        //TIM2_CCR1 = 100;
        servo_angle(120);
        //delay_ms(1);
        delay(9000000);

        //TIM2_CCR1 = 60;
        //delay_ms(1);
        servo_angle(50);
        //delay(9000000);

    }

    return 0;
}
