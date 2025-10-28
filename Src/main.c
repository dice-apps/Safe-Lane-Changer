//Working code, modification of HAL code
//Operation Successful
//MCU Clock speed set at 8MHz
//PSC TIM2 = 200 - 1
//DC Motor Control Code and Servo

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
#define GPIOA_ODR		(*(volatile unsigned int*)(GPIOA_BASE + 0x0C))

// TIM2 base
#define TIM2_BASE       0x40000000UL
#define TIM2_CR1        (*(volatile unsigned int*)(TIM2_BASE + 0x00))
#define TIM2_CCMR1      (*(volatile unsigned int*)(TIM2_BASE + 0x18))
#define TIM2_CCER       (*(volatile unsigned int*)(TIM2_BASE + 0x20))
#define TIM2_PSC        (*(volatile unsigned int*)(TIM2_BASE + 0x28))
#define TIM2_ARR        (*(volatile unsigned int*)(TIM2_BASE + 0x2C))
#define TIM2_CCR1       (*(volatile unsigned int*)(TIM2_BASE + 0x34))
#define TIM2_EGR        (*(volatile unsigned int*)(TIM2_BASE + 0x14))

// TIM3 base
#define TIM3_BASE		0x40000400
#define TIM3_CR1        (*(volatile unsigned int*)(TIM3_BASE + 0x00))
#define TIM3_CCMR1      (*(volatile unsigned int*)(TIM3_BASE + 0x18))
#define TIM3_CCER       (*(volatile unsigned int*)(TIM3_BASE + 0x20))
#define TIM3_PSC        (*(volatile unsigned int*)(TIM3_BASE + 0x28))
#define TIM3_ARR        (*(volatile unsigned int*)(TIM3_BASE + 0x2C))
#define TIM3_CCR1       (*(volatile unsigned int*)(TIM3_BASE + 0x34))
#define TIM3_EGR        (*(volatile unsigned int*)(TIM3_BASE + 0x14))


// SysTick base
#define SYSTICK_BASE    0xE000E010UL
#define SYSTICK_CTRL    (*(volatile unsigned int*)(SYSTICK_BASE + 0x00))
#define SYSTICK_LOAD    (*(volatile unsigned int*)(SYSTICK_BASE + 0x04))
#define SYSTICK_VAL     (*(volatile unsigned int*)(SYSTICK_BASE + 0x08))

// Alternate Functions
#define AFIO_BASE		0x40010000UL
#define AFIO_MAPR		(*(volatile unsigned int*)(AFIO_BASE + 0x04))

// ========================
// Bit definitions
// ========================

// RCC
/*#define RCC_IOPAEN      (1 << 2)   // GPIOA clock enable
#define RCC_TIM2EN      (1 << 0)   // TIM2 clock enable
*/

// TIM2
/*#define TIM_CR1_CEN     (1 << 0)
#define TIM_CR1_ARPE    (1 << 7)
#define TIM_EGR_UG      (1 << 0)
#define TIM_CCMR1_OC1M_PWM1 (6 << 4)
#define TIM_CCMR1_OC1PE (1 << 3)
#define TIM_CCER_CC1E   (1 << 0)
*/

// SysTick
/*#define SYSTICK_CLKSRC  (1 << 2)
#define SYSTICK_ENABLE  (1 << 0)
#define SYSTICK_COUNTFLAG (1 << 16)
*/
// ========================
// Global variables
// ========================
//volatile uint32_t pwm_val = 0;
//volatile unsigned char direction = 0;

#define RCC_CR      (*(volatile unsigned int*)(RCC_BASE + 0x00))
#define RCC_CFGR    (*(volatile unsigned int*)(RCC_BASE + 0x04))
#define FLASH_ACR   (*(volatile unsigned int*)0x40022000)

/*void clock_init(void) {
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
}*/


// ========================
// Delay using SysTick
// ========================
/*void delay_ms(unsigned int ms)
{
    SYSTICK_LOAD = 72000 - 1;     // 72MHz/1000 = 72k ticks/ms
    SYSTICK_VAL = 0;
    SYSTICK_CTRL = SYSTICK_CLKSRC | SYSTICK_ENABLE;

    for(unsigned int i = 0; i < ms; i++) {
        while(!(SYSTICK_CTRL & SYSTICK_COUNTFLAG));
    }
    SYSTICK_CTRL = 0;
}
*/
void servo_angle(uint8_t angle)
{
	uint16_t pulse = 50 + ((50*angle)/120);

	TIM2_CCR1 = pulse;
}

void delay(uint32_t time)
{
	while(time--);
}

void pin_enable(void)
{
	//PA0-> TIMER2
	//PA6-> TIMER3

	// 1. Enable clocks
    RCC_APB2ENR |= (1 << 2);   // GPIOA clock enable
    RCC_APB1ENR |= (1 << 0);   // TIM2 clock enable for Servo PWM -> 50Hz
    RCC_APB1ENR |= (1 << 1); //TIM3 clock enable for DC motor PWM -> 100kHz
    RCC_APB2ENR |= (1 << 0); //AFIO clock enable, to define AF


    // PA0 = AF Push-Pull (MODE=11, CNF=10), TIM2_CH1
	// PA6 = AF Push-Pull (MODE=11, CNF=10), TIM3_CH1
	// PA2 = GPO Push Pull (MODE=01, CNF=00)
	// PA3 = GPO Push Pull (MODE=01, CNF=00)

    GPIOA_CRL &= ~(0xFFFF << 0); //Clear bits
    GPIOA_CRL |=  (0xB << 0); //Set PA0
    GPIOA_CRL &= ~(0xF << 24); //Set PA6 bits initially to zero
    GPIOA_CRL |= (0xB << 24); //Set PA6
    GPIOA_CRL |= (0x1 << 8); //Set PA2
    GPIOA_CRL |= (0x1 << 12); //Set PA3

    //Setting no re-map for alternate function pins PA0 and PA6
    AFIO_MAPR &= ~(0x3 << 8); //Set TIM1 no re-map (00)
    AFIO_MAPR &= ~(0x3 << 10); //Set TIM3 not re-map (00)
}

void TIM2_init(void)
{
    //Configure TIM2
    TIM2_PSC = 200-1; //Prescaler
    TIM2_ARR = 800-1; //Period
    TIM2_CCR1 = 75; // Duty cycle start/ Reference position could be provided

    TIM2_CCMR1 &= ~(0x7 << 4);
    TIM2_CCMR1 |= (0x6 << 4) | (1 << 3); //(6 << 4) | (1 << 3)
    TIM2_CCER |= (1 << 0); //Set output of pwm as output of corresponding pin

    TIM2_CR1 |= (1 << 7); //Auto reload/ pre-load enable
    TIM2_EGR |= (1 << 0); //Set UG
    TIM2_CR1 |= (1 << 0); //Counter enabled
}

void TIM3_init(void)
{
    //Configure TIM2
    TIM3_PSC = 7; //Pre-scaler
    TIM3_ARR = 1000-1; //Period, TIM3 considered as 8MHz as operated in default SYSCLK of 8MHz
    TIM3_CCR1 = 500; // Duty cycle start (50% duty)/ Reference speed could be provided

    TIM3_CCMR1 &= ~(0x7 << 4);
    TIM3_CCMR1 |= (0x6 << 4) | (1 << 3);
    TIM3_CCER |= (1 << 0); //Set output of pwm as output of corresponding pin

    TIM3_CR1 |= (1 << 7); //Auto reload/ pre-load enable
    TIM3_EGR |= (1 << 0); //Set UG
    TIM3_CR1 |= (1 << 0); //Counter enabled
}



void forward_dir(void)
{
	GPIOA_ODR |= (1 << 2); //PA2 high
	GPIOA_ODR &= ~(1 << 3); //PA3 low
}

void backward_dir(void)
{
	GPIOA_ODR &= ~(1 << 2); //PA2 low
	GPIOA_ODR |= (1 << 3); //PA3 high
}

void stop(void)
{
	GPIOA_ODR &= ~((1 << 2) | (1 << 3)); //PA2 and PA3 low
}

void speed(int speed) //Input 'speed' should be in a range of 0 - 100. Note: Error handling done
{
	int duty;

	if ((speed > 0) | (speed < 90)) //Safe limits are set to the motor, assuming speed 100 is the maximum
	{
		duty = (10 * speed); //This happens with ((1000/100) * speed) where thousand is the total period TIM_ARR and 100 is the range of the speed variable.
		TIM3_CCR1 = duty;
	}
	else if(speed == 0)
	{
		stop();
	}
}

void set_dir_dcmotor(uint8_t dir)
{
	//dir = 1-> Forward direction
	//dir = 0-> Backward direction
	//dir = any-> Stop
	if(dir == 1)
	{
		forward_dir();
	}
	else if(dir == 0)
	{
		backward_dir();
	}
	else stop();
}

// ========================
// Main Program
// ========================
int main(void)
{
    //clock_init();
	pin_enable();
	TIM2_init();
	TIM3_init();







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

        /*//TIM2_CCR1 =100;
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
        //delay(9000000);*/

    	set_dir_dcmotor(1);
    	//servo_angle(100);

    	delay(9000000);

    	set_dir_dcmotor(0);

    	delay(9000000);

    	set_dir_dcmotor(2);

    	delay(9000000);

    	speed(40);
    	set_dir_dcmotor(1);
    	delay(9000000);
    	speed(85);
    	delay(9000000);
    	speed(40);
    	delay(9000000);


    }

    return 0;
}
