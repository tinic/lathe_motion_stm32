#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include "stm32f1xx.h"
#include "globals.h"
#include "main.h"
#include "libdivide.h"

// led
#define LED_DELAY_MS        128
#define LED_PORT            GPIOC
#define LED_CR              CRH
#define LED_SET             GPIO_BSRR_BS13
#define LED_RESET           GPIO_BSRR_BR13
#define LED_PORT_RESET_BITS GPIO_CRH_MODE13 | GPIO_CRH_CNF13
#define LED_PORT_SET_BITS   GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0
#define LED_CLOCK           RCC_APB2ENR_IOPCEN

// usart defines
#define USART_BAUD          115200
#define USART_BUFFER_SIZE   16

// delay defines
#define USART_DELAY_MS      8
#define POS_DELAY_MS		128

#define PRINT_BUF_LEN 64

typedef enum
{
    led_idle = 0, led_on, led_off
} led_state_t;

volatile led_state_t    led_state       = led_idle;
volatile led_state_t    led_state_next  = led_on;
volatile uint16_t       led_delay_count = 0;

typedef enum
{
    usart_idle = 0, usart_send
} usart_state_t;

typedef struct {
    uint8_t buffer[USART_BUFFER_SIZE];
    uint8_t head_pos;
    uint8_t tail_pos;
} rx_buffer_t;

volatile usart_state_t  usart_state         = usart_idle;
volatile uint16_t       usart_delay_count   = 0;
volatile rx_buffer_t    uart_buffer         = { {0}, 0, 0 };

volatile uint16_t       pos_delay_count   = 0;


//-----------------------------------------------------------------
// controller state, initialized to sane values in init_constants()
//-----------------------------------------------------------------

static int32_t state_steppers_sync_stop = 0;
static int32_t state_steppers_idle = 0;

static int32_t error_state_out_of_sync = 0;
static int32_t error_index_offset = 0;

static int64_t absolute_pos = 0;
static int64_t absolute_idx = 0;
static int32_t absolute_tick = 0;

static int32_t current_index_delta = 0;
static int32_t current_rpm = 0; 

static int64_t absolute_pos_start_offset = 0;

static int64_t stepper_actual_pos_z = 0;
static int32_t stepper_dir_z = 0;
static int32_t stepper_mul_z = 0;
static int32_t stepper_div_z = 0;
static libdivide::libdivide_s64_t stepper_div_z_opt;

static int64_t stepper_actual_pos_x = 0;
static int32_t stepper_dir_x = 0;
static int32_t stepper_mul_x = 0;
static int32_t stepper_div_x = 0;
static libdivide::libdivide_s64_t stepper_div_x_opt;

//-----------------------------------------------------------------

static void put_char(uint8_t byte)
{
    if(byte == '\n') {
        put_char('\r');
    }
    USART1->DR = (int)(byte);
    while (!(USART1->SR & USART_SR_TXE));
}

static int put_line(const char *string)
{
    int count = 0;
    
    while(*string) {
        put_char(*string);
        string++;
        count++;
    }
    put_char('\n');
    
    return(count);
}

static void buffer_char(uint8_t c)
{
    int i = (uart_buffer.head_pos + 1) % USART_BUFFER_SIZE;
    
    if (i != uart_buffer.tail_pos)
    {
        uart_buffer.buffer[uart_buffer.head_pos] = c;
        uart_buffer.head_pos = i;
    }
}

static uint8_t get_char(uint8_t *c)
{
    // if head_pos = tail_pos, there are no characters in buffer
    if(uart_buffer.head_pos == uart_buffer.tail_pos)
        return 0;
    
    // put char from buffer into var pointed to by c
    *c = uart_buffer.buffer[uart_buffer.tail_pos];
    uart_buffer.tail_pos = (uart_buffer.tail_pos + 1) % USART_BUFFER_SIZE;
    
    return 1;
}

static void echo(void)
{
    uint8_t in_char;

    if(get_char(&in_char))
    {
        switch(in_char)
        {
            case 0x0D:
                put_char('\n');
                break;
            default:
                put_char(in_char);
        }
    }
}

static void simple_outputchar(char **str, char c)
{
	if (str) {
		**str = c;
		++(*str);
	} 
	else {
		put_char(c);
	}
}

enum flags {
	PAD_ZERO	= 1,
	PAD_RIGHT	= 2,
};

static int prints(char **out, const char *string, int width, int flags)
{
	int pc = 0, padchar = ' ';

	if (width > 0) {
		int len = 0;
		const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (flags & PAD_ZERO)
			padchar = '0';
	}
	if (!(flags & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			simple_outputchar(out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		simple_outputchar(out, *string);
		++pc;
	}
	for ( ; width > 0; --width) {
		simple_outputchar(out, padchar);
		++pc;
	}

	return pc;
}

static int simple_outputi(char **out, int i, int base, int sign, int width, int flags, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	char *s;
	int t, neg = 0, pc = 0;
	unsigned int u = i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints(out, print_buf, width, flags);
	}

	if (sign && base == 10 && i < 0) {
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {
		t = u % base;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= base;
	}

	if (neg) {
		if( width && (flags & PAD_ZERO) ) {
			simple_outputchar (out, '-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}

	return pc + prints (out, s, width, flags);
}


static int simple_vsprintf(char **out, const char *format, va_list ap)
{
	int width, flags;
	int pc = 0;
	char scr[2];
	union {
		char c;
		char *s;
		int i;
		unsigned int u;
		void *p;
	} u;

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = flags = 0;
			if (*format == '\0')
				break;
			if (*format == '%')
				goto out;
			if (*format == '-') {
				++format;
				flags = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				flags |= PAD_ZERO;
			}
			if (*format == '*') {
				width = va_arg(ap, int);
				format++;
			} else {
				for ( ; *format >= '0' && *format <= '9'; ++format) {
					width *= 10;
					width += *format - '0';
				}
			}
			switch (*format) {
				case('d'):
					u.i = va_arg(ap, int);
					pc += simple_outputi(out, u.i, 10, 1, width, flags, 'a');
					break;

				case('u'):
					u.u = va_arg(ap, unsigned int);
					pc += simple_outputi(out, u.u, 10, 0, width, flags, 'a');
					break;

				case('x'):
					u.u = va_arg(ap, unsigned int);
					pc += simple_outputi(out, u.u, 16, 0, width, flags, 'a');
					break;

				case('X'):
					u.u = va_arg(ap, unsigned int);
					pc += simple_outputi(out, u.u, 16, 0, width, flags, 'A');
					break;

				case('c'):
					u.c = va_arg(ap, int);
					scr[0] = u.c;
					scr[1] = '\0';
					pc += prints(out, scr, width, flags);
					break;

				case('s'):
					u.s = va_arg(ap, char *);
					pc += prints(out, u.s ? u.s : "(null)", width, flags);
					break;
				default:
					break;
			}
		}
		else {
out:
			simple_outputchar (out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	return pc;
}

static int simple_printf(const char *fmt, ...)
{
	va_list ap;
	int r;

	va_start(ap, fmt);
	r = simple_vsprintf(0, fmt, ap);
	va_end(ap);

	return r;
}

static int simple_sprintf(char *buf, const char *fmt, ...)
{
	va_list ap;
	int r;

	va_start(ap, fmt);
	r = simple_vsprintf(&buf, fmt, ap);
	va_end(ap);

	return r;
}

static uint32_t rtc_get_counter_val(void)
{
    return (RTC->CNTH << 16) | RTC->CNTL;
}

#ifdef __cplusplus
extern "C" {
#endif  // #ifdef __cplusplus

void HardFault_Handler(void)
{
	for (;;) {
		LED_PORT->BSRR  = LED_RESET;
	}
}

void BusFault_Handler(void)
{
	for (;;) {
		LED_PORT->BSRR  = LED_RESET;
	}
}

void SysTick_Handler(void)
{
	absolute_tick++;

	echo();

    // update heartbeat LED delay counter and toggle state when needed
    led_delay_count = ( (led_delay_count + 1) % LED_DELAY_MS );
    if(led_delay_count == 0) {
        led_state       = led_state_next;
        led_state_next  = led_idle;
    }

    // update the usart counter and change state if needed
    usart_delay_count = ( (usart_delay_count + 1) % USART_DELAY_MS );
    if(usart_delay_count == 0) {
        usart_state       = usart_send;
    }

    pos_delay_count = ( (pos_delay_count + 1) % POS_DELAY_MS );
    if(pos_delay_count == 0) {
		current_rpm = 60000 / current_index_delta;

		int16_t cnt = TIM3->CNT;
	    simple_printf("g: %d s: %d i: %d r: %d s: %d\n", (int32_t)(absolute_pos + cnt), (int32_t)stepper_actual_pos_z, (int32_t)absolute_idx, current_rpm, (int32_t)SystemCoreClock);

		if (error_state_out_of_sync) {
			simple_printf("error: out of sync!\n");
			error_state_out_of_sync = 0;
		}

		if (error_index_offset) {
		    simple_printf("index error offset %d\n", error_index_offset);
			error_index_offset = 0;
		}
    }
}

void USART1_IRQHandler(void)
{
    if(USART1->SR & USART_SR_ORE) {
        // process overrun error if needed
    }
    
    // get character from data reg
    uint8_t in_char = (USART1->DR & 0xFF);

    buffer_char(in_char);
}

void EXTI1_IRQHandler(void)
{
	// Core critical section, nothing shall happen here
	__disable_irq();
	int16_t cnt = TIM3->CNT;
	TIM3->CNT = 0;

	absolute_pos += cnt;
	if (cnt < 0) {
		absolute_idx--;
	} else {
		absolute_idx++;
	}

	EXTI->PR |= EXTI_PR_PR1;

	if (cnt != 3 && cnt != -3 &&
		cnt != 2880 && cnt != -2880) {
		error_index_offset = (int32_t)cnt;
	}

	static int32_t last_absolute_tick = 0;
	current_index_delta = abs(absolute_tick - last_absolute_tick);
	last_absolute_tick = absolute_tick;
	__enable_irq();
}

void TIM2_IRQHandler(void)
{
	if ((TIM2->SR & TIM_SR_UIF)) {
		TIM2->SR &= ~(TIM_SR_UIF);

		// Do nothing if we are in stop sync mode
		if (state_steppers_sync_stop ||
			state_steppers_idle) {
			return;
		}

		// Protect against TIM3 IRQ race
		__disable_irq();
		int16_t cnt = TIM3->CNT;
		int64_t absolute_actual_pos = absolute_pos;
		__enable_irq();

		absolute_actual_pos +=  cnt - absolute_pos_start_offset;

		static uint32_t flip_z = 0;
		if (flip_z) {
			int64_t stepper_target_pos_z = libdivide::libdivide_s64_do(absolute_actual_pos * stepper_mul_z, &stepper_div_z_opt);
			int32_t szdelta = (int32_t)(stepper_target_pos_z - stepper_actual_pos_z);


			if (abs(szdelta) >= 2880) {
				error_state_out_of_sync = szdelta;
			}

			if (szdelta == 0) {
				// nop
			} else if (szdelta < 0) {
				if (stepper_dir_z != -1) {
					stepper_dir_z = -1;
					GPIOB->BSRR = GPIO_BSRR_BR14;
					// wait until next cycle to do the pulse
				} else {
					stepper_actual_pos_z--;
					GPIOB->BSRR = GPIO_BSRR_BS13;
					flip_z ^= 1;
				}
			} else if (szdelta > 0) {
				if (stepper_dir_z != +1) {
					stepper_dir_z = +1;
					GPIOB->BSRR = GPIO_BSRR_BS14;		
					// wait until next cycle to do the pulse
				} else {
					// now do the actual pulse
					stepper_actual_pos_z++;
					GPIOB->BSRR = GPIO_BSRR_BS13;
					flip_z ^= 1;
				}
			}
		} else {
			GPIOB->BSRR = GPIO_BSRR_BR13;
			flip_z ^= 1;
		}

		static uint32_t flip_x = 0;
		if (flip_x) {
			int64_t stepper_target_pos_x = libdivide::libdivide_s64_do(absolute_actual_pos * stepper_mul_x, &stepper_div_x_opt);
			int32_t sxdelta =  (int32_t)(stepper_target_pos_x - stepper_actual_pos_x);

			if (abs(sxdelta) >= 2880) {
				error_state_out_of_sync = sxdelta;
			}

			if (sxdelta == 0) {
				// nop
			} else if (sxdelta < 0) {
				if (stepper_dir_x != -1) {
					stepper_dir_x = -1;
				    GPIOB->BSRR = GPIO_BSRR_BR5;
					// wait until next cycle to do the pulse
				} else {
					stepper_actual_pos_x--;
					GPIOB->BSRR = GPIO_BSRR_BS11;
					flip_x ^= 1;
				}
			} else if (sxdelta > 0) {
				if (stepper_dir_x != +1) {
					stepper_dir_x = +1;
				    GPIOB->BSRR = GPIO_BSRR_BS5;
					// wait until next cycle to do the pulse
				} else {
					// now do the actual pulse
					stepper_actual_pos_x++;
					GPIOB->BSRR = GPIO_BSRR_BS11;
					flip_x ^= 1;
				}
			}
		} else {
			GPIOB->BSRR = GPIO_BSRR_BR11;
			flip_x ^= 1;
		}
	}
}

void TIM3_IRQHandler(void)
{
	if ((TIM3->SR & TIM_SR_UIF)) {
		TIM3->SR &= ~(TIM_SR_UIF);
	}
}

void TIM4_IRQHandler(void)
{
	if ((TIM4->SR & TIM_SR_UIF)) {
		TIM4->SR &= ~(TIM_SR_UIF);
	}
}

#ifdef __cplusplus
} // extern "C" {
#endif  // #ifdef __cplusplus

static void reset_stepper_offset_to_current_pos()
{
	// Protect against TIM3 IRQ race
	__disable_irq();
	int16_t cnt = TIM3->CNT;
	int64_t absolute_actual_pos = absolute_pos;
	__enable_irq();

	absolute_pos_start_offset = absolute_actual_pos + cnt;
}

static void maybe_enable_steppers()
{
	// disable stepper if no movement in z axis requested
	if (stepper_mul_z == 0) {
		// Z - disable
		GPIOB->BSRR = GPIO_BSRR_BS12; // set to high disables driver
	} else {
		// Z - enable
		GPIOB->BSRR = GPIO_BSRR_BR12; // set to low enables driver
	}

	// disable stepper if no movement in x axis requested
	if (stepper_mul_x == 0) {
		// X - disable
		//GPIOA->BSRR = GPIO_BSRR_BS15; // set to high disables driver
	} else {
		// X - enable
		//GPIOA->BSRR = GPIO_BSRR_BR15; // set to low enables driver
	}
}

static void set_stepper_sync_state(int32_t state)
{
	if (!state) {
		state_steppers_sync_stop = 1;
	} else {
		maybe_enable_steppers();
		// Make sure we go back into sync by resetting our offsets
		reset_stepper_offset_to_current_pos();
		state_steppers_sync_stop = 0;
	}
}

static void set_stepper_idle_state(int32_t state)
{
	// whatever we do we go into stop sync state
	set_stepper_sync_state(0);

	if (state) {
		// Z - disable
	    GPIOB->BSRR = GPIO_BSRR_BS12; // set to high disables driver
		// X - disable
	    //GPIOA->BSRR = GPIO_BSRR_BS15; // set to high disables driver
		state_steppers_idle = 1;
	} else {
		maybe_enable_steppers();
		state_steppers_idle = 0;
	}
}

static void init_constants() 
{
	// start in stop mode
	state_steppers_sync_stop = 1;
	state_steppers_idle = 0;

	absolute_pos = 0;
	absolute_idx = 0;
	absolute_tick = 0;
	current_rpm = 0; 

	absolute_pos_start_offset = 0;

	stepper_actual_pos_z = 0;
	stepper_dir_z = 0; // direction is undefined by default
	stepper_mul_z = 3200 / 4;
	stepper_div_z = 2800;
	stepper_div_z_opt = libdivide::libdivide_s64_gen(stepper_div_z);

	stepper_actual_pos_x = 0;
	stepper_dir_x = 0; // direction is undefined by default
	stepper_mul_x = 3200 / 4;
	stepper_div_x = 2880;
	stepper_div_x_opt = libdivide::libdivide_s64_gen(stepper_div_x);
}

static void init_systick(void)
{
    int tick_time = SystemCoreClock / 1000;  // Generate interrupt each 1 ms
    SysTick_Config(tick_time);               // Configure systick timer
}

static void init_clock(void)
{
    // Conf clock : 72MHz using HSE 8MHz crystal w/ PLL X 9 (8MHz x 9 = 72MHz)
    FLASH->ACR      |= FLASH_ACR_LATENCY_2; // Two wait states, per datasheet
    RCC->CFGR       |= RCC_CFGR_PPRE1_2;    // prescale AHB1 = HCLK/2
    RCC->CR         |= RCC_CR_HSEON;        // enable HSE clock
    while( !(RCC->CR & RCC_CR_HSERDY) );    // wait for the HSEREADY flag
    
    RCC->CFGR       |= RCC_CFGR_PLLSRC;     // set PLL source to HSE
    RCC->CFGR       |= RCC_CFGR_PLLMULL9;   // multiply by 9
    RCC->CR         |= RCC_CR_PLLON;        // enable the PLL
    while( !(RCC->CR & RCC_CR_PLLRDY) );    // wait for the PLLRDY flag
    
    RCC->CFGR       |= RCC_CFGR_SW_PLL;     // set clock source to pll

    while( !(RCC->CFGR & RCC_CFGR_SWS_PLL) );    // wait for PLL to be CLK
    
    SystemCoreClockUpdate();                // calculate the SYSCLOCK value
}

static void init_led(void)
{
    RCC->APB2ENR        |= LED_CLOCK;               // enable GPIO clock for LED
    LED_PORT->LED_CR    &= ~(LED_PORT_RESET_BITS);  // reset pin MODE / CNF
    LED_PORT->LED_CR    |=  (LED_PORT_SET_BITS);    // MODE: 50Mhz ouput CNF: PP
}

static void init_usart(uint32_t baudrate)
{
	// UART on pins PB6 and PB7
	RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN;  // enable AFIO clock
    RCC->APB2ENR    |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;// enable GPIOA, GPIOB, GPIOC clock
    RCC->APB2ENR    |= RCC_APB2ENR_USART1EN;    // enable USART1 clock

    //use AFIO_MAPR register to remap alternate functions to use USART 1 TX and RX on PB6 and PB7
    //Ref 7.4.2 and 6.3.7 in st manual
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
    
    GPIOB->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);  // reset PB6
    GPIOB->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);  // reset PB7
    
    GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0;  // 0b11 50MHz output

    GPIOB->CRL |= GPIO_CRL_CNF6_1;    // PB6: output @ 50MHz - Alt-function Push-pull
    GPIOB->CRL |= GPIO_CRL_CNF7_0;    // PB7 RX - Mode = 0b00 (input) - CNF = 0b01 (input floating)
    
    // configure USART1 registers
    uint32_t baud   = (uint32_t)(SystemCoreClock/baudrate);
    USART1->BRR     = baud;
    USART1->CR1     = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    
    // configure NVIC
    NVIC_EnableIRQ(USART1_IRQn);
	NVIC_SetPriority(USART1_IRQn, 255);
}

static void init_qenc()
{
	// setup A/B quadrature on timer 3 periph

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // enable TIM3 clock

    GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);  // reset PA8
    GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);  // reset PA9

    GPIOA->CRL |= GPIO_CRL_CNF6_1;
    GPIOA->CRL |= GPIO_CRL_CNF7_1;

	TIM3->SR = 0;
	TIM3->CNT = 0;

	TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S); 
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0; // connect TI1

	TIM3->CCMR1 &= ~(TIM_CCMR1_CC2S);
	TIM3->CCMR1 |= TIM_CCMR1_CC2S_0; // connect TI2

	TIM3->CCER &= ~(TIM_CCER_CC1P);  
	TIM3->CCER &= ~(TIM_CCER_CC1NP); 

	TIM3->CCER &= ~(TIM_CCER_CC2P);
	TIM3->CCER &= ~(TIM_CCER_CC2NP); 

	TIM3->SMCR &= ~(TIM_SMCR_SMS);
	TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;

	TIM3->CR1  |= TIM_CR1_CEN; // turn on counter

    NVIC_EnableIRQ(TIM3_IRQn);
	NVIC_SetPriority(TIM3_IRQn, 3);

	// setup Z/Index on pin PA1

	RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN;  // enable AFIO clock

	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA; // Select EXT1

	EXTI->IMR |= EXTI_IMR_MR1;
	EXTI->FTSR |= EXTI_FTSR_TR1;

    // configure NVIC
    NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_SetPriority(EXTI1_IRQn, 0); // highest/critical priority in system
}

static void init_stepper_pins()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN; // Enable GPIOB and GPIOA block

	// Z - enable
	GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
    GPIOB->CRH |= GPIO_CRH_MODE12_1 | GPIO_CRH_MODE12_0;
    GPIOB->BSRR = GPIO_BSRR_BR12; // set to low enables driver

	// Z - pulse/step
	GPIOB->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOB->CRH |= GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0;
    GPIOB->BSRR = GPIO_BSRR_BS13;

	// Z - dir
	GPIOB->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14);
    GPIOB->CRH |= GPIO_CRH_MODE14_1 | GPIO_CRH_MODE14_0;
    GPIOB->BSRR = GPIO_BSRR_BS14;

	// X - enable
	GPIOB->CRH &= ~(GPIO_CRH_MODE15 | GPIO_CRH_CNF15);
    GPIOB->CRH |= GPIO_CRH_MODE15_1 | GPIO_CRH_MODE15_0;
    GPIOB->BSRR = GPIO_BSRR_BR15; // set to low enables driver

	// X - pulse/step
	GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
    GPIOB->CRH |= GPIO_CRH_MODE11_1 | GPIO_CRH_MODE11_0;
    GPIOB->BSRR = GPIO_BSRR_BS11;

	// X - dir
	GPIOB->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    GPIOB->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_MODE5_0;
    GPIOB->BSRR = GPIO_BSRR_BS5;
}

static void init_stepper_sync_timer()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // enable TIM3 clock

    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;
    TIM2->PSC = 1;
    TIM2->ARR = 500; // This seems to be working fine at 1200rpm and 1:1 ratio
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->EGR = TIM_EGR_UG; // Generate Update Event to copy ARR to its shadow
    TIM2->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 1); // second highest priority in system
}

static void init_jog_timer()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  // enable TIM3 clock

    TIM4->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;
    TIM4->PSC = 1;
    TIM4->ARR = 32768;
    TIM4->DIER |= TIM_DIER_UIE;
    TIM4->EGR = TIM_EGR_UG; // Generate Update Event to copy ARR to its shadow
    TIM4->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, 2); // third highest priority in system
}

static void init_hardware(void)
{ 
	init_constants();
    init_clock();
    init_led();
    init_usart(USART_BAUD);
    init_systick();
	init_qenc();
	init_stepper_pins();
	init_stepper_sync_timer();
	init_jog_timer();

	set_stepper_sync_state(1);
}

static void led_heartbeat(void)
{
    switch(led_state)
    {
        case led_on:
            led_state       = led_idle;
            led_state_next  = led_off;
            LED_PORT->BSRR  = LED_SET;
            break;
            
        case led_off:
            led_state       = led_idle;
            led_state_next  = led_on;
            LED_PORT->BSRR  = LED_RESET;
            break;
            
        default:
            break;
    }
}

int main(void) 
{
    init_hardware();
	while (1) {
		led_heartbeat();
		__WFI();
    }
    return 0;
}

