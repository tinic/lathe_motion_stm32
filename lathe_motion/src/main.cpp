#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

//#define STM32

#ifdef STM32

#ifdef STM32F103xB
#include "stm32f1xx.h"
#endif  // #ifdef STM32F103xB

#ifdef STM32F767xx
#include "stm32f7xx.h"
#endif  // #ifdef STM32F767xx

// led
#define LED_DELAY_MS        128

#ifdef STM32F103xB
#define LED_PORT            GPIOC
#define LED_SET             GPIO_BSRR_BS13
#define LED_RESET           GPIO_BSRR_BR13

#define PIN_ENA_1_Z GPIOB->BSRR = GPIO_BSRR_BS15
#define PIN_ENA_0_Z GPIOB->BSRR = GPIO_BSRR_BR15

#define PIN_PUL_1_Z GPIOB->BSRR = GPIO_BSRR_BS13
#define PIN_PUL_0_Z GPIOB->BSRR = GPIO_BSRR_BR13

#define PIN_DIR_1_Z GPIOB->BSRR = GPIO_BSRR_BS14
#define PIN_DIR_0_Z GPIOB->BSRR = GPIO_BSRR_BR14

#define PIN_ENA_1_X GPIOB->BSRR = GPIO_BSRR_BS15
#define PIN_ENA_0_X GPIOB->BSRR = GPIO_BSRR_BR15

#define PIN_PUL_1_X GPIOB->BSRR = GPIO_BSRR_BS11
#define PIN_PUL_0_X GPIOB->BSRR = GPIO_BSRR_BR11

#define PIN_DIR_1_X GPIOB->BSRR = GPIO_BSRR_BS5
#define PIN_DIR_0_X GPIOB->BSRR = GPIO_BSRR_BR5

#endif   // #ifdef STM32F103xB

#ifdef STM32F767xx
#define LED_PORT            GPIOB
#define LED_SET             GPIO_BSRR_BS_14
#define LED_RESET           GPIO_BSRR_BR_14

#define PIN_ENA_1_Z GPIOE->BSRR = GPIO_BSRR_BS_9
#define PIN_ENA_0_Z GPIOE->BSRR = GPIO_BSRR_BR_9

#define PIN_PUL_1_Z GPIOE->BSRR = GPIO_BSRR_BS_10
#define PIN_PUL_0_Z GPIOE->BSRR = GPIO_BSRR_BR_10

#define PIN_DIR_1_Z GPIOE->BSRR = GPIO_BSRR_BS_11
#define PIN_DIR_0_Z GPIOE->BSRR = GPIO_BSRR_BR_11

#define PIN_ENA_1_X GPIOE->BSRR = GPIO_BSRR_BS_12
#define PIN_ENA_0_X GPIOE->BSRR = GPIO_BSRR_BR_12

#define PIN_PUL_1_X GPIOE->BSRR = GPIO_BSRR_BS_13
#define PIN_PUL_0_X GPIOE->BSRR = GPIO_BSRR_BR_13

#define PIN_DIR_1_X GPIOE->BSRR = GPIO_BSRR_BS_14
#define PIN_DIR_0_X GPIOE->BSRR = GPIO_BSRR_BR_14

#endif  // #ifdef STM32F767xx

static int16_t qep_counter() {
	return TIM3->CNT;
}

static void set_qep_counter(int16_t value) {
	TIM3->CNT = value;
}

#else  // #ifdef STM32

#include <cstdlib>
#include <deque>
#include <iostream>
#include <chrono>
#include <thread>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#define PIN_ENA_1_Z
#define PIN_ENA_0_Z

#define PIN_PUL_1_Z
#define PIN_PUL_0_Z

#define PIN_DIR_1_Z
#define PIN_DIR_0_Z

#define PIN_ENA_1_X
#define PIN_ENA_0_X

#define PIN_PUL_1_X
#define PIN_PUL_0_X

#define PIN_DIR_1_X
#define PIN_DIR_0_X

static void __disable_irq() { }
static void __enable_irq() { }

static int16_t qep_counter_int16 = 0;

static int16_t qep_counter() {
	return qep_counter_int16;
}

static void set_qep_counter(int16_t value) {
	qep_counter_int16 = value;
}

#endif  // #ifdef STM32

static uint32_t parse_hex08(const uint8_t *buf) {
	uint32_t value = 0;
	for (uint32_t c=0; c<8; c++) {
		if (buf[c] >= 0x30 && buf[c] <= 0x39) {
			value |= (buf[c] - 0x30       ) << ((7-c)*4);
		}
		if (buf[c] >= 0x41 && buf[c] <= 0x46) {
			value |= (buf[c] - 0x41 + 0x0A) << ((7-c)*4);
		}
	}
	return value;
}

static void write_hex08(uint8_t *buf, uint32_t value) {
	static const char *ntoh = "0123456789ABCDEF";
	for (uint32_t c=0; c<8; c++) {
		*buf++ = ntoh[(value >> ((7-c)*4)) & 0x0F];
	}
}

static uint32_t gcd_impl(uint32_t u, uint32_t v)
{
    int shift;
    if (u == 0) return v;
    if (v == 0) return u;
    shift = __builtin_ctz(u | v);
    u >>= __builtin_ctz(u);
    do {
        v >>= __builtin_ctz(v);
        if (u > v) {
            uint32_t t = v;
            v = u;
            u = t;
        }
        v = v - u;
    } while (v != 0);
    return u << shift;
}

static int32_t gcd(int32_t u, int32_t v) {
    return int32_t(gcd_impl(abs(u),abs(v)));
}

static inline uint32_t divlu(uint32_t u1, uint32_t u0, uint32_t v) {
	const uint32_t b = 65536;
 
	int32_t s = __builtin_clz(v);

	v = v << s;

	uint32_t vn1 = v >> 16;
	uint32_t vn0 = v & 0xFFFF;

	uint32_t un32 = (u1 << s) | ((u0 >> (32 - s)) & ((-s) >> 31));
	uint32_t un10 = u0 << s;

	uint32_t un1 = un10 >> 16;
	uint32_t un0 = un10 & 0xFFFF;

	uint32_t q1 = un32 / vn1;

	uint32_t rhat = un32 - q1*vn1;

again1:
	if (q1 >= b || q1*vn0 > b*rhat + un1) {
		q1 = q1 - 1;
		rhat = rhat + vn1;
		if (rhat < b) {
			goto again1;
		}
	}

	uint32_t un21 = un32*b + un1 - q1*v;

	uint32_t q0 = un21 / vn1;

	rhat = un21 - q0*vn1;

again2:
	if (q0 >= b || q0*vn0 > b*rhat + un0) {
		q0 = q0 - 1;
		rhat = rhat + vn1;
		if (rhat < b) {	
			goto again2;
		}
	}

	return q1*b + q0;
}

static inline int32_t divls(int32_t u1, uint32_t u0, int32_t v) {
	int32_t uneg = u1 >> 31;
	if (uneg) {
		u0 = -u0;
		int32_t borrow = (u0 != 0);
		u1 = -u1 - borrow;
	}

	int32_t vneg = v >> 31;
	v = (v ^ vneg) - vneg;

	int32_t q = divlu(u1, u0, v);

	int32_t diff = uneg ^ vneg;
	q = (q ^ diff) - diff;
	return q;
}

typedef enum
{
    led_idle = 0, led_on, led_off
} led_state_t;

volatile led_state_t    led_state       = led_idle;
volatile led_state_t    led_state_next  = led_on;
volatile uint16_t       led_delay_count = 0;

//-----------------------------------------------------------------
// controller state, initialized to sane values in init_constants()
//-----------------------------------------------------------------

static const int32_t one_active_axis_timer = 0x200;
static const int32_t all_active_axis_timer = 0x280;

static int32_t absolute_pos = 0;
static int32_t absolute_idx = 0;
static int32_t absolute_tick = 0;

static int32_t current_index_delta = 0;
static int32_t current_rpm = 0; 

static int32_t wait_for_index_zero = 0;
static int32_t index_zero_occured = 0;

static int32_t absolute_pos_start_offset = 0;

static int32_t stepper_target_axis = 0;

static int32_t stepper_actual_pos_z = 0;
static int32_t stepper_end_pos_z = 0;
static int32_t stepper_off_z = 0;

static int32_t stepper_follow_mul_z = 0;
static int32_t stepper_follow_div_z = 0;

static int32_t stepper_actual_pos_x = 0;
static int32_t stepper_end_pos_x = 0;
static int32_t stepper_off_x = 0;

static int32_t stepper_follow_mul_x = 0;
static int32_t stepper_follow_div_x = 0;

static int32_t stepper_actual_pos_d = 0;
static int32_t stepper_end_pos_d = 0;
static int32_t stepper_off_d = 0;

static int32_t stepper_follow_mul_d = 0;
static int32_t stepper_follow_div_d = 0;

static int32_t cycle_counter = 0;

enum run_mode {
    run_mode_idle,
    run_mode_none,
    
    run_mode_follow_z,
    run_mode_follow_x,
    run_mode_follow_d,
    run_mode_follow_zxd,
    
    run_mode_cycle,
    run_mode_cycle_pause
};

static run_mode current_run_mode = run_mode_follow_z;
static run_mode previous_run_mode = run_mode_follow_z;

static const char int2hex[]= {
    '0','1','2','3','4','5','6','7',
    '8','9','A','B','C','D','E','F'
}; 

static uint16_t CRC16(const uint8_t *data, size_t len)
{
    static const uint16_t crcTable[] = {
        0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
        0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
        0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
        0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
        0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
        0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
        0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
        0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
        0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
        0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
        0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
        0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
        0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
        0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
        0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
        0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
        0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
        0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
        0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
        0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
        0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
        0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
        0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
        0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
        0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
        0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
        0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
        0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
        0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
        0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
        0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
        0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040 
    };
    uint16_t crcWord = 0xFFFF;
    while (len--) {
        uint8_t d = *data++ ^ uint8_t(crcWord);
        crcWord >>= 8;
        crcWord ^= crcTable[d];
    }
    return crcWord;
}

#define USART_BAUD 115200
#define USART_BUFFER_SIZE 128

static void put_char(uint8_t byte)
{
#ifdef STM32

#ifdef STM32F103xB
    USART1->DR = (int)(byte);
    while (!(USART1->SR & USART_SR_TXE));
#endif  // #ifdef STM32F103xB

#ifdef STM32F767xx
    USART1->RDR = (int)(byte);
    while (!(USART1->ISR & USART_ISR_TXE));
#endif  // #ifdef STM32F767xx

#else  // #ifdef STM32



#endif  // #ifdef STM32
}

typedef struct {
    uint8_t buffer[USART_BUFFER_SIZE];
    uint8_t head_pos;
    uint8_t tail_pos;
} rx_buffer_t;

volatile rx_buffer_t uart_buffer = { { 0 }, 0, 0 };

static void buffer_char(uint8_t c)
{
    __disable_irq();

    int i = (uart_buffer.head_pos + 1) % USART_BUFFER_SIZE;
    
    if (i != uart_buffer.tail_pos) {
        uart_buffer.buffer[uart_buffer.head_pos] = c;
        uart_buffer.head_pos = i;
    }

    __enable_irq();
}

static uint8_t get_char(uint8_t *c)
{
    __disable_irq();

    if(uart_buffer.head_pos == uart_buffer.tail_pos) {
        __enable_irq();
        return 0;
    }
    
    *c = uart_buffer.buffer[uart_buffer.tail_pos];
    uart_buffer.tail_pos = (uart_buffer.tail_pos + 1) % USART_BUFFER_SIZE;
    
    __enable_irq();
    return 1;
}

volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;

static void parse(void);
static void set_current_run_mode(run_mode mode);

#ifdef __cplusplus
extern "C" {
#endif  // #ifdef __cplusplus

void HardFault_Handler(void)
{
#ifdef STM32
    for (;;) {
        LED_PORT->BSRR  = LED_RESET;
    }
#endif  // #ifdef STM32
}

void BusFault_Handler(void)
{
#ifdef STM32
    for (;;) {
        LED_PORT->BSRR  = LED_RESET;
    }
#endif  // #ifdef STM32
}

void SysTick_Handler(void)
{
    absolute_tick++;
    
    parse();

#ifdef STM32
    led_delay_count = ( (led_delay_count + 1) % LED_DELAY_MS );
    if(led_delay_count == 0) {
        led_state       = led_state_next;
        led_state_next  = led_idle;
    }
    
    IWDG->KR  = 0xAAAA;
#endif  // #ifdef STM32
}

void USART1_IRQHandler(void)
{
#ifdef STM32

#ifdef STM32F103xB
    uint8_t in_char = (USART1->DR & 0xFF);
    buffer_char(in_char);
#endif  // #ifdef STM32F103xB

#ifdef STM32F767xx
    uint8_t in_char = (USART1->RDR & 0xFF);
    buffer_char(in_char);
#endif  // #ifdef STM32F767xx

#endif  // #ifdef STM32
}

void EXTI1_IRQHandler(void)
{
    // Core critical section, nothing shall happen here
    __disable_irq();

    int16_t cnt = qep_counter();
    set_qep_counter(0);

    absolute_pos += cnt;

    if (cnt < 0) {
        absolute_idx--;
    } else {
        absolute_idx++;
    }

#ifdef STM32
    EXTI->PR |= EXTI_PR_PR1;
#endif  // #ifdef STM32

    if (cnt == 2880 || cnt == -2880) {
        index_zero_occured = 1;
    }

    static int32_t last_absolute_tick = 0;
    current_index_delta = abs(absolute_tick - last_absolute_tick);
    last_absolute_tick = absolute_tick;
    __enable_irq();
}

static struct cycle_buffer {

	struct cycle_entry {
		uint8_t target_axs;
		uint8_t wait_for_index_zero;
		uint16_t dummy;
		int32_t target_pos;
		int32_t stepper_mul_z;
		int32_t stepper_div_z;
		int32_t stepper_mul_x;
		int32_t stepper_div_x;
		int32_t stepper_mul_d;
		int32_t stepper_div_d;
	};

	void reset() {
		buf_pos = 0;
		buf_index = 0;
		memset(&entry,0,sizeof(entry));
		next();
	}

	void clear() {
		buf_pos = 0;
		buf_len = 0;
		buf_index = 0;
		buf_index_length = 0;
		memset(&entry,0,sizeof(entry));
	}
	
	const cycle_entry &current() const {
		return entry;
	}
	
	uint32_t index() const {
		return buf_index - 1;
	}

	uint32_t index_length() const {
		return buf_index_length;
	}
	
	bool empty() const {
		return buf_len == 0;
	}
	
	bool atend() const {
		return buf_pos >= buf_len;
	}
	
	bool atlimit() const {
		return buf_len >= (sizeof(buf) - sizeof(entry));
	}
	
	#define TARGET_AXS_MASK 0x03
	#define WAIT_FOR_ZERO   0x04

	#define Z_MULDIV_CHG    0x08
	#define X_MULDIV_CHG    0x10
	#define D_MULDIV_CHG    0x20

	#define Z_MULDIV_MSK    0x03
	#define Z_MULDIV_32_FLG 0x01
	#define Z_MULDIV_24_FLG 0x02
	#define Z_MULDIV_16_FLG 0x03

	#define X_MULDIV_MSK    0x0C
	#define X_MULDIV_32_FLG 0x04
	#define X_MULDIV_24_FLG 0x08
	#define X_MULDIV_16_FLG 0x0C

	#define D_MULDIV_MSK    0x30
	#define D_MULDIV_32_FLG 0x10
	#define D_MULDIV_24_FLG 0x20
	#define D_MULDIV_16_FLG 0x30

	#define T_MSK    		0xC0
	#define T_32_FLG 		0x40
	#define T_24_FLG 		0x80
	#define T_16_FLG 		0xC0
		
	void next() {
		uint8_t flags0 = buf[buf_pos++];
		uint8_t flags1 = buf[buf_pos++];
		
		entry.target_axs = flags0 & TARGET_AXS_MASK;
		entry.wait_for_index_zero = flags0 & WAIT_FOR_ZERO;

		if ((flags1 & T_32_FLG) == T_32_FLG) {
			entry.target_pos = read_int32();
		} else if ((flags1 & T_32_FLG) == T_24_FLG) {
			entry.target_pos = read_int24();
		} else if ((flags1 & T_32_FLG) == T_16_FLG) {
			entry.target_pos = read_int16();
		} else {
			entry.target_pos = 0;
		}

		if ((flags0 & Z_MULDIV_CHG) != 0) {
			if ((flags1 & Z_MULDIV_MSK) == Z_MULDIV_32_FLG) {
				entry.stepper_mul_z = read_int32();
				entry.stepper_div_z = read_int32();
			} else if ((flags1 & Z_MULDIV_MSK) == Z_MULDIV_24_FLG) {
				entry.stepper_mul_z = read_int24();
				entry.stepper_div_z = read_int24();
			} else if ((flags1 & Z_MULDIV_MSK) == Z_MULDIV_16_FLG) {
				entry.stepper_mul_z = read_int16();
				entry.stepper_div_z = read_int16();
			} else {
				entry.stepper_mul_z = 0;
				entry.stepper_div_z = 0;
			}
		}

		if ((flags0 & X_MULDIV_CHG) != 0) {
			if ((flags1 & X_MULDIV_MSK) == X_MULDIV_32_FLG) {
				entry.stepper_mul_x = read_int32();
				entry.stepper_div_x = read_int32();
			} else if ((flags1 & X_MULDIV_MSK) == X_MULDIV_24_FLG) {
				entry.stepper_mul_x = read_int24();
				entry.stepper_div_x = read_int24();
			} else if ((flags1 & X_MULDIV_MSK) == X_MULDIV_16_FLG) {
				entry.stepper_mul_x = read_int16();
				entry.stepper_div_x = read_int16();
			} else {
				entry.stepper_mul_x = 0;
				entry.stepper_div_x = 0;
			}
		}

		if ((flags0 & D_MULDIV_CHG) != 0) {
			if ((flags1 & D_MULDIV_MSK) == D_MULDIV_32_FLG) {
				entry.stepper_mul_d = read_int32();
				entry.stepper_div_d = read_int32();
			} else if ((flags1 & D_MULDIV_MSK) == D_MULDIV_24_FLG) {
				entry.stepper_mul_d = read_int24();
				entry.stepper_div_d = read_int24();
			} else if ((flags1 & D_MULDIV_MSK) == D_MULDIV_16_FLG) {
				entry.stepper_mul_d = read_int16();
				entry.stepper_div_d = read_int16();
			} else {
				entry.stepper_mul_d = 0;
				entry.stepper_div_d = 0;
			}
		}
		
		buf_index ++;
	}

	void push(const cycle_entry &push_entry) {
		uint8_t flags0 = push_entry.target_axs;
		uint8_t flags1 = 0;

		// always changes, so do not optimize change
		if ((abs(push_entry.target_pos) >> 25) != 0) {
			flags1 |= T_32_FLG;
		} else if ((abs(push_entry.target_pos) >> 17) != 0) {
			flags1 |= T_24_FLG;
		} else {
			flags1 |= T_16_FLG;
		}

		if (push_entry.stepper_mul_z != entry.stepper_mul_z ||
			push_entry.stepper_div_z != entry.stepper_div_z ) {
			flags0 |= Z_MULDIV_CHG;
			if (push_entry.stepper_mul_z || push_entry.stepper_div_z) {
				if (((abs(push_entry.stepper_mul_z) | abs(push_entry.stepper_div_z)) >> 25) != 0) {
					flags1 |= Z_MULDIV_32_FLG;
				} else if (((abs(push_entry.stepper_mul_z) | abs(push_entry.stepper_div_z)) >> 17) != 0) {
					flags1 |= Z_MULDIV_24_FLG;
				} else {
					flags1 |= Z_MULDIV_16_FLG;
				}
			}
		}

		if (push_entry.stepper_mul_x != entry.stepper_mul_x ||
			push_entry.stepper_div_x != entry.stepper_div_x ) {
			flags0 |= X_MULDIV_CHG;
			if (push_entry.stepper_mul_x || push_entry.stepper_div_x) {
				if (((abs(push_entry.stepper_mul_x) | abs(push_entry.stepper_div_x)) >> 25) != 0) {
					flags1 |= X_MULDIV_32_FLG;
				} else if (((abs(push_entry.stepper_mul_x) | abs(push_entry.stepper_div_x)) >> 17) != 0) {
					flags1 |= X_MULDIV_24_FLG;
				} else {
					flags1 |= X_MULDIV_16_FLG;
				}
			}
		}
		
		if (push_entry.stepper_mul_d != entry.stepper_mul_d ||
			push_entry.stepper_div_d != entry.stepper_div_d ) {
			flags0 |= D_MULDIV_CHG;
			if (push_entry.stepper_mul_d || push_entry.stepper_div_d) {
				if (((abs(push_entry.stepper_mul_d) | abs(push_entry.stepper_div_d)) >> 25) != 0) {
					flags1 |= X_MULDIV_32_FLG;
				} else if (((abs(push_entry.stepper_mul_d) | abs(push_entry.stepper_div_d)) >> 17) != 0) {
					flags1 |= X_MULDIV_24_FLG;
				} else {
					flags1 |= X_MULDIV_16_FLG;
				}
			}
		}

		flags0 |= push_entry.wait_for_index_zero ? WAIT_FOR_ZERO : 0;
		
		buf[buf_len++] = flags0;
		buf[buf_len++] = flags1;

		if ((flags1 & T_MSK) == T_32_FLG) {
			write_int32(push_entry.target_pos);
		} else if ((flags1 & T_MSK) == T_24_FLG) {
			write_int24(push_entry.target_pos);
		} else if ((flags1 & T_MSK) == T_16_FLG) {
			write_int16(push_entry.target_pos);
		}
		
		if ((flags1 & Z_MULDIV_MSK) == Z_MULDIV_32_FLG) {
			write_int32(push_entry.stepper_mul_z);
			write_int32(push_entry.stepper_div_z);
		} else if ((flags1 & Z_MULDIV_MSK) == Z_MULDIV_24_FLG) {
			write_int24(push_entry.stepper_mul_z);
			write_int24(push_entry.stepper_div_z);
		} else if ((flags1 & Z_MULDIV_MSK) == Z_MULDIV_16_FLG) {
			write_int16(push_entry.stepper_mul_z);
			write_int16(push_entry.stepper_div_z);
		} 
		
		if ((flags1 & X_MULDIV_MSK) == X_MULDIV_32_FLG) {
			write_int32(push_entry.stepper_mul_x);
			write_int32(push_entry.stepper_div_x);
		} else if ((flags1 & X_MULDIV_MSK) == X_MULDIV_24_FLG) {
			write_int24(push_entry.stepper_mul_x);
			write_int24(push_entry.stepper_div_x);
		} else if ((flags1 & X_MULDIV_MSK) == X_MULDIV_16_FLG) {
			write_int16(push_entry.stepper_mul_x);
			write_int16(push_entry.stepper_div_x);
		} 
		
		if ((flags1 & D_MULDIV_MSK) == D_MULDIV_32_FLG) {
			write_int32(push_entry.stepper_mul_d);
			write_int32(push_entry.stepper_div_d);
		} else if ((flags1 & D_MULDIV_MSK) == D_MULDIV_24_FLG) {
			write_int24(push_entry.stepper_mul_d);
			write_int24(push_entry.stepper_div_d);
		} else if ((flags1 & D_MULDIV_MSK) == D_MULDIV_16_FLG) {
			write_int16(push_entry.stepper_mul_d);
			write_int16(push_entry.stepper_div_d);
		} 
		
		entry = push_entry;
		
		buf_index_length++;
	}
	
private:

	int32_t read_int32() {
		int32_t res = int32_t((buf[buf_pos+0] << 24) | 
							  (buf[buf_pos+1] << 16) | 
							  (buf[buf_pos+2] <<  8) | 
							  (buf[buf_pos+3] <<  0)); 
		buf_pos += 4;
		return res;
	}

	int32_t read_int24() {
		int32_t res = int32_t((int32_t(int8_t(buf[buf_pos+0])) << 16) | 
							  (               buf[buf_pos+1]   <<  8) | 
							  (               buf[buf_pos+2]   <<  0)); 
		buf_pos += 3;
		return res;
	}

	int32_t read_int16() {
		int32_t res = int32_t(int16_t((buf[buf_pos+0] <<  8) | 
							          (buf[buf_pos+1] <<  0))); 
		buf_pos += 2;
		return res;
	}
	
	void write_int32(int32_t val) {
		buf[buf_len++] = (uint32_t(val) >> 24) & 0xFF;
		buf[buf_len++] = (uint32_t(val) >> 16) & 0xFF;
		buf[buf_len++] = (uint32_t(val) >>  8) & 0xFF;
		buf[buf_len++] = (uint32_t(val) >>  0) & 0xFF;
	}

	void write_int24(int32_t val) {
		buf[buf_len++] = (uint32_t(val) >> 16) & 0xFF;
		buf[buf_len++] = (uint32_t(val) >>  8) & 0xFF;
		buf[buf_len++] = (uint32_t(val) >>  0) & 0xFF;
	}

	void write_int16(int16_t val) {
		buf[buf_len++] = (uint32_t(val) >>  8) & 0xFF;
		buf[buf_len++] = (uint32_t(val) >>  0) & 0xFF;
	}

	cycle_entry entry;

#ifdef STM32

#ifdef STM32F103xB
	uint8_t buf[16384];
#endif  // #ifdef STM32F103xB

#ifdef STM32F767xx
	uint8_t buf[262144];
#endif  // #ifdef STM32F767xx

#else  // #ifdef STM32
	uint8_t buf[16384];
#endif   // #ifdef STM32

	uint32_t buf_index;
	uint32_t buf_index_length;

	uint32_t buf_pos;
	uint32_t buf_len;
} cycle_buffer;

static inline void calc_step(int32_t &stepper_target_pos, int32_t absolute_actual_pos, int32_t stepper_follow_mul, int32_t stepper_follow_div, int32_t stepper_off) {
	if (stepper_follow_mul == 0) {
		stepper_target_pos = stepper_off;      
	} else {
		int64_t u = int64_t(absolute_actual_pos) * int64_t(stepper_follow_mul);
		int32_t v = stepper_follow_div;
		stepper_target_pos = divls(int32_t(u>>32),uint32_t(u), v); 
		stepper_target_pos += stepper_off;
	}
}

static inline void reload_cycle() {
	auto &e = cycle_buffer.current();
	
	stepper_target_axis = e.target_axs;

	stepper_follow_mul_z = e.stepper_mul_z;
	stepper_follow_mul_x = e.stepper_mul_x;
	stepper_follow_mul_d = e.stepper_mul_d;

	stepper_follow_div_z = e.stepper_div_z;
	stepper_follow_div_x = e.stepper_div_x;
	stepper_follow_div_d = e.stepper_div_d;

	stepper_end_pos_z = 0;
	stepper_end_pos_x = 0;
	stepper_end_pos_d = 0;

	switch(e.target_axs) {
		case 0: {
			stepper_end_pos_z = e.target_pos;
		} break;
		case 1: {
			stepper_end_pos_x = e.target_pos;
		} break;
		case 2: {
			stepper_end_pos_d = e.target_pos;
		} break;
	}

	if (e.wait_for_index_zero) {
		wait_for_index_zero = 1;
		index_zero_occured = 0;
	}
}

void TIM2_IRQHandler(void)
{
    static int32_t stepper_dir_z = 0; // direction is undefined by default
    static int32_t stepper_dir_x = 0; // direction is undefined by default
    static int32_t stepper_dir_d = 0; // direction is undefined by default

#ifdef STM32
    if ((TIM2->SR & TIM_SR_UIF)) {
        TIM2->SR &= ~(TIM_SR_UIF);

        *DWT_CYCCNT = 0;
#endif  // #ifdef STM32

        // Do nothing if we are in stop sync mode
        if (current_run_mode == run_mode_none ||
            current_run_mode == run_mode_idle) {
            return;
        }
        
        // Protect against TIM3 IRQ race
        __disable_irq();
        int16_t cnt = qep_counter();
        int32_t absolute_actual_pos = absolute_pos;

        absolute_actual_pos += cnt - absolute_pos_start_offset;

        if (wait_for_index_zero) {
            if (index_zero_occured) {
                index_zero_occured = 0;
                wait_for_index_zero = 0;
                absolute_pos_start_offset = 0;
                absolute_pos = cnt;
                absolute_actual_pos = 0;
                stepper_off_z = stepper_actual_pos_z;
                stepper_off_x = stepper_actual_pos_x;
                stepper_off_d = stepper_actual_pos_d;
            } else {
                // Do nothing until we hit index zero
                __enable_irq();
                return;
            }
        }
        __enable_irq();

        static uint32_t flip_z = 0;
        static uint32_t flip_x = 0;
        static uint32_t flip_d = 0;

		int32_t stepper_target_pos_z = stepper_actual_pos_z;
		int32_t stepper_target_pos_x = stepper_actual_pos_x;
		int32_t stepper_target_pos_d = stepper_actual_pos_d;

		if (flip_z || flip_x || flip_d) switch (current_run_mode) {
			case    run_mode_follow_zxd: {
						if (flip_z) {
							calc_step(stepper_target_pos_z, absolute_actual_pos, stepper_follow_mul_z, stepper_follow_div_z, stepper_off_z);
						}
						if (flip_x) {
							calc_step(stepper_target_pos_x, absolute_actual_pos, stepper_follow_mul_x, stepper_follow_div_x, stepper_off_x);
						}
						if (flip_d) {
							calc_step(stepper_target_pos_d, absolute_actual_pos, stepper_follow_mul_d, stepper_follow_div_d, stepper_off_d);
						}
					} break;
			case    run_mode_follow_z: {
						if (flip_z) {
							calc_step(stepper_target_pos_z, absolute_actual_pos, stepper_follow_mul_z, stepper_follow_div_z, stepper_off_z);
						}
					} break;
			case    run_mode_follow_x: {
						if (flip_x) {
							calc_step(stepper_target_pos_x, absolute_actual_pos, stepper_follow_mul_x, stepper_follow_div_x, stepper_off_x);
						}
					} break;
			case    run_mode_follow_d: {
						if (flip_d) {
							calc_step(stepper_target_pos_d, absolute_actual_pos, stepper_follow_mul_d, stepper_follow_div_d, stepper_off_d);
						}
					} break;
			case    run_mode_cycle_pause: {
						return;
					} break;
			case    run_mode_cycle: {

						bool do_next_cycle_step = false;
						switch(stepper_target_axis) {
							case 0: {
								if (stepper_follow_mul_z <= 0) {
									if (stepper_actual_pos_z >= stepper_end_pos_z) {
										do_next_cycle_step = true;
									}   
								} else {
									if (stepper_actual_pos_z <= stepper_end_pos_z) {
										do_next_cycle_step = true;
									}   
								}
							} break;
							case 1: {
								if (stepper_follow_mul_x <= 0) {
									if (stepper_actual_pos_x >= stepper_end_pos_x) {
										do_next_cycle_step = true;
									}
								} else {
									if (stepper_actual_pos_x <= stepper_end_pos_x) {
										do_next_cycle_step = true;
									}
								}
							} break;
							case 2: {
								if (stepper_follow_mul_d <= 0) {
									if (stepper_actual_pos_d >= stepper_end_pos_d) {
										do_next_cycle_step = true;
									}
								} else {
									if (stepper_actual_pos_d <= stepper_end_pos_d) {
										do_next_cycle_step = true;
									}
								}
							} break;
						}

						if (do_next_cycle_step) {
							__disable_irq();
							stepper_off_z = stepper_actual_pos_z;
							stepper_off_x = stepper_actual_pos_x;
							stepper_off_d = stepper_actual_pos_d;
							absolute_pos = 0;
							absolute_pos_start_offset = cnt;
							cycle_buffer.next();
							reload_cycle();
							__enable_irq();
							if (cycle_buffer.atend()) {
								cycle_buffer.reset();
                                set_current_run_mode(run_mode_none);
                                return;
                            }
						}

						if (flip_z) {
							calc_step(stepper_target_pos_z, absolute_actual_pos, stepper_follow_mul_z, stepper_follow_div_z, stepper_off_z);
						}
						if (flip_x) {
							calc_step(stepper_target_pos_x, absolute_actual_pos, stepper_follow_mul_x, stepper_follow_div_x, stepper_off_x);
						}
						if (flip_d) {
							calc_step(stepper_target_pos_d, absolute_actual_pos, stepper_follow_mul_d, stepper_follow_div_d, stepper_off_d);
						}

					} break;
			default: {
					} break;
		}
            
        if (flip_z) {
            int32_t szdelta = stepper_target_pos_z - stepper_actual_pos_z;
            if (szdelta < 0) {
                if (stepper_dir_z != -1) {
                    stepper_dir_z = -1;
                    PIN_DIR_0_Z;
                    // wait until next cycle to do the pulse
                } else {
                    stepper_actual_pos_z--;
                    PIN_PUL_1_Z;
                    flip_z ^= 1;
                }
            } else if (szdelta > 0) {
                if (stepper_dir_z != +1) {
                    stepper_dir_z = +1;
                    PIN_DIR_1_Z;        
                    // wait until next cycle to do the pulse
                } else {
                    // now do the actual pulse
                    stepper_actual_pos_z++;
                    PIN_PUL_1_Z;
                    flip_z ^= 1;
                }
            }
        } else {
            PIN_PUL_0_Z;
            flip_z ^= 1;
        }

        if (flip_x) {
            int32_t sxdelta = stepper_target_pos_x - stepper_actual_pos_x;
            if (sxdelta < 0) {
                if (stepper_dir_x != -1) {
                    stepper_dir_x = -1;
                    PIN_DIR_0_X;
                    // wait until next cycle to do the pulse
                } else {
                    stepper_actual_pos_x--;
                    PIN_PUL_1_X;
                    flip_x ^= 1;
                }
            } else if (sxdelta > 0) {
                if (stepper_dir_x != +1) {
                    stepper_dir_x = +1;
                    PIN_DIR_1_X;
                    // wait until next cycle to do the pulse
                } else {
                    // now do the actual pulse
                    stepper_actual_pos_x++;
                    PIN_PUL_1_X;
                    flip_x ^= 1;
                }
            }
        } else {
            PIN_PUL_0_X;
            flip_x ^= 1;
        }

        if (flip_d) {
            int32_t sddelta = stepper_target_pos_d - stepper_actual_pos_d;
            if (sddelta < 0) {
                if (stepper_dir_d != -1) {
                    stepper_dir_d = -1;
                    // wait until next cycle to do the pulse
                } else {
                    stepper_actual_pos_d--;
                    flip_d ^= 1;
                }
            } else if (sddelta > 0) {
                if (stepper_dir_d != +1) {
                    stepper_dir_d = +1;
                    // wait until next cycle to do the pulse
                } else {
                    // now do the actual pulse
                    stepper_actual_pos_d++;
                    flip_d ^= 1;
                }
            }
        } else {
            flip_d ^= 1;
        }
        
#ifdef STM32
        cycle_counter = *DWT_CYCCNT;
    }
#endif  // #ifdef STM32
}

void TIM3_IRQHandler(void)
{
#ifdef STM32
    if ((TIM3->SR & TIM_SR_UIF)) {
        TIM3->SR &= ~(TIM_SR_UIF);
    }
#endif  // #ifdef STM32
}

#ifdef __cplusplus
} // extern "C" {
#endif  // #ifdef __cplusplus

static void set_zero_pos()
{
    // Protect against TIM3 IRQ race
    __disable_irq();
    absolute_pos = 0;
    absolute_pos_start_offset = 0;
    stepper_actual_pos_z = 0;
    stepper_actual_pos_x = 0;
    stepper_actual_pos_d = 0;
    stepper_off_z = 0;
    stepper_off_x = 0;
    stepper_off_d = 0;
    wait_for_index_zero = 1;
    index_zero_occured = 0;
    __enable_irq();
}

static void maybe_enable_steppers()
{
    if (current_run_mode == run_mode_idle) {
        // Z axis
        PIN_ENA_1_Z; // set to high disables driver
        // X axis
        PIN_ENA_1_X; // set to high disables driver
        return;
    }
    if (current_run_mode == run_mode_none) {
        switch (previous_run_mode) {
            case    run_mode_follow_z: {
                        // X axis 
                        PIN_ENA_1_X; // set to high disables driver
                    } break;
            case    run_mode_follow_x: {
                        // Z axis
                        PIN_ENA_1_Z; // set to high disables driver
                    } break;
            case    run_mode_follow_zxd: {
                        // X axis
                        PIN_ENA_1_X; // set to high disables driver
                        //Z axis
                        PIN_ENA_1_Z; // set to high disables driver
                    } break;
            default: {
                        // Z axis
                        PIN_ENA_1_Z; // set to high disables driver
                        // X axis
                        PIN_ENA_1_X; // set to high disables driver
                    } break;
        }
        return;
    }

    switch (current_run_mode) {
        case    run_mode_follow_z: {
                    // X axis
                    PIN_ENA_1_X; // set to high disables driver
                    // Z axis
                    PIN_ENA_0_Z; // set to low enables driver
                } break;
        case    run_mode_follow_x: {
                    // Z axis
                    PIN_ENA_1_Z; // set to high disables driver
                    // X axis
                    PIN_ENA_0_X; // set to low enables driver
                } break;
        case    run_mode_follow_d: {
                    // Z axis
                    PIN_ENA_1_Z; // set to high disables driver
                    // X axis
                    PIN_ENA_1_X; // set to high enables driver
                } break;
        default: {
                    // X axis
                    PIN_ENA_0_X; // set to low enables driver
                    // Z axis
                    PIN_ENA_0_Z; // set to low enables driver
                } break;
    }
}

static void set_current_run_mode(run_mode mode)
{
    if (mode != current_run_mode) {
        __disable_irq();

        int16_t cnt = qep_counter();

        stepper_off_z = stepper_actual_pos_z;
        stepper_off_x = stepper_actual_pos_x;
        stepper_off_d = stepper_actual_pos_d;

        absolute_pos = 0;
        absolute_pos_start_offset = cnt;

        current_run_mode = mode;
        __enable_irq();
    }
    switch (current_run_mode) {
        case    run_mode_follow_z:
        case    run_mode_follow_x:
        case    run_mode_follow_d: {
#ifdef STM32
                    TIM2->ARR = one_active_axis_timer;
#endif  // #ifdef STM32
                } break;
        default: {
#ifdef STM32
                    TIM2->ARR = all_active_axis_timer;
#endif  // #ifdef STM32
                } break;
    }
    maybe_enable_steppers();
}

static void set_follow_values(int32_t axis, int32_t mul, int32_t div)
{
    if (div == 0) {
        return;
    }

    __disable_irq();

	int16_t cnt = qep_counter();

    int32_t gcd_v = gcd(mul, div);
    mul /= gcd_v;
    div /= gcd_v;

    if (axis == 0) {
        stepper_follow_mul_z = mul;
        stepper_follow_div_z = (mul != 0) ? div : 1;

        stepper_off_z = stepper_actual_pos_z;

        absolute_pos = 0;
        absolute_pos_start_offset = cnt;
    }

    if (axis == 1) {
        stepper_follow_mul_x = mul;
        stepper_follow_div_x =(mul != 0) ? div : 1;

        stepper_off_x = stepper_actual_pos_x;

        absolute_pos = 0;
        absolute_pos_start_offset = cnt;
    }

    if (axis == 2) {
        stepper_follow_mul_d = mul;
        stepper_follow_div_d =(mul != 0) ? div : 1;

        stepper_off_d = stepper_actual_pos_d;

        absolute_pos = 0;
        absolute_pos_start_offset = cnt;
    }
                        
    __enable_irq();
}

static void send_bad_crc_response()
{
    const char *response = "BADCRC83AA\n";
    for (size_t c=0; c<11; c++) {
        put_char(response[c]);
    }
}

static void send_invalid_response()
{
    const char *response = "INVALID6E66\n";
    for (size_t c=0; c<12; c++) {
        put_char(response[c]);
    }
}

static void send_ok_response()
{
    const char *response = "OKB775\n";
    for (size_t c=0; c<7; c++) {
        put_char(response[c]);
    }
}

static void parse(void) {
    uint8_t in_char;
    static char buf[128];
    static size_t buf_pos = 0;
    while(get_char(&in_char)) {
        if(in_char != '\n') {
            buf[buf_pos++] = in_char;
            if (buf_pos > 127) {
                buf_pos = 0;
                break;
            }
        } else {
            if (buf_pos<4) {
                send_invalid_response();
                buf_pos = 0;
                break; 
            }

            const size_t size_of_crc = 4;

            // Check CRC16 for every command
            uint16_t crc = CRC16(reinterpret_cast<const uint8_t *>(buf), buf_pos - size_of_crc);
            if ((char(int2hex[((crc>>12)&0xF)]) != buf[buf_pos-4])||
                (char(int2hex[((crc>> 8)&0xF)]) != buf[buf_pos-3])||
                (char(int2hex[((crc>> 4)&0xF)]) != buf[buf_pos-2])||
                (char(int2hex[((crc>> 0)&0xF)]) != buf[buf_pos-1])) {
                send_bad_crc_response();
                buf_pos = 0;
                break;
            }

            // Command valid, execute
            switch(buf[0]) {

                // Set X/Z feed rate
                case 'D':
                case 'X':
                case 'Z': {
                            if (buf_pos < (1 + 16 + size_of_crc)) {
                                send_invalid_response();
                            } else {
                                uint32_t mul = 0;
                                uint32_t div = 0;
								mul = uint32_t(parse_hex08((const uint8_t *)&buf[1]));
								div = uint32_t(parse_hex08((const uint8_t *)&buf[9]));
                                if (div <= 0) {
                                    send_invalid_response();
                                    break;
                                }
                                set_follow_values(buf[0] == 'Z' ? 0 : (buf[0] == 'X' ? 1 : 2), int32_t(mul), int32_t(div));
                                send_ok_response();
                            }
                        } break;

                // Follow mode
                case 'F': {
                            if (buf_pos < (1 + 1 + size_of_crc) || (buf[1] != 'Z' && buf[1] != 'X' && buf[1] != 'D' && buf[1] != 'B')) {
                                send_invalid_response();
                            } else {
                                switch(buf[1]) {
                                    case    'Z': {
                                                set_current_run_mode(run_mode_follow_z);
                                            } break;
                                    case    'X': {
                                                set_current_run_mode(run_mode_follow_x);
                                            } break;
                                    case    'D': {
                                                set_current_run_mode(run_mode_follow_d);
                                            } break;
                                    case    'B': {
                                                set_current_run_mode(run_mode_follow_zxd);
                                            } break;
                                }
                                send_ok_response();
                            }
                        } break;

                // Halt and Idle!
                case 'H': {
                            if (buf_pos < (1 + 1 + size_of_crc) || (buf[1] != '1' && buf[1] != '0')) {
                                send_invalid_response();
                            } else {
                                set_current_run_mode(buf[1] == '1' ? run_mode_idle : run_mode_none);
                                send_ok_response();
                            }
                        } break;
                        
                case 'C': {
                            static int32_t prev_target_z = 0;
                            static int32_t prev_target_x = 0;
                            static int32_t prev_target_d = 0;

                            size_t pos = 1;
                            bool ok_to_run = true;

                            if (buf_pos < (1 + 1 + size_of_crc)) {
                                send_invalid_response();
                                break;
                            }

                            if (buf[pos] != 'X' && 
                                buf[pos] != 'Z' &&
                                buf[pos] != 'C' &&
                                buf[pos] != 'D' &&
                                buf[pos] != 'S' && 
                                buf[pos] != 'R' && 
                                buf[pos] != 'P') {
                                ok_to_run = false;
                                send_invalid_response();
                                break;
                            }

                            if (buf[pos] == 'S') {
                            	cycle_buffer.reset();
                                if (cycle_buffer.empty() ||
                                    cycle_buffer.atend()) {
                                    send_invalid_response();
                                    break;
                                }
                                reload_cycle();
                                set_zero_pos();
                                set_current_run_mode(run_mode_cycle);
                                send_ok_response();
                                break;
                            }

                            if (buf[pos] == 'C') {
                            	if (current_run_mode == run_mode_cycle_pause) {
	                                set_current_run_mode(run_mode_cycle);
	                            }
                                send_ok_response();
                                break;
                            }

                            if (buf[pos] == 'P') {
                            	if (current_run_mode == run_mode_cycle) {
									set_current_run_mode(run_mode_cycle_pause);
								}
                                send_ok_response();
                                break;
                            }

                            set_current_run_mode(run_mode_none);

                            if (buf[pos] == 'R') {
                            	cycle_buffer.clear();
                                prev_target_z = 0;
                                prev_target_x = 0;
                                prev_target_d = 0;
                                send_ok_response();
                                break;
                            }

                            if (buf_pos < (1 + 1 + size_of_crc + 64)) {
                                send_invalid_response();
                                break;
                            }

                            cycle_buffer::cycle_entry entry;
                            entry.target_axs = (buf[pos] == 'X') ? 1 : ( (buf[pos] == 'D') ? 2 : 0); pos++;

                            entry.target_pos = int32_t(parse_hex08((const uint8_t *)&buf[pos])); pos += 8;
                            entry.stepper_mul_z = int32_t(parse_hex08((const uint8_t *)&buf[pos])); pos += 8;
                            entry.stepper_div_z = int32_t(parse_hex08((const uint8_t *)&buf[pos])); pos += 8;
                            entry.stepper_mul_x = int32_t(parse_hex08((const uint8_t *)&buf[pos])); pos += 8;
                            entry.stepper_div_x = int32_t(parse_hex08((const uint8_t *)&buf[pos])); pos += 8;
                            entry.stepper_mul_d = int32_t(parse_hex08((const uint8_t *)&buf[pos])); pos += 8;
                            entry.stepper_div_d = int32_t(parse_hex08((const uint8_t *)&buf[pos])); pos += 8;
                            entry.wait_for_index_zero = int32_t(parse_hex08((const uint8_t *)&buf[pos])); pos += 8;

                            entry.stepper_mul_z = -entry.stepper_mul_z;
                            entry.stepper_mul_x = -entry.stepper_mul_x;
                            entry.stepper_mul_d = -entry.stepper_mul_d;

                            if (entry.stepper_div_z <= 0 ||
                                entry.stepper_div_x <= 0 ||
                                entry.stepper_div_d <= 0) {
                                ok_to_run = false;
                            }

                            int32_t gcd_z = gcd(entry.stepper_mul_z, entry.stepper_div_z); entry.stepper_mul_z /= gcd_z; entry.stepper_div_z /= gcd_z;
                            int32_t gcd_x = gcd(entry.stepper_mul_x, entry.stepper_div_x); entry.stepper_mul_x /= gcd_x; entry.stepper_div_x /= gcd_x;
                            int32_t gcd_d = gcd(entry.stepper_mul_d, entry.stepper_div_d); entry.stepper_mul_d /= gcd_d; entry.stepper_div_d /= gcd_d;

                            switch(entry.target_axs) {
                                case 0: {
                                    if (entry.stepper_mul_z < 0 && entry.target_pos <= prev_target_z) {
                                        ok_to_run = false;
                                    }
                                    if (entry.stepper_mul_z > 0 && entry.target_pos >= prev_target_z) {
                                        ok_to_run = false;
                                    }
                                    if (entry.stepper_mul_z == 0 && entry.target_pos != prev_target_z) {
                                        ok_to_run = false;
                                    }
                                } break;
                                case 1: {
                                    if (entry.stepper_mul_x < 0 && entry.target_pos <= prev_target_x) {
                                        ok_to_run = false;
                                    }
                                    if (entry.stepper_mul_x > 0 && entry.target_pos >= prev_target_x) {
                                        ok_to_run = false;
                                    }
                                    if (entry.stepper_mul_x == 0 && entry.target_pos != prev_target_x) {
                                        ok_to_run = false;
                                    }
                                } break;
                                case 2: {
                                    if (entry.stepper_mul_d < 0 && entry.target_pos <= prev_target_d) {
                                        ok_to_run = false;
                                    }
                                    if (entry.stepper_mul_d > 0 && entry.target_pos >= prev_target_d) {
                                        ok_to_run = false;
                                    }
                                    if (entry.stepper_mul_d == 0 && entry.target_pos != prev_target_d) {
                                        ok_to_run = false;
                                    }
                                } break;
                            }

							switch(entry.target_axs) {
								case 0: {
									prev_target_z = entry.target_pos; 
								} break;
								case 1: {
									prev_target_x = entry.target_pos; 
								} break;
								case 2: {
									prev_target_d = entry.target_pos; 
								} break;
							}

							// push cycle
							cycle_buffer.push(entry);

                            if (cycle_buffer.atlimit()) {
                                ok_to_run = false;
                            }
							
                            if (ok_to_run) {
                                send_ok_response();
                            } else {
                            	cycle_buffer.clear();
								send_invalid_response();
                            }
                        } break;

                // Reset to zero
                case 'R': {
                            set_zero_pos();
                            send_ok_response();
                        } break;

                // Get raw status
                case 'S': {
							int16_t cnt = qep_counter();
                            uint8_t sts[256];
                            size_t pos = 0;
                            write_hex08(&sts[pos], uint32_t(absolute_pos + cnt)); pos += 8; 
                            write_hex08(&sts[pos], uint32_t(stepper_actual_pos_z)); pos += 8; 
                            write_hex08(&sts[pos], uint32_t(stepper_actual_pos_x)); pos += 8; 
                            write_hex08(&sts[pos], uint32_t(stepper_actual_pos_d)); pos += 8; 
                            write_hex08(&sts[pos], uint32_t(cnt)); pos += 8; 
                            write_hex08(&sts[pos], uint32_t(absolute_idx)); pos += 8; 
                            write_hex08(&sts[pos], uint32_t(current_index_delta)); pos += 8; 
                            if (current_run_mode == run_mode_cycle) {
                                write_hex08(&sts[pos], uint32_t(cycle_buffer.current().target_pos)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(cycle_buffer.current().stepper_mul_z)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(cycle_buffer.current().stepper_div_z)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(cycle_buffer.current().stepper_mul_x)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(cycle_buffer.current().stepper_div_x)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(cycle_buffer.current().stepper_mul_d)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(cycle_buffer.current().stepper_div_d)); pos += 8; 
                            } else {
                                write_hex08(&sts[pos], uint32_t(absolute_pos_start_offset)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(stepper_follow_mul_z)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(stepper_follow_div_z)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(stepper_follow_mul_x)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(stepper_follow_div_x)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(stepper_follow_mul_d)); pos += 8; 
                                write_hex08(&sts[pos], uint32_t(stepper_follow_div_d)); pos += 8; 
                            }
                            write_hex08(&sts[pos], uint32_t(cycle_counter)); pos += 8; 
                            write_hex08(&sts[pos], uint32_t((current_run_mode<<16) | (cycle_buffer.index_length()<<8) | (cycle_buffer.index()))); pos += 8; 
                            write_hex08(&sts[pos], uint32_t(absolute_tick)); pos += 8; 

                            uint16_t crc = CRC16(reinterpret_cast<const uint8_t *>(sts), pos);

                            sts[pos++] = uint8_t(int2hex[((crc>>12)&0xF)]);
                            sts[pos++] = uint8_t(int2hex[((crc>> 8)&0xF)]);
                            sts[pos++] = uint8_t(int2hex[((crc>> 4)&0xF)]);
                            sts[pos++] = uint8_t(int2hex[((crc>> 0)&0xF)]);
                            sts[pos++] = '\n';

                            // Send the ascii-fied data over UART
                            for (size_t c=0; c<pos; c++) {
                                put_char(sts[c]);
                            }
                        } break;

                default: {
                            send_invalid_response();
                        } break;

            }
            buf_pos = 0;
            break;
        }
    }
}

static void init_constants() 
{
    absolute_pos = 0;
    absolute_idx = 0;
    absolute_tick = 0;
    current_rpm = 0; 

    absolute_pos_start_offset = 0;

    stepper_actual_pos_z = 0;
    
    stepper_follow_mul_z = -60; // 0.1mm/rev
    stepper_follow_div_z = 1143;
    
    stepper_actual_pos_x = 0;
    
    stepper_follow_mul_x = 60 / 2; // 0.05mm/rev
    stepper_follow_div_x = 1143;

    stepper_actual_pos_d = 0;

    stepper_follow_mul_d = 0;
    stepper_follow_div_d = 1;
}

#ifndef STM32
class systick_timer {
public:
	systick_timer() {
		thread = std::thread([]() {
			while(1) {
			
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				SysTick_Handler();
				static int32_t counter = 0;
				if (++counter == 1000) {
					counter = 0 ;
					printf("%08d %08d %08d\n", stepper_actual_pos_z, stepper_actual_pos_x, stepper_actual_pos_d);
				}
			}
	 	});
	}
private:
	std::thread thread;
};
#endif  // #ifndef STM32

static void init_systick(void)
{
#ifdef STM32

    int tick_time = SystemCoreClock / 1000;  // Generate interrupt each 1 ms
    SysTick_Config(tick_time);               // Configure systick timer

#else  // #ifdef STM32

	static std::shared_ptr<systick_timer> systick_session;
    systick_session = std::make_shared<systick_timer>();

#endif  // #ifdef STM32
}

static void init_clock(void)
{
#ifdef STM32

#ifdef STM32F103xB
    // Conf clock : 72MHz or 128Mhz using HSE 8MHz crystal w/ PLL X 9/16 (8MHz x 9 = 72MHz, 8MHz x 16 = 128MHz)
    FLASH->ACR      |= FLASH_ACR_LATENCY_2; // Two wait states, per datasheet
    RCC->CFGR       |= RCC_CFGR_PPRE1_2;    // prescale AHB1 = HCLK/2
    RCC->CR         |= RCC_CR_HSEON;        // enable HSE clock
    while( !(RCC->CR & RCC_CR_HSERDY) );    // wait for the HSEREADY flag
    
    RCC->CFGR       |= RCC_CFGR_PLLSRC;     // set PLL source to HSE

#if 1 // 128Mhz
    RCC->CFGR       |= RCC_CFGR_PLLMULL16;   // multiply by 9
#else  // #if 1 // 128Mhz
    RCC->CFGR       |= RCC_CFGR_PLLMULL9;   // multiply by 9
#endif  // #if 1 // 128Mhz

    RCC->CR         |= RCC_CR_PLLON;        // enable the PLL
    while( !(RCC->CR & RCC_CR_PLLRDY) );    // wait for the PLLRDY flag
    
    RCC->CFGR       |= RCC_CFGR_SW_PLL;     // set clock source to pll

    while( !(RCC->CFGR & RCC_CFGR_SWS_PLL) );    // wait for PLL to be CLK
    
    SystemCoreClockUpdate();                // calculate the SYSCLOCK value
#endif  // #ifdef STM32F103xB

#ifdef STM32F767xx
    FLASH->ACR      |= FLASH_ACR_LATENCY_4WS; // Four wait states
    while ((FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_4WS);

    RCC->CR         |= RCC_CR_HSEON;        // enable HSE clock
    while( !(RCC->CR & RCC_CR_HSERDY) );    // wait for the HSEREADY flag

	//
	// Set to 216Mhz using HSE 8Mhz crystal w/ PLL
	//
	// Default on startup (96Mhz):
	//
	// PLLP  == 2
	// PLLN  == 192
	// PLLM  == 16
	//
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM); // PLLP = 2
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2; // PLLM = 4
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7; // PLLN = 216;
    RCC->CR |= RCC_CR_PLLON;        // enable the PLL
    while( !(RCC->CR & RCC_CR_PLLRDY) );    // wait for the PLLRDY flag

    RCC->CFGR |= RCC_CFGR_SW_PLL;     // set clock source to pll
    while( !(RCC->CFGR & RCC_CFGR_SWS_PLL) );    // wait for PLL to be CLK

    SystemCoreClockUpdate();                // calculate the SYSCLOCK value
#endif  // #ifdef STM32F767xx
#else  // #ifdef STM32

#endif  // #ifdef STM32
}

static void init_led(void)
{
#ifdef STM32
#ifdef STM32F103xB
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // enable GPIO clock for LED
    LED_PORT->CRH    &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);  // reset pin MODE / CNF
    LED_PORT->CRH    |=  (GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0);    // MODE: 50Mhz ouput CNF: PP
#endif  // #ifdef STM32F103xB

#ifdef STM32F767xx
    RCC->APB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable GPIO clock for LED
#endif  // #ifdef STM32F767xx
#endif  // #ifdef STM32
}

#ifndef STM32
class uart_listener {
public:
	uart_listener() :
		socket(service) ,
		acceptor(service, boost::asio::ip::tcp::endpoint(boost::asio::ip::address_v4::loopback(), 33333)) {
		acceptor.async_accept(socket, boost::bind(&uart_listener::handle_accept, this, boost::asio::placeholders::error));
		thread = std::thread([this]() { service.run(); });
	}
	void handle_accept(const boost::system::error_code& err) {
		uint8_t buf[256] = { 0 };
		while (1) {
			try {
				size_t len = socket.read_some(boost::asio::buffer(buf, sizeof(buf)));
				for (int32_t c = 0; c < len; c++) {
					buffer_char(buf[c]);
				}
			}
			catch(...) {
				return;
			}
		}
	}
private:
	boost::asio::io_service service;
	std::thread thread;
	boost::asio::ip::tcp::socket socket;
	boost::asio::ip::tcp::acceptor acceptor;
};
#endif  // #ifndef STM32

static void init_usart(uint32_t baudrate)
{
#ifdef STM32
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // enable USART1 clock

#ifdef STM32F103xB
    // UART on pins PB6 and PB7
    RCC->APB2ENR    |= RCC_APB2ENR_AFIOEN;  // enable AFIO clock
    RCC->APB2ENR    |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;// enable GPIOA, GPIOB, GPIOC clock

    //use AFIO_MAPR register to remap alternate functions to use USART 1 TX and RX on PB6 and PB7
    //Ref 7.4.2 and 6.3.7 in st manual
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;
    
    GPIOB->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);  // reset PB6
    GPIOB->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);  // reset PB7
    
    GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE6_0;  // 0b11 50MHz output

    GPIOB->CRL |= GPIO_CRL_CNF6_1;    // PB6: output @ 50MHz - Alt-function Push-pull
    GPIOB->CRL |= GPIO_CRL_CNF7_0;    // PB7 RX - Mode = 0b00 (input) - CNF = 0b01 (input floating)
    
#endif  // #ifdef STM32F103xB

#ifdef STM32F767xx
    RCC->APB1ENR |= RCC_AHB1ENR_GPIOAEN; // enable GPIO clock for Port A

    GPIOA->MODER &= ~(GPIO_MODER_MODER9);  // reset PA9
    GPIOA->MODER &= ~(GPIO_MODER_MODER10);  // reset PA10

    GPIOA->MODER |= GPIO_MODER_MODER9_1; // Set to alt mode
    GPIOA->MODER |= GPIO_MODER_MODER10_1; // Set to alt mode

    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH1);
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFRH2);

    GPIOA->AFR[1] |= GPIO_AFRH_AFRH1_0 | GPIO_AFRH_AFRH1_1 | GPIO_AFRH_AFRH1_2; // set to AF7 mode (USART1_TX)
    GPIOA->AFR[1] |= GPIO_AFRH_AFRH2_0 | GPIO_AFRH_AFRH2_1 | GPIO_AFRH_AFRH2_2; // set to AF7 mode (USART1_RX)

#endif  // #ifdef STM32F767xx

    // configure USART1 registers
    uint32_t baud   = (uint32_t)(SystemCoreClock/baudrate);
    USART1->BRR     = baud;
    USART1->CR1     = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    // configure NVIC
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 255);
#else  // #ifdef STM32

	static std::shared_ptr<uart_listener> uart_session;
    uart_session = std::make_shared<uart_listener>();

#endif  // #ifdef STM32
}

#ifndef STM32
class qenc_timer {
public:
	qenc_timer() {
		thread = std::thread([]() {
			const int32_t pulses_per_rev = 2880;
			const int32_t rpm = 500;
			const int32_t usecs = int32_t(1000000./((double(rpm)/60.0)*double(pulses_per_rev)));
			while(1) {
				std::this_thread::sleep_for(std::chrono::microseconds(usecs));
				qep_counter_int16 ++;
				TIM2_IRQHandler();
				if (qep_counter_int16 == pulses_per_rev) {
					EXTI1_IRQHandler();
				}
			}
	 	});
	}

private:
	std::thread thread;
};
#endif  // #ifndef STM32

static void init_qenc()
{
#ifdef STM32
    // setup A/B quadrature on timer 3 periph
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;  // enable TIM3 clock

#ifdef STM32F103xB
   // PA6 - A
   // PA7 - B
   // PA1 - Z
    GPIOA->CRL &= ~(GPIO_CRL_CNF6 | GPIO_CRL_MODE6);  // reset PA6
    GPIOA->CRL &= ~(GPIO_CRL_CNF7 | GPIO_CRL_MODE7);  // reset PA7

    GPIOA->CRL |= GPIO_CRL_CNF6_1;
    GPIOA->CRL |= GPIO_CRL_CNF7_1;
#endif  // #ifdef STM32F103xB

#ifdef STM32F767xx
   // PA6 - A (CN12 -> Pin 13)
   // PA7 - B (CN12 -> Pin 15)
   // PA1 - Z (CN11 -> Pin 30)
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOA->MODER &= ~(GPIO_MODER_MODER6);  // reset PA6
    GPIOA->MODER &= ~(GPIO_MODER_MODER7);  // reset PA7

    GPIOA->MODER |= GPIO_MODER_MODER6_1;
    GPIOA->MODER |= GPIO_MODER_MODER7_1;
    
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL6); 
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL7);

    GPIOA->AFR[0] |= GPIO_AFRL_AFRL6_2;
    GPIOA->AFR[0] |= GPIO_AFRL_AFRL7_2;
#endif  // #ifdef STM32F767xx

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

#ifdef STM32F103xB
    // setup Z/Index on pin PA1
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;  // enable AFIO clock
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI1_PA; // Select EXT1
#endif  // #ifdef STM32F103xB

#ifdef STM32F767xx
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // enable SYCFG clock
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA; // Select EXT1
#endif  // #ifdef STM32F767xx

    EXTI->IMR |= EXTI_IMR_MR1;
    EXTI->FTSR |= EXTI_FTSR_TR1;

    // configure NVIC
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_SetPriority(EXTI1_IRQn, 0); // highest/critical priority in system

#else  // #ifdef STM32

	static std::shared_ptr<qenc_timer> qenc_session;
    qenc_session = std::make_shared<qenc_timer>();

#endif  // #ifdef STM32
}

static void init_stepper_pins()
{
#ifdef STM32
#ifdef STM32F103xB
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN; // Enable GPIOB and GPIOA block

    // Z - enable
    GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
    GPIOB->CRH |= GPIO_CRH_MODE12_1 | GPIO_CRH_MODE12_0;
    PIN_ENA_0_X; // set to low enables driver

    // Z - pulse/step
    GPIOB->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOB->CRH |= GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0;
    PIN_PUL_1_Z;

    // Z - dir
    GPIOB->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14);
    GPIOB->CRH |= GPIO_CRH_MODE14_1 | GPIO_CRH_MODE14_0;
    PIN_DIR_1_Z;

    // X - enable
    GPIOB->CRH &= ~(GPIO_CRH_MODE15 | GPIO_CRH_CNF15);
    GPIOB->CRH |= GPIO_CRH_MODE15_1 | GPIO_CRH_MODE15_0;
    PIN_ENA_0_X; // set to low enables driver

    // X - pulse/step
    GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
    GPIOB->CRH |= GPIO_CRH_MODE11_1 | GPIO_CRH_MODE11_0;
    PIN_PUL_1_X;

    // X - dir
    GPIOB->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    GPIOB->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_MODE5_0;
    PIN_DIR_1_X;
#endif   // #ifdef STM32F103xB

#ifdef STM32F767xx
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    // Z - enable
    GPIOE->MODER &= ~(GPIO_MODER_MODER9); 
    GPIOE->MODER |= GPIO_MODER_MODER9_1; // output mode 
    GPIOE->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR9); 
    GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9_1; // high speed 
    PIN_ENA_0_X; // set to low enables driver

    // Z - pulse/step
    GPIOE->MODER &= ~(GPIO_MODER_MODER10); 
    GPIOE->MODER |= GPIO_MODER_MODER10_1;// output mode 
    GPIOE->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR10); 
    GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR10_1; // high speed 
    PIN_PUL_1_Z;

    // Z - dir
    GPIOE->MODER &= ~(GPIO_MODER_MODER11); 
    GPIOE->MODER |= GPIO_MODER_MODER11_1;// output mode 
    GPIOE->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR11); 
    GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR11_1; // high speed  
    PIN_DIR_1_Z;

    // X - enable
    GPIOE->MODER &= ~(GPIO_MODER_MODER12); 
    GPIOE->MODER |= GPIO_MODER_MODER12_1;// output mode 
    GPIOE->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR12); 
    GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR12_1; // high speed 
    PIN_ENA_0_X; // set to low enables driver

    // X - pulse/step
    GPIOE->MODER &= ~(GPIO_MODER_MODER13);
    GPIOE->MODER |= GPIO_MODER_MODER13_1;// output mode 
    GPIOE->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR13); 
    GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR13_1; // high speed 
    PIN_PUL_1_X;

    // X - dir
    GPIOE->MODER &= ~(GPIO_MODER_MODER14);
    GPIOE->MODER |= GPIO_MODER_MODER14_1;// output mode 
    GPIOE->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR14); 
    GPIOE->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR14_1; // high speed 
    PIN_DIR_1_X;
#endif   // #ifdef STM32F767xx
#endif  // #ifdef STM32
}

static void init_stepper_sync_timer()
{
#ifdef STM32
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // enable TIM2 clock

    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;
    TIM2->PSC = 1;
    TIM2->ARR = one_active_axis_timer; // This seems to be working fine at 1200rpm and 1:1 ratio
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->EGR = TIM_EGR_UG; // Generate Update Event to copy ARR to its shadow
    TIM2->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1); // second highest priority in system
#endif  // #ifdef STM32
}

void init_cycle_counter() {
#ifdef STM32
    volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;
    volatile uint32_t *DWT_LAR = (uint32_t *)0xE0001FB0;
    volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;

    *DEMCR |= 0x01000000;
    *DWT_LAR = 0xC5ACCE55; 
    *DWT_CONTROL |= 1;
#endif  // #ifdef STM32
}

static void init_watchdog(void)
{
#ifdef STM32
    // enable watchdog
    IWDG->KR  = 0xCCCC;
    IWDG->KR  = 0x5555;

    // 1s watchdog
    IWDG->PR  = 1; // 32kHz / 8
    IWDG->RLR = 0xFFF; // *4095 = 1.02375s
#endif  // #ifdef STM32
}

static void init_hardware(void)
{ 
    init_constants();
    init_clock();
    init_cycle_counter();
    init_led();
    init_usart(USART_BAUD);
    init_systick();
    init_qenc();
    init_stepper_pins();
    init_stepper_sync_timer();
    init_watchdog();
}

int main(void) 
{
    init_hardware();
    set_current_run_mode(run_mode_follow_z);
    while (1) {
#ifdef STM32
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
        __WFI();
#else  // #ifdef STM32

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

#endif  // #ifdef STM32
    }
    return 0;
}

