#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
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

static int32_t error_state_out_of_sync = 0;
static int32_t error_index_offset = 0;

static int64_t absolute_pos = 0;
static int64_t absolute_idx = 0;
static int32_t absolute_tick = 0;

static int32_t current_index_delta = 0;
static int32_t current_rpm = 0; 

static int32_t wait_for_index_zero = 0;
static int32_t index_zero_occured = 0;

static int64_t absolute_pos_start_offset = 0;

static int64_t stepper_actual_pos_z = 0;
static int64_t stepper_off_z = 0;

static int32_t stepper_follow_mul_z = 0;
static int32_t stepper_follow_div_z = 0;
static libdivide::libdivide_s64_t stepper_follow_div_z_opt;

static int32_t stepper_jog_dir_z = 0;
static int32_t stepper_jog_tck_z = 0;
static int32_t stepper_jog_tim_z = 0;

static int64_t stepper_actual_pos_x = 0;
static int64_t stepper_off_x = 0;

static int32_t stepper_follow_mul_x = 0;
static int32_t stepper_follow_div_x = 0;
static libdivide::libdivide_s64_t stepper_follow_div_x_opt;

static int32_t stepper_jog_dir_x = 0;
static int32_t stepper_jog_tck_x = 0;
static int32_t stepper_jog_tim_x = 0;

static int32_t cycle_counter = 0;

struct cycle_entry {
    int32_t target_axs;
    int64_t target_pos;
    int32_t stepper_mul_z;
    int32_t stepper_div_z;
    libdivide::libdivide_s64_t stepper_div_z_opt;
    int32_t stepper_mul_x;
    int32_t stepper_div_x;
    libdivide::libdivide_s64_t stepper_div_x_opt;
    int32_t wait_for_index_zero;
};

static cycle_entry current_cycle[64];
static size_t current_cycle_idx = 0;
static size_t current_cycle_len = 0;

enum run_mode {
    run_mode_idle,
    run_mode_none,
    
    run_mode_follow_z,
    run_mode_follow_x,
    run_mode_follow_zx,
    
    run_mode_jog_z,
    run_mode_jog_x,
    run_mode_jog_zx,
    
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
#define USART_BUFFER_SIZE   256

static void put_char(uint8_t byte)
{
    USART1->DR = (int)(byte);
    while (!(USART1->SR & USART_SR_TXE));
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

static void parse(void);

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
    
    parse();

    led_delay_count = ( (led_delay_count + 1) % LED_DELAY_MS );
    if(led_delay_count == 0) {
        led_state       = led_state_next;
        led_state_next  = led_idle;
    }
}

void USART1_IRQHandler(void)
{
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
    
    if (cnt == 2880 || cnt == -2880) {
        index_zero_occured = 1;
    }

    static int32_t last_absolute_tick = 0;
    current_index_delta = abs(absolute_tick - last_absolute_tick);
    last_absolute_tick = absolute_tick;
    __enable_irq();
}

volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;
#define DEMCT_TRCENA 0x01000000
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
#define CYCCNTENA (1<<0)

void TIM2_IRQHandler(void)
{
    static int32_t stepper_dir_x = 0; // direction is undefined by default
    static int32_t stepper_dir_z = 0; // direction is undefined by default

    if ((TIM2->SR & TIM_SR_UIF)) {
        TIM2->SR &= ~(TIM_SR_UIF);

        // Do nothing if we are in stop sync mode
        if (current_run_mode == run_mode_none ||
            current_run_mode == run_mode_idle) {
            return;
        }
        
        // Protect against TIM3 IRQ race
        __disable_irq();
        int16_t cnt = TIM3->CNT;
        int64_t absolute_actual_pos = absolute_pos;

        absolute_actual_pos += cnt - absolute_pos_start_offset;

        *DEMCR |= DEMCT_TRCENA;
        *DWT_CYCCNT = 0;
        *DWT_CONTROL |= CYCCNTENA;

        if (wait_for_index_zero) {
            if (index_zero_occured && current_run_mode != run_mode_cycle_pause) {
                wait_for_index_zero = 0;
                index_zero_occured = 0;
                absolute_pos_start_offset = cnt;
                absolute_pos = 0;
                absolute_actual_pos = 0;
                stepper_actual_pos_z = 0;
                stepper_actual_pos_x = 0;
                stepper_off_z = 0;
                stepper_off_x = 0;
            } else {
                // Do nothing until we hit index zero
                __enable_irq();
                return;
            }
        }
        __enable_irq();

        static uint32_t flip_z = 0;
        if (flip_z) {
            // by default, don't move
            int64_t stepper_target_pos_z = stepper_actual_pos_z;

            switch (current_run_mode) {
                case    run_mode_follow_zx:
                case    run_mode_follow_z: {
                            stepper_target_pos_z = libdivide::libdivide_s64_do(absolute_actual_pos * int64_t(stepper_follow_mul_z), &stepper_follow_div_z_opt);
                            stepper_target_pos_z += stepper_off_z;
                        } break;
                case    run_mode_jog_zx:    
                case    run_mode_jog_z: {
                            stepper_target_pos_z = stepper_actual_pos_z;
                            if (--stepper_jog_tck_z <= 0) {
                                // Reset
                                stepper_jog_tck_z = stepper_jog_tim_z;
                                // Advance
                                if (stepper_jog_dir_z < 0) {
                                    stepper_target_pos_z --;
                                }
                                if (stepper_jog_dir_z > 0) {
                                    stepper_target_pos_z ++;
                                }
                            }
                        } break;
                case    run_mode_cycle_pause:
                case    run_mode_cycle: {
                            int32_t mul_z = current_cycle[current_cycle_idx].stepper_mul_z;
                            const libdivide::libdivide_s64_t &div_z_opt = current_cycle[current_cycle_idx].stepper_div_z_opt;
                            stepper_target_pos_z = libdivide::libdivide_s64_do(absolute_actual_pos * int64_t(mul_z), &div_z_opt);
                            stepper_target_pos_z += stepper_off_z;
                        } break;
                default: {
                        } break;
            }
            
            int32_t szdelta = int32_t(stepper_target_pos_z - stepper_actual_pos_z);

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
            int64_t stepper_target_pos_x = stepper_actual_pos_x;
            
            switch (current_run_mode) {
                case    run_mode_follow_zx:
                case    run_mode_follow_x: {
                            stepper_target_pos_x = libdivide::libdivide_s64_do(absolute_actual_pos * int64_t(stepper_follow_mul_x), &stepper_follow_div_x_opt);
                            stepper_target_pos_x += stepper_off_x;
                        } break;
                case    run_mode_jog_zx:    
                case    run_mode_jog_x: {
                            if (--stepper_jog_tck_x <= 0) {
                                // Reset
                                stepper_jog_tck_x = stepper_jog_tim_x;
                                // Advance
                                if (stepper_jog_dir_x < 0) {
                                    stepper_target_pos_x --;
                                }
                                if (stepper_jog_dir_x > 0) {
                                    stepper_target_pos_x ++;
                                }
                            }
                        } break;
                case    run_mode_cycle_pause:
                case    run_mode_cycle: {
                            int32_t mul_x = current_cycle[current_cycle_idx].stepper_mul_x;
                            const libdivide::libdivide_s64_t &div_x_opt = current_cycle[current_cycle_idx].stepper_div_x_opt;
                            stepper_target_pos_x = libdivide::libdivide_s64_do(absolute_actual_pos * int64_t(mul_x), &div_x_opt);
                            stepper_target_pos_x += stepper_off_x;
                        } break;
                default: {
                        } break;
            }

            int32_t sxdelta = int32_t(stepper_target_pos_x - stepper_actual_pos_x);

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

        if (current_run_mode == run_mode_cycle ||
            current_run_mode == run_mode_cycle_pause) {
            bool do_next_cycle_step = false;
            if (current_cycle[current_cycle_idx].target_axs == 0) {
                if (stepper_actual_pos_z == (int64_t(current_cycle[current_cycle_idx].target_pos) + stepper_off_z)) {
                    stepper_off_z = stepper_actual_pos_z;
                    do_next_cycle_step = true;
                }
            }
            if (current_cycle[current_cycle_idx].target_axs == 1) {
                if (stepper_actual_pos_x == (int64_t(current_cycle[current_cycle_idx].target_pos) + stepper_off_x)) {
                    stepper_off_x = stepper_actual_pos_x;
                    do_next_cycle_step = true;
                }
            }
            if (do_next_cycle_step) {
                __disable_irq();
                absolute_pos = 0;
                absolute_pos_start_offset = cnt;
                current_cycle_idx++;
                if (current_cycle_idx == current_cycle_len) {
                    current_run_mode = run_mode_none;
                }
                if (current_cycle[current_cycle_idx].wait_for_index_zero) {
                    wait_for_index_zero = 1;
                    index_zero_occured = 0;
                }
                __enable_irq();
            }
        }

        cycle_counter = *DWT_CYCCNT;
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

static void set_zero_pos()
{
    // Protect against TIM3 IRQ race
    __disable_irq();
    absolute_pos = 0;
    absolute_pos_start_offset = 0;
    stepper_actual_pos_z = 0;
    stepper_actual_pos_x = 0;
    stepper_off_z = 0;
    stepper_off_x = 0;
    wait_for_index_zero = 1;
    index_zero_occured = 0;
    __enable_irq();
}

static void maybe_enable_steppers()
{
    if (current_run_mode == run_mode_idle) {
        // Z axis
        GPIOB->BSRR = GPIO_BSRR_BS12; // set to high disables driver
        // X axis
        GPIOB->BSRR = GPIO_BSRR_BS15; // set to high disables driver
        return;
    }
    if (current_run_mode == run_mode_none) {
        switch (previous_run_mode) {
            case    run_mode_follow_z: {
                        // X axis
                        GPIOB->BSRR = GPIO_BSRR_BS15; // set to high disables driver
                    } break;
            case    run_mode_follow_x: {
                        // Z axis
                        GPIOB->BSRR = GPIO_BSRR_BS12; // set to high disables driver
                    } break;
            case    run_mode_follow_zx: {
                        // NOP
                    } break;
            default: {
                        // Z axis
                        GPIOB->BSRR = GPIO_BSRR_BS12; // set to high disables driver
                        // X axis
                        GPIOB->BSRR = GPIO_BSRR_BS15; // set to high disables driver
                    } break;
        }
        return;
    }

    switch (current_run_mode) {
        case    run_mode_follow_z: {
                    // X axis
                    GPIOB->BSRR = GPIO_BSRR_BS15; // set to high disables driver
                    // Z axis
                    GPIOB->BSRR = GPIO_BSRR_BR12; // set to low enables driver
                } break;
        case    run_mode_follow_x: {
                    // Z axis
                    GPIOB->BSRR = GPIO_BSRR_BS12; // set to high disables driver
                    // X axis
                    GPIOB->BSRR = GPIO_BSRR_BR15; // set to low enables driver
                } break;
        default: {
                    // X axis
                    GPIOB->BSRR = GPIO_BSRR_BR15; // set to low enables driver
                    // Z axis
                    GPIOB->BSRR = GPIO_BSRR_BR12; // set to low enables driver
                } break;
    }
}

static void set_current_run_mode(run_mode mode)
{
    if (mode != current_run_mode) {
        __disable_irq();
        int16_t cnt = TIM3->CNT;

        stepper_off_z = stepper_actual_pos_z;
        stepper_off_x = stepper_actual_pos_x;

        absolute_pos = 0;
        absolute_pos_start_offset = cnt;

        current_run_mode = mode;
        __enable_irq();
    }
    switch (current_run_mode) {
        case    run_mode_follow_z:
        case    run_mode_follow_x: {
                    TIM2->ARR = 500;
                } break;
        default: {
                    // slow down as we now have two long divides potentially
                    TIM2->ARR = 1000;
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

    int16_t cnt = TIM3->CNT;

    if (axis == 0) {
        stepper_follow_mul_z = mul;
        stepper_follow_div_z = (mul != 0) ? div : 1;
        stepper_follow_div_z_opt = libdivide::libdivide_s64_gen(stepper_follow_div_z);

        stepper_off_z = stepper_actual_pos_z;

        absolute_pos = 0;
        absolute_pos_start_offset = cnt;
    }

    if (axis == 1) {
        stepper_follow_mul_x = mul;
        stepper_follow_div_x =(mul != 0) ? div : 1;
        stepper_follow_div_x_opt = libdivide::libdivide_s64_gen(stepper_follow_div_x);

        stepper_off_x = stepper_actual_pos_x;

        absolute_pos = 0;
        absolute_pos_start_offset = cnt;
    }
                        
    __enable_irq();
}

static void send_bad_crc_response()
{
    const char *response = "BADCRC83AA\n";
    for (size_t c=0; c<7; c++) {
        put_char(response[c]);
    }
}

static void send_invalid_response()
{
    const char *response = "INVALID6E66\n";
    for (size_t c=0; c<7; c++) {
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
    static char buf[256];
    static size_t buf_pos = 0;
    while(get_char(&in_char)) {
        if(in_char != '\n') {
            buf[buf_pos++] = in_char;
            if (buf_pos > 32) {
                buf_pos = 0;
                break;
            }
        } else {
            if (buf_pos<4) {
                send_bad_crc_response();
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
                case 'X':
                case 'Z': {
                            if (buf_pos < (1 + 16 + size_of_crc)) {
                                send_invalid_response();
                            } else {
                                uint32_t mul = 0;
                                uint32_t div = 0;
                                sscanf(&buf[1],"%08x%08x", (unsigned int *)&mul, (unsigned int *)&div);
                                if (div <= 0) {
                                    send_invalid_response();
                                    break;
                                }
                                set_follow_values(buf[0] == 'Z' ? 0 : 1, int32_t(mul), int32_t(div));
                                send_ok_response();
                            }
                        } break;

                // Follow mode
                case 'F': {
                            if (buf_pos < (1 + 1) || (buf[1] != 'Z' && buf[1] != 'X'  && buf[1] != 'B')) {
                                send_invalid_response();
                            } else {
                                switch(buf[1]) {
                                    case    'Z': {
                                                set_current_run_mode(run_mode_follow_z);
                                            } break;
                                    case    'X': {
                                                set_current_run_mode(run_mode_follow_x);
                                            } break;
                                    case    'B': {
                                                set_current_run_mode(run_mode_follow_zx);
                                            } break;
                                }
                                send_ok_response();
                            }
                        } break;

                // Halt and Idle!
                case 'H': {
                            if (buf_pos < (1 + 1) || (buf[1] != '1' && buf[1] != '0')) {
                                send_invalid_response();
                            } else {
                                set_current_run_mode(buf[1] == '1' ? run_mode_idle : run_mode_none);
                                send_ok_response();
                            }
                        } break;
                        
                // Run cycle
                case 'C': {
                            // Set to none while we are parsing
                            set_current_run_mode(run_mode_none);

                            size_t pos = 1;
                            bool ok_to_run = true;
                            int32_t prev_target_pos = 0;

                            current_cycle_len = 0;
                            current_cycle_idx = 0;

                            for ( ; pos < (buf_pos - size_of_crc - 1) ; ) {

                                if ((buf_pos - size_of_crc - pos) < 49) {
                                    ok_to_run = false;
                                    break;
                                }
                                
                                if (buf[pos] != 'X' && buf[pos] != 'Z') {
                                    ok_to_run = false;
                                    break;
                                }

                                cycle_entry entry;
                                entry.target_axs = (buf[pos] == 'X') ? 1 : 0; pos++; // 1
								
                                uint32_t upper;
                                uint32_t lower;
								sscanf(&buf[pos],"%08x%08x",(unsigned int *)&upper, (unsigned int *)&lower); pos += 16;
                                entry.target_pos = (int64_t(upper) << 32) | int64_t(lower);
								sscanf(&buf[pos],"%08x",(unsigned int *)&entry.stepper_mul_z); pos += 8;
								sscanf(&buf[pos],"%08x",(unsigned int *)&entry.stepper_div_z); pos += 8;
								sscanf(&buf[pos],"%08x",(unsigned int *)&entry.stepper_mul_x); pos += 8;
								sscanf(&buf[pos],"%08x",(unsigned int *)&entry.stepper_div_x); pos += 8;
								sscanf(&buf[pos],"%08x",(unsigned int *)&entry.wait_for_index_zero); pos += 8;

                                entry.stepper_div_z_opt = libdivide::libdivide_s64_gen(entry.stepper_div_z);
                                entry.stepper_div_x_opt = libdivide::libdivide_s64_gen(entry.stepper_div_x);
                                
                                if (entry.stepper_div_z < 0 ||
                                    entry.stepper_div_x < 0) {
                                    ok_to_run = false;
                                    break;
                                }
                                
                                // verify
                                if (entry.target_axs == 0) {
                                    if (entry.stepper_mul_z > 0 && prev_target_pos > entry.target_pos) {
                                        ok_to_run = false;
                                        break;
                                    }
                                    if (entry.stepper_mul_z < 0 && prev_target_pos < entry.target_pos) {
                                        ok_to_run = false;
                                        break;
                                    }
                                } else {
                                    if (entry.stepper_mul_x > 0 && prev_target_pos > entry.target_pos) {
                                        ok_to_run = false;
                                        break;
                                    }
                                    if (entry.stepper_mul_x < 0 && prev_target_pos < entry.target_pos) {
                                        ok_to_run = false;
                                        break;
                                    }
                                }
                                prev_target_pos = entry.target_pos;
                                
                                current_cycle[current_cycle_idx++]= entry;
                            }
							
                            if (ok_to_run && current_cycle_idx > 0) {
								current_cycle_len = current_cycle_idx;
								current_cycle_idx = 0;
                                set_current_run_mode(run_mode_cycle);
                                send_ok_response();
                            } else {
								current_cycle_len = 0;
								current_cycle_idx = 0;
								send_invalid_response();
                            }
                        } break;

                // Pause cycle
                case 'P': {
                            if (buf_pos < (1 + 1) || (buf[1] != '1' && buf[1] != '0') || current_run_mode != run_mode_cycle) {
                                send_invalid_response();
                            } else {
                                set_current_run_mode(buf[1] == '1' ? run_mode_cycle_pause : run_mode_cycle);
                                send_ok_response();
                            }
                        } break;
                        
                // Reset to zero
                case 'R': {
                            set_zero_pos();
                            send_ok_response();
                        } break;

                // Get raw status
                case 'S': {
                            int16_t cnt = TIM3->CNT;

                            char sts[256];
                            size_t pos = 0;
                            snprintf(&sts[pos], 32, "%08lX%08lX", uint32_t(uint64_t(absolute_pos + cnt  )>>32),
                                                                  uint32_t(uint64_t(absolute_pos + cnt  ))); pos += 16; // 1
                            snprintf(&sts[pos], 32, "%08lX%08lX", uint32_t(uint64_t(stepper_actual_pos_z + stepper_off_z)>>32),
                                                                  uint32_t(uint64_t(stepper_actual_pos_z + stepper_off_z))); pos += 16; // 2
                            snprintf(&sts[pos], 32, "%08lX%08lX", uint32_t(uint64_t(stepper_actual_pos_x + stepper_off_x)>>32),
                                                                  uint32_t(uint64_t(stepper_actual_pos_x + stepper_off_x))); pos += 16; // 3
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(cnt)); pos += 8; // 4
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(absolute_idx)); pos += 8; // 5
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(current_index_delta)); pos += 8; // 6
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(absolute_pos_start_offset)); pos += 8; // 7
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(stepper_follow_mul_z)); pos += 8; // 8
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(stepper_follow_div_z)); pos += 8; // 9
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(stepper_follow_mul_x)); pos += 8; // 10
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(stepper_follow_div_x)); pos += 8; // 11
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(cycle_counter)); pos += 8; // 12
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(error_index_offset)); pos += 8; // 13
                            snprintf(&sts[pos], 32, "%08lX", uint32_t(absolute_tick)); pos += 8; // 14

                            uint16_t crc = CRC16(reinterpret_cast<const uint8_t *>(sts), pos);

                            sts[pos++] = char(int2hex[((crc>>12)&0xF)]);
                            sts[pos++] = char(int2hex[((crc>> 8)&0xF)]);
                            sts[pos++] = char(int2hex[((crc>> 4)&0xF)]);
                            sts[pos++] = char(int2hex[((crc>> 0)&0xF)]);
                            sts[pos++] = '\n';

                            // Send the ascii-fied data over UART
                            for (size_t c=0; c<pos; c++) {
                                put_char(sts[c]);
                            }

                            // Clear errors
                            if (error_state_out_of_sync) {
                                error_state_out_of_sync = 0;
                            }

                            if (error_index_offset) {
                                error_index_offset = 0;
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
    stepper_follow_div_z_opt = libdivide::libdivide_s64_gen(stepper_follow_div_z);
    
    stepper_jog_dir_z = 0; // direction is undefined by default
    stepper_jog_tck_z = 100;
    stepper_jog_tim_z = 100;

    stepper_actual_pos_x = 0;
    
    stepper_follow_mul_x = 60 / 2; // 0.05mm/rev
    stepper_follow_div_x = 1143;
    stepper_follow_div_x_opt = libdivide::libdivide_s64_gen(stepper_follow_div_x);

    stepper_jog_dir_x = 0; // direction is undefined by default
    stepper_jog_tck_x = 100;
    stepper_jog_tim_x = 100;
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
}

int main(void) 
{
    init_hardware();
    set_current_run_mode(run_mode_follow_z);
    while (1) {
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
    }
    return 0;
}

