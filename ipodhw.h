#ifndef _IPODHW_H_
#define _IPODHW_H_

#include "bootloader.h"

#define IPOD_PP5002_LCD_BASE    0xC0001000
#define IPOD_PP5020_LCD_BASE    0x70003000

#define IPOD_PP5002_IDE_PRIMARY_BASE         0xC00031E0
#define IPOD_PP5002_IDE_PRIMARY_CONTROL      0xC00033F8
#define IPOD_PP5020_IDE_PRIMARY_BASE         0xC30001E0
#define IPOD_PP5020_IDE_PRIMARY_CONTROL      0xC30003F8

#define IPOD_LCD_FORMAT_2BPP   0x00
#define IPOD_LCD_FORMAT_RGB565 0x01

/* GPIO Ports for the PP5002 */
#define IPOD_PP5002_GPIO_BASE_ADDR   0xcf000000
#define IPOD_PP5002_GPIOA_ENABLE     0xcf000000
#define IPOD_PP5002_GPIOB_ENABLE     0xcf000004
#define IPOD_PP5002_GPIOC_ENABLE     0xcf000008
#define IPOD_PP5002_GPIOD_ENABLE     0xcf00000c
#define IPOD_PP5002_GPIOA_OUTPUT_EN  0xcf000010
#define IPOD_PP5002_GPIOB_OUTPUT_EN  0xcf000014
#define IPOD_PP5002_GPIOC_OUTPUT_EN  0xcf000018
#define IPOD_PP5002_GPIOD_OUTPUT_EN  0xcf00001c
#define IPOD_PP5002_GPIOA_OUTPUT_VAL 0xcf000020
#define IPOD_PP5002_GPIOB_OUTPUT_VAL 0xcf000024
#define IPOD_PP5002_GPIOC_OUTPUT_VAL 0xcf000028
#define IPOD_PP5002_GPIOD_OUTPUT_VAL 0xcf00002c
#define IPOD_PP5002_GPIOA_INPUT_VAL  0xcf000030
#define IPOD_PP5002_GPIOB_INPUT_VAL  0xcf000034
#define IPOD_PP5002_GPIOC_INPUT_VAL  0xcf000038
#define IPOD_PP5002_GPIOD_INPUT_VAL  0xcf00003c
#define IPOD_PP5002_GPIOA_INT_STAT   0xcf000040
#define IPOD_PP5002_GPIOB_INT_STAT   0xcf000044
#define IPOD_PP5002_GPIOC_INT_STAT   0xcf000048
#define IPOD_PP5002_GPIOD_INT_STAT   0xcf00004c
#define IPOD_PP5002_GPIOA_INT_EN     0xcf000050
#define IPOD_PP5002_GPIOB_INT_EN     0xcf000054
#define IPOD_PP5002_GPIOC_INT_EN     0xcf000058
#define IPOD_PP5002_GPIOD_INT_EN     0xcf00005c
#define IPOD_PP5002_GPIOA_INT_LEV    0xcf000060
#define IPOD_PP5002_GPIOB_INT_LEV    0xcf000064
#define IPOD_PP5002_GPIOC_INT_LEV    0xcf000068
#define IPOD_PP5002_GPIOD_INT_LEV    0xcf00006c
#define IPOD_PP5002_GPIOA_INT_CLR    0xcf000070
#define IPOD_PP5002_GPIOB_INT_CLR    0xcf000074
#define IPOD_PP5002_GPIOC_INT_CLR    0xcf000078
#define IPOD_PP5002_GPIOD_INT_CLR    0xcf00007c

/* GPIO Ports for the PP5020 */
#define IPOD_PP5020_GPIO_BASE_ADDR   0x6000d000
#define IPOD_PP5020_GPIOA_ENABLE     0x6000d000
#define IPOD_PP5020_GPIOB_ENABLE     0x6000d004
#define IPOD_PP5020_GPIOC_ENABLE     0x6000d008
#define IPOD_PP5020_GPIOD_ENABLE     0x6000d00c
#define IPOD_PP5020_GPIOA_OUTPUT_EN  0x6000d010
#define IPOD_PP5020_GPIOB_OUTPUT_EN  0x6000d014
#define IPOD_PP5020_GPIOC_OUTPUT_EN  0x6000d018
#define IPOD_PP5020_GPIOD_OUTPUT_EN  0x6000d01c
#define IPOD_PP5020_GPIOA_OUTPUT_VAL 0x6000d020
#define IPOD_PP5020_GPIOB_OUTPUT_VAL 0x6000d024
#define IPOD_PP5020_GPIOC_OUTPUT_VAL 0x6000d028
#define IPOD_PP5020_GPIOD_OUTPUT_VAL 0x6000d02c
#define IPOD_PP5020_GPIOA_INPUT_VAL  0x6000d030
#define IPOD_PP5020_GPIOB_INPUT_VAL  0x6000d034
#define IPOD_PP5020_GPIOC_INPUT_VAL  0x6000d038
#define IPOD_PP5020_GPIOD_INPUT_VAL  0x6000d03c
#define IPOD_PP5020_GPIOA_INT_STAT   0x6000d040
#define IPOD_PP5020_GPIOB_INT_STAT   0x6000d044
#define IPOD_PP5020_GPIOC_INT_STAT   0x6000d048
#define IPOD_PP5020_GPIOD_INT_STAT   0x6000d04c
#define IPOD_PP5020_GPIOA_INT_EN     0x6000d050
#define IPOD_PP5020_GPIOB_INT_EN     0x6000d054
#define IPOD_PP5020_GPIOC_INT_EN     0x6000d058
#define IPOD_PP5020_GPIOD_INT_EN     0x6000d05c
#define IPOD_PP5020_GPIOA_INT_LEV    0x6000d060
#define IPOD_PP5020_GPIOB_INT_LEV    0x6000d064
#define IPOD_PP5020_GPIOC_INT_LEV    0x6000d068
#define IPOD_PP5020_GPIOD_INT_LEV    0x6000d06c
#define IPOD_PP5020_GPIOA_INT_CLR    0x6000d070
#define IPOD_PP5020_GPIOB_INT_CLR    0x6000d074
#define IPOD_PP5020_GPIOC_INT_CLR    0x6000d078
#define IPOD_PP5020_GPIOD_INT_CLR    0x6000d07c

/* Timers for PP5002 */
#define IPOD_PP5002_TIMER1_CFG       0xcf001100
#define IPOD_PP5002_TIMER1_VAL       0xcf001104
#define IPOD_PP5002_TIMER2_CFG       0xcf001108
#define IPOD_PP5002_TIMER2_VAL       0xcf00110c
#define IPOD_PP5002_USEC_TIMER       0xcf001110
#define IPOD_PP5002_RTC              0xcf001114

/* Timers for PP5020 */
#define IPOD_PP5020_TIMER1_CFG       0x60005000
#define IPOD_PP5020_TIMER1_VAL       0x60005004
#define IPOD_PP5020_TIMER2_CFG       0x60005008
#define IPOD_PP5020_TIMER2_VAL       0x6000500c
#define IPOD_PP5020_USEC_TIMER       0x60005010
#define IPOD_PP5020_RTC              0x60005014

/* Device Controller for the PP5020 */
#define IPOD_PP5020_DEV_RS           0x60006004
#define IPOD_PP5020_DEV_RS2          0x60006008
#define IPOD_PP5020_DEV_EN           0x6000600c
#define IPOD_PP5020_DEV_EN2          0x60006010

typedef struct {
  uint32 hw_rev;
  uint32 lcd_base, lcd_busy_mask;
  uint32 usec_timer;
  uint32 ide_base, ide_control;
  uint32 mem_base, mem_size;
  uint32 iram_base, iram_full_size, iram_user_end;
  int32 lcd_height, lcd_width;
  int16 hw_ver;        // = hw_rev>>16
  uint8 lcd_format;
  uint8 lcd_type;
  uint8 lcd_is_grayscale;
} ipod_t;

void    ipod_init_hardware(void);
ipod_t *ipod_get_hwinfo(void);

#define TIMER_SECOND (1000000)
#define TIMER_MINUTE (60000000)

unsigned long timer_get_current(void);
int timer_passed(unsigned long clock_start, int usecs);
void lcd_wait_ready(void);
void lcd_prepare_cmd(int cmd);
void lcd_send_data(int data_hi, int data_lo);
void lcd_cmd_and_data16(int cmd, uint16 data);
void lcd_cmd_and_data_hi_lo(int cmd, int data_hi, int data_lo);
void lcd_set_contrast(int val);
int lcd_curr_contrast ();
void ipod_set_backlight(int on);
void ipod_reboot (void);
void pcf_standby_mode(void);
void ipod_i2c_init(void);
void ipod_beep(int duration_ms, int period);

#endif
