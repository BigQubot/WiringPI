/*
 * wiringPi.h:
 *	Arduino like Wiring library for the Raspberry Pi.
 *	Copyright (c) 2012-2017 Gordon Henderson
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#ifndef	__WIRING_PI_H__
#define	__WIRING_PI_H__

// C doesn't have true/false by default and I can never remember which
//	way round they are, so ...
//	(and yes, I know about stdbool.h but I like capitals for these and I'm old)

#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(!TRUE)
#endif

// GCC warning suppressor

#define	UNU	__attribute__((unused))
#define MAX_PIN_NUM        (0x40)

#define PI_MODEL_A               0
#define PI_MODEL_B               1
#define PI_MODEL_AP              2
#define PI_MODEL_BP              3
#define PI_MODEL_2               4
#define PI_ALPHA                 5
#define PI_MODEL_CM              6
#define PI_MODEL_07              7
#define PI_MODEL_CM3            10
#define PI_MODEL_ZERO_W         12
#define PI_MODEL_3P             13

#define PI_VERSION_1             0
#define PI_VERSION_1_1           1
#define PI_VERSION_1_2           2
#define PI_VERSION_2             3

#define PI_MAKER_SONY            0
#define PI_MAKER_EGOMAN          1
#define PI_MAKER_EMBEST          2
#define PI_MAKER_UNKNOWN         3

/*********** Allwinner H3 *************/
#define H3_GPIO_BASE_ADDR                       0x01C20000U
#define H3_R_GPIO_BASE_ADDR                     0x01F02000U
/*********** Allwinner H3 *************/

/*********** Allwinner H6 *************/
#define H6_GPIO_BASE_ADDR                       0x0300B000U
#define H6_R_GPIO_BASE_ADDR                     0x07022000U
/*********** Allwinner H6 *************/

typedef struct {
	unsigned int gpio_base_addr;
	unsigned int r_gpio_base_addr;
	unsigned int * gpio;
	unsigned int * r_gpio;
	unsigned int gpio_base_offset;
	unsigned int r_gpio_base_offset;
	unsigned int pwm_base_addr;
	unsigned int * pwm;
	unsigned int pwm_ctrl;
	unsigned int pwm_period;
	unsigned int pwm_clk;		// H616
	unsigned int pwm_en;		// H616
	unsigned int pwm_type;		// type:V1 H3/H6, type:V2 H616
	unsigned int pwm_bit_en; 	// SUNXI_PWM_CH0_EN
	unsigned int pwm_bit_act;	// SUNXI_PWM_CH0_ACT_STA
	unsigned int pwm_bit_sclk;	// SUNXI_PWM_SCLK_CH0_GATING
	unsigned int pwm_bit_mode;	// SUNXI_PWM_CH0_MS_MODE
	unsigned int pwm_bit_pulse;	// SUNXI_PWM_CH0_PUL_START
} sunxi_gpio_info;
#define GPIO_PWM                                GPIO_PWM_OP

//sunxi_pwm
#ifdef BPI
#define SUNXI_PWM_BASE                          (0x01c21400)
#define SUNXI_PWM_CTRL_REG                      (SUNXI_PWM_BASE)
#define SUNXI_PWM_CH0_PERIOD                    (SUNXI_PWM_BASE + 0x4)
#define SUNXI_PWM_CH1_PERIOD                    (SUNXI_PWM_BASE + 0x8)

#define SUNXI_PWM_CH0_EN                        (1 << 4)
#define SUNXI_PWM_CH0_ACT_STA                   (1 << 5)
#define SUNXI_PWM_SCLK_CH0_GATING               (1 << 6)
#define SUNXI_PWM_CH0_MS_MODE                   (1 << 7)    //pulse mode
#define SUNXI_PWM_CH0_PUL_START                 (1 << 8)

#else
#define SUNXI_PWM_BASE                          (sunxi_gpio_info_t.pwm_base_addr)
#define SUNXI_PWM_CTRL_REG                      (sunxi_gpio_info_t.pwm_ctrl)
#define SUNXI_PWM_PERIOD                        (sunxi_gpio_info_t.pwm_period)
#define SUNXI_PWM_CLK_REG                       (sunxi_gpio_info_t.pwm_clk)    // H616
#define SUNXI_PWM_EN_REG                        (sunxi_gpio_info_t.pwm_en)    // H616
#define SUNXI_PWM_TYPE                          (sunxi_gpio_info_t.pwm_type)
#define SUNXI_PWM_EN                            (sunxi_gpio_info_t.pwm_bit_en)
#define SUNXI_PWM_ACT_STA                       (sunxi_gpio_info_t.pwm_bit_act)
#define SUNXI_PWM_SCLK_GATING                   (sunxi_gpio_info_t.pwm_bit_sclk)
#define SUNXI_PWM_MS_MODE                       (sunxi_gpio_info_t.pwm_bit_mode)    //pulse mode
#define SUNXI_PWM_PUL_START                     (sunxi_gpio_info_t.pwm_bit_pulse)
#endif

#define H3_PWM_BASE                             (0x01c21400)
#define H6_PWM_BASE                             (0x0300A000)
#define H616_PWM_BASE                           (0x0300A000)

#define SUNXI_V1_PWM_TYPE                       (1)
#define SUNXI_V2_PWM_TYPE                       (2)

#define SUNXI_V1_PWM_EN_REG                     (SUNXI_PWM_BASE + 0x0)
#define SUNXI_V1_PWM_CLK_REG                    (SUNXI_PWM_BASE + 0x0)
#define SUNXI_V1_PWM_CTRL_REG                   (SUNXI_PWM_BASE + 0x0)
#define SUNXI_V1_PWM_CH0_PERIOD                 (SUNXI_PWM_BASE + 0x4)

#define SUNXI_V1_PWM_CH0_EN                     (1 << 4)
#define SUNXI_V1_PWM_CH0_ACT_STA                (1 << 5)
#define SUNXI_V1_PWM_SCLK_CH0_GATING            (1 << 6)
#define SUNXI_V1_PWM_CH0_MS_MODE                (1 << 7)    //pulse mode
#define SUNXI_V1_PWM_CH0_PUL_START              (1 << 8)

//-----------------------------------------------------
//SUNXI_V2_PWM Fixed parameters
#define SUNXI_V2_PWM_EN_REG                     (SUNXI_PWM_BASE + 0x40)
#define SUNXI_V2_PWM_ACT_STA                    (1 << 8)
#define SUNXI_V2_PWM_SCLK_GATING                (1 << 4)
#define SUNXI_V2_PWM_MS_MODE                    (1 << 9)
#define SUNXI_V2_PWM_PUL_START                  (1 << 10)

//SUNXI_V2_PWM Variable parameters
#define SUNXI_V2_PWM1_PERIOD                    (SUNXI_PWM_BASE + 0x84)
#define SUNXI_V2_PWM1_CTRL_REG                  (SUNXI_PWM_BASE + 0x80)
#define SUNXI_V2_PWM1_CLK_REG                   (SUNXI_PWM_BASE + 0x20)
#define SUNXI_V2_PWM1_EN                        (1 << 1)

#define SUNXI_V2_PWM2_PERIOD                    (SUNXI_PWM_BASE + 0xA4)
#define SUNXI_V2_PWM2_CTRL_REG                  (SUNXI_PWM_BASE + 0xA0)
#define SUNXI_V2_PWM2_CLK_REG                   (SUNXI_PWM_BASE + 0x24)
#define SUNXI_V2_PWM2_EN                        (1 << 2)

#define SUNXI_V2_PWM3_PERIOD                    (SUNXI_PWM_BASE + 0xC4)
#define SUNXI_V2_PWM3_CTRL_REG                  (SUNXI_PWM_BASE + 0xC0)
#define SUNXI_V2_PWM3_CLK_REG                   (SUNXI_PWM_BASE + 0x24)
#define SUNXI_V2_PWM3_EN                        (1 << 3)

#define SUNXI_V2_PWM4_PERIOD                    (SUNXI_PWM_BASE + 0xE4)
#define SUNXI_V2_PWM4_CTRL_REG                  (SUNXI_PWM_BASE + 0xE0)
#define SUNXI_V2_PWM4_CLK_REG                   (SUNXI_PWM_BASE + 0x28)
#define SUNXI_V2_PWM4_EN                        (1 << 4)

#define PWM_CLK_DIV_120  0
#define PWM_CLK_DIV_180  1
#define PWM_CLK_DIV_240  2
#define PWM_CLK_DIV_360  3
#define PWM_CLK_DIV_480  4
#define PWM_CLK_DIV_12K  8
#define PWM_CLK_DIV_24K  9
#define PWM_CLK_DIV_36K  10
#define PWM_CLK_DIV_48K  11
#define PWM_CLK_DIV_72K  12

#define SUNXI_PUD_OFF    0
#define SUNXI_PUD_UP     1
#define SUNXI_PUD_DOWN   2



// Mask for the bottom 64 pins which belong to the Raspberry Pi
//	The others are available for the other devices


#define	PI_GPIO_MASK	(0xFFFFFFC0)


// Handy defines
extern int wiringPiDebug;

// wiringPi modes

#define WPI_MODE_PINS            0
#define WPI_MODE_GPIO            1
#define WPI_MODE_GPIO_SYS        2
#define WPI_MODE_PHYS            3
#define WPI_MODE_PIFACE          4
#define WPI_MODE_UNINITIALISED  -1

// Pin modes

#define INPUT                    0
#define OUTPUT                   1
#define PWM_OUTPUT               2
#define GPIO_CLOCK               3
#define SOFT_PWM_OUTPUT          4
#define SOFT_TONE_OUTPUT         5
#define PWM_TONE_OUTPUT          6

#define LOW                      0
#define HIGH                     1

// Pull up/down/none

#define PUD_OFF                  0
#define PUD_DOWN                 1
#define PUD_UP                   2

// PWM

#define PWM_MODE_MS              0
#define PWM_MODE_BAL             1

// Interrupt levels

#define INT_EDGE_SETUP           0
#define INT_EDGE_FALLING         1
#define INT_EDGE_RISING          2
#define INT_EDGE_BOTH            3

// Pi model types and version numbers
//	Intended for the GPIO program Use at your own risk.

/* Allwinner H618 */
#define PI_MODEL_ZERO          0
#define PI_MODEL_BERRY        1


extern const char *piModelNames    [16] ;

extern const char *piRevisionNames [16] ;
extern const char *piMakerNames    [16] ;
extern const int   piMemorySize    [ 8] ;


//	Intended for the GPIO program Use at your own risk.

// Threads

#define	PI_THREAD(X)	void *X (UNU void *dummy)

// Failure modes

#define	WPI_FATAL	(1==1)
#define	WPI_ALMOST	(1==2)


// wiringPiNodeStruct:
//	This describes additional device nodes in the extended wiringPi
//	2.0 scheme of things.
//	It's a simple linked list for now, but will hopefully migrate to 
//	a binary tree for efficiency reasons - but then again, the chances
//	of more than 1 or 2 devices being added are fairly slim, so who
//	knows....

struct wiringPiNodeStruct
{
  int     pinBase ;
  int     pinMax ;

  int          fd ;	// Node specific
  unsigned int data0 ;	//  ditto
  unsigned int data1 ;	//  ditto
  unsigned int data2 ;	//  ditto
  unsigned int data3 ;	//  ditto

           void   (*pinMode)          (struct wiringPiNodeStruct *node, int pin, int mode) ;
           void   (*pullUpDnControl)  (struct wiringPiNodeStruct *node, int pin, int mode) ;
           int    (*digitalRead)      (struct wiringPiNodeStruct *node, int pin) ;
//unsigned int    (*digitalRead8)     (struct wiringPiNodeStruct *node, int pin) ;
           void   (*digitalWrite)     (struct wiringPiNodeStruct *node, int pin, int value) ;
//         void   (*digitalWrite8)    (struct wiringPiNodeStruct *node, int pin, int value) ;
           void   (*pwmWrite)         (struct wiringPiNodeStruct *node, int pin, int value) ;
           int    (*analogRead)       (struct wiringPiNodeStruct *node, int pin) ;
           void   (*analogWrite)      (struct wiringPiNodeStruct *node, int pin, int value) ;

  struct wiringPiNodeStruct *next ;
} ;

extern struct wiringPiNodeStruct *wiringPiNodes ;

// Export variables for the hardware pointers

extern volatile unsigned int *_wiringPiGpio ;
extern volatile unsigned int *_wiringPiPwm ;
extern volatile unsigned int *_wiringPiClk ;
extern volatile unsigned int *_wiringPiPads ;
extern volatile unsigned int *_wiringPiTimer ;
extern volatile unsigned int *_wiringPiTimerIrqRaw ;


// Function prototypes
//	c++ wrappers thanks to a comment by Nick Lott
//	(and others on the Raspberry Pi forums)

#ifdef __cplusplus
extern "C" {
#endif

// Data

// Internal

#ifdef CONFIG_BANANAPI
extern void piGpioLayoutOops (const char *why);
#endif

extern int wiringPiFailure (int fatal, const char *message, ...) ;

// Core wiringPi functions

extern struct wiringPiNodeStruct *wiringPiFindNode (int pin) ;
extern struct wiringPiNodeStruct *wiringPiNewNode  (int pinBase, int numPins) ;

extern void wiringPiVersion	(int *major, int *minor) ;
extern int  wiringPiSetup       (void) ;
extern int  wiringPiSetupSys    (void) ;
extern int  wiringPiSetupGpio   (void) ;
extern int  wiringPiSetupPhys   (void) ;

extern          void pinModeAlt          (int pin, int mode) ;
extern          void pinMode             (int pin, int mode) ;
extern          void pullUpDnControl     (int pin, int pud) ;
extern          int  digitalRead         (int pin) ;
extern          void digitalWrite        (int pin, int value) ;
extern unsigned int  digitalRead8        (int pin) ;
extern          void digitalWrite8       (int pin, int value) ;
extern          void pwmWrite            (int pin, int value) ;
extern          int  analogRead          (int pin) ;
extern          void analogWrite         (int pin, int value) ;

// PiFace specifics 
//	(Deprecated)

extern int  wiringPiSetupPiFace (void) ;
extern int  wiringPiSetupPiFaceForGpioProg (void) ;	// Don't use this - for gpio program only

// On-Board Raspberry Pi hardware specific stuff

extern          void piBoardId(int *model) ;
extern           int wpiPinToGpio(int wpiPin) ;
extern           int physPinToGpio(int physPin) ;
extern          void setPadDrive(int group, int value) ;
extern           int getAlt(int pin) ;
extern          void H618_set_pwm_reg(int pin,sunxi_gpio_info *sunxi_gpio_info_ptr) ;
extern          void sunxi_pwm_set_enable(int en) ;
extern          void pwmToneWrite(int pin, int freq) ;
extern          void pwmSetMode(int pin,int mode) ;
extern          void pwmSetRange(int pin,unsigned int range) ;
extern          void pwmSetClock(int pin,int divisor) ;
extern          void gpioClockSet(int pin, int freq) ;
extern unsigned  int digitalReadByte(void) ;
extern unsigned  int digitalReadByte2(void) ;
extern          void digitalWriteByte(int value) ;
extern          void digitalWriteByte2(int value) ;

// Interrupts
//	(Also Pi hardware specific)

extern int  waitForInterrupt    (int pin, int mS) ;
extern int  wiringPiISR         (int pin, int mode, void (*function)(void)) ;

// Threads

extern int  piThreadCreate      (void *(*fn)(void *)) ;
extern void piLock              (int key) ;
extern void piUnlock            (int key) ;

// Schedulling priority

extern int piHiPri (const int pri) ;

// Extras from arduino land

extern void         delay             (unsigned int howLong) ;
extern void         delayMicroseconds (unsigned int howLong) ;
extern unsigned int millis            (void) ;
extern unsigned int micros            (void) ;

extern unsigned int readR(unsigned int addr);
extern void writeR(unsigned int val, unsigned int addr);
extern int BananaPi_get_gpio_mode(int pin);
extern int BananaPi_set_gpio_mode(int pin, int mode);
extern int BananaPi_digitalRead(int pin);
extern int BananaPi_digitalWrite(int pin, int value);
extern int BananaPi_set_gpio_alt(int pin, int mode);
extern void BananaPi_set_gpio_pullUpDnControl (int pin, int pud);
extern void sunxi_pwm_set_act(int pin,int act_cys);
extern void sunxi_pwm_set_period(int pin,unsigned int period_cys);
extern void sunxi_pwm_set_clk(int pin,int clk);
extern void sunxi_pwm_set_tone(int pin,int freq);

void set_soc_info(void);

#ifdef __cplusplus
}
#endif

#endif
