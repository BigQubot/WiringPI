/*
 * wiringPi:
 *	Arduino look-a-like Wiring library for the Raspberry Pi
 *	Copyright (c) 2012-2017 Gordon Henderson
 *	Additional code for pwmSetClock by Chris Hall <chris@kchall.plus.com>
 *
 *	Thanks to code samples from Gert Jan van Loo and the
 *	BCM2835 ARM Peripherals manual, however it's missing
 *	the clock section /grr/mutter/
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

// Revisions:
//	19 Jul 2012:
//		Moved to the LGPL
//		Added an abstraction layer to the main routines to save a tiny
//		bit of run-time and make the clode a little cleaner (if a little
//		larger)
//		Added waitForInterrupt code
//		Added piHiPri code
//
//	 9 Jul 2012:
//		Added in support to use the /sys/class/gpio interface.
//	 2 Jul 2012:
//		Fixed a few more bugs to do with range-checking when in GPIO mode.
//	11 Jun 2012:
//		Fixed some typos.
//		Added c++ support for the .h file
//		Added a new function to allow for using my "pin" numbers, or native
//			GPIO pin numbers.
//		Removed my busy-loop delay and replaced it with a call to delayMicroseconds
//
//	02 May 2012:
//		Added in the 2 UART pins
//		Change maxPins to numPins to more accurately reflect purpose


#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>

#include <limits.h>
#include "softPwm.h"
#include "softTone.h"

#include "wiringPi.h"
#include "../version.h"

// Environment Variables

#define	ENV_DEBUG	"WIRINGPI_DEBUG"
#define	ENV_CODES	"WIRINGPI_CODES"
#define	ENV_GPIOMEM	"WIRINGPI_GPIOMEM"

int pwmmode = 0;


const char * int2bin(uint32_t param) {
	int bits = sizeof(uint32_t)*CHAR_BIT;
	static char buffer[sizeof(uint32_t)*CHAR_BIT + 1];
	char chars[2] = {'0', '1'};
	int i,j,offset;

        for (i = 0; i < bits; i++) {
		 j = bits - i - 1;
		 offset = (param & (1 << j)) >> j;
		 buffer[i] = chars[offset];
        }

	buffer[bits] = '\0';
	return buffer;
}

static int BANANAPI_PIN_MASK_M4_BERRY[12][32] =  //[BANK]  [INDEX]
{
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PA
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PB
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PC
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PD
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PE
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PF
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PG
	{-1,-1, 2, 3, 4, 5, 6, 7, 8, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PH
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PI
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PJ
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PK
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PE
};

static int BANANAPI_PIN_MASK_M4_ZERO[12][32] =  //[BANK]  [INDEX]
{
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PA
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PB
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PC
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PD
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PE
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PF
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15,16,17,18,19,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PG
	{-1,-1, 2, 3, 4, 5, 6, 7, 8, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PH
	{ 0, 1, 2, 3, 4,-1,-1,-1,-1, 9,10,11,12,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PI
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PJ
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PK
	{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,},//PE
};

int (*BANANAPI_PIN_MASK)[32];

// Extend wiringPi with other pin-based devices and keep track of
//	them in this structure

struct wiringPiNodeStruct *wiringPiNodes = NULL ;

// BCM Magic

#define	BCM_PASSWORD		0x5A000000


// The BCM2835 has 54 GPIO pins.
//	BCM2835 data sheet, Page 90 onwards.
//	There are 6 control registers, each control the functions of a block
//	of 10 pins.
//	Each control register has 10 sets of 3 bits per GPIO pin - the ALT values
//
//	000 = GPIO Pin X is an input
//	001 = GPIO Pin X is an output
//	100 = GPIO Pin X takes alternate function 0
//	101 = GPIO Pin X takes alternate function 1
//	110 = GPIO Pin X takes alternate function 2
//	111 = GPIO Pin X takes alternate function 3
//	011 = GPIO Pin X takes alternate function 4
//	010 = GPIO Pin X takes alternate function 5
//
// So the 3 bits for port X are:
//	X / 10 + ((X % 10) * 3)

// Port function select bits

#define	FSEL_INPT		0b000
#define	FSEL_OUTP		0b001
#define	FSEL_ALT0		0b100
#define	FSEL_ALT1		0b101
#define	FSEL_ALT2		0b110
#define	FSEL_ALT3		0b111
#define	FSEL_ALT4		0b011
#define	FSEL_ALT5		0b010

// Access from ARM Running Linux
//	Taken from Gert/Doms code. Some of this is not in the manual
//	that I can find )-:
//
// Updates in September 2015 - all now static variables (and apologies for the caps)
//	due to the Pi v2, v3, etc. and the new /dev/gpiomem interface

static volatile unsigned int GPIO_PADS ;
static volatile unsigned int GPIO_CLOCK_BASE ;

#ifndef CONFIG_BANANAPI
static volatile unsigned int GPIO_BASE ;
#endif

static volatile unsigned int GPIO_TIMER ;
//static volatile unsigned int GPIO_PWM;

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

static unsigned int usingGpioMem    = FALSE ;
static          int wiringPiSetuped = FALSE ;
static          int wiringPiSysSetuped = FALSE ;


// PWM
//	Word offsets into the PWM control region

#define	PWM_CONTROL 0
#define	PWM_STATUS  1
#define	PWM0_RANGE  4
#define	PWM0_DATA   5
#define	PWM1_RANGE  8
#define	PWM1_DATA   9

//	Clock regsiter offsets

#define	PWMCLK_CNTL	40
#define	PWMCLK_DIV	41

#define	PWM0_MS_MODE    0x0080  // Run in MS mode
#define	PWM0_USEFIFO    0x0020  // Data from FIFO
#define	PWM0_REVPOLAR   0x0010  // Reverse polarity
#define	PWM0_OFFSTATE   0x0008  // Ouput Off state
#define	PWM0_REPEATFF   0x0004  // Repeat last value if FIFO empty
#define	PWM0_SERIAL     0x0002  // Run in serial mode
#define	PWM0_ENABLE     0x0001  // Channel Enable

#define	PWM1_MS_MODE    0x8000  // Run in MS mode
#define	PWM1_USEFIFO    0x2000  // Data from FIFO
#define	PWM1_REVPOLAR   0x1000  // Reverse polarity
#define	PWM1_OFFSTATE   0x0800  // Ouput Off state
#define	PWM1_REPEATFF   0x0400  // Repeat last value if FIFO empty
#define	PWM1_SERIAL     0x0200  // Run in serial mode
#define	PWM1_ENABLE     0x0100  // Channel Enable

// Timer
//	Word offsets

#define	TIMER_LOAD	(0x400 >> 2)
#define	TIMER_VALUE	(0x404 >> 2)
#define	TIMER_CONTROL	(0x408 >> 2)
#define	TIMER_IRQ_CLR	(0x40C >> 2)
#define	TIMER_IRQ_RAW	(0x410 >> 2)
#define	TIMER_IRQ_MASK	(0x414 >> 2)
#define	TIMER_RELOAD	(0x418 >> 2)
#define	TIMER_PRE_DIV	(0x41C >> 2)
#define	TIMER_COUNTER	(0x420 >> 2)

// Locals to hold pointers to the hardware

static volatile unsigned int *gpio ;
static volatile unsigned int *pwm ;
static volatile unsigned int *clk ;
static volatile unsigned int *pads ;
//static volatile unsigned int *timer ;
//static volatile unsigned int *timerIrqRaw ;

// Export variables for the hardware pointers

volatile unsigned int *_wiringPiGpio ;
volatile unsigned int *_wiringPiPwm ;
volatile unsigned int *_wiringPiClk ;
volatile unsigned int *_wiringPiPads ;
volatile unsigned int *_wiringPiTimer ;
volatile unsigned int *_wiringPiTimerIrqRaw ;


// Data for use with the boardId functions.
//	The order of entries here to correspond with the PI_MODEL_X
//	and PI_VERSION_X defines in wiringPi.h
//	Only intended for the gpio command - use at your own risk!

// piGpioBase:
//	The base address of the GPIO memory mapped hardware IO

#define	GPIO_PERI_BASE_OLD	0x20000000
#define	GPIO_PERI_BASE_NEW	0x3F000000

static volatile unsigned int piGpioBase = 0 ;

// Time for easy calculations

static uint64_t epochMilli, epochMicro ;

// Misc

static int wiringPiMode = WPI_MODE_UNINITIALISED ;
static volatile int    pinPass = -1 ;
static pthread_mutex_t pinMutex ;

// Debugging & Return codes

int wiringPiDebug       = FALSE ;
//int wiringPiDebug     = TRUE ;
int wiringPiReturnCodes = FALSE ;

// Use /dev/gpiomem ?

int wiringPiTryGpioMem  = FALSE ;

sunxi_gpio_info sunxi_gpio_info_t;

// sysFds:
//	Map a file descriptor from the /sys/class/gpio/gpioX/value

static int sysFds[384] =
  {
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,
    -1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1-1,-1,-1,-1
  };

// ISR Data

static void (*isrFunctions [64])(void) ;


// Doing it the Arduino way with lookup tables...
//	Yes, it's probably more innefficient than all the bit-twidling, but it
//	does tend to make it all a bit clearer. At least to me!

// pinToGpio:
//	Take a Wiring pin (0 through X) and re-map it to the GPIO pin
//	Cope for 3 different board revisions here.

static int BananaPiModel;

static int *pinToGpio ;

int pinToGpio_M4_BERRY[64] =
{
	208, 207, 211, 198, 199, 226, 203,
	227, 194, 200, 201, 231, 232, 193,
	230, 229, 233, 210, 209, 195, 196,
	192, 197, 204, 228, 202, 206, 205,

	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,			// ... 31
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,			// ... 47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,			// ... 63
};

int pinToGpio_M4_ZERO[64] =
{
	208, 207, 211, 198, 199, 226, 203,
	227, 194, 200, 201, 231, 232, 193,
	230, 229, 233, 210, 209, 195, 196,
	192, 197, 204, 228, 202, 206, 205,

	// Padding:
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,			// ... 31
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,			// ... 47
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,			// ... 63
};

// physToGpio:
//	Take a physical pin (1 through 26) and re-map it to the GPIO pin
//	Cope for 2 different board revisions here.
//	Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56

static int *physToGpio ;

int physToGpio_M4_BERRY[64] =
{
	 -1,	 	// 0
	 -1,  -1,	// 1, 2
	208,  -1,
	207,  -1,
	211, 198,
	 -1, 199,
	226, 203,
	227,  -1,
	194, 200,
	 -1, 201,
	231,  -1,
	232, 193,
	230, 229,
	 -1, 233,	// 25, 26
	210, 209,	// 27
	195,  -1,	// 29
	196, 192,	// 31
	197,  -1,	// 33
	204, 228,	// 35
	202, 206,	// 37
	 -1, 205,	// 39

	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,		// ... 49
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
};

int physToGpio_M4_ZERO[64] =
{
	 -1,	 	// 0
	 -1,  -1,	// 1, 2
	208,  -1,
	207,  -1,
	211, 198,
	 -1, 199,
	226, 203,
	227,  -1,
	194, 200,
	 -1, 201,
	231,  -1,
	232, 193,
	230, 229,
	 -1, 233,	// 25, 26
	210, 209,	// 27
	195,  -1,	// 29
	196, 192,	// 31
	197,  -1,	// 33
	204, 228,	// 35
	202, 206,	// 37
	 -1, 205,	// 39

	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,		// ... 49
	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
};

// gpioToGPFSEL:
//	Map a BCM_GPIO pin to it's Function Selection
//	control port. (GPFSEL 0-5)
//	Groups of 10 - 3 bits per Function - 30 bits per port

static uint8_t gpioToGPFSEL [] =
{
  0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,1,
  2,2,2,2,2,2,2,2,2,2,
  3,3,3,3,3,3,3,3,3,3,
  4,4,4,4,4,4,4,4,4,4,
  5,5,5,5,5,5,5,5,5,5,
} ;


// gpioToShift
//	Define the shift up for the 3 bits per pin in each GPFSEL port

static uint8_t gpioToShift [] =
{
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
  0,3,6,9,12,15,18,21,24,27,
} ;


// gpioToGPSET:
//	(Word) offset to the GPIO Set registers for each GPIO pin

static uint8_t gpioToGPSET [] =
{
   7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
   8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
} ;

// gpioToGPCLR:
//	(Word) offset to the GPIO Clear registers for each GPIO pin

static uint8_t gpioToGPCLR [] =
{
  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,11,
} ;


// gpioToGPLEV:
//	(Word) offset to the GPIO Input level registers for each GPIO pin

static uint8_t gpioToGPLEV [] =
{
  13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,13,
  14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,
} ;


#ifdef notYetReady
// gpioToEDS
//	(Word) offset to the Event Detect Status

static uint8_t gpioToEDS [] =
{
  16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
  17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,
} ;

// gpioToREN
//	(Word) offset to the Rising edge ENable register

static uint8_t gpioToREN [] =
{
  19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,
  20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,
} ;

// gpioToFEN
//	(Word) offset to the Falling edgde ENable register

static uint8_t gpioToFEN [] =
{
  22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,
  23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,
} ;
#endif


// GPPUD:
//	GPIO Pin pull up/down register

#define	GPPUD	37

// gpioToPUDCLK
//	(Word) offset to the Pull Up Down Clock regsiter

static uint8_t gpioToPUDCLK [] =
{
  38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,38,
  39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,39,
} ;


// gpioToPwmALT
//	the ALT value to put a GPIO pin into PWM mode

static uint8_t gpioToPwmALT [] =
{
          0,         0,         0,         0,         0,         0,         0,         0,	//  0 ->  7
          0,         0,         0,         0, FSEL_ALT0, FSEL_ALT0,         0,         0, 	//  8 -> 15
          0,         0, FSEL_ALT5, FSEL_ALT5,         0,         0,         0,         0, 	// 16 -> 23
          0,         0,         0,         0,         0,         0,         0,         0,	// 24 -> 31
          0,         0,         0,         0,         0,         0,         0,         0,	// 32 -> 39
  FSEL_ALT0, FSEL_ALT0,         0,         0,         0, FSEL_ALT0,         0,         0,	// 40 -> 47
          0,         0,         0,         0,         0,         0,         0,         0,	// 48 -> 55
          0,         0,         0,         0,         0,         0,         0,         0,	// 56 -> 63
} ;


// gpioToPwmPort
//	The port value to put a GPIO pin into PWM mode

/*static uint8_t gpioToPwmPort [] =
{
          0,         0,         0,         0,         0,         0,         0,         0,	//  0 ->  7
          0,         0,         0,         0, PWM0_DATA, PWM1_DATA,         0,         0, 	//  8 -> 15
          0,         0, PWM0_DATA, PWM1_DATA,         0,         0,         0,         0, 	// 16 -> 23
          0,         0,         0,         0,         0,         0,         0,         0,	// 24 -> 31
          0,         0,         0,         0,         0,         0,         0,         0,	// 32 -> 39
  PWM0_DATA, PWM1_DATA,         0,         0,         0, PWM1_DATA,         0,         0,	// 40 -> 47
          0,         0,         0,         0,         0,         0,         0,         0,	// 48 -> 55
          0,         0,         0,         0,         0,         0,         0,         0,	// 56 -> 63

} ;*/

// gpioToGpClkALT:
//	ALT value to put a GPIO pin into GP Clock mode.
//	On the Pi we can really only use BCM_GPIO_4 and BCM_GPIO_21
//	for clocks 0 and 1 respectively, however I'll include the full
//	list for completeness - maybe one day...

#define	GPIO_CLOCK_SOURCE	1

// gpioToGpClkALT0:

static uint8_t gpioToGpClkALT0 [] =
{
          0,         0,         0,         0, FSEL_ALT0, FSEL_ALT0, FSEL_ALT0,         0,	//  0 ->  7
          0,         0,         0,         0,         0,         0,         0,         0, 	//  8 -> 15
          0,         0,         0,         0, FSEL_ALT5, FSEL_ALT5,         0,         0, 	// 16 -> 23
          0,         0,         0,         0,         0,         0,         0,         0,	// 24 -> 31
  FSEL_ALT0,         0, FSEL_ALT0,         0,         0,         0,         0,         0,	// 32 -> 39
          0,         0, FSEL_ALT0, FSEL_ALT0, FSEL_ALT0,         0,         0,         0,	// 40 -> 47
          0,         0,         0,         0,         0,         0,         0,         0,	// 48 -> 55
          0,         0,         0,         0,         0,         0,         0,         0,	// 56 -> 63
} ;

// gpioToClk:
//	(word) Offsets to the clock Control and Divisor register

static uint8_t gpioToClkCon [] =
{
         -1,        -1,        -1,        -1,        28,        30,        32,        -1,	//  0 ->  7
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1, 	//  8 -> 15
         -1,        -1,        -1,        -1,        28,        30,        -1,        -1, 	// 16 -> 23
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 24 -> 31
         28,        -1,        28,        -1,        -1,        -1,        -1,        -1,	// 32 -> 39
         -1,        -1,        28,        30,        28,        -1,        -1,        -1,	// 40 -> 47
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 48 -> 55
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 56 -> 63
} ;

static uint8_t gpioToClkDiv [] =
{
         -1,        -1,        -1,        -1,        29,        31,        33,        -1,	//  0 ->  7
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1, 	//  8 -> 15
         -1,        -1,        -1,        -1,        29,        31,        -1,        -1, 	// 16 -> 23
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 24 -> 31
         29,        -1,        29,        -1,        -1,        -1,        -1,        -1,	// 32 -> 39
         -1,        -1,        29,        31,        29,        -1,        -1,        -1,	// 40 -> 47
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 48 -> 55
         -1,        -1,        -1,        -1,        -1,        -1,        -1,        -1,	// 56 -> 63
} ;


/*
 * Functions
 *********************************************************************************
 */


/*
 * wiringPiFailure:
 *	Fail. Or not.
 *********************************************************************************
 */

int wiringPiFailure (int fatal, const char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  if (!fatal && wiringPiReturnCodes)
    return -1 ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  fprintf (stderr, "%s", buffer) ;
  exit (EXIT_FAILURE) ;

  return 0 ;
}


/*
 * setupCheck
 *	Another sanity check because some users forget to call the setup
 *	function. Mosty because they need feeding C drip by drip )-:
 *********************************************************************************
 */

static void setupCheck (const char *fName)
{
	if (!wiringPiSetuped)
	{
		fprintf (stderr, "%s: You have not called one of the wiringPiSetup\n"
				"  functions, so I'm aborting your program before it crashes anyway.\n", fName) ;
		exit (EXIT_FAILURE) ;
	}
}

/*
 * gpioMemCheck:
 *	See if we're using the /dev/gpiomem interface, if-so then some operations
 *	can't be done and will crash the Pi.
 *********************************************************************************
 */

static void usingGpioMemCheck (const char *what)
{
  if (usingGpioMem)
  {
    fprintf (stderr, "%s: Unable to do this when using /dev/gpiomem. Try sudo?\n", what) ;
    exit (EXIT_FAILURE) ;
  }
}

/*
 * piGpioLayout:
 *	Return a number representing the hardware revision of the board.
 *	This is not strictly the board revision but is used to check the
 *	layout of the GPIO connector - and there are 2 types that we are
 *	really interested in here. The very earliest Pi's and the
 *	ones that came after that which switched some pins ....
 *
 *	Revision 1 really means the early Model A and B's.
 *	Revision 2 is everything else - it covers the B, B+ and CM.
 *		... and the Pi 2 - which is a B+ ++  ...
 *		... and the Pi 0 - which is an A+ ...
 *
 *	The main difference between the revision 1 and 2 system that I use here
 *	is the mapping of the GPIO pins. From revision 2, the Pi Foundation changed
 *	3 GPIO pins on the (original) 26-way header - BCM_GPIO 22 was dropped and
 *	replaced with 27, and 0 + 1 - I2C bus 0 was changed to 2 + 3; I2C bus 1.
 *
 *	Additionally, here we set the piModel2 flag too. This is again, nothing to
 *	do with the actual model, but the major version numbers - the GPIO base
 *	hardware address changed at model 2 and above (not the Zero though)
 *
 *********************************************************************************
 */

void piGpioLayoutOops (const char *why)
{
	fprintf (stderr, "Oops: Unable to determine board revision from /etc/bananapi-release or /etc/armbian-release.\n") ;
	fprintf (stderr, " -> %s\n", why) ;
	fprintf (stderr, " ->  You'd best google the error to find out why.\n") ;
	exit (EXIT_FAILURE) ;
}

void piBoardId (int * model)
{
	FILE *cpuFd ;
	char line [40] ;
	char *c;
	char revision[40];
	unsigned int i = 0;

	if ((cpuFd = fopen ("/etc/bananapi-release", "r")) == NULL)
		if ((cpuFd = fopen ("/etc/armbian-release", "r")) == NULL)
			piGpioLayoutOops ("Unable to open /etc/bananapi-release or /etc/armbian-release.");

	while (fgets (line, 40, cpuFd) != NULL)
	if (strncmp (line, "BOARD=", 6) == 0)
		break ;

	fclose (cpuFd) ;

	if (strncmp (line, "BOARD=", 6) != 0)
		piGpioLayoutOops ("No \"Revision\" line") ;

	if (wiringPiDebug)
		printf ("piBoardId: Revision string: %s\n", line) ;

	// Chomp trailing CR/NL
	for (c = &line [strlen (line) - 1] ; (*c == '\n') || (*c == '\r') ; --c)
		*c = 0 ;

	// Need to work out if it's using the new or old encoding scheme:
	// Scan to the first character of the revision number
	for (c = line ; *c ; ++c)
    	if (*c == '=')
      		break ;

	if (*c != '=')
    	piGpioLayoutOops ("Revision line (no equal)");

	c++;
	for (i = 0; *c ; ++c)
		revision[i++] = *c;

	revision[i] = '.';

	if (wiringPiDebug)
		printf ("piBoardId: Board string: %s\n", revision) ;

	/**/ if (strncmp(revision, "bpi-m4berry.",    	   12) == 0) { *model = PI_MODEL_BERRY; }
	else if (strncmp(revision, "bpi-m4zero.",         11) == 0) { *model = PI_MODEL_ZERO; }

	if (wiringPiDebug)
		printf("piBoardId: model = %d\n", *model);
}

/*
 * wpiPinToGpio:
 *	Translate a wiringPi Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int wpiPinToGpio (int wpiPin)
{
  return pinToGpio [wpiPin & 63] ;
}


/*
 * physPinToGpio:
 *	Translate a physical Pin number to native GPIO pin number.
 *	Provided for external support.
 *********************************************************************************
 */

int physPinToGpio (int physPin)
{
  return physToGpio [physPin & 63] ;
}


/*
 * setPadDrive:
 *	Set the PAD driver value
 *********************************************************************************
 */

void setPadDrive (int group, int value)
{
  uint32_t wrVal ;

  if ((wiringPiMode == WPI_MODE_PINS) || (wiringPiMode == WPI_MODE_PHYS) || (wiringPiMode == WPI_MODE_GPIO))
  {
    if ((group < 0) || (group > 2))
      return ;

    wrVal = BCM_PASSWORD | 0x18 | (value & 7) ;
    *(pads + group + 11) = wrVal ;

    if (wiringPiDebug)
    {
      printf ("setPadDrive: Group: %d, value: %d (%08X)\n", group, value, wrVal) ;
      printf ("Read : %08X\n", *(pads + group + 11)) ;
    }
  }
}


/*
 * getAlt:
 *	Returns the ALT bits for a given port. Only really of-use
 *	for the gpio readall command (I think)
 *********************************************************************************
 */
int getAlt (int pin)
{
	int alt;

	pin &= 63;

	if (wiringPiMode == WPI_MODE_PINS)
		pin = pinToGpio [pin];
	else if (wiringPiMode == WPI_MODE_PHYS)
		pin = physToGpio [pin];
	else if (wiringPiMode != WPI_MODE_GPIO)
		return 0;

	alt = BananaPi_get_gpio_mode(pin);

	return alt;
}


void print_pwm_reg()
{
    uint32_t val = readR(SUNXI_PWM_CTRL_REG);
    uint32_t val2 = readR(SUNXI_PWM_PERIOD);

    if (wiringPiDebug) {
#ifdef BPI
        printf("SUNXI_PWM_CTRL_REG: %s\n", int2bin(val));
        printf("SUNXI_PWM_PERIOD: %s\n", int2bin(val2));
#else
        printf("SUNXI_PWM_CTRL_REG: [%x]\n", val);
        printf("SUNXI_PWM_PERIOD: [%x]\n", val2);
#endif
    }
}

void sunxi_pwm_set_enable_v2(int en)
{
    int val = 0;

    // CLK
    val = readR(SUNXI_PWM_CLK_REG);

    if (en) {
        val |= (SUNXI_PWM_SCLK_GATING);
    } else {
        val &= ~(SUNXI_PWM_SCLK_GATING);
    }

    if (wiringPiDebug)
	 printf(">>function%s,no:%d,enable? :0x%x\n", __func__, __LINE__, val);

    writeR(val, SUNXI_PWM_CLK_REG);
    delay(1);
    val = readR(SUNXI_PWM_EN_REG);

    // EN
    if (en) {
        val |= (SUNXI_PWM_EN);
    } else {
        val &= ~(SUNXI_PWM_EN);
    }

    if (wiringPiDebug)
	 printf(">>function%s,no:%d,enable? :0x%x\n", __func__, __LINE__, val);

    writeR(val, SUNXI_PWM_EN_REG);
    delay(1);

    // debug
    print_pwm_reg();
}


void sunxi_pwm_set_enable(int en)
{
	int val = 0;

	switch (BananaPiModel)
	{
		case PI_MODEL_BERRY:

			if(SUNXI_PWM_TYPE == SUNXI_V2_PWM_TYPE) {
				sunxi_pwm_set_enable_v2(en);
				return;
			}

			val = readR(SUNXI_PWM_CTRL_REG);

			if (en) {
				val |= (SUNXI_PWM_EN | SUNXI_PWM_SCLK_GATING);
			} else {
				val &= ~(SUNXI_PWM_EN | SUNXI_PWM_SCLK_GATING);
			}

			if (wiringPiDebug)
				printf(">>function%s,no:%d,enable? :0x%x\n", __func__, __LINE__, val);

			writeR(val, SUNXI_PWM_CTRL_REG);
			delay(1);
			print_pwm_reg();

			break;
		case PI_MODEL_ZERO:

			if(SUNXI_PWM_TYPE == SUNXI_V2_PWM_TYPE) {
				sunxi_pwm_set_enable_v2(en);
				return;
			}

			val = readR(SUNXI_PWM_CTRL_REG);

			if (en) {
				val |= (SUNXI_PWM_EN | SUNXI_PWM_SCLK_GATING);
			} else {
				val &= ~(SUNXI_PWM_EN | SUNXI_PWM_SCLK_GATING);
			}

			if (wiringPiDebug)
				printf(">>function%s,no:%d,enable? :0x%x\n", __func__, __LINE__, val);

			writeR(val, SUNXI_PWM_CTRL_REG);
			delay(1);
			print_pwm_reg();

			break;

		default:
			break;
	}
}


void sunxi_pwm_set_mode(int mode)
{
    int val = 0;

    val = readR(SUNXI_PWM_CTRL_REG);
    mode &= 1; //cover the mode to 0 or 1

    if (mode) { //pulse mode
        val |= (SUNXI_PWM_MS_MODE | SUNXI_PWM_PUL_START);
        pwmmode = 1;
    } else { //cycle mode
        val &= ~(SUNXI_PWM_MS_MODE);  //0<<9  MS_mode
        pwmmode = 0;
    }
    val |= (SUNXI_PWM_ACT_STA);  //1<<8  ACT_HIGH

    if (wiringPiDebug)
        printf(">>function%s,no:%d,mode? :0x%x\n", __func__, __LINE__, val);

    writeR(val, SUNXI_PWM_CTRL_REG);
    delay(1);
    print_pwm_reg();
}

void sunxi_pwm_set_tone(int pin,int freq)
{
	int div ;
	unsigned int range ;
	unsigned int val;

	if (wiringPiDebug)
		printf(">>func:%s no:%d\n", __func__, __LINE__);

	switch (BananaPiModel)
	{
		case PI_MODEL_BERRY:

			H618_set_pwm_reg(pin,&sunxi_gpio_info_t);

			if (freq == 0)
				sunxi_pwm_set_act (pin, 0);             // Off
			else {
				div = readR(SUNXI_PWM_CTRL_REG);
				div &= 0x00ff;  //The lower 8 bits determine the frequency division
				div += 1;       //The actual frequency division value is (div + 1)
				range = 24000000 / (div * freq);  //The default pwm clock frequency is 24MHz

				sunxi_pwm_set_period (pin,range);
				sunxi_pwm_set_act (pin, range / 2);

				if (wiringPiDebug)
					printf("div:%d range:%d\n",div,range);
			}

			break;
		case PI_MODEL_ZERO:

			H618_set_pwm_reg(pin,&sunxi_gpio_info_t);

			if (freq == 0)
				sunxi_pwm_set_act (pin, 0);             // Off
			else {
				div = readR(SUNXI_PWM_CTRL_REG);
				div &= 0x00ff;  //The lower 8 bits determine the frequency division
				div += 1;       //The actual frequency division value is (div + 1)
				range = 24000000 / (div * freq);  //The default pwm clock frequency is 24MHz

				sunxi_pwm_set_period (pin,range);
				sunxi_pwm_set_act (pin, range / 2);

				if (wiringPiDebug)
					printf("div:%d range:%d\n",div,range);
			}

			break;

		default:
			break;
	}
}

void sunxi_pwm_set_clk_v2(int clk)
{
    int val = 0;

    if (wiringPiDebug)
	printf(">>function%s,no:%d\n", __func__, __LINE__);

    sunxi_pwm_set_enable(0);
    val = readR( SUNXI_PWM_CTRL_REG);

    if (wiringPiDebug)
        printf("read reg val: 0x%x\n", val);

    //clear clk to 0
    val &= 0x0f00;

    clk = (clk - 1) < 0 ? 0 : (clk - 1);
    val |= (clk & 0xff); //todo check wether clk is invalid or not
    writeR(val, SUNXI_PWM_CTRL_REG);

    sunxi_pwm_set_enable(1);

    if (wiringPiDebug)
        printf(">>function%s,no:%d,clk? :0x%x\n", __func__, __LINE__, val);

    delay(1);
    print_pwm_reg();
}


void sunxi_pwm_set_clk(int pin,int clk)
{
	int val = 0;
	int regval = 0;

	if (wiringPiDebug)
		printf(">>func:%s no:%d\n", __func__, __LINE__);

	switch (BananaPiModel)
	{
		case PI_MODEL_BERRY:

			if ((clk < 1) || (clk > 256)) {
				fprintf (stderr, "gpio: clock must be between 1 and 256\n") ;
				exit (1) ;
			}

			H618_set_pwm_reg(pin,&sunxi_gpio_info_t);

			if(SUNXI_PWM_TYPE == SUNXI_V2_PWM_TYPE) {
				sunxi_pwm_set_clk_v2(clk);
				return;
			}

			if (wiringPiDebug)
				printf(">>function%s,no:%d\n", __func__, __LINE__);

			// sunxi_pwm_set_enable(0);
			val = readR(SUNXI_PWM_CTRL_REG);

			if (wiringPiDebug)
				printf("read reg val: 0x%x\n", val);

			//clear clk to 0
			val &= 0xf801f0;

			val |= ((clk & 0xf) << 15); //todo check wether clk is invalid or not
			writeR(val, SUNXI_PWM_CTRL_REG);

			sunxi_pwm_set_enable(1);

			if (wiringPiDebug)
				printf(">>function%s,no:%d,clk? :0x%x\n", __func__, __LINE__, val);

			delay(1);
			print_pwm_reg();

			break;

		case PI_MODEL_ZERO:

			if ((clk < 1) || (clk > 256)) {
				fprintf (stderr, "gpio: clock must be between 1 and 256\n") ;
				exit (1) ;
			}

			H618_set_pwm_reg(pin,&sunxi_gpio_info_t);

			if(SUNXI_PWM_TYPE == SUNXI_V2_PWM_TYPE) {
				sunxi_pwm_set_clk_v2(clk);
				return;
			}

			if (wiringPiDebug)
				printf(">>function%s,no:%d\n", __func__, __LINE__);

			// sunxi_pwm_set_enable(0);
			val = readR(SUNXI_PWM_CTRL_REG);

			if (wiringPiDebug)
				printf("read reg val: 0x%x\n", val);

			//clear clk to 0
			val &= 0xf801f0;

			val |= ((clk & 0xf) << 15); //todo check wether clk is invalid or not
			writeR(val, SUNXI_PWM_CTRL_REG);

			sunxi_pwm_set_enable(1);

			if (wiringPiDebug)
				printf(">>function%s,no:%d,clk? :0x%x\n", __func__, __LINE__, val);

			delay(1);
			print_pwm_reg();

			break;

		default:
			break;
	}
}

/**
 * ch0 and ch1 set the same,16 bit period and 16 bit act
 */
int sunxi_pwm_get_period(void)
{
    uint32_t period_cys = 0;
    period_cys = readR(SUNXI_PWM_PERIOD);

    if (wiringPiDebug) {
        printf("periodcys: %d\n", period_cys);
    }

    period_cys &= 0xffff0000; //get period_cys
    period_cys = period_cys >> 16;

    if (wiringPiDebug)
        printf(">>func:%s,no:%d,period/range:%d", __func__, __LINE__, period_cys);

    delay(1);
    return period_cys;
}

int sunxi_pwm_get_act(void)
{
    uint32_t period_act = 0;

    period_act = readR(SUNXI_PWM_PERIOD); //get ch1 period_cys
    period_act &= 0xffff; //get period_act

    if (wiringPiDebug)
        printf(">>func:%s,no:%d,period/range:%d", __func__, __LINE__, period_act);

    delay(1);
    return period_act;
}

void sunxi_pwm_set_period(int pin,unsigned int period_cys)
{
	uint32_t val = 0;
	uint32_t ccr = 0;

	if (wiringPiDebug)
		printf(">>func:%s no:%d\n", __func__, __LINE__);

	switch (BananaPiModel)
	{
		case PI_MODEL_BERRY:

			if ((period_cys < 1) || (period_cys > 65536)) {
				fprintf (stderr, "gpio: range must be between 1 and 65536\n") ;
				exit (1) ;
			}

			H618_set_pwm_reg(pin,&sunxi_gpio_info_t);

			period_cys -= 1;
			period_cys &= 0xffff; //set max period to 2^16
			period_cys = period_cys << 16;
			val = readR(SUNXI_PWM_PERIOD);

			if (wiringPiDebug)
				printf("read reg val: 0x%x\n", val);

			val &= 0x0000ffff;
			period_cys |= val;

			if (wiringPiDebug)
				printf("write reg val: 0x%x\n", period_cys);

			writeR(period_cys, SUNXI_PWM_PERIOD);
			delay(1);
			val = readR(SUNXI_PWM_PERIOD);

			if (wiringPiDebug)
				printf("readback reg val: 0x%x\n", val);

			print_pwm_reg();

			break;
		case PI_MODEL_ZERO:

			if ((period_cys < 1) || (period_cys > 65536)) {
				fprintf (stderr, "gpio: range must be between 1 and 65536\n") ;
				exit (1) ;
			}

			H618_set_pwm_reg(pin,&sunxi_gpio_info_t);

			period_cys -= 1;
			period_cys &= 0xffff; //set max period to 2^16
			period_cys = period_cys << 16;
			val = readR(SUNXI_PWM_PERIOD);

			if (wiringPiDebug)
				printf("read reg val: 0x%x\n", val);

			val &= 0x0000ffff;
			period_cys |= val;

			if (wiringPiDebug)
				printf("write reg val: 0x%x\n", period_cys);

			writeR(period_cys, SUNXI_PWM_PERIOD);
			delay(1);
			val = readR(SUNXI_PWM_PERIOD);

			if (wiringPiDebug)
				printf("readback reg val: 0x%x\n", val);

			print_pwm_reg();

			break;

		default:
			break;
	}
}

void sunxi_pwm_set_act(int pin,int act_cys)
{
	uint32_t per0 = 0;
	uint32_t arr = 0;

	switch (BananaPiModel)
	{
		case PI_MODEL_BERRY:

			if ((act_cys < 0) || (act_cys > 65535)) {
				fprintf (stderr, "gpio: range must be between 0 and 65535\n");
				exit (1) ;
			}

			H618_set_pwm_reg(pin,&sunxi_gpio_info_t);

			arr = sunxi_pwm_get_period();

			if (wiringPiDebug)
				printf("==> no:%d period now is :%d,act_val to be set:%d\n", __LINE__, arr, act_cys);

			if ((uint32_t)act_cys > (arr+1)) {
				printf("val pwmWrite 0 <= X <= 1024\n");
				printf("Or you can set new range by yourself by pwmSetRange(range)\n");
				return;
			}

			//keep period the same, clear act_cys to 0 first
			if (wiringPiDebug)
				printf(">>func:%s no:%d\n", __func__, __LINE__);
			per0 = readR(SUNXI_PWM_PERIOD);

			if (wiringPiDebug)
				printf("read reg val: 0x%x\n", per0);

			per0 &= 0xffff0000;
			act_cys &= 0xffff;
			act_cys |= per0;

			if (wiringPiDebug)
				printf("write reg val: 0x%x\n", act_cys);

			writeR(act_cys, SUNXI_PWM_PERIOD);
			delay(1);
			print_pwm_reg();

			break;
		case PI_MODEL_ZERO:

			if ((act_cys < 0) || (act_cys > 65535)) {
				fprintf (stderr, "gpio: range must be between 0 and 65535\n");
				exit (1) ;
			}

			H618_set_pwm_reg(pin,&sunxi_gpio_info_t);

			arr = sunxi_pwm_get_period();

			if (wiringPiDebug)
				printf("==> no:%d period now is :%d,act_val to be set:%d\n", __LINE__, arr, act_cys);

			if ((uint32_t)act_cys > (arr+1)) {
				printf("val pwmWrite 0 <= X <= 1024\n");
				printf("Or you can set new range by yourself by pwmSetRange(range)\n");
				return;
			}

			//keep period the same, clear act_cys to 0 first
			if (wiringPiDebug)
				printf(">>func:%s no:%d\n", __func__, __LINE__);
			per0 = readR(SUNXI_PWM_PERIOD);

			if (wiringPiDebug)
				printf("read reg val: 0x%x\n", per0);

			per0 &= 0xffff0000;
			act_cys &= 0xffff;
			act_cys |= per0;

			if (wiringPiDebug)
				printf("write reg val: 0x%x\n", act_cys);

			writeR(act_cys, SUNXI_PWM_PERIOD);
			delay(1);
			print_pwm_reg();

			break;

		default:
			break;
	}
}


/*
 * pwmSetMode:
 *	Select the native "balanced" mode, or standard mark:space mode
 *********************************************************************************
 */

void pwmSetMode(int pin,int mode)
{
    if (BananaPiModel == PI_MODEL_BERRY || BananaPiModel == PI_MODEL_BERRY) {
        H618_set_pwm_reg(pin,&sunxi_gpio_info_t);
    }

    sunxi_pwm_set_mode(mode);
    return;
}

/*
 * pwmSetRange:
 *	Set the PWM range register. We set both range registers to the same
 *	value. If you want different in your own code, then write your own.
 *********************************************************************************
 */

void pwmSetRange(int pin,unsigned int range)
{
	if ((pin & PI_GPIO_MASK) == 0) {
		if (wiringPiMode == WPI_MODE_PINS)
			pin = pinToGpio[pin];
		else if (wiringPiMode == WPI_MODE_PHYS)
			pin = physToGpio[pin];
		else if (wiringPiMode != WPI_MODE_GPIO)
			return;
	}

	if (-1 == pin) {
		printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
		return;
	}

	sunxi_pwm_set_period(pin,range);
	return;
}

/*
 * pwmSetClock:
 *	Set/Change the PWM clock. Originally my code, but changed
 *	(for the better!) by Chris Hall, <chris@kchall.plus.com>
 *	after further study of the manual and testing with a 'scope
 *********************************************************************************
 */

void pwmSetClock(int pin,int divisor)
{
	if ((pin & PI_GPIO_MASK) == 0) {
		if (wiringPiMode == WPI_MODE_PINS)
			pin = pinToGpio[pin];
		else if (wiringPiMode == WPI_MODE_PHYS)
			pin = physToGpio[pin];
		else if (wiringPiMode != WPI_MODE_GPIO)
			return;
	}

	if (-1 == pin) {
		printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
		return;
	}

	sunxi_pwm_set_clk(pin,divisor);
	return;
}

/*
 * gpioClockSet:
 *	Set the frequency on a GPIO clock pin
 *********************************************************************************
 */

void gpioClockSet (int pin, int freq)
{
  int divi, divr, divf ;

  pin &= 63 ;

  /**/ if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio [pin] ;
  else if (wiringPiMode != WPI_MODE_GPIO)
    return ;
  
  divi = 19200000 / freq ;
  divr = 19200000 % freq ;
  divf = (int)((double)divr * 4096.0 / 19200000.0) ;

  if (divi > 4095)
    divi = 4095 ;

  *(clk + gpioToClkCon [pin]) = BCM_PASSWORD | GPIO_CLOCK_SOURCE ;		// Stop GPIO Clock
  while ((*(clk + gpioToClkCon [pin]) & 0x80) != 0)				// ... and wait
    ;

  *(clk + gpioToClkDiv [pin]) = BCM_PASSWORD | (divi << 12) | divf ;		// Set dividers
  *(clk + gpioToClkCon [pin]) = BCM_PASSWORD | 0x10 | GPIO_CLOCK_SOURCE ;	// Start Clock
}


/*
 * wiringPiFindNode:
 *      Locate our device node
 *********************************************************************************
 */

struct wiringPiNodeStruct *wiringPiFindNode (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  while (node != NULL)
    if ((pin >= node->pinBase) && (pin <= node->pinMax))
      return node ;
    else
      node = node->next ;

  return NULL ;
}


/*
 * wiringPiNewNode:
 *	Create a new GPIO node into the wiringPi handling system
 *********************************************************************************
 */

static         void pinModeDummy             (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int mode)  { return ; }
static         void pullUpDnControlDummy     (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int pud)   { return ; }
//static unsigned int digitalRead8Dummy        (UNU struct wiringPiNodeStruct *node, UNU int UNU pin)            { return 0 ; }
//static         void digitalWrite8Dummy       (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static          int digitalReadDummy         (UNU struct wiringPiNodeStruct *node, UNU int UNU pin)            { return LOW ; }
static         void digitalWriteDummy        (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static         void pwmWriteDummy            (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }
static          int analogReadDummy          (UNU struct wiringPiNodeStruct *node, UNU int pin)            { return 0 ; }
static         void analogWriteDummy         (UNU struct wiringPiNodeStruct *node, UNU int pin, UNU int value) { return ; }

struct wiringPiNodeStruct *wiringPiNewNode (int pinBase, int numPins)
{
  int    pin ;
  struct wiringPiNodeStruct *node ;

// Minimum pin base is 64

  if (pinBase < 64)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: pinBase of %d is < 64\n", pinBase) ;

// Check all pins in-case there is overlap:

  for (pin = pinBase ; pin < (pinBase + numPins) ; ++pin)
    if (wiringPiFindNode (pin) != NULL)
      (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Pin %d overlaps with existing definition\n", pin) ;

  node = (struct wiringPiNodeStruct *)calloc (sizeof (struct wiringPiNodeStruct), 1) ;	// calloc zeros
  if (node == NULL)
    (void)wiringPiFailure (WPI_FATAL, "wiringPiNewNode: Unable to allocate memory: %s\n", strerror (errno)) ;

  node->pinBase          = pinBase ;
  node->pinMax           = pinBase + numPins - 1 ;
  node->pinMode          = pinModeDummy ;
  node->pullUpDnControl  = pullUpDnControlDummy ;
  node->digitalRead      = digitalReadDummy ;
//node->digitalRead8     = digitalRead8Dummy ;
  node->digitalWrite     = digitalWriteDummy ;
//node->digitalWrite8    = digitalWrite8Dummy ;
  node->pwmWrite         = pwmWriteDummy ;
  node->analogRead       = analogReadDummy ;
  node->analogWrite      = analogWriteDummy ;
  node->next             = wiringPiNodes ;
  wiringPiNodes          = node ;

  return node ;
}


#ifdef notYetReady
/*
 * pinED01:
 * pinED10:
 *	Enables edge-detect mode on a pin - from a 0 to a 1 or 1 to 0
 *	Pin must already be in input mode with appropriate pull up/downs set.
 *********************************************************************************
 */

void pinEnableED01Pi (int pin)
{
  pin = pinToGpio [pin & 63] ;
}
#endif

/*
 *********************************************************************************
 * Core Functions
 *********************************************************************************
 */

/*
 * pinModeAlt:
 *	This is an un-documented special to let you set any pin to any mode
 *********************************************************************************
 */

void pinModeAlt (int pin, int mode)
{
  int fSel, shift ;

  setupCheck ("pinModeAlt") ;

}


/*
 * pinMode:
 *	Sets the mode of a pin to be input, output or PWM output
 *********************************************************************************
 */
void pinMode (int pin, int mode)
{
	struct wiringPiNodeStruct *node = wiringPiNodes ;

	setupCheck ("pinMode") ;

	if (wiringPiDebug)
		printf("PinMode: pin:%d,mode:%d\n", pin, mode);

	if ((pin & PI_GPIO_MASK) == 0) {
		if (wiringPiMode == WPI_MODE_PINS) {
			pin = pinToGpio[pin];
		} else if (wiringPiMode == WPI_MODE_PHYS)
			pin = physToGpio[pin];
		else if (wiringPiMode != WPI_MODE_GPIO)
			return;

		if (-1 == pin) {
			printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", 
					__func__,  __LINE__, pin);
			return;
		}

		if (mode == INPUT) {
			BananaPi_set_gpio_mode(pin, INPUT);
			return;
		} 
		else if (mode == OUTPUT) {
			BananaPi_set_gpio_mode(pin, OUTPUT);
			return ;
		}
		else if (mode == PWM_OUTPUT) {
			BananaPi_set_gpio_mode(pin, PWM_OUTPUT);
			return;
		}
		else
			return;
	} 
	else
	{
		if ((node = wiringPiFindNode (pin)) != NULL)
			node->pinMode(node, pin, mode);

		return ;
	}
}


/*
 * pullUpDownCtrl:
 *	Control the internal pull-up/down resistors on a GPIO pin.
 *********************************************************************************
 */

void pullUpDnControl (int pin, int pud)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  setupCheck ("pullUpDnControl") ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
  {
    /**/ if (wiringPiMode == WPI_MODE_PINS)
      pin = pinToGpio [pin] ;
    else if (wiringPiMode == WPI_MODE_PHYS)
      pin = physToGpio [pin] ;
    else if (wiringPiMode != WPI_MODE_GPIO)
      return ;

	if (pud == PUD_OFF) {
		BananaPi_set_gpio_pullUpDnControl(pin, PUD_OFF);
	}
	else if (pud == PUD_UP){
		BananaPi_set_gpio_pullUpDnControl(pin, PUD_UP);
	}
	else if (pud == PUD_DOWN){
		BananaPi_set_gpio_pullUpDnControl(pin, PUD_DOWN);
	}
  }
  else						// Extension module
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->pullUpDnControl (node, pin, pud) ;
    return ;
  }
}


/*
 * digitalRead:
 *	Read the value of a given Pin, returning HIGH or LOW
 *********************************************************************************
 */
int digitalRead (int pin)
{
	char c ;
	int ret;
	struct wiringPiNodeStruct *node = wiringPiNodes ;

	if ((pin & PI_GPIO_MASK) == 0)
	{
		if (wiringPiMode == WPI_MODE_GPIO_SYS)	// Sys mode
		{
			if (sysFds [pin] == -1)
				return LOW ;

			ret = lseek(sysFds [pin], 0L, SEEK_SET) ;
			ret = read(sysFds [pin], &c, 1);

			if (ret < 0)
				return -1;
			
			return (c == '0') ? LOW : HIGH ;
		} 
		else if (wiringPiMode == WPI_MODE_PINS)
			pin = pinToGpio[pin];
		else if (wiringPiMode == WPI_MODE_PHYS)
			pin = physToGpio[pin];
		else if (wiringPiMode != WPI_MODE_GPIO)
			return -1;

		if (pin == -1)
		{
			printf("[%s %d]Pin %d is invalid, please check it over!\n", __func__, __LINE__, pin);
			return LOW;
		}

		return BananaPi_digitalRead(pin);   
	}
	else 
	{
		if ((node = wiringPiFindNode (pin)) == NULL)
			return LOW ;
		
		return node->digitalRead (node, pin) ;
	}
}


/*
 * digitalRead8:
 *	Read 8-bits (a byte) from given start pin.
 *********************************************************************************

unsigned int digitalRead8 (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
    return 0 ;
  else
  {
    if ((node = wiringPiFindNode (pin)) == NULL)
      return LOW ;
    return node->digitalRead8 (node, pin) ;
  }
}
 */


/*
 * digitalWrite:
 *	Set an output bit
 *********************************************************************************
 */
void digitalWrite (int pin, int value)
{
	struct wiringPiNodeStruct *node = wiringPiNodes ;
	int ret;

	if ((pin & PI_GPIO_MASK) == 0) 
	{
		if (wiringPiMode == WPI_MODE_GPIO_SYS)
		{
			if (sysFds [pin] != -1) {
				if (value == LOW)
				{
					ret = write (sysFds [pin], "0\n", 2);
					if (ret < 0)
						return;
				}
				else
				{
					ret = write (sysFds [pin], "1\n", 2);
					if (ret < 0)
						return;
				}
			}
			return ;
		} 
		else if (wiringPiMode == WPI_MODE_PINS) 
		{
			pin = pinToGpio[pin];
		}
		else if (wiringPiMode == WPI_MODE_PHYS)
			pin = physToGpio[pin];
		else 
			return;

		if(-1 == pin)
		{
			printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
			printf("[%s:L%d] the mode is: %d, please check it over!\n", __func__,  __LINE__, wiringPiMode);

			return;
		}

		BananaPi_digitalWrite(pin, value);	  
	} 
	else 
	{
		if ((node = wiringPiFindNode(pin)) != NULL)
			node->digitalWrite(node, pin, value);
	}

	return;
}

/*
 * digitalWrite8:
 *	Set an output 8-bit byte on the device from the given pin number
 *********************************************************************************

void digitalWrite8 (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((pin & PI_GPIO_MASK) == 0)		// On-Board Pin
    return ;
  else
  {
    if ((node = wiringPiFindNode (pin)) != NULL)
      node->digitalWrite8 (node, pin, value) ;
  }
}
 */

/*
 * pwmWrite:
 *	Set an output PWM value
 *********************************************************************************
 */

void pwmWrite(int pin, int value) {
	struct wiringPiNodeStruct *node = wiringPiNodes;

	if (pinToGpio == 0 || physToGpio == 0) {
		printf("please call wiringPiSetup first.\n");
		return;
	}

	if (pwmmode == 1) {
		sunxi_pwm_set_mode(1);
	} else {
		sunxi_pwm_set_mode(0);
	}

	// On-Board Pin needto fix me Jim
	if (pin < MAX_PIN_NUM) {
		if (wiringPiMode == WPI_MODE_PINS)
			pin = pinToGpio [pin];
		else if (wiringPiMode == WPI_MODE_PHYS)
			pin = physToGpio[pin];
		else if (wiringPiMode != WPI_MODE_GPIO)
			return;

		if (-1 == pin) {
			printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__, __LINE__, pin);
			return;
		}
		sunxi_pwm_set_act(pin,value);
	} else {
		printf("not on board :%s,%d\n", __func__, __LINE__);
		if ((node = wiringPiFindNode(pin)) != NULL) {
			if (wiringPiDebug)
				printf("Jim find node%s,%d\n", __func__, __LINE__);
			node->digitalWrite(node, pin, value);
		}
	}

	if (wiringPiDebug)
		printf("this fun is ok now %s,%d\n", __func__, __LINE__);
 
	return;
}

/*
 * analogRead:
 *	Read the analog value of a given Pin. 
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

int analogRead (int pin)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((node = wiringPiFindNode (pin)) == NULL)
    return 0 ;
  else
    return node->analogRead (node, pin) ;
}


/*
 * analogWrite:
 *	Write the analog value to the given Pin. 
 *	There is no on-board Pi analog hardware,
 *	so this needs to go to a new node.
 *********************************************************************************
 */

void analogWrite (int pin, int value)
{
  struct wiringPiNodeStruct *node = wiringPiNodes ;

  if ((node = wiringPiFindNode (pin)) == NULL)
    return ;

  node->analogWrite (node, pin, value) ;
}


/*
 * pwmToneWrite:
 *	Pi Specific.
 *      Output the given frequency on the Pi's PWM pin
 *********************************************************************************
 */

void pwmToneWrite (int pin, int freq)
{
	if ((pin & PI_GPIO_MASK) == 0) {
		if (wiringPiMode == WPI_MODE_PINS)
			pin = pinToGpio[pin];
		else if (wiringPiMode == WPI_MODE_PHYS)
			pin = physToGpio[pin];
		else if (wiringPiMode != WPI_MODE_GPIO)
			return;
	}

	if (-1 == pin) {
		printf("[%s:L%d] the pin:%d is invaild,please check it over!\n", __func__,  __LINE__, pin);
		return;
	}

	sunxi_pwm_set_tone(pin,freq);
	return;
}


/*
 * digitalWriteByte:
 * digitalReadByte:
 *	Pi Specific
 *	Write an 8-bit byte to the first 8 GPIO pins - try to do it as
 *	fast as possible.
 *	However it still needs 2 operations to set the bits, so any external
 *	hardware must not rely on seeing a change as there will be a change 
 *	to set the outputs bits to zero, then another change to set the 1's
 *	Reading is just bit fiddling.
 *	These are wiringPi pin numbers 0..7, or BCM_GPIO pin numbers
 *	17, 18, 22, 23, 24, 24, 4 on a Pi v1 rev 0-3
 *	17, 18, 27, 23, 24, 24, 4 on a Pi v1 rev 3 onwards or B+, 2, 3, zero
 *********************************************************************************
 */

void digitalWriteByte (const int value)
{
	int mask = 1 ;
	int pin ;

	for (pin = 0 ; pin < 8 ; ++pin) {
		digitalWrite (pin, (value >> pin) & mask) ;
	}
	
	return ;
}

unsigned int digitalReadByte (void)
{
	int pin, x ;
	uint32_t data = 0 ;

	for (pin = 7 ; pin >= 0 ; --pin){
		x = digitalRead(pin);
		
		data = (data << 1) | x ;
	}
	
	return data ;
}

/*
 * digitalWriteByte2:
 * digitalReadByte2:
 *	Pi Specific
 *	Write an 8-bit byte to the second set of 8 GPIO pins. This is marginally
 *	faster than the first lot as these are consecutive BCM_GPIO pin numbers.
 *	However they overlap with the original read/write bytes.
 *********************************************************************************
 */

void digitalWriteByte2 (const int value)
{
  register int mask = 1 ;
  register int pin ;

  /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)
  {
    for (pin = 20 ; pin < 28 ; ++pin)
    {
      digitalWrite (pin, value & mask) ;
      mask <<= 1 ;
    }
    return ;
  }
  else
  {
    *(gpio + gpioToGPCLR [0]) = (~value & 0xFF) << 20 ; // 0x0FF00000; ILJ > CHANGE: Old causes glitch
    *(gpio + gpioToGPSET [0]) = ( value & 0xFF) << 20 ;
  }
}

unsigned int digitalReadByte2 (void)
{
  int pin, x ;
  uint32_t data = 0 ;

  /**/ if (wiringPiMode == WPI_MODE_GPIO_SYS)
  {
    for (pin = 20 ; pin < 28 ; ++pin)
    {
      x = digitalRead (pin) ;
      data = (data << 1) | x ;
    }
  }
  else 
    data = ((*(gpio + gpioToGPLEV [0])) >> 20) & 0xFF ; // First bank for these pins

  return data ;
}


/*
 * waitForInterrupt:
 *	Pi Specific.
 *	Wait for Interrupt on a GPIO pin.
 *	This is actually done via the /sys/class/gpio interface regardless of
 *	the wiringPi access mode in-use. Maybe sometime it might get a better
 *	way for a bit more efficiency.
 *********************************************************************************
 */

int waitForInterrupt (int pin, int mS)
{
  int fd, x ;
  uint8_t c ;
  struct pollfd polls ;
  int ret;

  /**/ if (wiringPiMode == WPI_MODE_PINS)
    pin = pinToGpio [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    pin = physToGpio [pin] ;

  if ((fd = sysFds [pin]) == -1)
    return -2 ;

// Setup poll structure

  polls.fd     = fd ;
  polls.events = POLLPRI | POLLERR ;

// Wait for it ...

  x = poll (&polls, 1, mS) ;

// If no error, do a dummy read to clear the interrupt
//	A one character read appars to be enough.

  if (x > 0)
  {
    lseek (fd, 0, SEEK_SET) ;	// Rewind
    ret = read (fd, &c, 1) ;	// Read & clear
   if (ret < 0)
  	return -1;
  }

  return x ;
}


/*
 * interruptHandler:
 *	This is a thread and gets started to wait for the interrupt we're
 *	hoping to catch. It will call the user-function when the interrupt
 *	fires.
 *********************************************************************************
 */

static void *interruptHandler (UNU void *arg)
{
  int myPin ;

  (void)piHiPri (55) ;	// Only effective if we run as root

  myPin   = pinPass ;
  pinPass = -1 ;

  for (;;)
    if (waitForInterrupt (myPin, -1) > 0)
      isrFunctions [myPin] () ;

  return NULL ;
}


/*
 * wiringPiISR:
 *	Pi Specific.
 *	Take the details and create an interrupt handler that will do a call-
 *	back to the user supplied function.
 *********************************************************************************
 */

int wiringPiISR (int pin, int mode, void (*function)(void))
{
  pthread_t threadId ;
  const char *modeS ;
  char fName   [64] ;
  char  pinS [8] ;
  pid_t pid ;
  int   count, i ;
  char  c ;
  int   bcmGpioPin ;
  int ret;

  if ((pin < 0) || (pin > 63))
    return wiringPiFailure (WPI_FATAL, "wiringPiISR: pin must be 0-63 (%d)\n", pin) ;

  /**/ if (wiringPiMode == WPI_MODE_UNINITIALISED)
    return wiringPiFailure (WPI_FATAL, "wiringPiISR: wiringPi has not been initialised. Unable to continue.\n") ;
  else if (wiringPiMode == WPI_MODE_PINS)
    bcmGpioPin = pinToGpio [pin] ;
  else if (wiringPiMode == WPI_MODE_PHYS)
    bcmGpioPin = physToGpio [pin] ;
  else
    bcmGpioPin = pin ;

// Now export the pin and set the right edge
//	We're going to use the gpio program to do this, so it assumes
//	a full installation of wiringPi. It's a bit 'clunky', but it
//	is a way that will work when we're running in "Sys" mode, as
//	a non-root user. (without sudo)

  if (mode != INT_EDGE_SETUP)
  {
    /**/ if (mode == INT_EDGE_FALLING)
      modeS = "falling" ;
    else if (mode == INT_EDGE_RISING)
      modeS = "rising" ;
    else
      modeS = "both" ;

    sprintf (pinS, "%d", pin) ;

    if ((pid = fork ()) < 0)	// Fail
      return wiringPiFailure (WPI_FATAL, "wiringPiISR: fork failed: %s\n", strerror (errno)) ;

    if (pid == 0)	// Child, exec
    {
      /**/ if (access ("/usr/local/bin/gpio", X_OK) == 0)
      {
	execl ("/usr/local/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL) ;
	return wiringPiFailure (WPI_FATAL, "wiringPiISR: execl failed: %s\n", strerror (errno)) ;
      }
      else if (access ("/usr/bin/gpio", X_OK) == 0)
      {
	execl ("/usr/bin/gpio", "gpio", "edge", pinS, modeS, (char *)NULL) ;
	return wiringPiFailure (WPI_FATAL, "wiringPiISR: execl failed: %s\n", strerror (errno)) ;
      }
      else
	return wiringPiFailure (WPI_FATAL, "wiringPiISR: Can't find gpio program\n") ;
    }
    else		// Parent, wait
      wait (NULL) ;
  }

// Now pre-open the /sys/class node - but it may already be open if
//	we are in Sys mode...

  if (sysFds [bcmGpioPin] == -1)
  {
    sprintf (fName, "/sys/class/gpio/gpio%d/value", bcmGpioPin) ;
    if ((sysFds [bcmGpioPin] = open (fName, O_RDWR)) < 0)
      return wiringPiFailure (WPI_FATAL, "wiringPiISR: unable to open %s: %s\n", fName, strerror (errno)) ;
  }

// Clear any initial pending interrupt

  ioctl (sysFds [bcmGpioPin], FIONREAD, &count) ;
  for (i = 0 ; i < count ; ++i){
    ret = read (sysFds [bcmGpioPin], &c, 1) ;
	if (ret < 0)
		return -1;
  }

  isrFunctions [pin] = function ;

  pthread_mutex_lock (&pinMutex) ;
    pinPass = pin ;
    pthread_create (&threadId, NULL, interruptHandler, NULL) ;
    while (pinPass != -1)
      delay (1) ;
  pthread_mutex_unlock (&pinMutex) ;

  return 0 ;
}


/*
 * initialiseEpoch:
 *	Initialise our start-of-time variable to be the current unix
 *	time in milliseconds and microseconds.
 *********************************************************************************
 */

static void initialiseEpoch (void)
{
#ifdef	OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  epochMilli = (uint64_t)tv.tv_sec * (uint64_t)1000    + (uint64_t)(tv.tv_usec / 1000) ;
  epochMicro = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)(tv.tv_usec) ;
#else
  struct timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  epochMilli = (uint64_t)ts.tv_sec * (uint64_t)1000    + (uint64_t)(ts.tv_nsec / 1000000L) ;
  epochMicro = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec /    1000L) ;
#endif
}


/*
 * delay:
 *	Wait for some number of milliseconds
 *********************************************************************************
 */

void delay (unsigned int howLong)
{
  struct timespec sleeper, dummy ;

  sleeper.tv_sec  = (time_t)(howLong / 1000) ;
  sleeper.tv_nsec = (long)(howLong % 1000) * 1000000 ;

  nanosleep (&sleeper, &dummy) ;
}


/*
 * delayMicroseconds:
 *	This is somewhat intersting. It seems that on the Pi, a single call
 *	to nanosleep takes some 80 to 130 microseconds anyway, so while
 *	obeying the standards (may take longer), it's not always what we
 *	want!
 *
 *	So what I'll do now is if the delay is less than 100uS we'll do it
 *	in a hard loop, watching a built-in counter on the ARM chip. This is
 *	somewhat sub-optimal in that it uses 100% CPU, something not an issue
 *	in a microcontroller, but under a multi-tasking, multi-user OS, it's
 *	wastefull, however we've no real choice )-:
 *
 *      Plan B: It seems all might not be well with that plan, so changing it
 *      to use gettimeofday () and poll on that instead...
 *********************************************************************************
 */

void delayMicrosecondsHard (unsigned int howLong)
{
  struct timeval tNow, tLong, tEnd ;

  gettimeofday (&tNow, NULL) ;
  tLong.tv_sec  = howLong / 1000000 ;
  tLong.tv_usec = howLong % 1000000 ;
  timeradd (&tNow, &tLong, &tEnd) ;

  while (timercmp (&tNow, &tEnd, <))
    gettimeofday (&tNow, NULL) ;
}

void delayMicroseconds (unsigned int howLong)
{
  struct timespec sleeper ;
  unsigned int uSecs = howLong % 1000000 ;
  unsigned int wSecs = howLong / 1000000 ;

  /**/ if (howLong ==   0)
    return ;
  else if (howLong  < 100)
    delayMicrosecondsHard (howLong) ;
  else
  {
    sleeper.tv_sec  = wSecs ;
    sleeper.tv_nsec = (long)(uSecs * 1000L) ;
    nanosleep (&sleeper, NULL) ;
  }
}


/*
 * millis:
 *	Return a number of milliseconds as an unsigned int.
 *	Wraps at 49 days.
 *********************************************************************************
 */

unsigned int millis (void)
{
  uint64_t now ;

#ifdef	OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000 + (uint64_t)(tv.tv_usec / 1000) ;

#else
  struct  timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  now  = (uint64_t)ts.tv_sec * (uint64_t)1000 + (uint64_t)(ts.tv_nsec / 1000000L) ;
#endif

  return (uint32_t)(now - epochMilli) ;
}


/*
 * micros:
 *	Return a number of microseconds as an unsigned int.
 *	Wraps after 71 minutes.
 *********************************************************************************
 */

unsigned int micros (void)
{
  uint64_t now ;
#ifdef	OLD_WAY
  struct timeval tv ;

  gettimeofday (&tv, NULL) ;
  now  = (uint64_t)tv.tv_sec * (uint64_t)1000000 + (uint64_t)tv.tv_usec ;
#else
  struct  timespec ts ;

  clock_gettime (CLOCK_MONOTONIC_RAW, &ts) ;
  now  = (uint64_t)ts.tv_sec * (uint64_t)1000000 + (uint64_t)(ts.tv_nsec / 1000) ;
#endif


  return (uint32_t)(now - epochMicro) ;
}

/*
 * wiringPiVersion:
 *	Return our current version number
 *********************************************************************************
 */

void wiringPiVersion (int *major, int *minor)
{
  *major = VERSION_MAJOR ;
  *minor = VERSION_MINOR ;
}

void set_soc_info(void)
{
	switch (BananaPiModel)
	{
		case PI_MODEL_ZERO:
			sunxi_gpio_info_t.gpio_base_addr = H6_GPIO_BASE_ADDR;
			sunxi_gpio_info_t.r_gpio_base_addr = H6_R_GPIO_BASE_ADDR;
			sunxi_gpio_info_t.gpio_base_offset = 0x0;
			sunxi_gpio_info_t.r_gpio_base_offset = 0x0;
			sunxi_gpio_info_t.pwm_base_addr = H6_PWM_BASE;
			break;
		case PI_MODEL_BERRY:
			sunxi_gpio_info_t.gpio_base_addr = H6_GPIO_BASE_ADDR;
			sunxi_gpio_info_t.r_gpio_base_addr = H6_R_GPIO_BASE_ADDR;
			sunxi_gpio_info_t.gpio_base_offset = 0x0;
			sunxi_gpio_info_t.r_gpio_base_offset = 0x0;
			sunxi_gpio_info_t.pwm_base_addr = H6_PWM_BASE;
			break;
		default:
			break;
	}

	switch (BananaPiModel)
	{
		case PI_MODEL_ZERO:
			sunxi_gpio_info_t.pwm_en = SUNXI_V2_PWM_EN_REG;
			sunxi_gpio_info_t.pwm_type = SUNXI_V2_PWM_TYPE;	// H616
			sunxi_gpio_info_t.pwm_bit_act = SUNXI_V2_PWM_ACT_STA;
			sunxi_gpio_info_t.pwm_bit_sclk = SUNXI_V2_PWM_SCLK_GATING;
			sunxi_gpio_info_t.pwm_bit_mode = SUNXI_V2_PWM_MS_MODE;
			sunxi_gpio_info_t.pwm_bit_pulse = SUNXI_V2_PWM_PUL_START;
			break;
		case PI_MODEL_BERRY:
			sunxi_gpio_info_t.pwm_en = SUNXI_V2_PWM_EN_REG;
			sunxi_gpio_info_t.pwm_type = SUNXI_V2_PWM_TYPE;	// H616
			sunxi_gpio_info_t.pwm_bit_act = SUNXI_V2_PWM_ACT_STA;
			sunxi_gpio_info_t.pwm_bit_sclk = SUNXI_V2_PWM_SCLK_GATING;
			sunxi_gpio_info_t.pwm_bit_mode = SUNXI_V2_PWM_MS_MODE;
			sunxi_gpio_info_t.pwm_bit_pulse = SUNXI_V2_PWM_PUL_START;
			break;
		default:
			break;
	}

}

/*
 * wiringPiSetup:
 *	Must be called once at the start of your program execution.
 *
 * Default setup: Initialises the system into wiringPi Pin mode and uses the
 *	memory mapped hardware directly.
 *
 * Changed now to revert to "gpio" mode if we're running on a Compute Module.
 *********************************************************************************
 */

int wiringPiSetup (void)
{
	int fd;

	// It's actually a fatal error to call any of the wiringPiSetup routines more than once,
	//	(you run out of file handles!) but I'm fed-up with the useless twats who email
	//	me bleating that there is a bug in my code, so screw-em.

	if (wiringPiSetuped)
		return 0 ;

	wiringPiSetuped = TRUE ;

	if (getenv (ENV_DEBUG) != NULL)
		wiringPiDebug = TRUE ;

	if (getenv (ENV_CODES) != NULL)
		wiringPiReturnCodes = TRUE ;

	if (wiringPiDebug)
	printf ("wiringPi: wiringPiSetup called\n") ;

	piBoardId (&BananaPiModel) ;

	wiringPiMode = WPI_MODE_PINS ;

	switch (BananaPiModel)
	{
		case PI_MODEL_BERRY:
			pinToGpio =  pinToGpio_M4_BERRY;
			physToGpio = physToGpio_M4_BERRY;
			BANANAPI_PIN_MASK = BANANAPI_PIN_MASK_M4_BERRY;
			break;
		case PI_MODEL_ZERO:
			pinToGpio =  pinToGpio_M4_ZERO;
			physToGpio = physToGpio_M4_ZERO;
			BANANAPI_PIN_MASK = BANANAPI_PIN_MASK_M4_ZERO;
			break;
		default:
			printf ("Oops - unable to determine board type... model: %d\n", BananaPiModel);
			break ;
	}

	set_soc_info();

	// Open the master /dev/ memory control device
	// Device strategy: December 2016:
	//	Try /dev/mem. If that fails, then 
	//	try /dev/gpiomem. If that fails then game over.
	if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC)) < 0){
		if ((fd = open ("/dev/gpiomem", O_RDWR | O_SYNC | O_CLOEXEC) ) >= 0){	// We're using gpiomem
			piGpioBase   = 0 ;
			usingGpioMem = TRUE ;
		}
		else
			return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: Unable to open /dev/mem or /dev/gpiomem: %s.\n"
			"  Aborting your program because if it can not access the GPIO\n"
			"  hardware then it most certianly won't work\n"
			"  Try running with sudo?\n", strerror (errno)) ;
	}

	switch (BananaPiModel)
	{
		case PI_MODEL_ZERO:
	
			sunxi_gpio_info_t.pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, sunxi_gpio_info_t.pwm_base_addr);
			if ((int32_t)(unsigned long)sunxi_gpio_info_t.pwm == -1)
				return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (PWM) failed: %s\n", strerror(errno));

			sunxi_gpio_info_t.gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, sunxi_gpio_info_t.gpio_base_addr);
			if ((int32_t)(unsigned long)sunxi_gpio_info_t.gpio == -1)
				return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror(errno));

			sunxi_gpio_info_t.r_gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, sunxi_gpio_info_t.r_gpio_base_addr);
			if ((int32_t)(unsigned long)sunxi_gpio_info_t.r_gpio == -1)
				return wiringPiFailure(WPI_ALMOST,  "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror(errno));

			break;
		case PI_MODEL_BERRY:
	
			sunxi_gpio_info_t.pwm = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, sunxi_gpio_info_t.pwm_base_addr);
			if ((int32_t)(unsigned long)sunxi_gpio_info_t.pwm == -1)
				return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (PWM) failed: %s\n", strerror(errno));

			sunxi_gpio_info_t.gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, sunxi_gpio_info_t.gpio_base_addr);
			if ((int32_t)(unsigned long)sunxi_gpio_info_t.gpio == -1)
				return wiringPiFailure(WPI_ALMOST, "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror(errno));

			sunxi_gpio_info_t.r_gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, sunxi_gpio_info_t.r_gpio_base_addr);
			if ((int32_t)(unsigned long)sunxi_gpio_info_t.r_gpio == -1)
				return wiringPiFailure(WPI_ALMOST,  "wiringPiSetup: mmap (GPIO) failed: %s\n", strerror(errno));

			break;

		default:

			printf("model %d is error\n", BananaPiModel);
			break ;
	}

	initialiseEpoch () ;

	return 0 ;
}

/*
 * wiringPiSetupGpio:
 *	Must be called once at the start of your program execution.
 *
 * GPIO setup: Initialises the system into GPIO Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupGpio (void)
{
	(void)wiringPiSetup () ;

	if (wiringPiDebug)
		printf ("wiringPi: wiringPiSetupGpio called\n") ;

	wiringPiMode = WPI_MODE_GPIO ;

	return 0 ;
}


/*
 * wiringPiSetupPhys:
 *	Must be called once at the start of your program execution.
 *
 * Phys setup: Initialises the system into Physical Pin mode and uses the
 *	memory mapped hardware directly.
 *********************************************************************************
 */

int wiringPiSetupPhys (void)
{
  (void)wiringPiSetup () ;

  if (wiringPiDebug)
    printf ("wiringPi: wiringPiSetupPhys called\n") ;

  wiringPiMode = WPI_MODE_PHYS ;

  return 0 ;
}

/*
 * wiringPiSetupSys:
 *	Must be called once at the start of your program execution.
 *
 * Initialisation (again), however this time we are using the /sys/class/gpio
 *	interface to the GPIO systems - slightly slower, but always usable as
 *	a non-root user, assuming the devices are already exported and setup correctly.
 */

int wiringPiSetupSys (void)
{
	int pin ;
	int tmpGpio;
	char fName [128] ;

	if (wiringPiSysSetuped)
		return 0 ;

	wiringPiSysSetuped = TRUE ;

	if (getenv (ENV_DEBUG) != NULL)
		wiringPiDebug = TRUE ;

	if (getenv (ENV_CODES) != NULL)
		wiringPiReturnCodes = TRUE ;

	if (wiringPiDebug)
		printf ("wiringPi: wiringPiSetupSys called\n") ;

	piBoardId (&BananaPiModel) ;

	switch (BananaPiModel)
	{
		case PI_MODEL_BERRY:
			pinToGpio =  pinToGpio_M4_BERRY;
			physToGpio = physToGpio_M4_BERRY;
			break;
		case PI_MODEL_ZERO:
			pinToGpio =  pinToGpio_M4_ZERO;
			physToGpio = physToGpio_M4_ZERO;
			break;
		default:
			break ;
	}

	// Open and scan the directory, looking for exported GPIOs, and pre-open
	//	the 'value' interface to speed things up for later
	for (pin = 1 ; pin < 28 ; ++pin)
	{
		tmpGpio = pinToGpio[pin];
		sprintf (fName, "/sys/class/gpio/gpio%d/value", tmpGpio) ;
		sysFds [pin] = open (fName, O_RDWR) ;
	}

	initialiseEpoch () ;

	wiringPiMode = WPI_MODE_GPIO_SYS;

	return 0 ;
}


/*
 * Read register value helper  
 */
unsigned int readR(unsigned int addr)
{
	unsigned int val = 0;
	unsigned int mmap_base;
	unsigned int mmap_seek;

	switch (BananaPiModel)
	{
		default:
			
			val = 0;
			mmap_base = (addr & 0xfffff000);
			mmap_seek = ((addr - mmap_base) >> 2);
			if(mmap_base == SUNXI_PWM_BASE) {
				val = *(sunxi_gpio_info_t.pwm + mmap_seek);
				if (wiringPiDebug) {
					printf("BPI: PWM: readR addr[%x]=[%x]\n",addr,val);
				}
				return val;
			}

			if (addr >= sunxi_gpio_info_t.r_gpio_base_addr)
				val = *(sunxi_gpio_info_t.r_gpio + mmap_seek);
			else
				val = *(sunxi_gpio_info_t.gpio + mmap_seek);

			return val;
			
			break;
	}

	return -1;
}

/*
 * Wirte value to register helper
 */
void writeR(unsigned int val, unsigned int addr)
{
	unsigned int mmap_base;
	unsigned int mmap_seek;

	switch (BananaPiModel)
	{

		default:

			mmap_base = (addr & 0xfffff000);
			mmap_seek = ((addr - mmap_base) >> 2);
			if (mmap_base == SUNXI_PWM_BASE) {
				*(sunxi_gpio_info_t.pwm + mmap_seek) = val;
				if (wiringPiDebug ){
					printf("BPI: PWM: writeR addr[%x]=[%x]\n",addr,val);
				}
				return;
			}
				        
			if (addr >= sunxi_gpio_info_t.r_gpio_base_addr)
				*(sunxi_gpio_info_t.r_gpio + mmap_seek) = val;
			else
				*(sunxi_gpio_info_t.gpio + mmap_seek) = val;

			break;
	}
}

int BananaPi_get_gpio_mode(int pin)
{
	unsigned int regval = 0;
	unsigned int bank   = pin >> 5;
	unsigned int index  = pin - (bank << 5);
	unsigned int phyaddr = 0;
	unsigned char mode = -1;
	unsigned int grf_phyaddr = 0, ddr_phyaddr = 0;
	int offset;

	switch (BananaPiModel)
	{

		default:

			offset = ((index - ((index >> 3) << 3)) << 2);

			if (bank == 11)
				phyaddr = sunxi_gpio_info_t.r_gpio_base_addr + sunxi_gpio_info_t.r_gpio_base_offset + ((index >> 3) << 2);
			else
				phyaddr = sunxi_gpio_info_t.gpio_base_addr + sunxi_gpio_info_t.gpio_base_offset + (bank * 36) + ((index >> 3) << 2);

			/* Ignore unused gpio */
			if (BANANAPI_PIN_MASK[bank][index] != -1)
			{
				regval = readR(phyaddr);
				mode = (regval >> offset) & 7;
			}

			return mode;

			break;
	}

	return -1;
}

void H618_set_pwm_reg(int pin,sunxi_gpio_info *sunxi_gpio_info_ptr)
{
	switch(pin) {
		case 227:
		case 267:
			sunxi_gpio_info_ptr->pwm_period = SUNXI_V2_PWM1_PERIOD;
			sunxi_gpio_info_ptr->pwm_ctrl = SUNXI_V2_PWM1_CTRL_REG;
			sunxi_gpio_info_ptr->pwm_clk = SUNXI_V2_PWM1_CLK_REG;
			sunxi_gpio_info_ptr->pwm_bit_en = SUNXI_V2_PWM1_EN;
			break;
		case 226:
		case 268:
			sunxi_gpio_info_ptr->pwm_period = SUNXI_V2_PWM2_PERIOD;
			sunxi_gpio_info_ptr->pwm_ctrl = SUNXI_V2_PWM2_CTRL_REG;
			sunxi_gpio_info_ptr->pwm_clk = SUNXI_V2_PWM2_CLK_REG;
			sunxi_gpio_info_ptr->pwm_bit_en = SUNXI_V2_PWM2_EN;
			break;
		case 225:
		case 270:
			sunxi_gpio_info_ptr->pwm_period = SUNXI_V2_PWM4_PERIOD;
			sunxi_gpio_info_ptr->pwm_ctrl = SUNXI_V2_PWM4_CTRL_REG;
			sunxi_gpio_info_ptr->pwm_clk = SUNXI_V2_PWM4_CLK_REG;
			sunxi_gpio_info_ptr->pwm_bit_en = SUNXI_V2_PWM4_EN;
			break;
		case 224:
		case 269:
			sunxi_gpio_info_ptr->pwm_period = SUNXI_V2_PWM3_PERIOD;
			sunxi_gpio_info_ptr->pwm_ctrl = SUNXI_V2_PWM3_CTRL_REG;
			sunxi_gpio_info_ptr->pwm_clk = SUNXI_V2_PWM3_CLK_REG;
			sunxi_gpio_info_ptr->pwm_bit_en = SUNXI_V2_PWM3_EN;
			break;
		default:
			fprintf(stderr,"gpio: the pin you choose doesn't support hardware PWM\n");
	}
}


/*
 * Set GPIO Mode
 */
int BananaPi_set_gpio_mode(int pin, int mode)
{
	unsigned int regval = 0;
	unsigned int bank   = pin >> 5;
	unsigned int index  = pin - (bank << 5);
	unsigned int phyaddr = 0;
	int offset;
	unsigned int cru_phyaddr =0, grf_phyaddr = 0, gpio_phyaddr = 0, ddr_phyaddr = 0;
	unsigned int cru_val = 0;
	unsigned int temp = 0;
	unsigned int bit_enable;
	unsigned int grf_val = 0;

	switch (BananaPiModel)
	{
		default:

			offset = ((index - ((index >> 3) << 3)) << 2);

			if (bank == 11)
				phyaddr = sunxi_gpio_info_t.r_gpio_base_addr + sunxi_gpio_info_t.r_gpio_base_offset + ((index >> 3) << 2);
			else
				phyaddr = sunxi_gpio_info_t.gpio_base_addr + sunxi_gpio_info_t.gpio_base_offset + (bank * 36) + ((index >> 3) << 2);

			/* Ignore unused gpio */
			if (BANANAPI_PIN_MASK[bank][index] != -1)
			{
				regval = readR(phyaddr);
				if (wiringPiDebug)
					printf("Before read reg val: 0x%x offset:%d\n",regval,offset);
			
				if (wiringPiDebug)
					printf("Register[%#x]: %#x index:%d\n", phyaddr, regval, index);

				/* Set Input */
				if(INPUT == mode)
				{
					regval &= ~(7 << offset);
					writeR(regval, phyaddr);
					regval = readR(phyaddr);

					if (wiringPiDebug)
						printf("Input mode set over reg val: %#x\n",regval);
				}
				else if(OUTPUT == mode)
				{
					/* Set Output */
					regval &= ~(7 << offset);
					regval |=  (1 << offset);

					if (wiringPiDebug)
						printf("Out mode ready set val: 0x%x\n",regval);

					writeR(regval, phyaddr);
					regval = readR(phyaddr);

					if (wiringPiDebug)
						printf("Out mode get value: 0x%x\n",regval);
				}
				else if(PWM_OUTPUT == mode)
				{
					if (wiringPiDebug)
						printf("BPI: try wiringPi pin %d for PWM pin\n", pin);

					if (BananaPiModel == PI_MODEL_BERRY && pin != 179 && pin != 224 && pin != 225 && pin != 226 && pin != 227) {
						printf("the pin you choose doesn't support hardware PWM\n");
						printf("BPI:you can select wiringPi pin 179,224,225,226,227 for PWM pin\n");
						printf("or you can use it in softPwm mode\n");
						exit(1);
					} else if (BananaPiModel == PI_MODEL_ZERO && pin != 179 && pin != 224 && pin != 225 && pin != 226 && pin != 227) {
						printf("the pin you choose doesn't support hardware PWM\n");
						printf("BPI:you can select wiringPi pin 179,224,225,226,227 for PWM pin\n");
						printf("or you can use it in softPwm mode\n");
						exit(1);
					}

					H618_set_pwm_reg(pin,&sunxi_gpio_info_t);

					// set pin PWMx to pwm mode
					regval &= ~(7 << offset);
					if (BananaPiModel == PI_MODEL_ZERO)
						regval |= (0x5 << offset); // ALT5 PWM
					else if (BananaPiModel == PI_MODEL_BERRY)
						regval |= (0x5 << offset);
					else
						regval |= (0x3 << offset); // ALT3 PWM
					if (wiringPiDebug)
						printf(">>>>>line:%d PWM mode ready to set val: 0x%x\n", __LINE__, regval);

					writeR(regval, phyaddr);
					delayMicroseconds(200);
					regval = readR(phyaddr);

					if (wiringPiDebug)
						 printf("<<<<<PWM mode set over reg val: 0x%x\n", regval);

					//clear all reg
					writeR(0, SUNXI_PWM_CTRL_REG);
					writeR(0, SUNXI_PWM_PERIOD);

					//set default M:S to 1/2
					sunxi_pwm_set_period(pin,1024);
					sunxi_pwm_set_act(pin,512);
					sunxi_pwm_set_mode(PWM_MODE_MS);

					if (BananaPiModel == PI_MODEL_BERRY || BananaPiModel == PI_MODEL_ZERO)
						sunxi_pwm_set_clk(pin,1);  //default clk:24M
					else
						sunxi_pwm_set_clk(pin,PWM_CLK_DIV_120); //default clk:24M/120
					delayMicroseconds(200);
				}
				else
				{
					printf("Unknow mode\n");
				}
			}
			else
			{
				printf("Pin mode failed!\n");
			}

			break;
	}

	return 0;
}

int BananaPi_set_gpio_alt(int pin, int mode)
{
	unsigned int regval = 0;
	unsigned int bank   = pin >> 5;
	unsigned int index  = pin - (bank << 5);
	unsigned int phyaddr = 0;
	int offset = ((index - ((index >> 3) << 3)) << 2);

	if (bank == 11)
		phyaddr = sunxi_gpio_info_t.r_gpio_base_addr + ((index >> 3) << 2);
	else
		phyaddr = sunxi_gpio_info_t.gpio_base_addr + (bank * 36) + ((index >> 3) << 2);

	/* Ignore unused gpio */
	if (BANANAPI_PIN_MASK[bank][index] != -1)
	{
		if (wiringPiDebug)
			printf("Register[%#x]: %#x index:%d\n", phyaddr, regval, index);

		regval = readR(phyaddr);
		regval &= ~(7 << offset);
		regval |=  (mode << offset);
		writeR(regval, phyaddr);
	}
	else
	{
		printf("Pin alt mode failed!\n");
	}

	return 0;
}

/*
 * BananaPi Digital write 
 */
int BananaPi_digitalWrite(int pin, int value)
{
	unsigned int bank   = pin >> 5;
	unsigned int index  = pin - (bank << 5);
	unsigned int phyaddr = 0;
	unsigned int regval = 0;
	unsigned int cru_phyaddr =0, gpio_phyaddr = 0, dr_phyaddr = 0;
	unsigned int cru_val = 0;
	unsigned int temp = 0;
	unsigned int bit_enable = 0;
	unsigned int offset;

	switch (BananaPiModel)
	{
		default:
			
			if (bank == 11)
			{
				phyaddr = sunxi_gpio_info_t.r_gpio_base_addr + sunxi_gpio_info_t.r_gpio_base_offset + 0x10;
			}
			else
			{
				phyaddr = sunxi_gpio_info_t.gpio_base_addr + sunxi_gpio_info_t.gpio_base_offset + (bank * 36) + 0x10;
			}
			
			/* Ignore unused gpio */
			if (BANANAPI_PIN_MASK[bank][index] != -1)
			{
				regval = readR(phyaddr);
				if (wiringPiDebug)
					printf("befor write reg val: 0x%x,index:%d\n", regval, index);
				
				if(0 == value)
				{
					regval &= ~(1 << index);
					writeR(regval, phyaddr);
					regval = readR(phyaddr);
					if (wiringPiDebug)
						printf("LOW val set over reg val: 0x%x\n", regval);
				} 
				else
				{
					regval |= (1 << index);
					writeR(regval, phyaddr);
					regval = readR(phyaddr);
					if (wiringPiDebug)
						printf("HIGH val set over reg val: 0x%x\n", regval);
				}
			} 
			else
			{
				printf("Pin mode failed!\n");
			}

			break;
	}

	return 0;
}

/*
 * BananaPi Digital Read
 */
int BananaPi_digitalRead(int pin)
{
	int bank = pin >> 5;
	int index = pin - (bank << 5);
	int val;
	unsigned int phyaddr = 0;

	switch (BananaPiModel)
	{
		default:

			if (bank == 11) 
			{
				phyaddr = sunxi_gpio_info_t.r_gpio_base_addr + sunxi_gpio_info_t.r_gpio_base_offset + 0x10;
			}
			else
			{
				phyaddr = sunxi_gpio_info_t.gpio_base_addr + sunxi_gpio_info_t.gpio_base_offset + (bank * 36) + 0x10;
			}

			break;

	}

	if (BANANAPI_PIN_MASK[bank][index] != -1)
	{
		val = readR(phyaddr);

		if (BananaPiModel == PI_MODEL_BERRY) {
			val = val >> index;
		} 
		else if (BananaPiModel == PI_MODEL_ZERO) {
			val = val >> index;
		} 
		else{
		}

		val &= 1;

		if (wiringPiDebug)
			printf("Read reg val: 0x%#x, bank:%d, index:%d\n", val, bank, index);

		return val;
	}

	return 0;
}

void BananaPi_set_gpio_pullUpDnControl (int pin, int pud)
{
	unsigned int bank = pin >> 5;
	unsigned int index = pin - (bank << 5);
	unsigned int regval;
	unsigned int offset;
	unsigned int phyaddr = 0;
	unsigned int bit_enable;
	unsigned int bit_value = 0;

	switch (BananaPiModel)
	{

		default:
			//int offset = ((index - ((index >> 4) << 4)) << 1);
			offset = ((index % 16) << 1);

			if (bank == 11)
				phyaddr = sunxi_gpio_info_t.r_gpio_base_addr + sunxi_gpio_info_t.r_gpio_base_offset + ((index >> 4) << 2) + 0x1c;
			else
				phyaddr = sunxi_gpio_info_t.gpio_base_addr + sunxi_gpio_info_t.gpio_base_offset + (bank * 36) + ((index >> 4) << 2) + 0x1c;

			bit_enable = 0;

			/* */if (PUD_UP == pud)
				bit_value = 1;
			else if (PUD_DOWN == pud)
				bit_value = 2;
			else if (PUD_OFF == pud)
				bit_value = 0;

			break;
	}

	/* Ignore unused gpio */
	if (BANANAPI_PIN_MASK[bank][index] != -1)
	{
		if (wiringPiDebug)
			printf("bank: %d, index: %d\n", bank, index);

		regval = readR(phyaddr);

		if (wiringPiDebug)
			printf("read val(%#x) from register[%#x]\n", regval, phyaddr);

		/* clear bit */
		regval &= ~(3 << offset);

		/* bit write enable*/
		regval |= bit_enable;

		/* set bit */
		regval |= (bit_value & 3) << offset;

		if (wiringPiDebug)
			printf("write val(%#x) to register[%#x]\n", regval, phyaddr);

		writeR(regval, phyaddr);
		regval = readR(phyaddr);

		if (wiringPiDebug)
			printf("over reg val: %#x\n", regval);
	}
}
