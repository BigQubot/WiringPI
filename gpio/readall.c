/*
 * readall.c:
 *	The readall functions - getting a bit big, so split them out.
 *	Copyright (c) 2012-2018 Gordon Henderson
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


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <wiringPi.h>

extern int wpMode ;

#ifndef TRUE
#  define       TRUE    (1==1)
#  define       FALSE   (1==2)
#endif

/*
 * doReadallExternal:
 *	A relatively crude way to read the pins on an external device.
 *	We don't know the input/output mode of pins, but we can tell
 *	if it's an analog pin or a digital one...
 *********************************************************************************
 */

static void doReadallExternal (void)
{
  int pin ;

  printf ("+------+---------+--------+\n") ;
  printf ("|  Pin | Digital | Analog |\n") ;
  printf ("+------+---------+--------+\n") ;

  for (pin = wiringPiNodes->pinBase ; pin <= wiringPiNodes->pinMax ; ++pin)
    printf ("| %4d |  %4d   |  %4d  |\n", pin, digitalRead (pin), analogRead (pin)) ;

  printf ("+------+---------+--------+\n") ;
}

/*
 * doReadall:
 *	Read all the GPIO pins
 *	We also want to use this to read the state of pins on an externally
 *	connected device, so we need to do some fiddling with the internal
 *	wiringPi node structures - since the gpio command can only use
 *	one external device at a time, we'll use that to our advantage...
 *********************************************************************************
 */

static char * alts_common [] =
{
  "IN", "OUT", "ALT2", "ALT3", "ALT4", "ALT5", "ALT6", "OFF"
};

static char ** alts = alts_common;

static int physToWpi_M4_BERRY[64] =
{
	-1,     // 0
	-1, -1, // 1, 2
	 0, -1, // 3, 4
	 1, -1, // 5, 6
   	 2,  3, // 7, 8
	-1,  4, // 9, 10
	 5,  6, //11, 12
	 7, -1, //13, 14
	 8,  9, //15, 16
	-1, 10, //17, 18
	11, -1, //19, 20
	12, 13, //21, 22
	14, 15, //23, 24
	-1, 16, //25, 26
	17, 18, //27, 28
	19, -1, //29, 30
	20, 21, //31, 32
	22, -1, //33, 34
	23, 24, //35, 36
	25, 26, //37, 38
	-1, 27, //39, 40

	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
	-1, -1, -1, -1, -1, -1, -1, -1 								// 56-> 63
};

static char * physNames_M4_BERRY[64] =
{
    NULL,
	"    3.3V", "5V      ",
	"   SDA.4", "5V      ",
	"   SCL.4", "GND     ",
	"   PWM.1", "TXD.1   ",
	"     GND", "RXD.1   ",
	"   TXD.5", "PCM_BCLK",
	"   RXD.5", "GND     ",
	"    PG02", "RTS.1   ",
	"    3.3V", "CTS.1   ",
	"  MOSI.1", "GND     ",
	"  MISO.1", "PG01    ",
	"  SCLK.1", "CS0.1   ",
	"     GND", "CS1.1   ",
	"   SDA.3", "SCL.3   ",
	"    PG03", "GND     ",
	"    PG04", "PG00    ",
	"    PG05", "GND     ",
	"PCM_LRCK", "SPDIF   ",
	"PCM_MCLK", "PCM_IN  ",
	"     GND", "PCM_OUT ",
};

static int physToWpi_M4_ZERO[64] =
{
	-1,     // 0
	-1, -1, // 1, 2
	 0, -1, // 3, 4
	 1, -1, // 5, 6
   	 2,  3, // 7, 8
	-1,  4, // 9, 10
	 5,  6, //11, 12
	 7, -1, //13, 14
	 8,  9, //15, 16
	-1, 10, //17, 18
	11, -1, //19, 20
	12, 13, //21, 22
	14, 15, //23, 24
	-1, 16, //25, 26
	17, 18, //27, 28
	19, -1, //29, 30
	20, 21, //31, 32
	22, -1, //33, 34
	23, 24, //35, 36
	25, 26, //37, 38
	-1, 27, //39, 40

	-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
	-1, -1, -1, -1, -1, -1, -1, -1 								// 56-> 63
};

static char * physNames_M4_ZERO[64] =
{
    NULL,
	"    3.3V", "5V      ",
	"   SDA.4", "5V      ",
	"   SCL.4", "GND     ",
	"   PWM.1", "TXD.1   ",
	"     GND", "RXD.1   ",
	"   TXD.5", "PCM_BCLK",
	"   RXD.5", "GND     ",
	"    PG02", "RTS.1   ",
	"    3.3V", "CTS.1   ",
	"  MOSI.1", "GND     ",
	"  MISO.1", "PG01    ",
	"  SCLK.1", "CS0.1   ",
	"     GND", "CS1.1   ",
	"   SDA.3", "SCL.3   ",
	"    PG03", "GND     ",
	"    PG04", "PG00    ",
	"    PG05", "GND     ",
	"PCM_LRCK", "SPDIF   ",
	"PCM_MCLK", "PCM_IN  ",
	"     GND", "PCM_OUT ",
};


static int * physToWpi;
static char ** physNames;


/*
 * readallPhys:
 *	Given a physical pin output the data on it and the next pin:
 *| BCM | wPi |   Name  | Mode | Val| Physical |Val | Mode | Name    | wPi | BCM |
 *********************************************************************************
 */
void readallPhys (int physPin)
{
	int pin ;

	if (physPinToGpio (physPin) == -1)
		printf (" |      |    ") ;
	else
		printf (" | %4d | %3d", physPinToGpio (physPin), physToWpi [physPin]) ;

	printf (" | %s", physNames[physPin]);
	

	if (physToWpi [physPin] == -1)
	{
		printf (" |        |  ") ;
	}
	else
	{
		if (wpMode == WPI_MODE_GPIO)
			pin = physPinToGpio (physPin) ;
		else if (wpMode == WPI_MODE_PHYS)
			pin = physPin ;
		else
			pin = physToWpi [physPin] ;

		printf (" | %6s", alts[getAlt(pin)]) ;
		printf (" | %d", digitalRead (pin)) ;
	}

	// Pin numbers:
	printf (" | %2d", physPin) ;
	++physPin ;
	printf (" || %-2d", physPin) ;

	// Same, reversed
	if (physToWpi [physPin] == -1)
		printf (" |   |       ") ;
	else
	{
		if (wpMode == WPI_MODE_GPIO)
			pin = physPinToGpio (physPin) ;
		else if (wpMode == WPI_MODE_PHYS)
			pin = physPin ;
		else
			pin = physToWpi [physPin] ;

		printf (" | %d", digitalRead (pin)) ;
		printf (" | %-6s", alts [getAlt (pin)]) ;
	}

	printf (" | %-5s", physNames [physPin]) ;

	if (physToWpi     [physPin] == -1)
		printf (" |     |     ") ;
	else
		printf (" | %-3d | %-4d", physToWpi [physPin], physPinToGpio (physPin)) ;

	printf (" |\n") ;
}


/*
 * allReadall:
 *	Read all the pins regardless of the model. Primarily of use for
 *	the compute module, but handy for other fiddling...
 *********************************************************************************
 */

static void allReadall (void)
{
  int pin ;

  printf ("+-----+------+-------+      +-----+------+-------+\n") ;
  printf ("| Pin | Mode | Value |      | Pin | Mode | Value |\n") ;
  printf ("+-----+------+-------+      +-----+------+-------+\n") ;

  for (pin = 0 ; pin < 14 ; ++pin)
  {
    printf ("| %3d ", pin) ;
    printf ("| %-4s ", alts [getAlt (pin)]) ;
    printf ("| %s  ", digitalRead (pin) == HIGH ? "High" : "Low ") ;
    printf ("|      ") ;
    printf ("| %3d ", pin + 14) ;
    printf ("| %-4s ", alts [getAlt (pin + 14)]) ;
    printf ("| %s  ", digitalRead (pin + 14) == HIGH ? "High" : "Low ") ;
    printf ("|\n") ;
  }

  printf ("+-----+------+-------+      +-----+------+-------+\n") ;

}


/*
 * abReadall:
 *	Read all the pins on the model A or B.
 *********************************************************************************
 */

void abReadall (int model, int rev)
{
  int pin ;
  char *type ;

  if (model == PI_MODEL_A)
    type = " A" ;
  else
    if (rev == PI_VERSION_2)
      type = "B2" ;
    else
      type = "B1" ;

  printf (" +-----+-----+---------+------+---+-Model %s-+---+------+---------+-----+-----+\n", type) ;
  printf (" | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n") ;
  printf (" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  for (pin = 1 ; pin <= 26 ; pin += 2)
    readallPhys (pin) ;

  if (rev == PI_VERSION_2) // B version 2
  {
    printf (" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
    for (pin = 51 ; pin <= 54 ; pin += 2)
      readallPhys (pin) ;
  }

  printf (" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  printf (" | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n") ;
  printf (" +-----+-----+---------+------+---+-Model %s-+---+------+---------+-----+-----+\n", type) ;
}


/*
 * piPlusReadall:
 *	Read all the pins on the model A+ or the B+ or actually, all 40-pin Pi's
 *********************************************************************************
 */

static void plus2header (int model)
{
  /**/ if (model == PI_MODEL_AP)
    printf (" +-----+-----+---------+------+---+---Pi A+--+---+------+---------+-----+-----+\n") ;
  else if (model == PI_MODEL_BP)
    printf (" +-----+-----+---------+------+---+---Pi B+--+---+------+---------+-----+-----+\n") ;
  else if (model == PI_MODEL_ZERO)
    printf (" +-----+-----+---------+------+---+-Pi Zero--+---+------+---------+-----+-----+\n") ;
  else if (model == PI_MODEL_ZERO_W)
    printf (" +-----+-----+---------+------+---+-Pi ZeroW-+---+------+---------+-----+-----+\n") ;
  else if (model == PI_MODEL_2)
    printf (" +-----+-----+---------+------+---+---Pi 2---+---+------+---------+-----+-----+\n") ;
  else if (model == PI_MODEL_3P)
    printf (" +-----+-----+---------+------+---+---Pi 3+--+---+------+---------+-----+-----+\n") ;
  else
    printf (" +-----+-----+---------+------+---+---Pi ?---+---+------+---------+-----+-----+\n") ;
}


static void piPlusReadall (int model)
{
  int pin ;

  plus2header (model) ;

  printf (" | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n") ;
  printf (" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  for (pin = 1 ; pin <= 40 ; pin += 2)
    readallPhys (pin) ;
  printf (" +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  printf (" | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n") ;

  plus2header (model) ;
}

/*
 * ReadAll 
 */
void BananaPiReadAll(int model)
{
	int pin;
	int tmp = wiringPiDebug;
	wiringPiDebug = FALSE;

	switch (model)
	{
		case PI_MODEL_BERRY:
			printf (" +------+-----+----------+--------+---+  BPI-M4B +---+--------+----------+-----+------+\n");
			physToWpi =  physToWpi_M4_BERRY;
			physNames =  physNames_M4_BERRY;
			alts = alts_common;
			break;
		case PI_MODEL_ZERO:
			printf (" +------+-----+----------+--------+---+  BPI-M4Z  +---+--------+----------+-----+------+\n");
			physToWpi =  physToWpi_M4_ZERO;
			physNames =  physNames_M4_ZERO;
			alts = alts_common;
			break;
		default:
			printf ("Oops - unable to determine board type... model: %d\n", model);
			break ;
	}

	printf (" | GPIO | wPi |   Name   |  Mode  | V | Physical | V |  Mode  | Name     | wPi | GPIO |\n");
	printf (" +------+-----+----------+--------+---+----++----+---+--------+----------+-----+------+\n");

	switch (model)
	{
		case PI_MODEL_BERRY:
		case PI_MODEL_ZERO:
			for (pin = 1 ; pin <= 40; pin += 2)
				readallPhys(pin);
			break;
		default:
			printf ("Oops - unable to determine board type... model: %d\n", model);
			break ;
	}

	printf (" +------+-----+----------+--------+---+----++----+---+--------+----------+-----+------+\n");
	printf (" | GPIO | wPi |   Name   |  Mode  | V | Physical | V |  Mode  | Name     | wPi | GPIO |\n");

	switch (model)
	{
		case PI_MODEL_BERRY:
			printf (" +------+-----+----------+--------+---+  BPI-M4B +---+--------+----------+-----+------+\n");
			break;
		case PI_MODEL_ZERO:
			printf (" +------+-----+----------+--------+---+  BPI-M4Z +---+--------+----------+-----+------+\n");
			break;
		default:
			printf ("Oops - unable to determine board type... model: %d\n", model);
			break ;
	}

    wiringPiDebug = tmp;
}

/*
 * doReadall:
 *	Generic read all pins called from main program. Works out the Pi type
 *	and calls the appropriate function.
 *********************************************************************************
 */

void doReadall (void)
{
	int model = -1;

	if (wiringPiNodes != NULL)	// External readall
	{
		doReadallExternal () ;
		return ;
	}

	piBoardId (&model) ;

	BananaPiReadAll(model);
}


/*
 * doAllReadall:
 *	Force reading of all pins regardless of Pi model
 *********************************************************************************
 */

void doAllReadall (void)
{
  allReadall () ;
}


/*
 * doQmode:
 *	Query mode on a pin
 *********************************************************************************
 */

void doQmode (int argc, char *argv [])
{
  int pin ;

  if (argc != 3)
  {
    fprintf (stderr, "Usage: %s qmode pin\n", argv [0]) ;
    exit (EXIT_FAILURE) ;
  }

  pin = atoi (argv [2]) ;
  printf ("%s\n", alts [getAlt (pin)]) ;
}
