/*
# This file is based on pxauto of the PiXtend(R) Project.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "pxstream.h"
#include <stdbool.h>

#define BIT(BYTE, BITNUM) (BYTE >> BITNUM) & 0x01
#define VERSION "0.0.1"

static int tCount;

//PiXtend input data
struct pixtIn InputData, oldInputData;

//PiXtend output data
struct pixtOut OutputData, oldOutputData;

//PiXtend DAC Output Data
struct pixtOutDAC OutputDataDAC, oldOutputDataDAC;

bool printOnlyChanged = true;

timer_t gTimerid;

//Time Base for timer_callback() calls
void start_timer(void)
{
	struct itimerspec value;
	value.it_value.tv_sec = 0;
	value.it_value.tv_nsec = 200000000;
	value.it_interval.tv_sec = 0;
	value.it_interval.tv_nsec = 200000000;
	timer_create(CLOCK_REALTIME, NULL, &gTimerid);
	timer_settime(gTimerid, 0, &value, NULL);
}

void stop_timer(void)
{
	struct itimerspec value;
	value.it_value.tv_sec = 0;
	value.it_value.tv_nsec = 0;
	value.it_interval.tv_sec = 0;
	value.it_interval.tv_nsec = 0;
	timer_settime(gTimerid, 0, &value, NULL);
}

void help()
{
	printf("PiXtend Stream Tool - www.goroot.de - Version %s\n", VERSION);
	printf("This tool will stream all inputs / outputs in ascii format to stdout\n\n");
	printf("usage: sudo pxstream [OPTION]\n");
	printf("\n");
	printf("available options:\n");
	printf("\t-h \t\t\t\t\tprint this help\n");
	printf("\t-u \t\t\t\t\tonly print values that changed\n");
}

/**
 * Prints a value if it changed, it is the first loop iteration, or always, if printonlychanged is disabled
 * 
 * @param valueIdentifier ID String for the value
 * @param old Old value
 * @param current Current value
 */
void printValueUint8(char *valueIdentifier, uint8_t old, uint8_t current)
{
	if (!printOnlyChanged || tCount == 1 || old != current)
	{
		printf("%s %d\n", valueIdentifier, current);
	}
}

/**
 * Prints a value if it changed, it is the first loop iteration, or always, if printonlychanged is disabled
 * 
 * @param valueIdentifier ID String for the value
 * @param old Old value
 * @param current Current value
 */
void printValueFloat(char *valueIdentifier, float old, float current)
{
	if (!printOnlyChanged || tCount == 1 || old != current)
	{
		printf("%s %f\n", valueIdentifier, current);
	}
}

/**
 * Prints a value if it changed, it is the first loop iteration, or always, if printonlychanged is disabled
 * 
 * @param valueIdentifier ID String for the value
 * @param old Old value
 * @param current Current value
 */
void printValueUint16(char *valueIdentifier, uint16_t old, uint16_t current)
{
	if (!printOnlyChanged || tCount == 1 || old != current)
	{
		printf("%s %d\n", valueIdentifier, current);
	}
}

/**
 * Prints whole byte and individual bits
 * 
 * @param valueIdentifierFormatString Format string containing %s as placeholder for the bit ID
 * @param old Old value
 * @param current Current value
 * @param bitsToPrint Number of bits to print
 */
void printByteAndBits(char *valueIdentifierFormatString, uint8_t old, uint8_t current, uint8_t bitsToPrint)
{
	if (!printOnlyChanged || tCount == 1 || old != current)
	{
		printf("%sB %d\n", valueIdentifierFormatString, current);
		for (uint8_t i = 0; i < bitsToPrint; i++)
		{
			uint8_t oldBit = BIT(old, i);
			uint8_t currentBit = BIT(current, i);
			if (!printOnlyChanged || tCount == 1 || oldBit != currentBit)
			{
				printf("%s%d %d\n", valueIdentifierFormatString, i, currentBit);
			}
		}
	}
}

void printValues()
{
	/*
InputData TODO:

	uint16_t wTemp0;
	uint16_t wTemp1;
	uint16_t wTemp2;
	uint16_t wTemp3;
	uint16_t wHumid0;
	uint16_t wHumid1;
	uint16_t wHumid2;
	uint16_t wHumid3;

	float rTemp0;
	float rTemp1;
	float rTemp2;
	float rTemp3;
	float rHumid0;
	float rHumid1;
	float rHumid2;
	float rHumid3;


*/
	printValueUint8("IN_UCSTATUS", oldInputData.byUcStatus, InputData.byUcStatus);
	printByteAndBits("IN_GPIO", oldInputData.byGpioIn, InputData.byGpioIn, 4);
	printValueFloat("IN_RAI0", oldInputData.rAi0, InputData.rAi0);
	printValueFloat("IN_RAI1", oldInputData.rAi1, InputData.rAi1);
	printValueFloat("IN_RAI2", oldInputData.rAi2, InputData.rAi2);
	printValueFloat("IN_RAI3", oldInputData.rAi3, InputData.rAi3);

	printValueUint16("IN_WAI0", oldInputData.wAi0, InputData.wAi0);
	printValueUint16("IN_WAI1", oldInputData.wAi1, InputData.wAi1);
	printValueUint16("IN_WAI2", oldInputData.wAi2, InputData.wAi2);
	printValueUint16("IN_WAI3", oldInputData.wAi3, InputData.wAi3);

	printByteAndBits("IN_D", oldInputData.byDigIn, InputData.byDigIn, 8);

	if (!printOnlyChanged || tCount == 1 || oldInputData.byUcVersionH != InputData.byUcVersionH)
	{
		printf("IN_UCVER %d.%d\n", InputData.byUcVersionH, InputData.byUcVersionL);
	}

	printValueUint8("OUT_AICTRL0", oldOutputData.byAiCtrl0, OutputData.byAiCtrl0);
	printValueUint8("OUT_AICTRL1", oldOutputData.byAiCtrl1, OutputData.byAiCtrl1);

	printValueUint16("OUT_PWM0", oldOutputData.wPwm0, OutputData.wPwm0);
	printValueUint16("OUT_PWM1", oldOutputData.wPwm1, OutputData.wPwm1);

	printByteAndBits("OUT_GPIO", oldOutputData.byGpioOut, OutputData.byGpioOut, 4);
	printByteAndBits("OUT_D", oldOutputData.byDigOut, OutputData.byDigOut, 6);
	printByteAndBits("OUT_REL", oldOutputData.byRelayOut, OutputData.byRelayOut, 4);
	printValueUint8("OUT_PWMCTRL0", oldOutputData.byPwm0Ctrl0, OutputData.byPwm0Ctrl0);
	printValueUint8("OUT_PWMCTRL1", oldOutputData.byPwm0Ctrl1, OutputData.byPwm0Ctrl1);
	printValueUint8("OUT_PWMCTRL2", oldOutputData.byPwm0Ctrl2, OutputData.byPwm0Ctrl2);
	printValueUint8("OUT_GPIOCTRL", oldOutputData.byGpioCtrl, OutputData.byGpioCtrl);
	printValueUint8("OUT_UCCTRL", oldOutputData.byUcCtrl, OutputData.byUcCtrl);
	printValueUint8("OUT_PISTATUS", oldOutputData.byPiStatus, OutputData.byPiStatus);
	printValueUint8("OUT_AUX0", oldOutputData.byAux0, OutputData.byAux0);

	printValueUint8("OUT_AOUT0", oldOutputDataDAC.wAOut0, OutputDataDAC.wAOut0);
	printValueUint8("OUT_AOUT1", oldOutputDataDAC.wAOut1, OutputDataDAC.wAOut1);

	oldInputData = InputData;
	oldOutputData = OutputData;
	oldOutputDataDAC = OutputDataDAC;
}

// Timer Callback, executed every 200ms
void timer_callback(int sig)
{

	//Increment static counter
	tCount++;

	//Execute every 200ms
	autoMode();
	printValues();
}

int autoMode()
{

	//Exchange PiXtend Data
	Spi_AutoMode(&OutputData, &InputData);

	//Exchange PiXtendDAC Data
	Spi_AutoModeDAC(&OutputDataDAC);

	return (0);
}

void readCurrentOutputValues()
{
	OutputData.byRelayOut = Spi_Get_Relays();
	OutputData.byDigOut = Spi_Get_Dout();
	OutputData.byGpioOut = Spi_Get_Gpio();
}

int main()
{

	//Setup SPI using wiringPi
	Spi_Setup(0); //use SPI device 0.0 (PiXtend), exit on failure
	Spi_Setup(1); //use SPI device 0.1 (PiXtend DAC), exit on failure

	//Connect Timer Signal with 200ms refresh rate
	(void)signal(SIGALRM, timer_callback);

	//Read the current uC values into the structs, so that the first call to Spi_AutoMode doesn't change anything on the outputs
	readCurrentOutputValues();

	start_timer();

	//Main Loop
	while (1)
	{
	}

	return (0);
}
