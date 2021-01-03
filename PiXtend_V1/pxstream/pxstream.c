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

void printValues()
{
	/*
InputData TODO:
	uint16_t wAi0;
	uint16_t wAi1;
	uint16_t wAi2;
	uint16_t wAi3;

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
	if (!printOnlyChanged || tCount == 1 || oldInputData.byUcStatus != InputData.byUcStatus)
	{
		printf("UCSTATUS %d\n", InputData.byUcStatus);
	}

	if (!printOnlyChanged || tCount == 1 || oldInputData.byGpioIn != InputData.byGpioIn)
	{
		printf("GPIOIB %d \n", InputData.byGpioIn);
		printf("GPIOI0 %d \n", BIT(InputData.byGpioIn, 0));
		printf("GPIOI1 %d \n", BIT(InputData.byGpioIn, 1));
		printf("GPIOI2 %d \n", BIT(InputData.byGpioIn, 2));
		printf("GPIOI3 %d \n", BIT(InputData.byGpioIn, 3));
	}

	if (!printOnlyChanged || tCount == 1 || oldInputData.rAi0 != InputData.rAi0)
	{
		printf("AI0 %f\n", InputData.rAi0);
	}
	if (!printOnlyChanged || tCount == 1 || oldInputData.rAi1 != InputData.rAi1)
	{
		printf("AI1 %f\n", InputData.rAi1);
	}
	if (!printOnlyChanged || tCount == 1 || oldInputData.rAi2 != InputData.rAi2)
	{
		printf("AI2 %f\n", InputData.rAi2);
	}
	if (!printOnlyChanged || tCount == 1 || oldInputData.rAi3 != InputData.rAi3)
	{
		printf("AI3 %f\n", InputData.rAi3);
	}

	if (!printOnlyChanged || tCount == 1 || oldInputData.byDigIn != InputData.byDigIn)
	{
		printf("DINB %d\n", InputData.byDigIn);
		printf("DIN0 %d\n", BIT(InputData.byDigIn, 0));
		printf("DIN1 %d\n", BIT(InputData.byDigIn, 1));
		printf("DIN2 %d\n", BIT(InputData.byDigIn, 2));
		printf("DIN3 %d\n", BIT(InputData.byDigIn, 3));
		printf("DIN4 %d\n", BIT(InputData.byDigIn, 4));
		printf("DIN5 %d\n", BIT(InputData.byDigIn, 5));
		printf("DIN6 %d\n", BIT(InputData.byDigIn, 6));
		printf("DIN7 %d\n", BIT(InputData.byDigIn, 7));
	}

	if (!printOnlyChanged || tCount == 1 || oldInputData.byUcVersionH != InputData.byUcVersionH)
	{
		printf("UCVER %d.%d\n", InputData.byUcVersionH, InputData.byUcVersionL);
	}

	if (!printOnlyChanged || tCount == 1 || oldOutputData.byAiCtrl0 != OutputData.byAiCtrl0)
	{
		printf("AICTRL0 %d\n", OutputData.byAiCtrl0);
	}

	if (!printOnlyChanged || tCount == 1 || oldOutputData.byAiCtrl1 != OutputData.byAiCtrl1)
	{
		printf("AICTRL1 %d\n", OutputData.byAiCtrl1);
	}

	if (!printOnlyChanged || tCount == 1 || oldOutputData.wPwm0 != OutputData.wPwm0)
	{
		printf("PWM0 %d\n", OutputData.wPwm0);
	}
	if (!printOnlyChanged || tCount == 1 || oldOutputData.wPwm1 != OutputData.wPwm1)
	{
		printf("PWM1 %d\n", OutputData.wPwm1);
	}

	if (!printOnlyChanged || tCount == 1 || oldOutputData.byGpioOut != OutputData.byGpioOut)
	{
		printf("GPIOOB %d\n", OutputData.byGpioOut);
		printf("GPIOO0 %d\n", BIT(OutputData.byGpioOut, 0));
		printf("GPIOO1 %d\n", BIT(OutputData.byGpioOut, 1));
		printf("GPIOO2 %d\n", BIT(OutputData.byGpioOut, 2));
		printf("GPIOO3 %d\n", BIT(OutputData.byGpioOut, 3));
	}

	if (!printOnlyChanged || tCount == 1 || oldOutputData.byDigOut != OutputData.byDigOut)
	{
		printf("DOB %d\n", OutputData.byDigOut);
		printf("DO0 %d\n", BIT(OutputData.byDigOut, 0));
		printf("DO1 %d\n", BIT(OutputData.byDigOut, 1));
		printf("DO2 %d\n", BIT(OutputData.byDigOut, 2));
		printf("DO3 %d\n", BIT(OutputData.byDigOut, 3));
		printf("DO4 %d\n", BIT(OutputData.byDigOut, 4));
		printf("DO5 %d\n", BIT(OutputData.byDigOut, 5));
	}
	if (!printOnlyChanged || tCount == 1 || oldOutputData.byRelayOut != OutputData.byRelayOut)
	{
		printf("RELB %d\n", OutputData.byRelayOut);
		printf("REL0 %d\n", BIT(OutputData.byRelayOut, 0));
		printf("REL1 %d\n", BIT(OutputData.byRelayOut, 1));
		printf("REL2 %d\n", BIT(OutputData.byRelayOut, 2));
		printf("REL3 %d\n", BIT(OutputData.byRelayOut, 3));
	}
	//	printf("HEARTBEAT\n");

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
