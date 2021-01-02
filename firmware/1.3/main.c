/*
# This file is part of the PiXtend(R) Project.
#
# For more information about PiXtend(R) and this program,
# see <http://www.pixtend.de> or <http://www.pixtend.com>
#
# Copyright (C) 2016 Tobias Gall, Christian Strobel
# Qube Solutions UG (haftungsbeschränkt), Luitgardweg 18
# 71083 Herrenberg, Germany 
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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++					PiXtend Firmware 								+++++
//+++++					ATmega32A, Hardware: V1.3 Software: 1.3.2 		+++++
//+++++												           			+++++
//+++++					10. January 2016          						+++++
//+++++												           			+++++
//+++++					www.pixtend.de - www.qube-solutions.de			+++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++						ATmega32A Fusebit-Information				+++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//LOW-FUSE: 	0x3F	()
//HIGH-FUSE:	0xC9	()
//LOCK-FUSE:	0xFF	(has not to be modified)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++						Includes / Defines							+++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/crc16.h>

//Microcontroller Firmware Version
#define VERSIONL 2
#define VERSIONH 13

//Number of bytes which are transfered in "Automatic Mode"
#define NUMBER_OF_AUTOMATIC_MODE_BYTES 31

//GPIO 0 as Debug-Pin
#define GPIO_DEBUG 0

#define DEBUG_PIN_0_HIGH PORTA |= (1<<PA0);
#define DEBUG_PIN_0_LOW PORTA &= ~(1<<PA0);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++						    Global Variables			 			+++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

volatile uint8_t SPI_TRANSMIT_BUFFER[NUMBER_OF_AUTOMATIC_MODE_BYTES];
volatile uint8_t SPI_RECEIVE_BUFFER[NUMBER_OF_AUTOMATIC_MODE_BYTES];

volatile uint8_t spi_temp = 0;
volatile uint8_t spi_mode = 0;
volatile uint8_t spi_data_mask = 0;
volatile uint8_t spi_data_mask_set = 0;
volatile uint8_t spi_automatic_counter = 0;
volatile uint8_t spi_command = 0;
volatile uint8_t spi_receive_temp = 0;
volatile uint8_t spi_transmit_temp = 0;
volatile uint8_t spi_transfer_complete = 0;
volatile uint8_t spi_byte_cnt = 0;
volatile uint8_t spi_automatic_cycle_finished = 0;
volatile uint8_t spi_adc_request = 0;
volatile uint8_t spi_adc_request_channel = 0;
volatile uint8_t spi_adc_ctrl_temp0 = 0;
volatile uint8_t spi_adc_ctrl_temp1 = 0;
volatile uint8_t spi_pwm_temp0 = 0;
volatile uint8_t spi_pwm_temp1 = 0;
volatile uint8_t spi_pwm_ctrl_temp0 = 0;
volatile uint8_t spi_pwm_ctrl_temp1 = 0;
volatile uint8_t spi_pwm_ctrl_temp2 = 0;
volatile uint8_t spi_dht_request = 0;
volatile uint8_t spi_dht_request_channel = 0;

volatile uint16_t OCR1A_temp = 0;
volatile uint16_t OCR1B_temp = 0;

volatile uint8_t UC_INIT = 0;
volatile uint8_t ADC_VALUE[4][2];
volatile uint8_t DIGITAL_IN = 0;
volatile uint8_t GPIO_IN = 0;
volatile uint8_t GPIO_CTRL = 0;
volatile uint8_t PWM_CTRL[3];
volatile uint8_t ADC_CTRL[2];
volatile uint8_t UC_STATUS = 0;
volatile uint8_t DOUT_VALUE = 0;
volatile uint8_t RELAY_VALUE = 0;

volatile uint8_t HUMIDITY[4][2];
volatile uint8_t TEMPERATURE[4][2];

volatile uint8_t uc_control_reg_old = 0;
volatile uint8_t gpio_ctrl_old = 0;
volatile uint8_t pwm_ctrl_reg0_old = 0;
volatile uint8_t pwm_ctrl_reg1_old = 0;
volatile uint8_t pwm_ctrl_reg2_old = 0;

volatile uint8_t dht_channel_counter = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++							 Prototypes								+++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void get_adc_value(uint8_t channel);
void get_din_value(void);
void get_gpio_value(void);
void get_dht_value(uint8_t channel);
void get_dout_value(void);
void get_relay_value(void);

void set_gpio_ctrl(uint8_t reg);
void set_pwm_ctrl(uint8_t reg0, uint8_t reg1, uint8_t reg2);
void set_uc_ctrl(uint8_t reg);
void set_analog_ctrl(uint8_t reg0, uint8_t reg1);

void set_dout_value(uint8_t value);
void set_relay_value(uint8_t value);
void set_gpio_value(uint8_t value);
void set_pwm_value(uint8_t channel, uint8_t pwm0_value_low, uint8_t pwm0_value_high, uint8_t pwm1_value_low, uint8_t pwm1_value_high);

void set_raspberry_status_register(uint8_t reg);
void set_uc_status_register(uint8_t bit, uint8_t value);

uint8_t dht11_22(uint8_t channel);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++							 Functions  							+++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//----------------------------
//-- DHT11/22 sensor driver --
//----------------------------

uint8_t dht11_22(uint8_t channel)
{
	uint8_t timeout = 0;
	uint8_t dht_data[5]={0};
	uint8_t dht_byte = 0;

	DDRA &= ~(1 << channel); //set pin to input
	_delay_us(10);
	
	if (!(PINA & (1 << channel)))
	{
		//pin is low --> bus is not free
		return 1;
	}

	//set pin to output
	DDRA |= (1 << channel);		
	//set pin to low-level .--> uC start signal
	PINA &= ~(1 << channel);
	//start signal (min. 0.8ms max. 20ms)
	_delay_ms(20);				
	DDRA &= ~(1 << channel);

	//Bus master has released time (min. 20us, typ. 30us, max. 200us)
	timeout = 185;
	_delay_us(15); 
	while(PINA & (1 << channel)) 
	{
		_delay_us(1);
		if (!timeout--) 
		{
			return 2;
		}
	}

	//DHT11/22 response signal (min. 75us typ. 80us max. 85us)
	//response to low time
	timeout = 70;
	_delay_us(15); 
	while(!(PINA & (1 << channel))) 
	{
		_delay_us(1); 
		if (!timeout--) 
		{
			return 3;
		}
	}  
	
	//response to high time
	timeout = 70;
	_delay_us(15); 
	while(PINA & (1 << channel))
	{
		_delay_us(1);
		if (!timeout--)
		{
			return 4;
		}
	}


	//now it is time for real data!
	
	//Bit-timings:
	//			 				min 	typ 	max
	//signal 0 high time: 		22us  	26us  	30us
	//signal 1 high time: 		68us  	70us  	75us
	//signal 0,1 down time: 	48us  	50us  	55us

	
	for(uint8_t i = 0; i < 5; i++)
	{
		dht_byte = 0;
		for(uint8_t j = 1; j <= 8; j++) //receive 8 bits from dht11/22
		{
			// signal "0", "1" low time
			timeout = 55;
			while(!(PINA & (1 << channel))) 
			{
				_delay_us(1);
				if (!timeout--)
				{
					return 5;
				}
			}
			_delay_us(30);
			dht_byte <<= 1; //Add new lower byte
			
			if (PINA & (1 << channel)) //If pin is high after 30us --> bit=1 else bit=0
			{
				dht_byte |= 1;
				timeout = 45;  // 30us - 75us = 45us
				while(PINA & (1 << channel))
				{
					_delay_us(1);
					if (!timeout--)
					{
						return 6;
					}
				}
			}
		}
		
		dht_data[i] = dht_byte;
	}

	//build checksum and check it
	if (((dht_data[0]+dht_data[1]+dht_data[2]+dht_data[3]) & 0xFF) != dht_data[4])
	{
		return 7;
	}

	//No timeout or checksum-fail --> data is correct --> set global variables
	
	HUMIDITY[channel][0] = dht_data[1]; //humidity low byte
	HUMIDITY[channel][1] = dht_data[0]; //humidity high byte
	TEMPERATURE[channel][0] = dht_data[3]; //temperature low byte
	TEMPERATURE[channel][1] = dht_data[2]; //temperature high byte

	return 0;
}

//-----------------------------------
//-- GET values from analog inputs --
//-----------------------------------

void get_adc_value(uint8_t channel)
{
	uint8_t i = 0;
	uint8_t samples = 0;
	uint16_t ADC_temp = 0;
	uint8_t ADC_CTRL_temp = 0;
	
	//calculate the number of samples
	
	ADC_CTRL_temp = ADC_CTRL[0] >> (2*channel);
	ADC_CTRL_temp &= 0b00000011;
	
	if(ADC_CTRL_temp == 0b00000000)
	{
		samples = 10;
	}
	else if(ADC_CTRL_temp == 0b00000001)
	{
		samples = 1;
	}
	else if(ADC_CTRL_temp == 0b00000010)
	{
		samples = 5;
	}
	else if(ADC_CTRL_temp == 0b00000011)
	{
		samples = 50;
	}
	
	//set adc channel, but hold other information in ADMUX 
	ADMUX = (ADMUX & 0b11100000) | ((channel+4) & 0b00011111);
	
	//sample adc data
	for(i=0; i<samples; i++)
	{
		ADCSRA |= (1<<ADSC);                  
		while (ADCSRA & (1<<ADSC))	//wait for new adc value
		{
			;
		}
		
		ADC_temp += ADC;
	}
	
	//built average
	ADC_temp /= samples;
	
	//save data to global variables
	ADC_VALUE[channel][0] = ADC_temp & 0b11111111;
	ADC_VALUE[channel][1] = ADC_temp >> 8;
}

//------------------------------------
//-- GET values from digital inputs --
//------------------------------------

void get_din_value(void)
{
	//save pin states to global variable
	DIGITAL_IN = (PIND & 0b00001111) | (PINB << 4);
}

//---------------------------------
//-- GET values from gpio inputs --
//---------------------------------

void get_gpio_value(void)
{
	//save pin states to global variable
	GPIO_IN = (PINA & 0b00001111);
}

//---------------------------------
//--- GET value from DHT sensor ---
//---------------------------------

void get_dht_value(uint8_t channel)
{
	uint8_t dht_error = 0;
	
	if(channel > 3) //Automatic Mode
	{
		//DHT sensors should only be polled every two seconds
		//min. automatic-cylce time is 100ms --> each sensor will be polled at every twentieth automatic-cyle (20 x 100ms = 2s)
		
		switch(dht_channel_counter)
		{
			//poll DHT sensor 0
			case 0:
				channel = 0;
				if(GPIO_CTRL & 0b00010000)
				{
					dht_error = dht11_22(channel);
				}
				else
				{
					dht_error = 8; //sensor was not active
				}
				break;
			//poll DHT sensor 1
			case 5:
				channel = 1;
				if(GPIO_CTRL & 0b00100000)
				{
					dht_error = dht11_22(channel);
				}
				else
				{
					dht_error = 8; //sensor was not active
				}
				break;
			//poll DHT sensor 2
			case 10:
				channel = 2;
				if(GPIO_CTRL & 0b01000000)
				{
					dht_error = dht11_22(channel);
				}
				else
				{
					dht_error = 8; //sensor was not active
				}
				break;
			//poll DHT sensor 3
			case 15:
				channel = 3;
				if(GPIO_CTRL & 0b10000000)
				{
					dht_error = dht11_22(channel);
				}
				else
				{
					dht_error = 8; //sensor was not active
				}
				break;
			default:
				break;
		}
		dht_channel_counter++;
		if(dht_channel_counter > 19)
		{
			dht_channel_counter = 0;
		}
	}
	else //Manual Mode
	{
		//poll the requested DHT sensor
		if(GPIO_CTRL & (1 << (4+channel)))
		{
			dht_error = dht11_22(channel);
		}
	}
	
	if(dht_error == 0)
	{
		//No Error - nothing to do
	}
	else if(dht_error > 7)
	{
		//Sensor was not enabled --> value "0" signals, that sensor is disabled
		HUMIDITY[channel][0] = 0;
		HUMIDITY[channel][1] = 0;
		TEMPERATURE[channel][0] = 0; 
		TEMPERATURE[channel][1] = 0;
	}
	else
	{
		//Error! --> value "255" signals, that data acquisition failed
		HUMIDITY[channel][0] = 255;
		HUMIDITY[channel][1] = 255;
		TEMPERATURE[channel][0] = 255; 
		TEMPERATURE[channel][1] = 255;
	}
}

//-------------------------------------
//-- GET values from digital outputs --
//-------------------------------------

void get_dout_value(void)
{
	//save pin states to global variable
	DOUT_VALUE = (PINC & 0b00001111) | ((PIND & 0b10000000) >> 3) | ((PIND & 0b01000000) >> 1);
}

//-----------------------------------
//-- GET values from relay outputs --
//-----------------------------------

void get_relay_value(void)
{
	//save pin states to global variable
	RELAY_VALUE = (PINC & 0b11110000) >> 4;
}


//------------------------------------
//-- 	SET GPIO control register 	--
//------------------------------------

void set_gpio_ctrl(uint8_t reg)
{
	uint8_t i = 0;
	
	GPIO_CTRL = reg;
	
	//-- GPIO 0 is in DEBUG-MODE ? --
	if(GPIO_DEBUG)
	{
		i = 1;
	}
	//-------------------------------
	
	if(GPIO_CTRL != gpio_ctrl_old)
	{
		for(i=0;i<4;i++)
		{
			if(GPIO_CTRL & (1<<(4+i)))
			{
				//GPIO is in DHT Mode
			}
			else if(GPIO_CTRL & (1<<i))
			{
				//GPIO is output
				DDRA = DDRA | (1<<i);
			}
			else
			{
				//GPIO is Input
				DDRA = DDRA & ~(1<<i);
			}
		}
	}
	gpio_ctrl_old = GPIO_CTRL;
}

//------------------------------------
//-- 	SET PWM control registers 	--
//------------------------------------

void set_pwm_ctrl(uint8_t reg0, uint8_t reg1, uint8_t reg2)
{
	PWM_CTRL[0] = reg0;
	PWM_CTRL[1] = reg1;
	PWM_CTRL[2] = reg2;
	
	if(((pwm_ctrl_reg0_old & 0b11100000) != (reg0 & 0b11100000)) && (reg0 & 0b00000001))
	{
		//Prescaler has changed / pwm-mode was activated --> set new values
		TCCR1B = (TCCR1B & 0b11111000) | ((reg0 >> 5) & 0b00000111);
		ICR1 = (reg2 << 8) + reg1;
	}
	
	if((reg0 & 0b00000001) && ((reg1 != pwm_ctrl_reg1_old) || (reg2 != pwm_ctrl_reg2_old)))
	{
		ICR1 = (reg2 << 8) + reg1;
	}
	
	if((pwm_ctrl_reg0_old & 0b00000001) && (!(reg0 & 0b00000001)))
	{
		//Mode has changed to servo-mode --> set prescaler to 64, overflow after 20ms and 1ms "ON"-time
		TCCR1B = (TCCR1B & 0b11111000) | 0b00000011;
		ICR1 = 5000;
		OCR1A = 250;
		OCR1B = 250;
	}
	pwm_ctrl_reg0_old = reg0;
	pwm_ctrl_reg1_old = reg1;
	pwm_ctrl_reg2_old = reg2;
}

//------------------------------------
//-- SET values for digital outputs --
//------------------------------------

void set_dout_value(uint8_t value)
{
	uint8_t local_temp = 0;
	
	//split output value and set the pins
	PORTC = (PORTC & 0b11110000)  | (value & 0b00001111);
	local_temp = ((value << 3) & 0b10000000) | ((value << 1) & 0b01000000);
	PORTD = (PORTD & 0b00111111) | (local_temp & 0b11000000);
}

//---------------------------
//-- SET values for relays --
//---------------------------

void set_relay_value(uint8_t value)
{
	PORTC = (PORTC & 0b00001111) | ((value << 4) & 0b11110000);
}

//---------------------------------
//-- SET values for gpio outputs --
//---------------------------------

void set_gpio_value(uint8_t value)
{
	uint8_t i = 0;
	uint8_t temp = 0;
	
	for(i=0; i<4; i++)
	{
		//GPIO is output
		if((GPIO_CTRL & (1<<i)) && (!(GPIO_CTRL & (1<<(i+4)))))
		{
			//value is High
			if(value & (1<<i))
			{
				temp |= (1<<i);
			}
		}
	}
	PORTA = (PORTA & 0b11110000) | temp;
}

//------------------------------------------
//-- SET microcontroller control register --
//------------------------------------------

void set_uc_ctrl(uint8_t reg)
{	
	//only change Watchdog state if the "reg" has changed
	if(uc_control_reg_old == reg)
	{
		;
	}
	else
	{
		if(reg & 0b00000001)
		{
			//activate Watchdog with two-seconds-timeout
			wdt_enable(WDTO_2S);
		}
		else
		{
			//disable watchdog timer
			wdt_disable();
		}
	}
	
	uc_control_reg_old = reg;
}

//------------------------------------------
//------- SET ADC control registers --------
//------------------------------------------

void set_analog_ctrl(uint8_t reg0, uint8_t reg1)
{
	ADC_CTRL[0] = reg0;
	ADC_CTRL[1] = reg1;
	
	if(reg1 == 0)
	{	
		//default is Prescaler 128
		ADCSRA = (ADCSRA & 0b11111000) | 0b00000111;
	}
	else
	{
		ADCSRA = (ADCSRA & 0b11111000) | ((reg1 >> 5) & 0b00000111);
	}
}

//------------------------------
//-- SET values for PWM/Servo --
//------------------------------

void set_pwm_value(uint8_t channel, uint8_t pwm0_value_low, uint8_t pwm0_value_high, uint8_t pwm1_value_low, uint8_t pwm1_value_high)
{
	//channel definition:
	//value 0 --> only channel 0 is updated
	//value 1 --> only channel 1 is updated
	//value 2 --> both channels are updated
	
	//used for Automatic-Mode
	if(!(PWM_CTRL[0] & 0b00000001))
	{
		//PWM0 & PWM1: Servo-Mode
		
		if((channel == 0) || (channel == 2)) 
		{	//Need for Overdrive?
			if(PWM_CTRL[0] & 0b00000010)
			{
				//overdrive for servo0 enabled
				OCR1A = 250+pwm0_value_low+pwm0_value_high-128;
			}
			else
			{
				//overdrive for servo0 disabled
				OCR1A = 250+pwm0_value_low;
			}
		}
		
		if((channel == 1) || (channel == 2)) 
		{
			//Need for Overdrive?
			if(PWM_CTRL[0] & 0b00000100)
			{
				//overdrive for servo1 enabled
				OCR1B = 250+pwm1_value_low+pwm1_value_high-128;
			}
			else
			{
				//overdrive for servo1 disabled
				OCR1B = 250+pwm1_value_low;
			}
		}
	}
	else
	{
		//PWM0 & PWM1: PWM-Mode
		//Set values from pwm-bytes
		if((channel == 0) || (channel == 2))
		{
			OCR1A = (pwm0_value_high << 8) + pwm0_value_low;
		}
		if((channel == 1) || (channel == 2)) 
		{
			OCR1B = (pwm1_value_high << 8) + pwm1_value_low;
		}
	}
}

//-----------------------------------
//-- SET raspberry status register --
//-----------------------------------

void set_raspberry_status_register(uint8_t reg)
{
	; //to be defined in later version!
}

//-----------------------------------------
//-- SET microcontroller status register --
//-----------------------------------------

void set_uc_status_register(uint8_t bit, uint8_t value)
{
	if(value == 1)
	{
		UC_STATUS |= (1 << bit);
	}
	else
	{
		UC_STATUS &= ~((1 << bit));
	}
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++																	+++++
//+++++							Main Function							+++++
//+++++																	+++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main(void) 
{
	//disable interrupts (global)

	cli();
	
	//Define Input and Output Pins

	DDRA = 0b00000000;						//GPIOs Pin 0-3: Inputs, AIs Pin 4-7: Input
	DDRB = 0b01000000;						//DIs Pin 0-3: Input, SPI Pin 4-7: Inputs/Configurated by SPI
	DDRC = 0b11111111;						//DOs Pin 0-3: Output, Relais Pin 4-7: Output
	DDRD = 0b11110000;						//DIs Pin 0-3: Input, PWM Pins 4-5: Inputs, DOs Pin 6-7: Output
	
	//Set all Outputs Low, deactivate all input pull-ups
	
	PORTA = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;
	
	
	
	//-- GPIO 0 in DEBUG-MODE ? --
	if(GPIO_DEBUG)
	{
		DDRA |= (1<<PA0);					//GPIO 0 is output 
		set_uc_status_register(1, 1);		//Signal that GPIO 0 is in DEBUG-MODE
	}
	
	//--------------------------------
	//	Configurate the Peripherals	--
	//--------------------------------
	
	//Enable SPI interface and SPI interrupt source
	SPCR = (1<<SPE)|(1<<SPIE);
	
	//Timer 1 (16bit) for Servo/PWM-Functions
	//Fast-PWM-Mode, TOP is ICR1-Register
	TCCR1B |= (1<<WGM13)|(1<<WGM12);		
	TCCR1A |= (1<<WGM11)|(0<<WGM10);
	//Clear OC1A & OC1B on Compare Match
	TCCR1A |= (1<<COM1A1)|(1<<COM1B1);
	//Prescaler: 64		
	TCCR1B |= (1<<CS11)|(1<<CS10);
	
	//at initial state the timer is used for servo signals --> set standard servo timings
	//5000*0.004ms = 20ms cycle time
	ICR1 = 5000;
	//250*0.004ms = 1ms (servo 0°)
	OCR1A = 250;
	OCR1B = 250;
	
	//ADC: Use external reference at AREF-Pin
	//ADMUX |= (0<<REFS0) | (0<<REFS1);
	//ADC: Start with ADC4 (ANALOG_IN 0)		
	ADMUX |= (1<<MUX2);
	//ADC: Enable
	ADCSRA|=(1<<ADEN);
	//prescaler 128: f_adc = 125kHz --> convert every 13 cycles --> 9,615kHz --> each channel is updated every 0.4ms (2.5kHz)
	ADCSRA|=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);
	//start conversion --> first conversion
	ADCSRA|=(1<<ADSC);
	
	uint8_t i = 0;
	uint8_t ii = 0;
	uint16_t crc_transmit = 0xFFFF;
	uint16_t crc_receive = 0xFFFF;
	uint16_t crc_receive_raspberry = 0;
	
	//Enable interrups (global)
	sei();
	
	//+++++++++++++++++++++++++++++++++++++++++++
	//+++++++++		Infinite Loop		+++++++++
	//+++++++++++++++++++++++++++++++++++++++++++
	
	//get and set data for the next automatic_cycle
	
	while(1)
	{	
		if(spi_automatic_cycle_finished == 1)
		{
			if(GPIO_DEBUG)
			{
				DEBUG_PIN_0_HIGH
			}
			
			//Interrups disabled --> SPI communication is rejected
			cli();
			
			
			//CRC-Checksum - receive buffer
			crc_receive = 0xFFFF;
			for (i=0; i<(NUMBER_OF_AUTOMATIC_MODE_BYTES-2); i++)
			{
				crc_receive = _crc16_update(crc_receive,SPI_RECEIVE_BUFFER[i]);
			}
			
			crc_receive_raspberry = SPI_RECEIVE_BUFFER[29] | (SPI_RECEIVE_BUFFER[30] << 8);
			
			if(crc_receive_raspberry == crc_receive)
			{
				//data is correct
			}
			else
			{
				//data is currupt
				wdt_enable(WDTO_15MS);	//
				while(1); 				//infinite loop --> HW-Watchdog will expire and force uC-Reset
			}
			
			//Values will be updated, if uC has been initialized
			if(UC_INIT)
			{
				//get values
				
				get_din_value();
				get_gpio_value();
				
				for(i=0;i<4;i++)
				{
					get_adc_value(i);
				}
				
				get_dht_value(255);
				
				//set control values
				set_gpio_ctrl(SPI_RECEIVE_BUFFER[10]);
				set_uc_ctrl(SPI_RECEIVE_BUFFER[11]);
				set_analog_ctrl(SPI_RECEIVE_BUFFER[12], SPI_RECEIVE_BUFFER[13]);
				set_pwm_ctrl(SPI_RECEIVE_BUFFER[7], SPI_RECEIVE_BUFFER[8], SPI_RECEIVE_BUFFER[9]);
				
				//set outputs and PWM values
				set_dout_value(SPI_RECEIVE_BUFFER[0]);
				set_relay_value(SPI_RECEIVE_BUFFER[1]);
				set_gpio_value(SPI_RECEIVE_BUFFER[2]);
				set_pwm_value(2, SPI_RECEIVE_BUFFER[3], SPI_RECEIVE_BUFFER[4], SPI_RECEIVE_BUFFER[5], SPI_RECEIVE_BUFFER[6]);
				
				//write data from global variables to transmit buffer
				SPI_TRANSMIT_BUFFER[0] = DIGITAL_IN;
				SPI_TRANSMIT_BUFFER[9] = GPIO_IN;
				
				for(i=1, ii=0 ; i<9, ii<4 ; i+=2, ii++)
				{
					SPI_TRANSMIT_BUFFER[i] = ADC_VALUE[ii][0];
					SPI_TRANSMIT_BUFFER[i+1] = ADC_VALUE[ii][1];
				}
				
				for(i=0; i<4; i++)
				{
					SPI_TRANSMIT_BUFFER[18+2*i] = HUMIDITY[i][0]; //humidity low byte
					SPI_TRANSMIT_BUFFER[19+2*i] = HUMIDITY[i][1]; //humidity high byte
					SPI_TRANSMIT_BUFFER[10+2*i] = TEMPERATURE[i][0]; //temperature low byte
					SPI_TRANSMIT_BUFFER[11+2*i] = TEMPERATURE[i][1]; //temperature high byte
				}
				
				SPI_TRANSMIT_BUFFER[26] = VERSIONL;
				SPI_TRANSMIT_BUFFER[27] = VERSIONH;
				SPI_TRANSMIT_BUFFER[28] = UC_STATUS;
			}
			else //microcontroller is in INIT-State
			{
				if(SPI_RECEIVE_BUFFER[11] & 0b00010000)
				{
					//enter RUN-State
					UC_INIT = 1;
					set_uc_status_register(0, 1);
				}
				else
				{
					//no request to enter "RUN" --> stay in INIT-State
					set_uc_status_register(0, 0);
				}
				
				SPI_TRANSMIT_BUFFER[28] = UC_STATUS;
			}
			
			//CRC-Checksum - transmit buffer
			crc_transmit = 0xFFFF;
			for (i=0; i<(NUMBER_OF_AUTOMATIC_MODE_BYTES-2); i++)
			{
				crc_transmit = _crc16_update(crc_transmit,SPI_TRANSMIT_BUFFER[i]);
			}
			
			SPI_TRANSMIT_BUFFER[29] = crc_transmit & 0b11111111;	//CRC Checksum low byte
			SPI_TRANSMIT_BUFFER[30] = crc_transmit >> 8;			//CRC Checksum high byte
			
			
			//i'm done - data is up to date
			wdt_reset();
			spi_automatic_cycle_finished = 0;
			
			sei();	//interrups enabled --> spi communication is possible again (new automatic cycle or manual transfer is possible now)
		}
		//Only used for Manual-Mode
		else if(spi_adc_request == 1)
		{
			cli(); //interrups disabled --> spi communication is rejected
		
			//do adc convertion(s)
			get_adc_value(spi_adc_request_channel);
			
			spi_adc_request = 0;
			
			sei(); //interrups enabled --> spi communication is possible again (new automatic cycle or manual transfer is possible now)
		}
		//Only used for Manual-Mode
		else if(spi_dht_request == 1)
		{
			cli(); //interrups disabled --> spi communication is rejected
			
			//receive ne DHT11/22 value
			get_dht_value(spi_dht_request_channel);
			spi_dht_request = 0;
			
			sei(); //interrups enabled --> spi communication is possible again (new automatic cycle or manual transfer is possible now)
		}
		
		if(GPIO_DEBUG)
		{
			DEBUG_PIN_0_LOW
		}
	}
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++							Interrupt: SPI						+++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

ISR(SPI_STC_vect)
{
	spi_receive_temp = SPDR;
	
	//+++++++++++++++++++++++++++++++++++++++++
	//Process the reveived data from SPI (SPDR)
	//+++++++++++++++++++++++++++++++++++++++++
	
	//Last byte of the transfer?
	if(spi_transfer_complete == 1)
	{
		if(spi_mode == 2)
		{
			//signal to process the new data
			spi_automatic_cycle_finished = 1;
		}
		
		//The transfer is complete - Manual- or Automatic-Cycle - write last byte which was set by transfer-functions
		//clear variables for new Manual- or Automatic-Cyle
		spi_transfer_complete = 0;
		spi_mode = 0;
		spi_automatic_counter = 0;
		spi_data_mask_set = 0;
		spi_command = 0;
		spi_transmit_temp = 0;
	}
	else if(spi_receive_temp == 0b10000000 && spi_mode == 0)
	{
		//Get ready to transceive spi data in automatic-mode (2)
		spi_mode = 2;
		spi_transmit_temp = 0b10000000; //acknowledge - 128
		spi_automatic_counter = 0;
	}
	else if(spi_receive_temp == 0b10101010 && spi_mode == 0)
	{
		//Get ready to receive spi_command for manual-mode (1), only if no request is already running
		spi_mode = 1;
		spi_transmit_temp = 0b01010101; //acknowledge - 170
	}
	
	//+++++++++++++++++++++++++++++++++++++
	//+++++       Automatic-Mode	  +++++
	//+++++++++++++++++++++++++++++++++++++
	
	else if(spi_mode == 2)
	{
		if(spi_automatic_counter == 0 && spi_data_mask_set == 0)
		{
			spi_data_mask = spi_receive_temp;
			spi_data_mask_set = 1;
			//first byte of the transmit buffer has to be prepared
			spi_transmit_temp = SPI_TRANSMIT_BUFFER[0];
			//spi_automatic_counter++;
		}
		else
		{
			//Last byte of the cycle?
			if(spi_automatic_counter == NUMBER_OF_AUTOMATIC_MODE_BYTES-1)
			{
				//save the last received data byte into the receive buffer
				SPI_RECEIVE_BUFFER[spi_automatic_counter] = spi_receive_temp;
				//bye bye automatic mode
				spi_transmit_temp = 128;
				//transfer is complete
				spi_transfer_complete = 1;
			}
			else
			{
			//save the received data byte
			SPI_RECEIVE_BUFFER[spi_automatic_counter] = spi_receive_temp;
			//write data into transmit buffer - will be send at the next spi-transfer
			spi_transmit_temp = SPI_TRANSMIT_BUFFER[spi_automatic_counter+1];
			//increase counter
			spi_automatic_counter++;
			}
		}
		
		
	}
	
	//+++++++++++++++++++++++++++++++++++++
	//+++++       Manual-Mode	      +++++
	//+++++++++++++++++++++++++++++++++++++
	
	else if(spi_mode == 1 && spi_command == 0)
	{
		//Get ready to read/write data
		spi_command = spi_receive_temp;
		//acknowledge by sending the command back to Raspberry
		spi_transmit_temp = spi_receive_temp;
	}
	else if(spi_mode == 1 && spi_command != 0)
	{
		if(spi_command < 128) //read or write a value 
		{	
			//write digital Outputs
			if(spi_command == 0b00000001)
			{
				set_dout_value(SPDR);
				spi_transmit_temp = spi_receive_temp;
				
				//transfer complete
				spi_transfer_complete = 1;
			} 
			//read digital Inputs
			else if(spi_command == 0b00000010)
			{
				get_din_value();
				spi_transmit_temp = DIGITAL_IN;
				
				//transfer complete
				spi_transfer_complete = 1;
			}
			//read ADC_VALUE from channel 0
			else if(spi_command == 0b00000011)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = ADC_VALUE[0][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = ADC_VALUE[0][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that new ADC_VALUEs are requested
					spi_adc_request = 1;
					//...for channel 0
					spi_adc_request_channel = 0;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read ADC_VALUE from channel 1
			else if(spi_command == 0b00000100)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = ADC_VALUE[1][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = ADC_VALUE[1][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that new ADC_VALUEs are requested
					spi_adc_request = 1;
					//...for channel 0
					spi_adc_request_channel = 1;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read ADC_VALUE from channel 2
			else if(spi_command == 0b00000101)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = ADC_VALUE[2][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = ADC_VALUE[2][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that new ADC_VALUEs are requested
					spi_adc_request = 1;
					//...for channel 0
					spi_adc_request_channel = 2;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read ADC_VALUE from channel 3
			else if(spi_command == 0b00000110)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = ADC_VALUE[3][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = ADC_VALUE[3][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that new ADC_VALUEs are requested
					spi_adc_request = 1;
					//...for channel 0
					spi_adc_request_channel = 3;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//write Relays
			if(spi_command == 0b00000111)
			{
				set_relay_value(SPDR);
				spi_transmit_temp = spi_receive_temp;
				
				//transfer complete
				spi_transfer_complete = 1;
			} 
			//write GPIOs
			if(spi_command == 0b00001000)
			{
				set_gpio_value(SPDR);
				spi_transmit_temp = spi_receive_temp;
				
				//transfer complete
				spi_transfer_complete = 1;
			} 
			//read GPIOs
			else if(spi_command == 0b00001001)
			{
				get_gpio_value();
				spi_transmit_temp = GPIO_IN;
				
				//transfer complete
				spi_transfer_complete = 1;
			}
			//read temperature value 0 (DHT11/22 on GPIO 0)
			else if(spi_command == 0b00001010)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = TEMPERATURE[0][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = TEMPERATURE[0][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that a new DHT_VALUE is requested
					spi_dht_request = 1;
					//...for channel 0
					spi_dht_request_channel = 0;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read temperature value 1 (DHT11/22 on GPIO 1)
			else if(spi_command == 0b00001011)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = TEMPERATURE[1][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = TEMPERATURE[1][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that a new DHT_VALUE is requested
					spi_dht_request = 1;
					//...for channel 0
					spi_dht_request_channel = 1;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read temperature value 2 (DHT11/22 on GPIO 2)
			else if(spi_command == 0b00001100)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = TEMPERATURE[2][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = TEMPERATURE[2][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that a new DHT_VALUE is requested
					spi_dht_request = 1;
					//...for channel 0
					spi_dht_request_channel = 2;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read temperature value 3 (DHT11/22 on GPIO 3)
			else if(spi_command == 0b00001101)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = TEMPERATURE[3][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = TEMPERATURE[3][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that a new DHT_VALUE is requested
					spi_dht_request = 1;
					//...for channel 0
					spi_dht_request_channel = 3;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read humidity value 0 (DHT11/22 on GPIO 0)
			else if(spi_command == 0b00001110)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = HUMIDITY[0][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = HUMIDITY[0][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that a new DHT_VALUE is requested
					spi_dht_request = 1;
					//...for channel 0
					spi_dht_request_channel = 0;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read humidity value 1 (DHT11/22 on GPIO 1)
			else if(spi_command == 0b00001111)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = HUMIDITY[1][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = HUMIDITY[1][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that a new DHT_VALUE is requested
					spi_dht_request = 1;
					//...for channel 0
					spi_dht_request_channel = 1;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read humidity value 2 (DHT11/22 on GPIO 2)
			else if(spi_command == 0b00010000)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = HUMIDITY[2][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = HUMIDITY[2][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that a new DHT_VALUE is requested
					spi_dht_request = 1;
					//...for channel 0
					spi_dht_request_channel = 2;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read humidity value 3 (DHT11/22 on GPIO 3)
			else if(spi_command == 0b00010001)
			{	
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = HUMIDITY[3][0]; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = HUMIDITY[3][1]; //high-byte
					spi_byte_cnt = 0;
					
					//remind that a new DHT_VALUE is requested
					spi_dht_request = 1;
					//...for channel 0
					spi_dht_request_channel = 3;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read DO value
			else if(spi_command == 0b00010010)
			{
				get_dout_value();
				spi_transmit_temp = DOUT_VALUE;
				
				//transfer complete
				spi_transfer_complete = 1;
			}
			//read RELAY value
			else if(spi_command == 0b00010011)
			{
				get_relay_value();
				spi_transmit_temp = RELAY_VALUE;
				
				//transfer complete
				spi_transfer_complete = 1;
			}
		}
		else //read or write a register or a group of registers
		{
			//write servo value 0
			if(spi_command == 0b10000000)
			{
				set_pwm_value(0, SPDR, 0, 0, 0);
				
				spi_transmit_temp = spi_receive_temp;
				
				//transfer complete
				spi_transfer_complete = 1;
			}
			//write servo value 1
			else if(spi_command == 0b10000001)
			{
				set_pwm_value(1, 0, 0, SPDR, 0);
				
				spi_transmit_temp = spi_receive_temp;
				
				//transfer complete
				spi_transfer_complete = 1;
			}
			//write PWM value 0
			else if(spi_command == 0b10000010)
			{
				if(spi_byte_cnt == 0)
				{
					spi_pwm_temp0 = SPDR;
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_pwm_temp1 = SPDR;
					
					//all values are available now --> set registers
					set_pwm_value(0, spi_pwm_temp0, spi_pwm_temp1, 0, 0);
					
					//transfer complete
					spi_transfer_complete = 1;
					spi_byte_cnt = 0;
				}
				
				spi_transmit_temp = spi_receive_temp;
			}
			//write PWM value 1
			else if(spi_command == 0b10000011)
			{
				if(spi_byte_cnt == 0)
				{
					spi_pwm_temp0 = SPDR;
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_pwm_temp1 = SPDR;
					
					//all values are available now --> set registers
					set_pwm_value(1, 0, 0, spi_pwm_temp0, spi_pwm_temp1);
					
					//transfer complete
					spi_transfer_complete = 1;
					spi_byte_cnt = 0;
				}
				
				spi_transmit_temp = spi_receive_temp;
			}
			//set PWM control registers
			else if(spi_command == 0b10000100)
			{
				if(spi_byte_cnt == 0)
				{
					spi_pwm_ctrl_temp0 = SPDR;
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_pwm_ctrl_temp1 = SPDR;
					spi_byte_cnt = 2;
				}
				else if(spi_byte_cnt == 2)
				{
					spi_pwm_ctrl_temp2 = SPDR;
					
					//all values are available now --> set registers
					set_pwm_ctrl(spi_pwm_ctrl_temp0, spi_pwm_ctrl_temp1, spi_pwm_ctrl_temp2);
					
					//transfer complete
					spi_byte_cnt = 0;
					spi_transfer_complete = 1;
				}
				
				spi_transmit_temp = spi_receive_temp;
			}
			//set GPIO control register
			else if(spi_command == 0b10000101)
			{
				DEBUG_PIN_0_HIGH
				set_gpio_ctrl(SPDR);
				DEBUG_PIN_0_LOW
				spi_transmit_temp = spi_receive_temp;
				
				//transfer complete
				spi_transfer_complete = 1;
			}
			//set uC control register
			else if(spi_command == 0b10000110)
			{
				set_uc_ctrl(SPDR);
				spi_transmit_temp = spi_receive_temp;
				
				//transfer complete
				spi_transfer_complete = 1;
			}
			//set analog input control registers
			else if(spi_command == 0b10000111)
			{
				if(spi_byte_cnt == 0)
				{
					spi_adc_ctrl_temp0 = SPDR;
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_adc_ctrl_temp1 = SPDR;
					
					//all values are available now --> set registers
					set_analog_ctrl(spi_adc_ctrl_temp0, spi_adc_ctrl_temp1);
					
					//transfer complete
					spi_transfer_complete = 1;
					spi_byte_cnt = 0;
				}
				spi_transmit_temp = spi_receive_temp;
			}
			//set raspberry status register
			else if(spi_command == 0b10001000)
			{
				set_raspberry_status_register(SPDR);
				spi_transmit_temp = spi_receive_temp;
				
				//transfer complete
				spi_transfer_complete = 1;
			}
			//read uC version registers
			else if(spi_command == 0b10001001)
			{
				if(spi_byte_cnt == 0)
				{
					spi_transmit_temp = VERSIONL; //low-byte
					spi_byte_cnt = 1;
				}
				else if(spi_byte_cnt == 1)
				{
					spi_transmit_temp = VERSIONH; //high-byte
					spi_byte_cnt = 0;
					
					//transfer complete
					spi_transfer_complete = 1;
				}
			}
			//read uC status register
			else if(spi_command == 0b10001010)
			{
				spi_transmit_temp = UC_STATUS;
				
				//transfer complete
				spi_transfer_complete = 1;
			}
		}
	}
	else
	{
		//the byte/command is unknown or corrupt --> reset variables for new transfer
		spi_mode = 0;
		spi_automatic_counter = 0;
		spi_command = 0;
		spi_transmit_temp = spi_receive_temp;
	}
	
	//write data to the SPI transmit register of the uC
	SPDR = spi_transmit_temp;
}

//The End!