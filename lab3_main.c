#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"

#define F_CPU 16000000UL

#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>

#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#define largest_dis 60
#define smallest_dis 2

volatile int time_len=0;
volatile int edge=0;
char String[25];

volatile int high_or_low=0;//1 when it is high; 0 when it is low; start at low

volatile int rise_or_fall=1;//1 when looking for a rising edge; 0 when when looking for a falling edge

volatile int start_time=0;//The time that sending a trig pulse
volatile int end_time=0;//The time that receiving the first echo pulse

int time_tics=0;
int time_length=0;
unsigned int distance=0;

int c_or_d=1;//Continuous mode or discrete mode. Continuous by default.

int OCR0A_calculation(int distance)
{	
	int cal_OCR0A=0;
	float a=0;
	float b=0;
	if (c_or_d)// In continuous mode.
	{	
		//(OCR0A-60)/(120-60)=(distance-smallest_dis)/(largest_dis-smallest_dis)
		a=-60/(largest_dis-smallest_dis);
		b=(smallest_dis)*60/(largest_dis-smallest_dis)+120;
		cal_OCR0A=a*distance+b;
	}
	else //In discrete mode.
	{
		if ((distance>=2)&&(distance<10))
		cal_OCR0A=120;
		if ((distance>=10)&&(distance<20))
		cal_OCR0A=106;
		if ((distance>=20)&&(distance<30))
		cal_OCR0A= 95;
		if ((distance>=30)&&(distance<40))
		cal_OCR0A=89;
		if ((distance>=40)&&(distance<50))
		cal_OCR0A=80;
		if ((distance>=50)&&(distance<60))
		cal_OCR0A=71;
		if ((distance>=70)&&(distance<80))
		cal_OCR0A= 63;
		if (distance>=80)
		cal_OCR0A=60;
	}
	return cal_OCR0A;
}

int dutycycle_calculation(int OCR0A_num)
{
	int cal_OCR0B=0;
	float duty_cycle=0;
	
	duty_cycle=0.05*((int)(ADC/70)+1);
	cal_OCR0B=OCR0A_num*duty_cycle;
	return cal_OCR0B;
}


void Input_capture_ini(){
	DDRB &=~(1<<DDB5);		//Set PB0(ICP1) as an input pin, echo
	//Enable Timer/Counter1 input capture interrupt
	TIMSK1 |= (1<<ICIE1);

	//Prescale Timer1 by 64
	//frequency=16M/64=62.5k
	//period=1/62.5k=4us
	TCCR1B |=(1<<CS11);
	TCCR1B |=(1<<CS10);
	
	//Looking for a rising edge
	TCCR1B |= (1<<ICES1);
	
	//Write a logic one to ICF1 to clear it
	TIFR1 |= (1<<ICF1);
	
}

void Output_compare_ini(){
	
	DDRD |=(1<<DDD6);		//Set PD6(OC0A) as an output pin
	DDRD |=(1<<DDD5);		//Set PD5(OC0B) as an output pin	

	//Prescale Timer0 by 64
	//frequency=16M/64=250kHz
	//period=1/250k=4us
	TCCR0B |=(1<<CS00);
	TCCR0B |=(1<<CS01);
	TCCR0B &=~(1<<CS02);
	
	//Enable Timer0 to Phase Correct PWM mode (WGM02,01,00 1 0 1)
	TCCR0B |=(1<<WGM02);
	TCCR0A &=~(1<<WGM01);	
	TCCR0A |=(1<<WGM00);
		
	//Enable Timer0 compare match A interrupt
	TIMSK0 |= (1<<OCIE0A);

	//Enable Timer0 compare match B interrupt
	TIMSK0 |= (1<<OCIE0B);
	
	//Toggle PD6 (OC0A) on compare match
	TCCR0A |=(1<<COM0A0);
	TCCR0A &=~(1<<COM0A1);
	
	//Clear when upcounting set it when downcounting
	TCCR0A |=(1<<COM0B1);
	TCCR0A &=~(1<<COM0B0);
	
	OCR0A=30;//Begin compare quickly
}

void ADC_ini(){
	//Setup for ADC
	//Clear power reduction for ADC
	PRR &=~(1<<PRADC);
	
	//Select Vref=AVcc
	ADMUX |=(1<<REFS0);
	ADMUX &=~(1<<REFS1);
	
	//Set ADC clk divided by 128, 16M/128=125k
	ADCSRA|=(1<<ADPS0);
	ADCSRA|=(1<<ADPS1);
	ADCSRA|=(1<<ADPS2);
	
	//Select channel 0(MUX0,1,2,3 0 0 0 0)
	ADMUX &=~(1<<MUX0);
	ADMUX &=~(1<<MUX1);
	ADMUX &=~(1<<MUX2);
	ADMUX &=~(1<<MUX3);	
	
	//Set to free running
	ADCSRA |= (1<<ADATE);
	ADCSRB &= ~(1<<ADTS0);
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2);	
	
	//Disable digital input buffer on ADC pin
	DIDR0 |= (1<<ADC0D);
	
	//Enable ADC
	ADCSRA |= (1<<ADEN);
	
	//Start conversion
	ADCSRA |= (1<<ADSC);
}

void initialize(){
	cli();					//Disable all global interrupts
	
	DDRD |=(1<<DDD2);		//Set PD2 as an output pin, trig
	
	DDRB &=~(1<<DDB1);		//Set PC4 as an input pin, to choose mode
	
	//PORTB|=(1<<PORTB1);	//Enable pull-up resistor
	Output_compare_ini();
	Input_capture_ini();
	ADC_ini();

	sei();	//Enable global interrupts
	
}


ISR(TIMER0_COMPA_vect)
{
	
}

ISR(TIMER0_COMPB_vect)
{
	
}

ISR(TIMER1_CAPT_vect)
{
	if (rise_or_fall)//If it is a rising edge, record the start time
	{
		start_time=ICR1;
		rise_or_fall=0;
	}
	else
	{
		end_time=ICR1;//If it is a falling edge, record the end time
		rise_or_fall=1;

		time_tics=end_time-start_time;
		
		time_length=4*time_tics;
		distance=time_length/58;//Distance is expressed in cm.

		OCR0A=OCR0A_calculation(distance);		
		OCR0B=dutycycle_calculation(OCR0A);
		
		sprintf(String, " %d\n",OCR0B);
		UART_putstring(String);
		sprintf(String, " %d\n",c_or_d);
		UART_putstring(String);	
	}
	TCCR1B^=(1<<ICES1);
}

int main(void)
{

	UART_init(BAUD_PRESCALER);
	initialize();
	
	PORTD|=(1<<PORTD2);
	_delay_us(10);
	PORTD&=~(1<<PORTD2);

	while (1)
	{
		if (PINB & (1<<PINB1))
		{	c_or_d = 1 - c_or_d;
		    _delay_ms(100);
		}
		PORTD|=(1<<PORTD2);
		_delay_us(10);
		PORTD&=~(1<<PORTD2);
		_delay_ms(30);
	}
	
}
