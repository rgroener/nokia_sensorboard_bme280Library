/*
Nokia Sensorboard
* 
* TWI Bus scannen und antwortende Addressen ausgeben.
* 
* Probleme: 0x41 stopt programm
* 
* 
* 
*/


#define F_CPU 8000000UL  // 1 MHz
#include <avr/io.h>
#include "glcd/glcd.h"
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include "glcd/fonts/Liberation_Sans15x21_Numbers.h"
#include "glcd/fonts/font5x7.h"
#include <avr/pgmspace.h>
#include <util/delay.h>

#define T_RED !(PIND & (1<<PD5)) && (entprell == 0)
#define T_BLUE !(PIND & (1<<PD6)) && (entprell == 0)
#define T_GREEN !(PIND & (1<<PD2)) && (entprell == 0)
#define RELOAD_ENTPRELL 80

#define LED_EIN PORTC |= (1<<PC3)
#define LED_AUS	PORTC &= ~(1<<PC3)					//LED ausschalten

#define TW_START 0xA4 // send start condition (TWINT,TWSTA,TWEN)
#define TW_READY (TWCR & 0x80) // ready when TWINT returns to logic 1.
#define TW_STATUS (TWSR & 0xF8) // returns value of status register
#define TW_SEND 0x84 // send data (TWINT,TWEN)


#define F_SCL 100000L // I2C clock speed 100 kHz
//#define F_CPU 800000L // I2C clock speed 100 kHz

uint8_t flag_send, flag_send_index;
// String für Zahlenausgabe
char string[30] = "";
uint8_t addresse, stat, position;
uint8_t ms, ms10,ms100,sec,min,entprell, state;
uint8_t end_ms100, end_sec, end_min;
enum state{WAIT, COUNT, TIME, TIME_WAIT,FLIGHT_TIME};

static void setup(void)
{
	/* Set up glcd, also sets up SPI and relevent GPIO pins */
	glcd_init();
}

ISR (TIMER1_COMPA_vect)
{
	ms10++;
	if(entprell != 0)entprell--;
	if(ms10==10)	//10ms
	{
		ms10=0;
		ms100++;
	}
    if(ms100==10)	//100ms
	{
		ms100=0;
		sec++;
	}
	if(sec==60)	//Minute
	{
		sec=0;
		min++;
	}
}

void twi_Init(void)
{
 /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR =0;                         /* no prescaler */
  TWBR = ((8000000/400000)-16)/2;  /* (F_CPU / F_TWI) must be > 10 for stable operation */
}
 
void twi_Start(void)
{
	TWCR = ((1<<TWINT) | (1<<TWSTA) | (1<<TWEN));
	while((TWCR & (1<<TWINT)) == 0);
}

void twi_Stop(void)
{
	TWCR = ((1<<TWINT) | (1<<TWSTO) | (1<<TWEN));
} 

void twi_Write(uint8_t u8data)
{
	TWDR = u8data;
	TWCR = ((1<<TWINT) | (1<<TWEN));
	while((TWCR & (1<<TWINT)) == 0);
}
uint8_t twi_GetStatus(void)
{
	/*  0x08   Start condition sent
	 *  0x10   repeated start condition sent
	 *  0x18   SLA+W transmitted ACK received
	 *  0x20   SLA+W transmitted NACK received
	 *  0x28   data byte sent ACK received
	 *  0x30   data byte sent NACK received
	 *  0x38   Arbitration in SLA+W lost
	 */
	 
	uint8_t status;
	//mask status
	status = TWSR & 0xF8;
	return status;
}
int main(void)
{
	/* Backlight pin PL3, set as output, set high for 100% output */
	DDRB |= (1<<PB2);
	PORTB |= (1<<PB2);
	//PORTB &= ~(1<<PB2);
	
	
	DDRC |=(1<<PC3); 	//Ausgang LED
	PORTC |= (1<<PC3);	//Led ein
	
	
	DDRD &= ~((1<<PD6) | (1<<PD2) | (1<<PD5)); 	//Taster 1-3
	PORTD |= ((1<<PD6) | (1<<PD2) | (1<<PD5)); 	//PUllups für Taster einschalten
	
	DDRD &= ~(1<<PD4); //T0 Counter Input
	TCCR0B |= (1<<CS02) | (1<<CS01) | (1<<CS00);//Counter 0 enabled clock on rising edge
	
	//Timer 1 Configuration
	OCR1A = 0x009C;	//OCR1A = 0x3D08;==1sec
	
    TCCR1B |= (1 << WGM12);
    // Mode 4, CTC on OCR1A

    TIMSK1 |= (1 << OCIE1A);
    //Set interrupt on compare match

    TCCR1B |= (1 << CS12) | (1 << CS10);
    // set prescaler to 1024 and start the timer

    //sei();
    // enable interrupts
	
	setup();
	
	twi_Init();

	glcd_tiny_set_font(Font5x7,5,7,32,127);
	glcd_clear_buffer();
	glcd_clear();
		
	addresse=0;
	position=10;
	

	
		while(1) 
		{	
			for(addresse=0;addresse<255;addresse++)
			{
				//print addresse die gerade angeruffen wird
				if((addresse==0xa1)||(addresse==0xef))addresse++;//addresse 0xa1 bleibt stehen Grund momentan nicht bekannt
				sprintf(string,"check: 0x%02x",addresse);
				glcd_draw_string_xy(0, 0,string);
				
				twi_Start();									//TWI open
				twi_Write(addresse);
				if(0x18==twi_GetStatus())					//gibt ein Bauteil auf Addresse Antwort?
				{
					sprintf(string,"0x%x",addresse);
					glcd_draw_string_xy(0, position,string);	//Addresse von antwortendem Bauteil ausgeben
					position=position+10;								//cursor an naechste Position bewegen
				}
				twi_Stop();										//close TWI 
				
				
				_delay_ms(5);//Delay
				glcd_write();//Display beschreiben
			}
				
			glcd_draw_string_xy(0, 0,"scan finished");	
			glcd_write();//Display beschreiben
			
			while(1);//stop program
			
			
				
		
				
		
		
		
		if(T_RED)
		{
			flag_send=1;
			LED_AUS;
			flag_send_index=0;
		}
		
		if(T_BLUE)
		{
			LED_EIN;
			flag_send=0;
			flag_send_index=1;
		}
		
	
	
	}//End of while
	
	return 0;
}//end of main
