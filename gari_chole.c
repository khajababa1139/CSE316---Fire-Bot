#define F_CPU 1000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#ifndef USART_RS232_H_FILE_H_				/* Define library H file if not defined */
#define USART_RS232_H_FILE_H_

#define F_CPU 1000000UL						/* Define CPU clock Frequency e.g. here its 8MHz */
#include <avr/io.h>							/* Include AVR std. library file */
#define BAUD_PRESCALE (((F_CPU / (BAUDRATE * 16UL))) - 1)	/* Define prescale value */

void USART_Init(unsigned long);				/* USART initialize function */
char USART_RxChar();						/* Data receiving function */
void USART_TxChar(char);					/* Data transmitting function */
void USART_SendString(char*);				/* Send string of USART data function */


#endif										/* USART_RS232_H_FILE_H_ */

void USART_Init(unsigned long BAUDRATE)				/* USART initialize function */
{
	UCSRB |= (1 << RXEN) | (1 << TXEN);				/* Enable USART transmitter and receiver */
	UCSRC |= (1 << URSEL)| (1 << UCSZ0) | (1 << UCSZ1);	/* Write USCRC for 8 bit data and 1 stop bit */
	UBRRL = BAUD_PRESCALE;							/* Load UBRRL with lower 8 bit of prescale value */
	UBRRH = (BAUD_PRESCALE >> 8);					/* Load UBRRH with upper 8 bit of prescale value */
}

char USART_RxChar()									/* Data receiving function */
{
	if (UCSRA & (1 << RXC)) {
		return UDR;            // Data available
		} else {
		return '\0';          // No data
	}								/* Get and return received data */
}

void USART_TxChar(char data)						/* Data transmitting function */
{
	UDR = data;										/* Write data to be transmitting in UDR */
	while (!(UCSRA & (1<<UDRE)));					/* Wait until data transmit and buffer get empty */
}

void USART_SendString(char *str)					/* Send string of USART data function */
{
	int i=0;
	while (str[i]!=0)
	{
		USART_TxChar(str[i]);						/* Send each char of string till the NULL */
		i++;
	}
}
//

#define MPU_ADDRESS 0x68
// ----------- TWI / I2C -----------
void TWI_Init() {
	TWSR = 0x00;
	TWBR = 2; // 100kHz for 1MHz clock
	TWCR = (1 << TWEN);
}

void TWI_Start() {
	TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

void TWI_Stop() {
	TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
}

void TWI_Write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

uint8_t TWI_ReadACK() {
	TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

uint8_t TWI_ReadNACK() {
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

// ----------- MPU9250 -----------
void MPU_Write(uint8_t reg, uint8_t data) {
	TWI_Start();
	TWI_Write(MPU_ADDRESS << 1);
	TWI_Write(reg);
	TWI_Write(data);
	TWI_Stop();
}

int16_t MPU_Read16(uint8_t regH) {
	int16_t val;
	TWI_Start();
	TWI_Write(MPU_ADDRESS << 1);
	TWI_Write(regH);
	TWI_Start();
	TWI_Write((MPU_ADDRESS << 1) | 1);
	uint8_t high = TWI_ReadACK();
	uint8_t low = TWI_ReadNACK();
	TWI_Stop();
	val = (high << 8) | low;
	return val;
}

void MPU_Init() {
	MPU_Write(0x6B, 0x00); // PWR_MGMT_1 = 0 → wake up
	_delay_ms(100);
}


#define LED PORTB		/* connected LED on PORT pin */

void motor_init(){
	DDRA = 0xff;
	DDRB = 0xff;
}

void motor_forward(){
	//IN1 - HI, IN2 - LO, IN3 - HI, IN4 - LO
	PORTA = 0b00000101;
	PORTB = 0xff;
	
}

void motor_stop(){
	PORTA = 0b00000000;
	PORTB = 0x00;
}

void motor_reverse(){
	
	PORTA = 0b00001010;
	PORTB = 0xff;
}

void motor_right(){
	PORTA = 0b00001001;
	PORTB = 0xff;
}

void motor_left(){
	PORTA = 0b00000110;
	PORTB = 0xff;
}

void pump_up() {
	PORTA = 0b00100000;
	PORTB = 0xff;
}

void pump_down() {
	PORTA = 0b00010000;
	PORTB = 0xff;
}

int main(void)
{
	char buffer[10];
	int16_t ax, ay, az;
	OSCCAL = 0xAC;
	TWI_Init();
	MPU_Init();

	char Data_in;
	DDRB = 0xff;		/* make PORT as output port */
	USART_Init(9600);	/* initialize USART with 9600 baud rate */
	motor_init();
	LED = 0b00000001;
	
	DDRC &= ~(1 << PC6);
	PORTC &= ~(1 << PC6);
	
	DDRC &= ~(1 << PC7);
	PORTC &= ~(1 << PC7);
	
	DDRC &= ~(1 << PC5);
	PORTC &= ~(1 << PC5);
	
	while(1)
	{
		if(!(PINC & (1 << PC6)) || !(PINC & (1 << PC7)) || !(PINC & (1 << PC5))) {
			LED = 0b00000111;
			USART_SendString("FIRE DETECTED!! ");
		}
		else if((PINC & (1 << PC6)) || (PINC & (1 << PC7)) || (PINC & (1 << PC5))) {
			LED = 0b00000001;
			USART_SendString("No Fire ");
		} 
		
		ax = MPU_Read16(0x3B); // ACCEL_XOUT_H
		ax = ax / 17000;
		ay = MPU_Read16(0x3D);
		ay = ay / 100;
		az = MPU_Read16(0x3F);
		az = az / 1000;
		
		USART_SendString("MPU DATA-X: ");
		itoa(ax, buffer, 10);
		USART_SendString(buffer);

		USART_SendString(" Y: ");
		itoa(ay, buffer, 10);
		USART_SendString(buffer);

		USART_SendString(" Z: ");
		itoa(az, buffer, 10);
		USART_SendString(buffer);

		USART_SendString("\n");
		
		Data_in = USART_RxChar();	/* receive data from Bluetooth device*/
		
		if(Data_in == 'F')
		{
			LED = 0b00000011;	/* Turn ON LED */
			motor_forward();
			USART_SendString("LED_ON | Going Forward");/* send status of LED i.e. LED ON */
			
		}
		else if(Data_in == 'B')
		{
			LED = 0b00000011;	/* Turn ON LED */
			motor_reverse();
			USART_SendString("LED_ON | Going Reverse");
			/* send status of LED i.e. LED OFF */
		}
		else if(Data_in == 'S')
		{
			LED = 0b00000001;	/* Turn OFF LED */
			motor_stop();
			USART_SendString("LED_OFF | STOP");
			/* send status of LED i.e. LED OFF */
		}
		else if(Data_in == 'L')
		{
			LED = 0b00000011;
			motor_left();
			USART_SendString("LED_ON | Going LEFT");
			/* send status of LED i.e. LED OFF */
		}
		else if(Data_in == 'R')
		{
			LED = 0b00000011;	/* Turn ON LED */
			motor_right();
			USART_SendString("LED_ON | Going RIGHT");
			/* send status of LED i.e. LED OFF */
		}
		else if(Data_in == 'P')
		{
			LED = 0b00000011;	/* Turn ON LED */
			pump_up();
			USART_SendString("LED_ON | Pump UP");
			/* send status of LED i.e. LED OFF */
		}
		else if(Data_in == 'O')
		{
			LED = 0b00000011;	/* Turn ON LED */
			pump_down();
			USART_SendString("LED_ON | Pump DOWN");
			/* send status of LED i.e. LED OFF */
		}
		
		_delay_ms(100);  // Debounce delay
	}
}
