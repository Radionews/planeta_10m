/*
 * Новые мозги Планате-10М на atmega16
 *
 * Created: 01.03.2019 23:28:09
 * Author : Андрей
 */ 

#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>

#define F_CPU 1000000
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1
#define N_BUFF 82

#define RD 0x80
#define WR 0x40
#define MODE_0 0x80
#define A0 0
#define A1 1
#define CS0 6
#define CS1 7

volatile unsigned char NEW_DATA = 0;
//volatile unsigned char INDEX_R = 0;
volatile unsigned char DATA_AVAL = 0;

void USARTInit(unsigned int ubrr) {
	//  нормальный асинхронный двунаправленный режим работы
	//  UBRR = f / (16 * band)
	//  Установка скорости
	ubrr = 12;
	UBRRH = (unsigned char)(ubrr>>8);
	UBRRL = (unsigned char)(ubrr);
	
	//  RXC         - завершение приёма
	//  |TXC        - завершение передачи
	//  ||UDRE      - отсутствие данных для отправки
	//  |||FE       - ошибка кадра
	//  ||||DOR     - ошибка переполнение буфера
	//  |||||PE     - ошибка чётности
	//  ||||||U2X   - Двойная скорость
	//  |||||||MPCM - Многопроцессорный режим
	//  ||||||||
	//  76543210
	UCSRA = 0x02;
	
	//  RXCIE       - прерывание при приёме данных
	//  |TXCIE      - прерывание при завершение передачи
	//  ||UDRIE     - прерывание отсутствие данных для отправки
	//  |||RXEN     - разрешение приёма
	//  ||||TXEN    - разрешение передачи
	//  |||||UCSZ2  - UCSZ0:2 размер кадра данных
	//  ||||||RXB8  - 9 бит принятых данных
	//  |||||||TXB8 - 9 бит переданных данных
	//  ||||||||
	//  76543210
	
	//  разрешен приём и передача данных по UART
	UCSRB = 1<<RXCIE | 1<<RXEN | 1<<TXEN;
	
	//  URSEL        - всегда 1
	//  |UMSEL       - режим: 1-синхронный 0-асинхронный
	//  ||UPM1       - UPM0:  1 чётность
	//  |||UPM0      - UPM0:  1 чётность
	//  ||||USBS     - стоп биты: 0-1, 1-2
	//  |||||UCSZ1   - UCSZ0: 2 размер кадра данных
	//  ||||||UCSZ0  - UCSZ0: 2 размер кадра данных
	//  |||||||UCPOL - в синхронном режиме - тактирование
	//  ||||||||
	//  76543210
	//  8-битовая посылка, 2 стоп бита
	UCSRC = 1<<URSEL | 1<<USBS | 1<<UCSZ0 | 1<<UCSZ1;
}

void USARTTransmitChar(char c) {
	//  Устанавливается, когда регистр свободен
	while(!( UCSRA & (1<<UDRE)));
	UDR = c;
}

ISR(USART_RXC_vect)
{
	DATA_AVAL = 1;
	NEW_DATA = UDR;
}

unsigned char stroka [80] = {
	0,0,0,0,0,0,0,
	34,65,
	73,73,54,0,126,17,17,17,126,0,127,1,1,1,1,0,
	126,17,17,17,14,0,3,4,8,16,127,0,34,65,73,73,
	54,0,127,8,20,34,65,0,126,17,17,17,126,0,0,0,
	32,0,0,0,0,0,32,0,0,0,0,0,32,0,0,0,
	0,0,0,0,0,0,0
};

  
void write()
{
	PORTD &= ~WR;
	//_delay_ms(50);
	PORTD |= WR;
}

void init()
{
	DDRA = 0xFF;
	DDRB = 0x0F;
	DDRC = 0xFF;
	DDRD = 0xE0;
	PORTD = 0;
	//настройка всех портов МК на вывод
	PORTD |= RD;
	//опускание линии RD (чтения) в ноль, он нам вообще не нужен
	PORTA = MODE_0;
	//подготавливаем управляющее слово
	PORTC = (1<<CS0)|(1<<A0)|(1<<A1);
	write();
	PORTC = (1<<CS1)|(1<<A0)|(1<<A1);
	write();
	//настриваем в обе микросхемы режим 0
	//посылакем строб записи
	PORTA = 0;
	PORTC = 0;
}


int main(void)
{
	init();
	USARTInit(MYUBRR);
	sei();
	
	unsigned char y = 0;
	while (1) 
    {
		y = 0;
		
		if(DATA_AVAL == 1)
		{
			for(int l = 79; l>0;l--)
			{
				stroka[l] = stroka[l-1];
			}
			stroka[0] = NEW_DATA;
			DATA_AVAL = 0;
		}
		
		for (unsigned char x=0;x<5;x++)
		{
			PORTA = 0;
			PORTC = 0;
			PORTB = 0x0F;
			if(x>2) 
			{
				PORTC |= (1<<CS1);
				PORTC |= x-3;
			}
			else
			{
				PORTC |= (1<<CS0);
				PORTC |= x;
			}
			//write();
			//_delay_ms(500);
			
			for(unsigned char i = 0;i<16;i++)
			{
				PORTA = stroka[y];
				y++;
				PORTB = (~(i)&0x07);
				if(i>=8)
				{
					PORTB &= ~(1<<3);
					
				}
				else
				{
					PORTB |= (1<<3);
				}
				write();
				_delay_us(200);
				PORTA = 0;
				write();
			}
		}	
    }
}

