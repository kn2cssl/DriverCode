/*
 * GccApplication1.c
 *
 * Created: 7/8/2013 4:44:35 PM
 *  Author: Milad
 */

#include <asf.h>
#include <avr/io.h>
#include <stdio.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include "Initialize.h"
#define _Freq (1.0/(2.0*3.14*2.0))
//#define TCNT1 (int)(TCNT1L|(TCNT1H<<8))

char slave_address=0;
char answer_permission;
char send_buff;
char str[100];
int hall_flag=0,hall_dir=0;
int counter=0;
int ctrl=0;
int PWM;
signed long int RPM_setpoint=900;
int tmp_setpoint,tmp_rpmA,tmp_rpmB;
unsigned char pck_num = 0;
float RPM,kp,ki,kd;
uint16_t Time=0;
uint8_t Motor_Direction;
char test_driver=0b11;
struct Motor_Param 
{
	int Encoder;
	int Err,d,i;
	int Direction;
	int RPM;
	int RPM_last;
	int PWM;
	int HSpeed;
	int PWM1;
	signed int RPM_setpointB;
	signed int RPM_setpointA;
	
}M;

int main(void)
{  
	slave_address=ADD0|(ADD1<<1);
// Input/Output Ports initialization
// Port B initialization
// Func7=In Func6=In Func5=Out Func4=Out Func3=Out Func2=Out Func1=Out Func0=Out
// State7=T State6=T State5=0 State4=0 State3=0 State2=0 State1=0 State0=0
PORTB=0x00;
DDRB=0x3F;

// Port C initialization
// Func6=In Func5=In Func4=In Func3=In Func2=In Func1=In Func0=In
// State6=T State5=T State4=T State3=T State2=T State1=T State0=T
PORTC=0x00;
DDRC=0x00;

// Port D initialization
// Func7=In Func6=Out Func5=In Func4=Out Func3=Out Func2=In Func1=Out Func0=In
// State7=T State6=0 State5=T State4=0 State3=0 State2=T State1=0 State0=T
PORTD=0x00;
DDRD=0x5A;

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Fast PWM top=0xFF
// OC0A output: Disconnected
// OC0B output: Disconnected
TCCR0A=0x03;
TCCR0B=0x01;
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 1000.000 kHz
// Mode: Normal top=0xFFFF
// OC1A output: Discon.
// OC1B output: Discon.
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: On
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=0x00;
TCCR1B=0x02;
TCNT1H=0xB1;
TCNT1L=0xE0;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: 8000.000 kHz
// Mode: Fast PWM top=0xFF
// OC2A output: Disconnected
// OC2B output: Disconnected
ASSR=0x00;
TCCR2A=0x03;
TCCR2B=0x01;
TCNT2=0x00;
OCR2A=0x00;
OCR2B=0x00;

// External Interrupt(s) initialization
// INT0: On
// INT0 Mode: Rising Edge
// INT1: Off
// Interrupt on any change on pins PCINT0-7: Off
// Interrupt on any change on pins PCINT8-14: Off
// Interrupt on any change on pins PCINT16-23: On
EICRA=0x03;
EIMSK=0x01;
EIFR=0x01;
PCICR=0x04;
PCMSK2=0xA4;
PCIFR=0x04;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=0x00;

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=0x01;

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=0x00;

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: On
// USART Transmitter: Off
// USART0 Mode: Asynchronous
// USART Baud Rate: 9600
UCSR0A=0x00;
UCSR0B=0x90;
UCSR0C=0x06;
UBRR0H=0x00;
UBRR0L=0x33;
if (slave_address==test_driver)
{
	// USART initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART Receiver: On
	// USART Transmitter: On
	// USART0 Mode: Asynchronous
	// USART Baud Rate: 9600
	UCSR0A=0x00;
	UCSR0B=0x98;
	UCSR0C=0x06;
	UBRR0H=0x00;
	UBRR0L=0x33;
	
	PORTD=0x00;
}

// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
ADCSRB=0x00;
DIDR1=0x00;

// ADC initialization
// ADC disabled
ADCSRA=0x00;

// SPI initialization
// SPI disabled
SPCR=0x00;

// TWI initialization
// TWI disabled
TWCR=0x00;

// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/8k
// Watchdog Timer interrupt: Off
#pragma optsize-
asm("wdr");
WDTCSR=0x18;
WDTCSR=0x08;
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif
slave_address=ADD0|(ADD1<<1);
// Global enable interrupts
asm("sei");
DDRC|=(1<<PINC5);

    while(1)
    {
		while(ctrl)
		{
			M.RPM_last = M.RPM ; M.RPM=M.HSpeed;
			M.RPM = M.RPM_last + _FILTER_CONST *( M.RPM - M.RPM_last ) ;
			M.PWM = PD_CTRL ( (M.RPM_setpointB & 0x0ff)|((M.RPM_setpointA<<8) & 0xff00) , M.RPM , &M.Err , &M.d , &M.i ) ;
			
			if ( M.PWM<0)
			{
				Motor_Direction = 1 ;
				PWM = -M.PWM*2;
			}
							        
			else if (M.PWM >=0)
			{
				PWM = M.PWM * 2 ;
				Motor_Direction = 0 ;
		        
			}
			ctrl=0;
		}

		
		
		Motor_Update ( PWM , Motor_Direction ) ;
		
		
    }
}

void Motor_Update(uint8_t Speed, uint8_t Direction)
{
	 unsigned char Hall_State;
	 Hall_State = (HALL3<<2)|(HALL2<<1)|(HALL1);
	 LED_1  (HALL1);
	 LED_2  (HALL2);
	 LED_3  (HALL3);

	switch(Hall_State | ((Direction<<3)&0x8))
	{		
		case 1:
		case (6|0x8):
		M2p (OFF);
		M3p (OFF);
		M2n_PWM = 0; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm));
		M1n_PWM = 0; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm));
		M1p (ON);
		M3n_PWM = Speed; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm)) | (M3_TCCR_gm);
		break;
		
		case 3:
		case (4|0x8):
		M1p (OFF);
		M3p (OFF);
		M1n_PWM = 0; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm));
		M2n_PWM = 0; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm));
		M2p (ON);
		M3n_PWM = Speed; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm)) | (M3_TCCR_gm);
		break;
		
		case 2:
		case (5|0x8):
		M1p (OFF);
		M3p (OFF);
		M2n_PWM = 0; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm));
		M3n_PWM = 0; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm));
		M2p (ON);
		M1n_PWM = Speed; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm)) | (M1_TCCR_gm);
		break;
		
		case 6:
		case (1|0x8):
		M1p (OFF);
		M2p (OFF);
		M2n_PWM = 0; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm));
		M3n_PWM = 0; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm));
		M3p (ON);
		M1n_PWM = Speed; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm)) | (M1_TCCR_gm);
		break;
		
		case 4:
		case (3|0x8):
		M1p (OFF);
		M2p (OFF);
		M1n_PWM = 0; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm));
		M3n_PWM = 0; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm));
		M3p (ON);
		M2n_PWM = Speed; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm)) | (M2_TCCR_gm);
		break;
		
		case 5:
		case (2|0x8):
		M2p (OFF);
		M3p (OFF);
		M1n_PWM = 0; M1_TCCR = (M1_TCCR & (~M1_TCCR_gm));
		M3n_PWM = 0; M3_TCCR = (M3_TCCR & (~M3_TCCR_gm));
		M1p (ON);
		M2n_PWM = Speed; M2_TCCR = (M2_TCCR & (~M2_TCCR_gm)) | (M2_TCCR_gm);
		break;
		case 7:
		default:
		M1p (OFF);
		M2p (OFF);
		M3p (OFF);
		M1n_PWM = 0; M1_TCCR= (M1_TCCR & (~M1_TCCR_gm));
		M2n_PWM = 0; M2_TCCR= (M2_TCCR & (~M2_TCCR_gm));
		M3n_PWM = 0; M3_TCCR= (M3_TCCR & (~M3_TCCR_gm));
		break;

	}
}

inline int PD_CTRL (int Setpoint,int Feed_Back,int *PID_Err_past,int *d_past,float *i)
{
	

	int PID_Err=Setpoint-Feed_Back;

	
	int d=(PID_Err-(*PID_Err_past))*10 ;
	// d= (*d_past) +0.05*(d-(*d_past));

	d=(abs(d)<50)?0:d;

	d=(d>2400)?(0):d;
	d=(d<-2400)?(0):d;

	int p=PID_Err*kp;


	(*i)=(*i)+(ki*PID_Err)*.020;


	if ((*i)>80)
	(*i)=80;
	if ((*i)<-80)
	(*i)=-80;

	p=(p>127)?(127):p;
	p=(p<-127)?(-127):p;

	//PID_U_past=PID_U;
	int PID_U=p+(*i)+kd*d;//(0.5)*PID_Err2_M1+(1.5)*(PID_Err1_M1+PID_Err2_M1);//+(12.5)*(float)(PID_Err2_M1-PID_Err1_M1)/10.0; //kp=0.5  kd=9

	if(PID_U>127)
	PID_U=127;
	if( PID_U<-127)
	PID_U=-127;
	*PID_Err_past=PID_Err;
	*d_past=d;
	
	if(Setpoint==0 && abs(Feed_Back-Setpoint)<10)
	return 0;
	
	return PID_U;
	// direction =1;

}

ISR(USART_RX_vect)
{
char status,data;
status=UCSR0A;
data=UDR0;

if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
{
		Motor_Update ( M.PWM , Motor_Direction ) ;
	switch (pck_num)
	{
		case 0:
		if(data == '*')
		pck_num++;
		break;
		
		case 1:
		if(data == '~')
		pck_num++;
		else
		pck_num=0;
		break;
		
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		if(slave_address*2 == pck_num-2)
		tmp_rpmB=data;
		//tmp_setpoint = data & 0x0FF;
		if(slave_address*2 == pck_num-3)
		tmp_rpmA=data;
		//tmp_setpoint |= (data<<8) & 0xFF00;
		pck_num++;
		break;
		
		case 10:
		kp = (float)data/100.0; //Robot_D[RobotID].P
		pck_num++;
		break;
		
		case 11:
		ki = (float)data/100.0; //Robot_D[RobotID].I
		pck_num++;
		break;
		
		case 12:
		kd = (float)data/100.0; //Robot_D[RobotID].D
		pck_num++;
		break;
		
		case 13:
		if(data == '#' || data=='$')
		{
			asm("wdr");
			answer_permission=data;
			M.RPM_setpointB=tmp_rpmB;
			M.RPM_setpointA=tmp_rpmA;
			//RPM_setpoint = tmp_setpoint;
			//LED_1  (~READ_PIN(PORTB,0));
		}
		pck_num=0;
		break;
	}
}
}

ISR(INT0_vect)
{
	if ( HALL1 == 1 )
	{
		hall_dir = HALL2 ;
	}
}

ISR(PCINT2_vect)
{
	hall_flag ++ ;
	Motor_Update ( M.PWM , Motor_Direction ) ;

}


ISR(TIMER1_OVF_vect)
{
	
	//Motor_Update ( M.PWM , Motor_Direction ) ;
	TCNT1H=0xB1;
	TCNT1L=0xE0;

		T_20ms() ;
		ctrl=1;
		
				if(slave_address==0b11)
				{
					send_reply () ;
				}

	
}


void T_20ms(void)
{
	//LED_1  (~READ_PIN(PORTB,0));
	
	M.HSpeed = (float) ( hall_flag * 62.50 ) ;	//62.50=60s/(20ms*48)	48 = 3(number of hall sensors) * 8(number of pair poles) * 2
	M.HSpeed = ( hall_dir ) ? - M.HSpeed : M.HSpeed ;
	counter = 0 ;
	hall_flag = 0 ;

}

void send_reply(void)
{   
		
		USART_send ('*');
		
		send_buff = (((int)M.RPM) & 0x0ff) ;//HALL1;
		USART_send ( send_buff ) ;
		
		send_buff = ( ( ( (int) M.RPM ) & 0x0ff00 ) >>8 ) ;//HALL2;
		USART_send ( send_buff ) ;
		
		//send_buff = slave_address ;//HALL3;
		//USART_send ( send_buff ) ;
		//
		//send_buff = (((int)M.PWM) & 0x0ff) ;//HALL1;
		//USART_send ( send_buff ) ;
		
		//send_buff = ( ( ( (int) M.PWM ) & 0x0ff00 ) >>8 ) ;//HALL2;
		//USART_send ( send_buff ) ;
		
		//send_buff = ((int)M.RPM_setpointB) ;//HALL1;
		//USART_send ( send_buff ) ;
				//
		//send_buff =   ( (int) M.RPM_setpointA );//HALL2;
		//USART_send ( send_buff ) ;
		
		USART_send ('#') ;
}


//	usart functions

unsigned char USART_receive(void){
	
	while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
	
}

void USART_send( unsigned char data){
	
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0 = data;
	
}

void USART_putstring(char* StringPtr){
	
	while(*StringPtr != 0x00){
		USART_send(*StringPtr);
	StringPtr++;}
	
}