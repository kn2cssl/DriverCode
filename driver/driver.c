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
char slave_address=0;
void send_reply(void);
char data_test;
void USART_putstring(char* StringPtr);
void USART_send( unsigned char data);
unsigned char USART_receive(void);

//#define TCNT1 (int)(TCNT1L|(TCNT1H<<8))


signed long int master_setpoint=0x7fff;
signed char tmp_setpoint;

//float current_fd ;
float hall;

//char str[100];
float pwm;
float PID_Err2,PID_Err1,PID_U2,PID_U1,RPM,RPM_1,p,p_new,i,d,d_1,kp,ki,kd;

int enc;
unsigned char adc;

float adc_I,adc_I_1;
float disp_p,disp_i,disp_d,disp_u;

char Motor_Dir,direction;
float Motor_Dir1,Motor_Dir2;


uint16_t Time=0;

char data_w_index = 0;
char data_r_index = 0;
#define DATA_BUF_SIZE 50
char data_buf[DATA_BUF_SIZE];
unsigned char pck_num = 0;
bool data_rcv = 0;

int flg_ask=0;

float kp,ki,kd;
unsigned char kp_tmp,ki_tmp,kd_tmp,ask_tmp;

#define ADC_VREF_TYPE 0x20

uint8_t a,Motor_Direction;

signed long int motor_time,motor_time_tmp1,motor_time_tmp2,Motor_Speed,Motor_Speed_last;
char str[100];

#define ADC_VREF_TYPE 0x20

// Read the 8 most significant bits
// of the AD conversion result
unsigned char read_adc(unsigned char adc_input)
{
	ADMUX=adc_input | (ADC_VREF_TYPE & 0xff);
	// Delay needed for the stabilization of the ADC input voltage
	_delay_us(10);
	// Start the AD conversion
	ADCSRA|=0x40;
	// Wait for the AD conversion to complete
	while ((ADCSRA & 0x10)==0);
	ADCSRA|=0x10;
	return ADCH;
}
int main(void)
{  

// Input/Output Ports initialization
// Port B initialization
// Func7=In Func6=In Func5=Out Func4=Out Func3=Out Func2=Out Func1=Out Func0=Out
// State7=T State6=T State5=0 State4=0 State3=0 State2=0 State1=0 State0=0
PORTB=0x00;
DDRB=0x3F;


// Port C initialization
// Func6=In Func5=Out Func4=Out Func3=In Func2=In Func1=In Func0=In
// State6=T State5=0 State4=0 State3=T State2=T State1=T State0=T
PORTC=0x00;
DDRC=0x30;

// Port D initialization
// Func7=In Func6=Out Func5=In Func4=Out Func3=Out Func2=In Func1=Out Func0=In
// State7=T State6=0 State5=T State4=0 State3=0 State2=T State1=0 State0=T
PORTD=0x00;
DDRD=0x5A;

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 8000.000 kHz
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
// Clock value: Timer1 Stopped
// Mode: Normal top=0xFFFF
// OC1A output: Discon.
// OC1B output: Discon.
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=0x00;
TCCR1B=0x00;
TCNT1H=0x00;
TCNT1L=0x00;
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
// INT0: Off
// INT1: Off
// Interrupt on any change on pins PCINT0-7: Off
// Interrupt on any change on pins PCINT8-14: Off
// Interrupt on any change on pins PCINT16-23: On
EICRA=0x00;
EIMSK=0x00;
PCICR=0x04;
PCMSK2=0x04;
PCIFR=0x04;

// Timer/Counter 0 Interrupt(s) initialization
TIMSK0=0x00;

// Timer/Counter 1 Interrupt(s) initialization
TIMSK1=0x00;

// Timer/Counter 2 Interrupt(s) initialization
TIMSK2=0x00;

if (slave_address==0)
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
}
else
{
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
}




// Analog Comparator initialization
// Analog Comparator: Off
// Analog Comparator Input Capture by Timer/Counter 1: Off
ACSR=0x80;
ADCSRB=0x00;
DIDR1=0x00;

// ADC initialization
// ADC Clock frequency: 156.250 kHz
// ADC Voltage Reference: AREF pin
// ADC Auto Trigger Source: ADC Stopped
// Only the 8 most significant bits of
// the AD conversion result are used
// Digital input buffers on ADC0: On, ADC1: On, ADC2: On, ADC3: On
// ADC4: On, ADC5: On
DIDR0=0x00;
ADMUX=ADC_VREF_TYPE & 0xff;
ADCSRA=0x87;

// SPI initialization
// SPI disabled
SPCR=0x00;

// TWI initialization
// TWI disabled
TWCR=0x00;

// Watchdog Timer initialization
// Watchdog Timer Prescaler: OSC/8k
// Watchdog Timer interrupt: Off
//#pragma optsize-
//#asm("wdr")
//WDTCSR=0x1A;
//WDTCSR=0x0A;
//#ifdef _OPTIMIZE_SIZE_
//#pragma optsize+
//#endif
slave_address=ADD0|(ADD1<<1);
// Global enable interrupts
asm("sei");
DDRC|=(1<<PINC5);
    while(1)
    {
        adc_I_1=adc_I;
        adc= (char)(read_adc(7)&0x00ff);
        adc_I=adc*34.732;
        adc_I = adc_I_1 + /*((0.01/(f+0.01))*/ (0.02*(float)(adc_I-adc_I_1));
		
		// Place your code here
        if ( master_setpoint<0)
        {
	        Motor_Direction=1;
	        pwm = -master_setpoint*2;
	        //LED_2(~READ_PIN(PORTB,4));
        }
        else if (master_setpoint >=0)
        {
	        pwm = master_setpoint*2;
	        Motor_Direction= 0;
	         
        }
        //LED_3=~LED_3;
        if(TCNT0>100){
	        PORTC=PORTC&(~(1<<PINC5));
        }
        else
        {
	       PORTC=PORTC|(1<<PINC5);
        }

        pwm=80;
		//Motor_Direction= 0;
        //        M1p=0;
        //        M2p=0;
        //        M3p=0;
        //M1_TCCR = (M1_TCCR & (~M1_TCCR_gm)) | (M1_TCCR_gm)|0x03;
        //M2_TCCR = (M2_TCCR & (~M2_TCCR_gm)) | (M2_TCCR_gm)|0x03;
        // M3_TCCR = (M3_TCCR & (~M3_TCCR_gm)) | (M3_TCCR_gm)|0x03;
		
        Motor_Update(pwm,Motor_Direction);
		
		if(flg_ask==1)
		{
			
			send_reply();
			
			//UCSR0B=0x90;
			// DDRD.1=0;
		}
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



ISR(USART_RX_vect)
{
char status,data;
status=UCSR0A;
data=UDR0;

if ((status & (FRAMING_ERROR | PARITY_ERROR | DATA_OVERRUN))==0)
{
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
		if(slave_address == pck_num-2)
		tmp_setpoint = data & 0x0ff;
		pck_num++;
		break;
		case 6:
		ask_tmp=data;
		pck_num++;
		break;
		
		case 7:
		if(data == '#')
		{
			//LED_2  (~READ_PIN(PORTB,4));
			asm("wdr");
			//master_setpoint = tmp_setpoint;
			if (ask_tmp==slave_address)
			{   //LED_1  (~READ_PIN(PORTB,0));
				flg_ask=1;
			}
			
		}
		pck_num=0;
		break;
	}
}
}

ISR(PCINT2_vect)
{
	         Motor_Update(pwm,Motor_Direction);
	         //if(HALL1 == 1)
	         //{
	         //WRITE_PORT(PORTD,1, HALL2);}
			 //////////////////////////////////////for current sensing driver that sends direction on pin c.5
			 //if(slave_address==0 || slave_address==1)
			 //{
			 if(HALL1 == 1)
			 {
			 WRITE_PORT(PORTC,5, HALL2);}//}
}
void send_reply(void)
{   
	
		Motor_Update(pwm,Motor_Direction);
		USART_send('*');
		data_test=(((int)adc_I) & 0x0ff);//HALL1;
		Motor_Update(pwm,Motor_Direction);
		USART_send(data_test);
		data_test=((((int)adc_I)&0x0ff00)>>8);//HALL2;
		Motor_Update(pwm,Motor_Direction);
		USART_send(data_test);
		data_test=25;//slave_address;//HALL3;
		Motor_Update(pwm,Motor_Direction);
		USART_send(data_test);
		USART_send('#');
	     flg_ask=0;
		Motor_Update(pwm,Motor_Direction);
	//printf("*%c%c%c#",(((int)RPM) & 0x0ff),((((int)RPM)&0x0ff00)>>8),20);
	

	
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