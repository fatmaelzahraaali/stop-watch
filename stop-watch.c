/*
 * stop-watch.c
 *
 *  Created on: Sep 17, 2022
 *      Author: fatima
 */
#include<avr/interrupt.h>
#include<avr/io.h>
#include<util/delay.h>
unsigned char g_tick=0;
unsigned char sec1,sec2,min1,min2,hour1,hour2=0;

void timer1_ctc(void){
	//Non-PWM FOC1A=1
	TCCR1A=(1<<FOC1A)|(1<<FOC1B);
	 /*compare mode
	 *WGM12=1 ,WGM10=0, WGM11=0 ,WGM13=0  (mode 4)
	 * prescalar 1024 CS10=1 ,CS11=0, CS12=1
	 */
	TCCR1B=(1<<WGM12) | (1<<CS10)| (1<<CS12);
	TCNT1=0;    //initial value
	OCR1A= 1000; //COMPARE value 1000 ms=1sec
	sei();   //enable I-bit
	TIMSK=(1<<OCIE1A);    //interrupt enable
}
ISR(TIMER1_COMPA_vect)
{
      g_tick++;
      sec1++;
}
void int0_reset(void){
	DDRD &=~(1<<PD2);   //PD.2 input pin
	PORTD |=(1<<PD2);    //activate internal pull-up
	//INT0-falling edge isc01=1
	MCUCR |=(1<<ISC01);
	GICR |=(1<<INT0);    //INT0 enable
	sei();   //enable I-bit

}
ISR(INT0_vect)
{ //reset
	TCNT1=0;    //initial value
	sec1=0;
	sec2=0;
	min1=0;
	min2=0;
	hour1=0;
	hour2=0;


}
void int1_pause(void)
{
	DDRD &=~(1<<PD3);   //PD.4 	input pin
	//INT1-raising edge isc11=1, isc10=1
	MCUCR|=(1<<ISC10) |(1<<ISC11);
	GICR|=(1<<INT1);    //INT1 enable
	sei();   //enable I-bit

}
ISR(INT1_vect)
{	//timer stopped - no clocks
    TCCR1A=0;
	TCCR1B=0;
}
void int2_resume(void){
	DDRB &=~(1<<PB2);  //PB.2 input pin
	PORTB |=(1<<PB2);    //activate internal pull-up
	//INT2-falling edge isc2=0
	MCUCSR &=~(1<<ISC2);
	GICR |=(1<<INT2);    //INT2 enable
	sei();   //enable I-bit
}
ISR(INT2_vect)
{
      //timer resumed
	 /*compare mode
		 *WGM12=1 ,WGM10=0, WGM11=0 ,WGM13=0  (mode 4)
		 * prescalar 1024 CS10=1 ,CS11=0, CS12=1
		 */
	//TIMSK=(1<<OCIE1A);    //interrupt enable
	TCCR1B=(1<<WGM12) | (1<<CS10)| (1<<CS12);

}

void display(void ){
	         PORTA = (PORTA & 0xc0) |0x01;   //PA0 enabled sec1 activated
             PORTC = (PORTC & 0xF0) | (sec1 & 0x0F) ;
		      _delay_ms(4);
		     PORTA = (PORTA & 0xc0) | 0X02;   //PA1 enabled sec2 activated
		     PORTC = (PORTC & 0xF0) | (sec2 & 0x0F);
		   	 _delay_ms(4);
		   	 PORTA = (PORTA & 0xc0) | 0X04;   //PA2 enabled min1 activated
		     PORTC = (PORTC & 0xF0) | (min1 & 0x0F);
		   	 _delay_ms(4);
		   	 PORTA = (PORTA & 0xc0) | 0X08;   //PA3 enabled min2 activated
	    	 PORTC = (PORTC & 0xF0) | (min2 & 0x0F) ;
		   	 _delay_ms(4);
		   	 PORTA = (PORTA & 0xc0) | 0X10;   //PA4 enabled hour1 activated
		     PORTC = (PORTC & 0xF0) | (hour1 & 0x0F);
		     _delay_ms(4);
		     PORTA = (PORTA & 0xc0) | 0X20;   //PA5 enabled hour2 activated
		     PORTC = (PORTC & 0xF0) | (hour2 & 0x0F) ;
		     _delay_ms(4);
}
void time_check(void)
{
	   if(g_tick==10)
	   {       sec1=0;
		       g_tick=0;
		       sec2++;
		      if(sec2==6)
		       {
		           sec2=0;
		           min1++;
		       }
		      if(min1==10)
   	          {
		       	  min1=0;
		       	  min2++;
		       	  if(min2==6)
		       	  {
		       		  min2=0;
		       		  hour1++;
		       	  }
		      }


	         if(hour1==10)
	         {
	        	  hour1=0;
	        	  hour2++;
	         }
	         if(hour2==10 && hour1==10 && min2==6 && min1==10 && sec2==6 && sec1==10)
	        {
	        	 //reset
	        	 	TCNT1=0;    //initial value
	        	 	sec1=0;
	        	 	sec2=0;
	        	 	min1=0;
	        	 	min2=0;
	        	 	hour1=0;
	        	 	hour2=0;
	        }
	   }
}
int main(){

	DDRA|=0X3F;  //first six pins in portA - output pins - enables six 7-segments
	DDRC|=0X0F;   //first four pins in portC - output pins - connected to the decoder
	PORTC&=0XF0;   //initialize pc0 ,pc1,pc2,pc3=0
	int0_reset ();
	int1_pause();
	int2_resume();
	timer1_ctc();
   while(1){
	   display();
	   time_check();

   }

}

