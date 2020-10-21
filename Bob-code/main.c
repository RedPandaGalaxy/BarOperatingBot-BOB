/*
 * Bob-code.c
 *
 * Created: 09.12.2018 18:37:46
 * Author : MSI
 */
  #define F_CPU 16000000UL
  //necessary libraries
  #include <stdio.h>
  #include <avr/io.h>
  #include <util/delay.h>
  #include "i2cmaster.h"
  #include "usart.h"
  #include <avr/interrupt.h>
  #include "PCA9685_SPRO1MC.h"
  #include <math.h>
 
 #define CW 1
 #define CCW 2
 #define STOP 3
 #define M1 1
 #define M2 2
 #define left 0
 #define right 1
 
 //global variables ultrasonics
 //global variables ultrasonics
unsigned int wave_time_1=0,wave_time_2=0 ,wave_time_3=0,wave_time_4=0,wave_time_5=0,wave_time_6=0;//variables for time for ultrasonics (both ways)
unsigned char sensor_num=0;//sensor 1= sensor_num=1 So front sensor is 1 , left front is 2 , right front 3, left side 4, right side 5, back is 6
volatile unsigned int pos_wheel_1=0,pos_wheel_2=0; //90 teeth is one full rotation
volatile unsigned int count_wheel_1=0, count_wheel_2=0;
volatile unsigned int flag_start=0,flag=0,flag_stop=0;//flag variables to indicate wheether interrupt for buttons was executed

float widthWheel=0.25; // width between the drive wheels in meters
float diameterWheel=0.17; //wheel diameter in meters
float avgSpeed = 1.4; //estimated traveling speed for calculating stuff
int rotaryHole=90; //number of holes for optical sensor
unsigned int motor_speed_2=0; //initial speed of motor 1
unsigned int motor_speed_1=0; //initial speed of motor 2
float theta=0; //angle to turn
float distance=0;//distance to delivery in straight line
char obstacle_counter=0;char flag_crazy=0;

float bob_x=0, bob_y=0, bob_t=3.14/4; //initial conditions
//float t1_x=180,t1_y=180,t2_x=100,t2_y=30,t3_x=30,t3_y=140; //table coordinates in cm but should be in m (let's keep the same unit)
float t0_x=1,t0_y=1; //in meters

 unsigned int table_of_measurements[5]; //for storing 5 measurements for ultrasonic sensor
 unsigned int table_of_medians[5]; // sotrs median for each ultrasonic ; note that for sensor_num=1 the corresponding value is: table_of_medians[sensor_num-1] which is [0]
 unsigned int table_holes_1_greater[5];
 unsigned int table_holes_2_greter[5];
 unsigned int median_wheel1,median_wheel2;
 //in the end it would be better to change the first sensor (front one) to 0 but then the changes in functions/interrupts will be needed
 //so for the time being let it stay as it is ;)

void distanceToDelivery(float x, float y, float dx, float dy, float *d, float *t); 
float timeestimate(float distance);
unsigned int turn (float t,float pt);//need angle we want to turn and the current location of BOB
void ultrasonic_trigger(char which_sensor);
void set_motor_state(char which_motor, char state);
void set_pins(); //defining the I/O pins and Interrupts
void sort_and_measure(char which_sensor);
void real_turn(char direction);
void bob_location();
void speed_check();
float degrees_to_radians(int angle);

int main(void)
{
	uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the uart
	i2c_init();
	set_pins();
	flag_start=0;
	flag_stop=0; pos_wheel_1=0;pos_wheel_2=0;
	sensor_num=1;
	motor_speed_1=2250+400;
	motor_speed_2=3000+400;
	while(1)//the controller runs this forever
 	{		//motor 1 is on the right
		 //motor 2 is on the left from the front	
		while(1)
		{
			flag_start=0;
			if (flag_start==1)
			{
				flag_start=0;//clear the start flag variable
				flag_crazy=0;
				flag_stop=0;
				pos_wheel_1=0; //we clear holes counters for optical sensors
				pos_wheel_2=0;
				printf("Stops at flag_start loop" );
				break;//break the while loop
			}
		}
		while(1)
		{
				
				if(flag_crazy==1)
					{
						flag_crazy=0;
						break;
					}
				if (flag_stop==1)
				{
					flag_stop=0;
					break;
				}
				
				
				//start moving "forward"
								printf("start loop \n");
								PCA9685_set_pwm(M1_PWM,0,motor_speed_1); set_motor_state(M1,CCW);
								PCA9685_set_pwm(M2_PWM,0,motor_speed_2); set_motor_state(M2,CCW);
								_delay_ms(5);
								//checking for obstacle
								for (int i=1;i<4;i++)
								{
									sort_and_measure(i);
									printf("median: %d \n",table_of_medians[sensor_num-1]);
									if((table_of_medians[i-1]<150) && (table_of_medians[i-1]>30))
									{
										//stop BOB
										set_motor_state(M1,STOP); PCA9685_set_pwm(M1_PWM,0,0);
										set_motor_state(M2,STOP); PCA9685_set_pwm(M2_PWM,0,0);
										_delay_ms(1000);
										
										theta=degrees_to_radians(40);
										//turn 40 degrees right
										printf("turn right \n");
										real_turn(right);
										_delay_ms(1000);
										obstacle_counter++;
										break;
									}
								}
								if (obstacle_counter==4)
								{
									for (int i=0;i<2;i++)
									{
										theta=degrees_to_radians(360);
										real_turn(right);
										obstacle_counter=0;
									}
									PCA9685_set_pwm(M1_PWM,0,0); set_motor_state(M1,STOP);
									PCA9685_set_pwm(M2_PWM,0,0); set_motor_state(M2,STOP);
									flag_crazy=1;
									}
								}
		}
	}
void set_pins()//defines the pins-input/output ; sets the interrupts
{
	//motor PINS: A0 A1 A2 A3 (but all set high just in case)
		DDRA = 0x11111111; //output - motor input pins M2; //output - motor input pins M1
	
// 	optical sensors
 		DDRD &= ~(1<<PIND2); //input - wheel 1
 		DDRD &= ~(1<<PIND3);// input - wheel 2
 		EICRA |= (1<<ISC21) | (1<<ISC20); // set INT2 to trigger on rising edge - wheel 1
 		EIMSK |= (1 << INT2); // Turns on interrupt for INT0
 		EICRA |= (1<<ISC31) | (1<<ISC30); // set INT3 to trigger on rising edge - wheel 2
 		EIMSK |= (1 << INT3); // Turns on interrupt for INT3
// 	buttons
		//stop button
			DDRE &= ~(1<<PINE4); //STOP button input
			PORTE |= (1<<PINE4); //enable pull-up
			EICRB |= (1<<ISC40); // set INT4 to trigger on any edge
			EIMSK |= (1 << INT4); // Turns on interrupt for INT4
		//start button
		DDRE &= ~(1<<PINE5); //START button input
		PORTE |= (1<<PINE5); //enable pull-up
		EICRB |= (1<<ISC50); // set INT5 to trigger on any edge
		EIMSK |= (1 << INT5); // Turns on interrupt for INT0
	
// 	ultrasonics
 				DDRL = (1<<PINL1); //trigger pin L1
				DDRB &= ~(1<<PINB0); PORTB |= (1<<PINB0); //sensor 1 - in front PCIN0
				DDRB &= ~(1<<PINB1); PORTB |= (1<<PINB1); //sensor 2 - front left PCINT1
				DDRB &= ~(1<<PINB2); PORTB |= (1<<PINB2); //sensor 3 - front right PCIN2
				DDRB &= ~(1<<PINB3); PORTB |= (1<<PINB3); // sensor 4 - side left PCINT3
				DDRB &= ~(1<<PINB4); PORTB |= (1<<PINB4); //sensor 5 -side right PCINT4
				DDRB &= ~(1<<PINB5); PORTB |= (1<<PINB5); //sensor 6 - back PCINT5
				PCICR |= (1 << PCIE0); // set PCIE0 to enable the group for PCINT8...14
				PCMSK0 |= (1<< PCINT0) | (1<<PCINT1) | (1<<PCINT2) | (1<<PCINT3) | (1<<PCINT4) | (1<<PCINT5);
		
 		sei(); // turn on interrupts for all the sensors!!!
		 PCA9685_init_pwm(PWM_FREQUENCY_200);//we need to say at what frequency the PWM board should run at
		
}
void distanceToDelivery(float x, float y, float dx, float dy, float *d, float *t) //distance to delivery point
{
	//x,y- destination coordinates (cm), dx,dy- bob location,(cm) *d-distance bob needs to travel (cm), 
//*t-angle theta we need to cover for 0 degrees (current angle will be included in another function)
	float relativeX, relativeY;
	
	relativeX = dx - x;
	relativeY = dy - y;
	*d = sqrt((relativeX * relativeX) + (relativeY * relativeY));
	if (relativeX==0 || relativeY==0) //if it is 90n angle
	{
		*t = asin(relativeY / *d);
	}
	else                     // angles are in radians
	{	
		*t = atan(relativeY / relativeX);
	}
	if ((relativeX < 0) && (relativeY < 0)) // quadrant 3
	{
		*t = *t + 3.141592;
	}
	if  ((relativeY < 0) && (relativeX > 0)) // quadrant 4
	{
		*t = *t + 6.283185;
	}
	if  ((relativeY > 0) && (relativeX < 0)) // quadrant 2
	{
		*t = *t + 3.141592;
	}	
}
float degrees_to_radians(int angle)
{
	return(angle * (M_PI/ 180.0));
}
unsigned int turn (float t,float pt)
{ 
//t-angle to turn (radians) , pt-current angle of bob front (radians), x-turn direction)
//It is necessary to calculate t before using this function
//function distanceToDelivery should be used before this function
	unsigned int turnAmt; //number of teeth to make a desired turn
	//	turnAmt = ((turnAmt * (widthWheel / 2) )/(diameterWheel * 3.141592)) * rotaryHole; //when turning in positive direction
	turnAmt= ( (t*widthWheel)/(2*M_PI*diameterWheel)  )*90;
	return turnAmt; //returns the necessary number of teeth to make a desired turn
}
void real_turn(char direction)
{
	//sets the motor to spin in the right direction
	pos_wheel_1=0;pos_wheel_2=0;
	if (direction==left)
	{
		set_motor_state(M2,CCW);
		set_motor_state(M1,CW);
	}
	else //right
	{
		set_motor_state(M2,CW);
		set_motor_state(M1,CCW);
	}
	unsigned int teeth_count=2*turn(theta,bob_t);
	PCA9685_set_pwm(M1_PWM,0,3000);
	PCA9685_set_pwm(M2_PWM,0,2900);
	while(1)
	{
// 		printf("pos wheel 1: %d \n",pos_wheel_1);
// 		printf("pos wheel 2: %d \n",pos_wheel_2);
		if ((pos_wheel_1>=teeth_count) || (pos_wheel_2>=teeth_count))
		{
			set_motor_state(M1,STOP);
			set_motor_state(M2,STOP);
			PCA9685_set_pwm(M2_PWM,0,0);
			PCA9685_set_pwm(M1_PWM,0,0);
			bob_t=theta;
			printf("turn is done!\n");
			break;
		}
	}
	pos_wheel_1=0;
	pos_wheel_2=0;
}
void ultrasonic_trigger(char which_sensor)
{
	//all ultrasonics are connected to one trigger pin
	//trigger
	//We disable all echoes so that we can focus on one only
	PCMSK0 &= ~(1<< PCINT0);
	PCMSK0 &= ~(1<< PCINT1);
	PCMSK0 &= ~(1<< PCINT2);
	PCMSK0 &= ~(1<< PCINT3);
	PCMSK0 &= ~(1<< PCINT4);
	PCMSK0 &= ~(1<< PCINT5);
	switch(which_sensor)
	{
		case 1:
		{
			PCMSK0 |= (1<<PCINT0);
			break;
		}
		case 2:
		{
			PCMSK0 |= (1<<PCINT1);
			break;
		}
		case 3:
		{
			PCMSK0 |= (1<<PCINT2);
			break;
		}
		case 4:
		{
			PCMSK0 |= (1<<PCINT3);
			break;
		}
		case 5:
		{
			PCMSK0 |= (1<<PCINT4);
			break;
		}
		case 6:
		{
			PCMSK0 |= (1<<PCINT5);
			break;
		}
	}
	//actual trigger
	PORTL |= (1<<PINL1);
	_delay_us(15);
	PORTL &= ~(1<<PINL1);
	
}
void sort_and_measure(char which_sensor)
{
	for (int i=0;i<5;i++)//making 5 measurement for one sensor
	{
		ultrasonic_trigger(which_sensor);
		switch (which_sensor)//saving value in the array
		{
			case 1:
			{
				table_of_measurements[i]=wave_time_1;
				printf("%d   ",wave_time_1);
				break;
			}
			case 2:
			{
				table_of_measurements[i]=wave_time_2;
				//printf("%d   ", wave_time_2);
				break;
			}
			case 3:
			{
				table_of_measurements[i]=wave_time_3;
				//printf("%d   ",wave_time_3);
				break;
			}
			case 4:
			{
				table_of_measurements[i]=wave_time_4;
				//printf("%d   ",wave_time_4);
				break;
			}
			case 5:
			{
				table_of_measurements[i]=wave_time_5;
				//printf("%d   ",wave_time_5);
				break;
			}
			case 6:
			{
				table_of_measurements[i]=wave_time_6;
				//printf("%d   ",wave_time_6);
				break;
			}
		}
	}
	unsigned int temporary_value;
	for (int j=1;j<5;j++)//sorting the measurements in the array so that we can get the median
	{
		for (int i=0;i<5;i++)
		{
			if (table_of_measurements[i]<=table_of_measurements[j])
			{
				temporary_value=table_of_measurements[i];
				table_of_measurements[i]=table_of_measurements[j];
				table_of_measurements[j]=temporary_value;
			}
		}
	}
	table_of_medians[which_sensor-1]=table_of_measurements[2];//saving the medain for n-sensor
}
void set_motor_state(char which_motor, char state)
{
	switch (state)
	{
		case CW:
		{switch(which_motor)
			{
				case M2:
				PORTA |= (1<<PINA1);
				PORTA &=  ~(1<<PINA0);
				break;
				case M1:
				PORTA |= (1<<PINA2);
				PORTA &=  ~(1<<PINA3);
			}
			break;
		}
		case CCW:
		{
			switch(which_motor)
			{
				case M2:
				PORTA |= (1<<PINA0);
				PORTA &=  ~(1<<PINA1);
				break;
				case M1:
				PORTA |= (1<<PINA3);
				PORTA &=  ~(1<<PINA2);
			}
			break;
		}
		case STOP:
		{
			switch(which_motor)
		{
			case M2:
			PORTA &= ~(1<<PINA0);
			PORTA &=  ~(1<<PINA1);
			break;
			case M1:
			PORTA &= ~(1<<PINA2);
			PORTA &= ~(1<<PINA3);
		}
		break;
		}
	}
}
void bob_location()
{
	bob_x=pos_wheel_1/90*diameterWheel*3.141592*cos(bob_t);
	bob_y=pos_wheel_1/90*diameterWheel*3.141592*sin(bob_t);
}
void speed_check()
{
	unsigned ratio;
	unsigned int uplimit1=3300,uplimit2=3300,downlimit1=1800,downlimit2=1800;
	//If M1 spins faster than M2
	if (pos_wheel_1>pos_wheel_2)//usally 30-100;
	{
		ratio=(pos_wheel_1-pos_wheel_2)*2;
		if (ratio<3)
			ratio=0;
		motor_speed_1=motor_speed_1-ratio*10;
		motor_speed_2=motor_speed_2+ratio;
	}
	else
		if (pos_wheel_2>pos_wheel_1)
		{
			ratio=pos_wheel_2-pos_wheel_1;
			if (ratio<3)
				ratio=0;
			motor_speed_2=motor_speed_2-ratio*10;
			motor_speed_1=motor_speed_1+ratio;
		}
	if (motor_speed_1>uplimit1)
		motor_speed_1=uplimit1;
		
	if (motor_speed_2>uplimit2)
		motor_speed_2=uplimit2;
		
	if (motor_speed_1<downlimit1)
		motor_speed_1=downlimit1;
		
	if (motor_speed_2<downlimit2)
		motor_speed_2=downlimit2;
		
	PCA9685_set_pwm(M1_PWM,0,motor_speed_1);
	PCA9685_set_pwm(M2_PWM,0,motor_speed_2);		
}
ISR (INT2_vect) // count pos_wheel_2
{
	pos_wheel_2++;
	count_wheel_2++;
}
ISR (INT3_vect) // count pos_wheel_1
{
	pos_wheel_1++;
	count_wheel_1++;
}
 ISR (PCINT0_vect) // for ultrasonics sensors
{
	if (sensor_num==1)
	{
		if( (PINB & (1 << PINB0)))
		{
			// LOW to HIGH pin change 
			TCCR1B |= 1<<CS12 ;  //when 1st change on the pin appears - start the counter //prescaler 256
		}
		else
		{
			// HIGH to LOW pin change 
			wave_time_1=TCNT1; //save time
			TCCR1B = 0; //stop counter
			TCNT1 = 0; //reset stored counter value
		}
		
	}
	if (sensor_num==2)
	{
		if( (PINB & (1 << PINB1)))
		{
			// LOW to HIGH pin change 
			TCCR1B |= 1<<CS12 ;  //when 1st change on the pin appears - start the counter //prescaler 256
		}
		else
		{
			// HIGH to LOW pin change
			wave_time_2=TCNT1; //save time
			TCCR1B = 0; //stop counter
			TCNT1 = 0; //reset stored counter value
		}
	}
	if (sensor_num==3)
	{
		if( (PINB & (1 << PINB2)))
		{
			// LOW to HIGH pin change
			TCCR1B |= 1<<CS12 ;  //when 1st change on the pin appears - start the counter //prescaler 256
		}
		else
		{
			// HIGH to LOW pin change
			wave_time_3=TCNT1; //save time
			TCCR1B = 0; //stop counter
			TCNT1 = 0; //reset stored counter value
		}
	}
	if (sensor_num==4)
	{
		if( (PINB & (1 << PINB3)))
		{
			// LOW to HIGH pin change
			TCCR1B |= 1<<CS12 ;  //when 1st change on the pin appears - start the counter //prescaler 256
		}
		else
		{
			// HIGH to LOW pin change 
			wave_time_4=TCNT1; //save time
			TCCR1B = 0; //stop counter
			TCNT1 = 0; //reset stored counter value
		}
	}
	if (sensor_num==5)
	{
		if( (PINB & (1 << PINB4)))
		{
			// LOW to HIGH pin change
			TCCR1B |= 1<<CS12 ;  //when 1st change on the pin appears - start the counter //prescaler 256
		}
		else
		{
			// HIGH to LOW pin change
			wave_time_5=TCNT1; //save time
			TCCR1B = 0; //stop counter
			TCNT1 = 0; //reset stored counter value
		}
	}
	if (sensor_num==6)
	{
		if( (PINB & (1 << PINB5)))
		{
			// LOW to HIGH pin change
			TCCR1B |= 1<<CS12 ;  //when 1st change on the pin appears - start the counter //prescaler 256
		}
		else
		{
			// HIGH to LOW pin change 
			wave_time_6=TCNT1; //save time
			TCCR1B = 0; //stop counter
			TCNT1 = 0; //reset stored counter value
		}
	}
 }
 ISR (INT4_vect) //stop button
{
	set_motor_state(M1,STOP);
	set_motor_state(M2,STOP);
	PCA9685_set_pwm(M2_PWM,0,0);
	PCA9685_set_pwm(M1_PWM,0,0);
	flag_stop=1;
}//stop button
ISR (INT5_vect)//start button
{
	flag_start=1;
}//start button