// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder

// This is the code specific to my Zumo bot setup

// Zumo motor lib: https://github.com/pololu/zumo-shield
                                                                                                             
#include <ZumoMotors.h>


//----------------------------------------
// Bot config
//----------------------------------------

// motors
#define SWAP_MOTORS             1
#define RMOTOR_DIR              1L    // -1 to reverse, 1 for normal
#define LMOTOR_DIR              1L     // -1 to reverse, 1 for normal

// Navigator defines
#define WHEELBASE              nvMM(112.95)
#define WHEEL_DIAMETER          nvMM(38.55)
#define TICKS_PER_REV           1204
#define WHEEL_RL_SCALER         1.0f // Ed
#define WHEELBASE_SCALER        1.0f // Eb
#define DISTANCE_SCALER         1.0f // Es
 
// Pilot heading PID controller coefficients
#define Kp_HEADINGS             5.0f
#define Ki_HEADINGS             0.1f
#define Kd_HEADINGS             0.0f

// Pilot speed PID controller coefficients
#define Kp_SPEED                0.5f
#define Ki_SPEED                0.0f
#define Kd_SPEED                0.0f

// Pilot wheel PID controller coefficients
#define Kp_WHEEL                0.8f
#define Ki_WHEEL                0.1f
#define Kd_WHEEL                0.0f

#define MAX_SPEED               nvMM(100)

//----------------------------------------
// Pin Assignments
//----------------------------------------

#define BUTTON_PIN    12


//----------------------------------------
// Data and Classes
//----------------------------------------

ZumoMotors    motors;

void setup_encoder();
//----------------------------------------
//
//----------------------------------------

void init_bot()
{
  // do all bot initialization here

  // set up encoder
  setup_encoder();


}

//----------------------------------------
// Motor handler
//----------------------------------------

void motor_handler( Pilot *pilot, int16_t lmotor, int16_t rmotor)
{
 
  int16_t lspeed = ((lmotor*400L)/1024L)*LMOTOR_DIR;
  int16_t rspeed = ((rmotor*400L)/1024L)*RMOTOR_DIR;
  
  #if SWAP_MOTORS
    motors.setLeftSpeed( rspeed );
    motors.setRightSpeed( lspeed );
  #else
    motors.setRightSpeed( rspeed );
    motors.setLeftSpeed( lspeed );
  #endif

  #if MOTOR_INFO || TEST_MOTORS
  Serial.print(F("Motors: Left = "));
  Serial.print(lspeed);
  Serial.print(F(" ("));
  Serial.print(lmotor);
  Serial.print(F(")"));
  Serial.print(F(" - Right = "));
  Serial.print(rspeed);
  Serial.print(F(" ("));
  Serial.print(rmotor);
  Serial.println(F(")"));
  #endif
 
}


//----------------------------------------
// Encoder Code
//----------------------------------------

volatile int16_t en_lft_ticks = 0;
volatile int16_t en_rht_ticks = 0;
volatile bool en_error = false;
 
// hard coding the pins to port D  
char en_rApin = 2;
char en_rBpin = 3;
char en_lApin = 4;
char en_lBpin = 5; 

char en_lastLA;
char en_lastLB;
char en_lastRA;
char en_lastRB;

//----------------------------------------
//
//----------------------------------------

void setup_encoder()
{
  cli();
      PCMSK2 = ((1<<PCINT21)|(1<<PCINT20)|(1<<PCINT19)|(1<<PCINT18));
      PCICR = (1<<PCIE2);
      PCIFR = 0xFF;
  sei();
}

//----------------------------------------
// Ticks handler
//----------------------------------------
 
bool ticks_handler( Pilot *pilot, int16_t *lft, int16_t *rht)
{
  cli();
  *lft = en_lft_ticks;
  *rht = en_rht_ticks;
  en_lft_ticks = en_rht_ticks = 0;
  char error = en_error;
  en_error = false;
  sei();

  return !error;
}

//----------------------------------------
//
//----------------------------------------

void clear_ticks()
{
  cli();
  en_lft_ticks = en_rht_ticks = 0;
  en_error = false;
  sei();
}

//----------------------------------------
//
//----------------------------------------

void en_process( char Apin, char Bpin, char *lastA, char *lastB, volatile int16_t *ticks )
{
  char A = (digitalRead( Apin) == HIGH) ? 1 : 0;
  char B = (digitalRead( Bpin) == HIGH) ? 1 : 0;
  char lA = *lastA;
  char lB = *lastB;
  char dA = A!=lA;
  char dB = B!=lB;

  if( dA && dB )
  {
    // both should not change at the same time
    en_error = true;
  }
  else if ( dA || dB )
  {
    if (A^lB) 
    {
      *ticks += 1;
    }
    else if(B^lA)
    {
      *ticks -= 1;
    }
  }
  *lastA = A; 
  *lastB = B;
}

//----------------------------------------
//
//----------------------------------------

ISR(PCINT2_vect)
{
    en_process(en_lApin, en_lBpin, &en_lastLA, &en_lastLB, &en_lft_ticks);
    en_process(en_rApin, en_rBpin, &en_lastRA, &en_lastRB, &en_rht_ticks);
}


