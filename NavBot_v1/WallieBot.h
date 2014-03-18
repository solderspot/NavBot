// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder

// This is the code specific to my Pololu Classic chassis bot setup
// https://github.com/pololu/qik-arduino
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <PololuQik.h>




//----------------------------------------
// Bot config
//----------------------------------------

// motors
#define SWAP_MOTORS             0
#define RMOTOR_DIR              -1L    // -1 to reverse, 1 for normal
#define LMOTOR_DIR              -1L     // -1 to reverse, 1 for normal

// Navigator defines
#define WHEEL_BASE              nvMM(93.4)
#define WHEEL_DIAMETER          nvMM(42.6)
#define TICKS_PER_REV           48
#define FORWARD_HEADING_ADJUST  0.0011f
#define LEFT_HEADING_ADJUST     0.042f
#define RIGHT_HEADING_ADJUST    0.0f
#define FORWARD_DIST_ADJUST     0.0f
#define RIGHT_DIST_ADJUST       0.0f
// Pilot heading PID controller coefficients
#define Kp_HEADINGS             5.0f
#define Ki_HEADINGS             0.0f
#define Kd_HEADINGS             0.0f

// Pilot speed PID controller coefficients
#define Kp_SPEED                0.5f
#define Ki_SPEED                0.0f
#define Kd_SPEED                0.0f

// Pilot turn PID controller coefficients
#define Kp_TURN                 0.5f
#define Ki_TURN                 0.0f
#define Kd_TURN                 0.0f

#define MAX_SPEED               nvMM(100)

//----------------------------------------
// Pin Assignments
//----------------------------------------

#define BUTTON_PIN   A2 


//----------------------------------------
// Data and Classes
//----------------------------------------

char lftApin    = 3;
char lftBpin    = 4;
char rhtApin    = 6;
char rhtBpin    = 5;
char rxPin      = 10;
char txPin      = 9;
char resetPin   = 11;

PololuQik2s9v1 qik(rxPin, txPin, resetPin);

void setup_encoder();

//----------------------------------------
//
//----------------------------------------

void init_bot()
{
	// do all bot initialization here

	qik.init();
	#if MOTOR_INFO
	Serial.print("Qik Firmware version: ");
	Serial.write(qik.getFirmwareVersion());
	Serial.println();
	#endif

  // set up encoder
  setup_encoder();

  pilot.SetMinServiceInterval( nvMS(50));
  pilot.SetMinTurnSpeed( nvDEGREES(30) );
  pilot.SetTargetRadius( nvMM(20));

}

//----------------------------------------
// Motor handler
//----------------------------------------

void motor_handler( Pilot *pilot, int16_t lmotor, int16_t rmotor)
{
 
  // LMotorGain is used to simulate the
  // left motor as being more or less powerful
  // than the right. We use this to test the PID
  // controler. You can set LMOTOR_GAIN and
  // see how well PID can correct the motor
  // imbalance

  int16_t lspeed = ((lmotor*127L)/1024L)*LMOTOR_DIR;
  int16_t rspeed = ((rmotor*127L)/1024L)*RMOTOR_DIR;
  
  #if SWAP_MOTORS
	qik.setM0Speed(rspeed);
	qik.setM1Speed(lspeed);
  #else
	qik.setM0Speed(lspeed);
	qik.setM1Speed(rspeed);
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
//
//----------------------------------------

volatile int16_t en_lft_ticks = 0;
volatile int16_t en_rht_ticks = 0;
volatile bool en_error = false;
char en_lApin;
char en_lBpin;
char en_rApin;
char en_rBpin;

//----------------------------------------
//
//----------------------------------------

void en_init_pin( char *pin, char value)
{
  *pin = value;
  pinMode(value, INPUT);
  digitalWrite( value, HIGH);
}

//----------------------------------------
//
//----------------------------------------

void setup_encoder( )
{
  en_init_pin( &en_lApin, lftApin);
  en_init_pin( &en_lBpin, lftBpin);
  en_init_pin( &en_rApin, rhtApin);
  en_init_pin( &en_rBpin, rhtBpin);

  cli();
  // set timer2 interrupt at 1000 Hz
	// we are assuming a clk speed of 16MHz
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2  = 0;
  OCR2A = 249;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);   
  TIMSK2 |= (1 << OCIE2A);
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

ISR(TIMER2_COMPA_vect)
{
  // this routine gets called once every 1 millisecond

  static char lastLA = 0;
  static char lastLB = 0;
  static char lastRA = 0;
  static char lastRB = 0;

  en_process(en_lApin, en_lBpin, &lastLA, &lastLB, &en_lft_ticks);
  en_process(en_rApin, en_rBpin, &lastRA, &lastRB, &en_rht_ticks);
}


