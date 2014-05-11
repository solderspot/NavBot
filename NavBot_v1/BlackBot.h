// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder

// This is the code specific to black bot bot setup

//#include <Wire.h>
//#include <Adafruit_MotorShield.h>

#define USE_MOTORS    1

//----------------------------------------
// Bot config
//----------------------------------------

// motors
#define SWAP_MOTORS             1
#define RMOTOR_DIR              -1L    // -1 to reverse, 1 for normal
#define LMOTOR_DIR              -1L     // -1 to reverse, 1 for normal

// Navigator defines
#define WHEELBASE               nvMM(78.0)
#define WHEEL_DIAMETER          nvMM(32.0)
#define TICKS_PER_REV           909
#define WHEEL_RL_SCALER         (1.002463028f*1.000276208f)   // Ed
#define WHEELBASE_SCALER        (0.9918532967f*0.9990412995f) // Eb
#define DISTANCE_SCALER         1.004642139f  // Es

// Pilot heading PID controller coefficients
#define Kp_HEADINGS             5.0f
#define Ki_HEADINGS             0.1f
#define Kd_HEADINGS             0.0f

// Pilot speed PID controller coefficients
#define Kp_SPEED                0.5f
#define Ki_SPEED                0.0f
#define Kd_SPEED                0.0f

// Pilot wheel PID controller coefficients
#define Kp_WHEEL                2.0f
#define Ki_WHEEL                1.0f
#define Kd_WHEEL                1.0f

#define MAX_SPEED               nvMM(100)

//----------------------------------------
// Pin Assignments
//----------------------------------------

#define BUTTON_PIN    A1


//----------------------------------------
// Data and Classes
//----------------------------------------

#if USE_MOTORS
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *lm = AFMS.getMotor(SWAP_MOTORS ? 4 : 3);
Adafruit_DCMotor *rm = AFMS.getMotor(SWAP_MOTORS ? 3 : 4);
#endif
void setup_encoder();

//----------------------------------------
//
//----------------------------------------

void init_bot()
{
  // do all bot initialization here
#if USE_MOTORS
  AFMS.begin();  // create with the default frequency 1.6KHz
#endif  
  // set up encoder 
  setup_encoder();

}

void set_mspeed( Adafruit_DCMotor *mm, int16_t speed )
{
#if USE_MOTORS
  if (speed < 0 )
  {
    mm->setSpeed (-speed);
    mm->run(BACKWARD);
  }
  else
  {
    mm->setSpeed (speed);
    mm->run(FORWARD);
  }
#endif
}

//----------------------------------------
// Motor handler
//----------------------------------------

void motor_handler( Pilot *pilot, int16_t lmotor, int16_t rmotor)
{

  int16_t lspeed = ((lmotor*256L)/1024L)*LMOTOR_DIR;
  int16_t rspeed = ((rmotor*256L)/1024L)*RMOTOR_DIR;

#if USE_MOTORS
  set_mspeed( lm, lspeed);
  set_mspeed( rm, rspeed);
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

char en_lastLA;
char en_lastLB;
char en_lastRA;
char en_lastRB;


// hard coding the pins to port D  
char en_rApin = 2;
char en_rBpin = 3;
char en_lApin = 4;
char en_lBpin = 5; 

//----------------------------------------
//
//----------------------------------------

void setup_encoder()
{
  cli();
  PCMSK2 |= ((1<<PCINT21)|(1<<PCINT20)|(1<<PCINT19)|(1<<PCINT18));
  PCICR |= (1<<PCIE2);
  PCIFR |= (1<<PCIF2);
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
  en_lastLA = digitalRead( en_lApin );
  en_lastLB = digitalRead( en_lBpin );
  en_lastRA = digitalRead( en_rApin );
  en_lastRB = digitalRead( en_rBpin );
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



