// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder 

// include needed libraries
// standard arduino libs

// other libs
// Zumo motor lib: https://github.com/pololu/zumo-shield
#include <ZumoMotors.h>

// local includes
#include "Navigator.h"
#include "Pilot.h"

//----------------------------------------
// Serial output config
//----------------------------------------

#define SERIAL_BAUD   9600

#define MOTOR_INFO      0
#define BUTTON_INFO     0
#define TEST_ENCODERS   0
#define NAV_INFO        1
#define USE_PATHS       1

#define USE_SERIAL    (MOTOR_INFO|TEST_ENCODERS \
                       |USE_PATHS|BUTTON_INFO \
                       |NAV_INFO)

//----------------------------------------
// Bot config
//----------------------------------------

// motors
#define SWAP_MOTORS       0
#define RMOTOR_DIR        -1    // -1 to reverse, 1 for normal
#define LMOTOR_DIR        1     // -1 to reverse, 1 for normal

// Navigator defines
#define WHEEL_BASE      nvMM(83.5)
#define WHEEL_DIAMETER  nvMM(38.9)
#define TICKS_PER_REV   1204
#define HEADING_BIAS    0.0f

// Pilot heading PID controller coefficients
#define Kp_HEADINGS     0.0f
#define Ki_HEADINGS     0.0f
#define Kd_HEADINGS     0.0f

// Pilot speed PID controller coefficients
#define Kp_SPEED        0.0f
#define Ki_SPEED        0.0f
#define Kd_SPEED        0.0f


//----------------------------------------
// Defines
//----------------------------------------

#define LEFT            1
#define RIGHT           0
#define FORWARDS        1
#define BACKWARDS      -1
          
//----------------------------------------
// Pin Assignments
//----------------------------------------

char buttonPin  = 12;

//----------------------------------------
// Config
//----------------------------------------

// PID - use LMOTOR_GAIN to make the left motor more
// powerful or less pwerful than the right motor.
// Value is as a percentage, i.e. 110 means left
// motor will be 10% faster than the right. 90
// would mean the left motor is 10% slower
#define LMOTOR_GAIN       100
bool pushToStart        = true;


//----------------------------------------
// Data and Data Types 
//----------------------------------------

// Bot states
enum State
{
  INIT,
  RUNNING,
  WAITING,
  NUM_STATES
};

State state       = INIT;

//----------------------------------------
// Instance classes
//----------------------------------------

ZumoMotors    motors;
Navigator     navigator;
Pilot         pilot;

//----------------------------------------
// Simple Path control
//----------------------------------------

enum PathState
{
  PTH_WAITING,
  PTH_STOPPING,
  PTH_DONE
};

int16_t *pth_sequence = NULL;

PathState pth_state = PTH_DONE;

#define PTH_END         0
#define PTH_MOVE        1
#define PTH_TURN        2

int16_t squareSequence[] = 
{
  PTH_MOVE, 600, PTH_TURN, 90, 
  PTH_MOVE, 600, PTH_TURN, 90, 
  PTH_MOVE, 600, PTH_TURN, 90, 
  PTH_MOVE, 600,
  PTH_MOVE, -600, PTH_TURN, -90, 
  PTH_MOVE, -600, PTH_TURN, -90, 
  PTH_MOVE, -600, PTH_TURN, -90, 
  PTH_MOVE, -600,
  PTH_END 
};

int16_t backForthSequence[] = 
{
  PTH_MOVE, 2000,
  PTH_MOVE, -2000,
  PTH_END 
};

//----------------------------------------
// Forward reference
//----------------------------------------

Pilot::TicksHandler ticks_handler;
Pilot::MotorHandler motor_handler;

//----------------------------------------
// setup
//----------------------------------------

void setup() 
{

  #if USE_SERIAL
  Serial.begin(SERIAL_BAUD);
  Serial.println("NavBot V1!");
  #endif
  
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);

  // set up encoder
  setup_encoder();

  // set up navigation
  navigator.InitEncoder( WHEEL_DIAMETER, WHEEL_BASE, TICKS_PER_REV );
  navigator.SetEncoderHeadingBias( HEADING_BIAS );

  // set up pilot
  pilot.SetNavigator( navigator );
  pilot.SetTimeFunction( millis );
  pilot.SetMaxMoveSpeed( nvMM(30));
  pilot.SetMaxTurnSpeed( nvDEGREES(120) );
  pilot.SetTicksHandler( ticks_handler );
  pilot.SetMotorHandler( motor_handler );
  pilot.SetHeadingPID( Kp_HEADINGS, Ki_HEADINGS, Kd_HEADINGS);
  pilot.SetSpeedPID( Kp_SPEED, Ki_SPEED, Kd_SPEED);

}

//----------------------------------------
// loop - Main logic
//----------------------------------------

void loop() 
{
  
  handleButtonPress();

  if( pushToStart )
  {
    // don't do anything till the button is pressed
    return;
  }

  // handle each state
  switch(state)
  {
    case INIT:
    {
      Serial.println("Pilot.Reset()");
      pilot.Reset();
      #if USE_PATHS
        // set up path
      Serial.println("init_path()");
        init_path( backForthSequence );
      #else
        pilot.Move(nvMETERS(3));
      #endif
      state = RUNNING;
      break;
    }
    
    case RUNNING :
    {
      #if USE_PATHS
      //Serial.println("update_path()");
        update_path();
      #endif
      break;
    }
    case WAITING :
    {
      Serial.println("Waiting");
      break;
    }
  }

  //Serial.println("Pilot.Service()");
  pilot.Service();

  #if NAV_INFO
    outputNavInfo();
  #endif

}

void stop()
{
  pushToStart = true;
  pilot.Stop();
  state = INIT;
}

//----------------------------------------
// button logic
//----------------------------------------

void handleButtonPress(void)
{
  static char lastButton = HIGH;
  static char count = 0;           
    
  char button =  digitalRead( buttonPin );
    
  if( (button != lastButton) )
  {
    if(++count > 2) 
    {
      if( button == LOW )
      {
        #if BUTTON_INFO
          Serial.println("Button Pressed");
        #endif
        if( pushToStart )
        {
          pushToStart = false;
          state = INIT;
        }
        else
        {
          stop();
        }
      }
      lastButton = button;
      count = 0;
    }
  }
  else
  {
    count = 0;
  }
  #if TEST_ENCODERS
  {
    int16_t lft, rht;
    static int16_t lTotal = 0;
    static int16_t rTotal = 0;
    static int16_t lastlTotal = 0;
    static int16_t lastrTotal = 0;
    bool ok = get_ticks_since_last( &lft, &rht);
    lTotal += lft;
    rTotal += rht;
    
    if( lastlTotal != lTotal || lastrTotal != rTotal )
    {
      Serial.print("Encoders: lft = ");
      Serial.print(lTotal);
      Serial.print(" rht = ");
      Serial.print(rTotal);
      Serial.println( !ok ? " (error)" :"");
      lastlTotal = lTotal;
      lastrTotal = rTotal;
    }
  }
  #endif
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

  int16_t lspeed = ((lmotor*LMOTOR_GAIN)/100)*LMOTOR_DIR;
  int16_t rspeed = (rmotor)*RMOTOR_DIR;
  
  lspeed = map( lspeed, -1024, 1024, -400, 400); 
  lspeed = map( rspeed, -1024, 1024, -400, 400); 

  #if SWAP_MOTORS
    motors.setLeftSpeed( lspeed );
  #else
    motors.setRightSpeed( lspeed );
  #endif

  #if MOTOR_INFO
  Serial.print("LeftMotor: ");
  Serial.print(lspeed);
  Serial.print(" (");
  Serial.print(lmotor);
  Serial.println(")");
  #endif

  #if SWAP_MOTORS
    motors.setRightSpeed( rspeed );
  #else
    motors.setLeftSpeed( rspeed );
  #endif

  #if MOTOR_INFO
  Serial.print("RightMotor: ");
  Serial.print(rspeed);
  Serial.print(" (");
  Serial.print(rmotor);
  Serial.println(")");
  #endif
 
}

//----------------------------------------
// Ticks handler
//----------------------------------------

bool ticks_handler( Pilot *pilot, int16_t *lticks, int16_t *rticks)
{
    return get_ticks_since_last( lticks, rticks );
}

//----------------------------------------
// Simple Path control
//----------------------------------------

void init_path( int16_t *sequence )
{
  pth_sequence = sequence;
  pth_state = PTH_WAITING;
  pth_next();
}

//----------------------------------------
//
//----------------------------------------

void update_path()
{
  switch (pth_state)
  {
    case PTH_WAITING:
      if( pilot.IsDone())
      {
         pth_next();
      }
      break;
    default:
    case PTH_DONE:
      break;
  }
}

//----------------------------------------
//
//----------------------------------------

void pth_next()
{
  int16_t action; 

  if( !pth_sequence || pth_state == PTH_DONE  )
  {
    return;
  }

  action = *pth_sequence++;

  switch( action )
  {
    case PTH_MOVE:
      pilot.Move( *pth_sequence++);
      pth_state = PTH_WAITING;
      break;
    case PTH_TURN:
      pilot.Turn( *pth_sequence++);
      pth_state = PTH_WAITING;
      break;
    default:
    case PTH_END:
      stop();
      pth_state = PTH_DONE;      
      break;
  }
}


//----------------------------------------
//
//----------------------------------------

#if NAV_INFO
void outputNavInfo()
{
  static nvHeading lheading = nvDEGREES(1000);
  static nvRate lspeed = nvMETERS(10);
  static nvRate lturn = nvMETERS(10);
  static nvDistance lx = nvMM(0);
  static nvDistance ly = nvMM(0);

  nvPose  pose = navigator.Pose();

  if(  pose.heading != lheading
       || pose.position.x != lx
       || pose.position.y != ly
       || navigator.Speed() != lspeed
       || navigator.TurnRate() != lturn )
  {
    Serial.print("Nav - x:");
    Serial.print( pose.position.x );
    Serial.print(" y: ");
    Serial.print( pose.position.y );
    Serial.print(" h: ");
    Serial.print( pose.heading );
    Serial.print(" v: ");
    Serial.print( navigator.Speed() );
    Serial.print(" t: ");
    Serial.println( navigator.TurnRate() );
    lturn = navigator.TurnRate();
    lspeed = navigator.Speed();
    lheading = pose.heading;
    lx = pose.position.x;
    ly = pose.position.y;
  }

}

#endif
