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
#define NAV_INFO        0
#define USE_PATHS       1
#define TARGET_INFO     1
#define MEM_REPORT      0

#define USE_SERIAL    (MOTOR_INFO|TEST_ENCODERS \
                       |USE_PATHS|BUTTON_INFO \
                       |NAV_INFO|TARGET_INFO \
                       |MEM_REPORT|PLT_USE_SERIAL)

//----------------------------------------
// Bot config
//----------------------------------------

// motors
#define SWAP_MOTORS             1
#define RMOTOR_DIR              1L    // -1 to reverse, 1 for normal
#define LMOTOR_DIR              1L     // -1 to reverse, 1 for normal

// Navigator defines
#define WHEEL_BASE              nvMM(112.95)
#define WHEEL_DIAMETER          nvMM(38.55)
#define TICKS_PER_REV           1204
#define FORWARD_HEADING_ADJUST  -0.0015f
#define LEFT_HEADING_ADJUST    -0.0010f
#define RIGHT_HEADING_ADJUST    0.0045f
#define FORWARD_DIST_ADJUST     0.06f
#define RIGHT_DIST_ADJUST       -0.285f
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
#define LMOTOR_GAIN       100L
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
  PTH_MOVE, 600, PTH_TURN, 180,
  PTH_MOVE, 600, PTH_TURN, -90, 
  PTH_MOVE, 600, PTH_TURN, -90, 
  PTH_MOVE, 600, PTH_TURN, -90, 
  PTH_MOVE, 600, PTH_TURN, 180,
  PTH_END 
};

int16_t backForthSequence[] = 
{
  PTH_MOVE, 2000,
  PTH_MOVE, -2000,
  PTH_END 
};


int16_t turningSequence[] = 
{
  PTH_TURN, 180,
  PTH_TURN, -180,
  PTH_TURN, -90,
  PTH_TURN, -90,
  PTH_TURN, -90,
  PTH_TURN, -90,
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
  Serial.println(F("NavBot V1!"));
  #endif
  
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);

  // set up encoder
  setup_encoder();

  // set up navigation
  navigator.InitEncoder( WHEEL_DIAMETER, WHEEL_BASE, TICKS_PER_REV );
  navigator.SetHeadingAdjust( nvADJUST_FORWARD, FORWARD_HEADING_ADJUST );
  navigator.SetHeadingAdjust( nvADJUST_LEFT, LEFT_HEADING_ADJUST );
  navigator.SetHeadingAdjust( nvADJUST_RIGHT, RIGHT_HEADING_ADJUST );
  navigator.SetDistanceAdjust( nvADJUST_FORWARD, FORWARD_DIST_ADJUST );
  navigator.SetDistanceAdjust( nvADJUST_RIGHT, RIGHT_DIST_ADJUST );
  // set up pilot
  pilot.SetNavigator( navigator );
  pilot.SetTimeFunction( millis );
  pilot.SetTicksHandler( ticks_handler );
  pilot.SetMotorHandler( motor_handler );
  pilot.SetHeadingPID( Kp_HEADINGS, Ki_HEADINGS, Kd_HEADINGS);
  pilot.SetSpeedPID( Kp_SPEED, Ki_SPEED, Kd_SPEED);
  pilot.SetTurnPID( Kp_TURN, Ki_TURN, Kd_TURN);
  pilot.SetMinMoveSpeed( nvMM(10));
  pilot.SetMaxMoveSpeed( MAX_SPEED );
  pilot.SetMinTurnSpeed( nvDEGREES(15) );
  pilot.SetMaxTurnSpeed( nvDEGREES(50) );
  pilot.SetMinServiceInterval( nvMS(10) );

  #if MEM_REPORT
  memReport();
  Serial.print(F("Navigator: "));
  Serial.print(sizeof(navigator));
  Serial.print(F("bytes Pilot: "));
  Serial.print(sizeof(pilot));
  Serial.println(F("bytes"));
  #endif
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
      pilot.Reset();
      #if USE_PATHS
        // set up path
        init_path( squareSequence );
      #else
        //pilot.MoveBy(nvMETERS(2));
        pilot.SpinBy(nvDEGREES(-360*5));
      #endif
      state = RUNNING;
      break;
    }
    
    case RUNNING :
    {
      #if USE_PATHS
        update_path();
      #else
        if( pilot.IsDone())
        {
           #if TARGET_INFO
           printNavInfo();
           #endif
           state = WAITING;
        }
      #endif
      break;
    }
    case WAITING :
    {
      break;
    }
  }

  //Serial.println(F("Pilot.Service()"));
  pilot.Service();

  #if NAV_INFO
    outputNavInfo();
  #endif

}

void stop()
{
  pushToStart = true;
  pilot.Stop();
  while ( !pilot.IsDone() )
  {
    pilot.Service();
  }
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
          Serial.println(F("Button Pressed"));
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
      Serial.print(F("Encoders: lft = "));
      Serial.print(lTotal);
      Serial.print(F(" rht = "));
      Serial.print(rTotal);
      Serial.println(!ok ? F(" (error)") :F(""));
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

  int16_t lspeed = ((lmotor*400L)/1024L)*LMOTOR_DIR;
  int16_t rspeed = ((rmotor*400L)/1024L)*RMOTOR_DIR;
  
  #if SWAP_MOTORS
    motors.setLeftSpeed( lspeed );
  #else
    motors.setRightSpeed( lspeed );
  #endif

  #if MOTOR_INFO
  Serial.print(F("LeftMotor: "));
  Serial.print(lspeed);
  Serial.print(F(" ("));
  Serial.print(lmotor);
  Serial.println(F(")"));
  #endif

  #if SWAP_MOTORS
    motors.setRightSpeed( rspeed );
  #else
    motors.setLeftSpeed( rspeed );
  #endif

  #if MOTOR_INFO
  Serial.print(F("RightMotor: "));
  Serial.print(rspeed);
  Serial.print(F(" ("));
  Serial.print(rmotor);
  Serial.println(F(")"));
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
      if( pilot.IsDone() && !navigator.InMotion())
      {
         #if TARGET_INFO
         printNavInfo();
         #endif
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
      pilot.MoveBy( *pth_sequence++);
      pth_state = PTH_WAITING;
      break;
    case PTH_TURN:
      pilot.TurnBy( nvDEGREES(*pth_sequence++));
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
    printNavInfo();
    lturn = navigator.TurnRate();
    lspeed = navigator.Speed();
    lheading = pose.heading;
    lx = pose.position.x;
    ly = pose.position.y;
  }

}

#endif

#if NAV_INFO || TARGET_INFO
void printNavInfo( void )
{
    nvPose pose = navigator.Pose();
    Serial.print(F("Nav - x:"));
    Serial.print( pose.position.x );
    Serial.print(F(" y: "));
    Serial.print( pose.position.y );
    Serial.print(F(" h: "));
    Serial.print( pose.heading );
    Serial.print(F(" v: "));
    Serial.print( navigator.Speed() );
    Serial.print(F(" t: "));
    Serial.println( navigator.TurnRate() );

}
#endif

#if MEM_REPORT
void memReport( void )
{
    int stack;
    extern int __bss_start;
    extern int __data_start;
    extern int __heap_start;
    extern void *__brkval;
    Serial.print(F("SRAM: data - "));
    Serial.print(((uint16_t)&__bss_start) - ((uint16_t)&__data_start));
    Serial.print(F(" bss - "));
    Serial.print(((uint16_t)&__heap_start) - ((uint16_t)&__bss_start));
    Serial.print(F(" free - "));
    Serial.println( ((uint16_t)&stack) - (__brkval!=0 ? ((uint16_t)__brkval) : ((uint16_t)&__heap_start)));
}               
#endif

