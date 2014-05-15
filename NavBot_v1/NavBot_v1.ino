// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder 

// local includes
#include "Navigator.h"
#include "Pilot.h"
#include "NavBot.h"

//----------------------------------------
// Config Logic
//----------------------------------------

#define CFG_TEST_ENCODERS     0     // print encoder ticks as they change
#define CFG_TEST_MOTORS       0     // verify motor wiring
#define CFG_SQUARE_TEST       0
#define CFG_CALIBRATE_MOVE    1     // straight line movement
#define CFG_CALIBRATE_TURNS   0     // turning only test

#define CAL_DISTANCE      2     // meters to move for CALIBRATE_MOVE
#define CAL_TURNS         5     // num of turns for CALIBRATE_TURNS (+ right, - left)

#define SQUARE_SIZE       800   // size of square test sides in mm

//----------------------------------------
// Serial output config
//----------------------------------------

#define SERIAL_BAUD   9600

#define MOTOR_INFO      0       // print motor values
#define BUTTON_INFO     0       // print button state
#define NAV_INFO        0       // print nav data
#define TARGET_INFO     1       // print nav data at way points
#define MEM_REPORT      0       // print memory usage at startup

//----------------------------------------
// Your Paths
//----------------------------------------

int16_t my_path[] = 
{
  PTH_MOVE, 1000, PTH_TURN, 180,
  PTH_MOVE, 1000, PTH_TURN, 180,
  PTH_END 
};

int16_t *run_sequence = my_path; 


//----------------------------------------
// Include Bot Specific Code
//----------------------------------------

// the arduino ide does not seem to allow
// nested includes so you must first include 
// any header files  needed by the bot's code

// ZUMO_BOT
//#include <ZumoMotors.h>
//#include "ZumoBot.h"

// WALLIE_BOT
//#include <Wire.h>
#include <SoftwareSerial.h>
#include <PololuQik.h>
#include "WallieBot.h"

// BLACK_BOT
//#include <Wire.h>
//#include <Adafruit_MotorShield.h>
//#include "BlackBot.h"

// MY_BOT
//#inlcude <...your needed header files...>
//#include "MyBot.h"

          
//----------------------------------------
// Config
//----------------------------------------

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

int16_t fullSquare[] = 
{
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, 90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, 90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, 90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, 180,
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, -90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, -90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, -90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, 180,
  PTH_END 
};

int16_t cwSquare[] = 
{
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, 90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, 90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, 90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, 90,
  PTH_END 
};

int16_t ccwSquare[] = 
{
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, -90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, -90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, -90, 
  PTH_MOVE, SQUARE_SIZE, PTH_TURN, -90,
  PTH_END 
};

int16_t *squarePath = cwSquare;

//----------------------------------------
// Validate Config
//----------------------------------------

#if (CFG_TEST_ENCODERS+CFG_TEST_MOTORS+CFG_SQUARE_TEST+CFG_CALIBRATE_MOVE+CFG_CALIBRATE_TURNS) > 1
#error Only one CFG_XXX can be set to 1. The rest must be 0
#endif


//----------------------------------------
// setup
//----------------------------------------

void setup() 
{

  
  pinMode(BUTTON_PIN, INPUT);
  digitalWrite(BUTTON_PIN, HIGH);

 
  // set up navigation
  navigator.InitEncoder( WHEEL_DIAMETER, WHEELBASE, TICKS_PER_REV );
  navigator.SetDistanceScaler( DISTANCE_SCALER );
  navigator.SetWheelbaseScaler( WHEELBASE_SCALER );
  navigator.SetWheelRLScaler( WHEEL_RL_SCALER );

  // set up pilot
  pilot.SetNavigator( navigator );
  pilot.SetTimeFunction( millis );
  pilot.SetTicksHandler( ticks_handler );
  pilot.SetMotorHandler( motor_handler );

  #if USE_SERIAL
  Serial.begin(SERIAL_BAUD);
  Serial.println(F("NavBot V1!"));
  #endif
  init_bot();


  #if MEM_REPORT
  memReport();
  Serial.print(F("Navigator: "));
  Serial.print(sizeof(navigator));
  Serial.print(F(" Pilot: "));
  Serial.println(sizeof(pilot));
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
      #if CFG_CALIBRATE_MOVE
        //pilot.SetCalibrationMode( true );
        pilot.MoveBy(nvMETERS(CAL_DISTANCE));
      #elif CFG_CALIBRATE_TURNS
        //pilot.SetCalibrationMode( true );
        pilot.SpinBy(nvDEGREES(360*CAL_TURNS));
      #elif CFG_TEST_MOTORS
          while(1)
          {
            motor_handler( &pilot, 0, 256 );
            delay(2000);
            motor_handler( &pilot, 256, 0 );
            delay(2000);
          }
      #else

        #if CFG_SQUARE_TEST
          //pilot.SetCalibrationMode( true );
          init_path( squarePath );
        #else
          init_path( run_sequence );
        #endif
      #endif
      state = RUNNING;
      break;
    }
    
    case RUNNING :
    {
      #if CFG_CALIBRATE_MOVE || CFG_CALIBRATE_TURNS
        if( pilot.IsDone())
        {
           #if TARGET_INFO
           printNavInfo();
           #endif
           state = WAITING;
        }
      #else
        update_path();
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
    
  char button =  digitalRead( BUTTON_PIN );
    
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
  #if CFG_TEST_ENCODERS
  {
    int16_t lft, rht;
    static int16_t lTotal = 0;
    static int16_t rTotal = 0;
    static int16_t lastlTotal = 0;
    static int16_t lastrTotal = 0;
    bool ok = ticks_handler( &pilot,  &lft, &rht);
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
  static nvHeading lheading = nvDEGREES(SQUARE_SIZE);
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

