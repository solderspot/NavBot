
//----------------------------------------
// Bot config
//----------------------------------------

// The following defines are used by the main sketch
// 
// Navigator defines
#define WHEEL_BASE              nvMM(<val>)		// millimeters
#define WHEEL_DIAMETER          nvMM(<val>)		// millimeters
#define TICKS_PER_REV           <val>

#define BUTTON_PIN    			<val>

// adjust headings
#define FORWARD_HEADING_ADJUST  0.0f
#define LEFT_HEADING_ADJUST     0.0f
#define RIGHT_HEADING_ADJUST    0.0f

// adjust distances
#define FORWARD_DIST_ADJUST     0.0f

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
// Motor config
//----------------------------------------

// use these to correct for incorrect motor wiring
#define SWAP_MOTORS             0
#define RMOTOR_DIR              1L    // -1 to reverse, 1 for normal
#define LMOTOR_DIR              1L    // -1 to reverse, 1 for normal


//----------------------------------------
//
//----------------------------------------

void init_bot()
{
  // do all bot initialization here

  // you can also access the pilot and navigator if
  // want to do furtner inititalization that's
  // specific to your bot

  // e.g.
  //pilot.SetMinServiceInterval( nvMS(20));

}

//----------------------------------------
// Motor handler
//----------------------------------------

void motor_handler( Pilot *pilot, int16_t lmotor, int16_t rmotor)
{
 
	// convert lmotor and rmotor to your motor controller's range 
  int16_t lspeed = ((lmotor*127L)/1024L)*LMOTOR_DIR;
  int16_t rspeed = ((rmotor*127L)/1024L)*RMOTOR_DIR;
  
  // put your motor code in here  
  #if SWAP_MOTORS
    //setLeftMotor( rspeed );
	//setRightMotor( lspeed );
  #else
	//setLeftMotor( lspeed );
	//setRightMotor( rspeed );
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
// Ticks handler
//----------------------------------------
 
bool ticks_handler( Pilot *pilot, int16_t *lft, int16_t *rht)
{
  // read the encoders and set lft and rht
  *lft = <left tick count>;
  *rht = <left tick count>;
  return true;
}



