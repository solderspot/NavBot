
//----------------------------------------
// Bot config
//----------------------------------------

// The following defines are used by the main sketch
// 
// Navigator defines
#define WHEELBASE               nvMM(<val>)     // millimeters
#define WHEEL_DIAMETER          nvMM(<val>)     // millimeters
#define TICKS_PER_REV           <val>

#define BUTTON_PIN              <val>

// correct for systematic errors
#define WHEEL_RL_SCALER         1.0f  // Ed
#define WHEELBASE_SCALER        1.0f  // Eb

// correct distance 
#define DISTANCE_SCALER         1.0f  // Es


//----------------------------------------
// Motor config
//----------------------------------------

// use these to correct for incorrect motor wiring
#define SWAP_MOTORS             0
#define RMOTOR_DIR              1L    // -1L to reverse, 1L for normal
#define LMOTOR_DIR              1L    // -1L to reverse, 1L for normal


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
  *lft = <left tick count since last call>;
  *rht = <left tick count since last call>;

  // return true if successful or false if there was an error
  return true;
}



