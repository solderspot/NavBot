![image](http://solderspot.files.wordpress.com/2014/02/calvin-and-hobbes.jpg?w=150)

#The Pilot

The Pilot controls the physical movement of the bot. Its primary goal is to move the bot accurately. 
As we are using dead reckoning for navigation this means avoiding sudden accelerations or decelerations 
that could cause loss of traction, which means anticipating arrival points ahead of time and slowing 
down gradually to a stop at the destination. It also needs to use PID controllers to minimize "errors" 
for speed control, heading control and turing, as well as dealing with systemic issues like 
alignment and motor characteristics.

On the other hand, the external functionality of the Pilot is straight forward. We simply tell it either a heading we want it to turn to, a distance to travel or an x/y coordinate to move to.

##Setting up the Pilot

I've tried to design the Pilot so it can be integrated into most any C++ environment, not just the Arduino. To facilitate this the Pilot's setup code is a little more involved than most.

First step is to create a Pilot instance instance:

	#include "Pilot.h"

	Pilot pilot;


In the setup phase we need provide the Pilot with four external services:

	//----------------------------------------
	// setup
	//----------------------------------------

	void setup()
	{
		:  
		:  
		// set up pilot
		pilot.SetNavigator( navigator );
		pilot.SetTimeFunction( millis );
		pilot.SetTicksHandler( ticks_handler );
		pilot.SetMotorHandler( motor_handler );
		:  
		:  
	}


The first is a Navigator instance. The Pilot needs this so it can "see" where it is and where it is going.

The second is a time function that returns an unsigned 32 bit integer of the current time in milliseconds. 
An example is the Arduino's **mills()** function. It requires this for all time keeping needs.

The third is a function that returns the tick counts for each of the wheels. The function takes three 
parameters: a pointer to the pilot instance and two pointers to signed 16 bit integers. It writes the 
encoder tick counts to the integers and returns true if there are no encoder errors, or returns false 
if there is a problem:

	//----------------------------------------
	// Ticks handler
	//----------------------------------------

	bool ticks_handler( Pilot *pilot, int16_t *lticks, int16_t *rticks)
	{
		if( encoder_error())
		{
			//error - reset the encoder
			encoder_reset()
			return false;
		}

		// no errors, records the current tick count
		*lticks = encoder_left_ticks();
		*rticks = encoder_right_ticks();

		// reset counts back to zero
		encoder_clear_ticks();
		return true;
	}

It is important that the handler returns the number of ticks counted since it was last called and not the total ticks since the program started.

The fourth is a function to update the motors with new power values. This function takes three parameters: a pointer to the pilot instance and two signed 16 bit integers for left and right power levels:


	//----------------------------------------
	// Motor handler
	//----------------------------------------

	void motor_handler( Pilot *pilot, int16_t lmotor, int16_t rmotor)
	{
		// set motor power levels
		// lmotor and rmotor have ranges from -1024 to 1024
	}

The motor power levels range from -1024 (full reverse) to 1024 (full forwards). You would need to convert these values to suit the motor driver interface your bot uses. Typically drivers use 8 bit values.

Finally we need to call **Service()**:


	pilot.Service();


as frequently as possible. This call will handle all the pilot logic like reading the encoder ticks, update the navigator and adjusting the motor power levels. It's the brains of the operation.

##Configuring

There are various settings you can change from their defaults:

	pilot.SetMinServiceInterval( nvMS(20) );
	pilot.SetMinMoveSpeed( nvMM(10));
	pilot.SetMaxMoveSpeed( nvMM(30));
	pilot.SetMinTurnSpeed( nvDEGREES(10) );
	pilot.SetMaxTurnSpeed( nvDEGREES(45) );

The "min" service interval is the minimum amount of time, in milliseconds, the Pilot will wait between service updates. The actual time between updates will depend on how frequently **Service()** is called. If you set a minimum interval of 10 ms but call **Service()** every 100 ms then the effective service interval is 100 ms.

The effective service interval is important. It should be frequent and regular. If the service interval is long then the pilot's behavior will be course and jerky. If too short then it might become erratic due to encoder sampling being too small. At this point I'm not sure what a good interval is. It will certainly depend on the resolution of the encoders.

The speed settings are a way to optimize the bot's navigation performance. Higher speeds usually result in more tire/track slippage which will reduce accuracy of navigation more quickly. Setting a minimum speed can help avoid the problem where one motor will turn before the other when at the power threshold for movement. This is particularly troublesome when turning in place where we really need both motors to turn together.

I also plan to add a "calibration" mode to the Pilot so it can detect the ideal minimum speeds by sensing motor imbalance via the wheel encoders. In fact I plan to make the Pilot as self calibrating as possible. I'm also hoping that it is possible to have default PID configurations that work for most setups.

##Controlling Movement

We can tell the Pilot to either **TurnBy()** a number of degrees, **TurnTo()** a specific heading, **MoveBy()** a certain distance or **MoveTo()** a specific position:

	// turn 35 degrees to the right
	pilot.TurnBy( nvDEGREES(35) );

	while( !pilot.IsDone())
	{
	  pilot.Service();
	}

	// turn 80 degrees to the left
	pilot.TurnBy( nvDEGREES(-80) );

	while( !pilot.IsDone())
	{
	  pilot.Service();
	}

	// turn to heading 270 (EAST)
	pilot.TurnTo( nvDEGREES(270) );

	while( !pilot.IsDone())
	{
	  pilot.Service();
	}

	// move 35 cm in the current heading
	pilot.MoveBy( nvMM(350) );

	while( !pilot.IsDone())
	{
	  pilot.Service();
	}

	// turn to 25cm east and 40 cm north of origin
	pilot.MoveTo( nvMM(250), nvMM(400) );

	while( !pilot.IsDone())
	{
	  pilot.Service();
	}

	// we can use the navigator to move
	// 25 cm east and 40 cm north of our
	// current location
	nvPosition pos = navigator->Position();
	pos.x += nvMM(250);
	pos.y += nvMM(400);
	pilot.MoveTo( pos );

	while( !pilot.IsDone())
	{
	  pilot.Service();
	}

We can also tell the Pilot to **Stop()**:

	// stop moving
	pilot.Stop();

	while( !pilot.IsDone())
	{
	  pilot.Service();
	}

Note that stopping is not immediate. Abrupt stops may cause the bot to skid and that would not be taken into account by the wheel encoders and so our Navigator's position information will be off. Instead the Pilot gently decelerates to a stop.

##Current Status

The Pilot is still work in progress. I have the basic turning functionality working but there is still a lot left to do.

