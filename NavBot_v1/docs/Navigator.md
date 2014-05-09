#Navigator

One of the core components of NavBot is the Navigator. Its job is to encapsulate the localization information of the robot, i.e. where in the world it is.

For this implementation I am only considering a two dimensional world as the bot will mostly likely only travel around my office floor or some other flat surface. 

At some later point if we want the bot to travel over general terrain we'd need to factor in elevation too.

But for now the navigator tracks the following data:

 * **Position (x,y)** - The x and y location of the bot from its center, in millimeters, from some origin (0,0)
 * **Heading (h)** - The orientation of the bot in degrees from North about its center.
 * **Speed (v)** - The speed of motion, in millimeters per second, in the current heading from its center.
 * **Turn Rate** - The degrees per second the bot is turning about its center. 

![image](http://solderspot.files.wordpress.com/2014/02/navbot-pose.png?w=600)

##Implementation

I've implemented the Navigator as it's own C++ class and tried to make it as modular as possible so it can be deployed to any application requiring localization.

For simplicity it uses floating point math. However, as I also want to be able to change over to fixed point math, Navigator abstracts out the some basic data types:

	//----------------------------------------
	// Base Types
	//----------------------------------------

	typedef float       nvCoord;        // millimeters
	typedef float       nvDegrees;      // degrees 
	typedef float       nvRadians;      // radians 
	typedef nvDegrees   nvHeading;      // degrees from North
	typedef float       nvRate;         // change per second
	typedef float       nvDistance;     // millimeters
	typedef uint32_t    nvTime;         // time in milliseconds

And as you can see from the comments we are using millimeters, degrees and milliseconds as the units of measurement.

Everything is floating point except time, which is an unsigned 32 bit integer.

Each type has an "nv" prefix to avoid possible name clashes with other libraries or code.

To make the code more readable, and to facilitate possible conversion to fixed point at a later date, Navigator provides some helper macros:

	//----------------------------------------
	// Helper Macros
	//----------------------------------------

	#define nvMM(D)         ((nvDistance)(D))
	#define nvMETERS(D)     ((nvDistance)((D)*1000))
	#define nvMS(MS)        ((nvTime)(MS))
	#define nvSECONDS(S)    ((nvTime)((S)*1000))
	#define nvDEGREES(D)    ((nvDegrees)(D))
	#define nvRADIANSS(R)   ((nvRadians)(R))

	#define nvNORTH         nvDEGREES(0)
	#define nvNORTHEAST     nvDEGREES(45)
	#define nvEAST          nvDEGREES(90)
	#define nvSOUTHEAST     nvDEGREES(135)
	#define nvSOUTH         nvDEGREES(180)
	#define nvSOUTHWEST     nvDEGREES(225)
	#define nvWEST          nvDEGREES(270)
	#define nvNORTHWEST     nvDEGREES(315)

Navigator also uses the following data abstractions:

	//----------------------------------------
	// nvPosition
	//----------------------------------------

	struct nvPosition
	{
		nvCoord     x;              // mm from origin
		nvCoord     y;              // mm from origin
	};

	//----------------------------------------
	// nvPose
	//----------------------------------------

	struct nvPose
	{
		nvPosition  position;       // mm from (0, 0)
		nvHeading   heading;        // degrees from North
	};


##Using Navigator

To use Navigator we first need to create an instance of the class:


	#include "Navigator.h"

	Navigator navigator;


Then in **setup()** we need to initialize the instance:


	// Navigator defines
	#define WHEEL_BASE      nvMM(83.5)
	#define WHEEL_DIAMETER  nvMM(35.4)
	#define TICKS_PER_REV   1204

	setup()
	{
	    :
	    :
		 // set up navigation
		 navigator.InitEncoder( WHEEL_DIAMETER, WHEEL_BASE, TICKS_PER_REV );

		:
		:
	}

This particular implementation uses dead reckoning via wheel encoders so we pass in the nominal wheel diameter, wheelbase distance and the number of encoder ticks per revolution. 

![image](http://solderspot.files.wordpress.com/2014/02/screen-shot-2014-03-06-at-4-42-21-pm.png?w=600)

With this information the Navigator can calculate the number of millimeters travelled per encoder tick by taking the circumference of the wheels and dividing it by the number of encoder ticks.

By default the Navigator assumes the bot is at location (0, 0) and heading (0). If we want to set a different starting location we use these two functions:

    navigator.SetStartPosition( nvMM(200), nvMM(300));
    navigator.SetStartHeading( nvEAST );

or

    nvPose pose;

    pose.position.x = nvMM(200);
    pose.position.y = nvMM(300);
    pose.heading = nvEAST;

    navigator.SetStartPose( pose );

To enable Navigator to track the movement of the robot we need first call its **Reset()** method and then regularly call the **UpdateTicks()** method.

So first call **Reset()**:

    navigator.Reset( millis() );

passing in the current time in milliseconds. This will initialize the navigator and set its location to the starting position and heading.

After **Reset()** we must regularly call **UpdateTicks()** thus:

    navigator.UpdateTicks( lticks, rticks, millis() );

passing in the number of encoder ticks for the left and right wheels since we last called **UpdateTicks()** and/or **Reset()**, and the current time in milliseconds.

Using the tick counts and time the navigator can calculate changes in the bot's location. 

Navigator will recalculate changes in position every 10 ms by default. You can change the minimum interval by calling **SetMinInterval()**: 

    navigator.SetMinInterval( nvMS(50) );

Once the minimum time interval has elapsed it calculates the new location of the bot based on the tick counts and the amount of time elapsed.

At any time we can query this information from the navigator by using various getter methods:

    nvPose  pose = navigator.Pose();

    Serial.print("Heading: ");
    Serial.println( pose.heading );

    Serial.print("Position: (");
    Serial.print( pose.position.x );
    Serial.print(", ");
    Serial.print( pose.position.y );
    Serial.println(")");

    Serial.print("Speed: ");
    Serial.println( navigator.Speed() );

    Serial.print("Turn Rate: ");
    Serial.println( navigator.TurnRate() );

  

 
