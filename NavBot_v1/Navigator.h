// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder 

#ifndef __NAVIGATOR_H
#define __NAVIGATOR_H

#include <stdint.h>
#include <math.h>


//----------------------------------------
// Constants
//----------------------------------------

#define nvMAX_TIME	0xffffffff

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

//----------------------------------------
// Helper Functions
//----------------------------------------

// Time delta function that handles wrapround
inline nvTime 		nvDeltaTime( nvTime last, nvTime now)	{ return now >= last ?  now - last : nvMAX_TIME - last + now + 1; }
// radians <-> degrees
inline nvDegrees 	nvRadToDeg( nvRadians rad ) { return (rad*180.0f)/M_PI; }
inline nvRadians	nvDegToRad( nvDegrees deg ) { return (deg*M_PI)/180.0f; }
inline nvDegrees	nvClipDegrees( nvDegrees deg ) { return deg - ( ((int32_t)deg/360L)*360.0f); }
inline nvDegrees	nvClipHeading( nvDegrees deg ) { deg = nvClipDegrees(deg); return deg < 0.00f ? deg + 360.0f : deg; }
inline nvRadians	nvClipRadians( nvRadians rad ) { rad -= ( ((int32_t)(rad/(2.0f*M_PI)))*2*M_PI); return rad < 0.00f ? rad + 2.0f*M_PI : rad;}

//----------------------------------------
// nvPosition
//----------------------------------------

struct nvPosition
{
    nvCoord     x;              // mm
    nvCoord     y;              // mm
};

//----------------------------------------
// nvPose
//----------------------------------------

struct nvPose
{
    nvPosition  position;       // mm from (0, 0)
    nvHeading   heading;        // degrees from North
};

//----------------------------------------
// Navigator
//----------------------------------------

class Navigator
{
    public:

        Navigator();

        // methods
        void            InitEncoder( nvDistance wheel_diameter, nvDistance wheel_base, uint16_t ticks_per_revolution );
        void            Reset( nvTime now );
        bool            UpdateTicks( int16_t lticks, int16_t rticks, nvTime now );

		// getters
        nvPose          Pose( void ) { return m_pose; }
        nvPosition      Position( void ) { return m_pose.position; }
        nvHeading       Heading( void ) { return m_pose.heading; }
        nvRate          Speed( void ) { return m_speed; }
        nvRate          TurnRate( void ) { return m_turn_rate; }
 		bool			IsMoving( void ) { return m_speed != 0.0f; }
		bool			IsTurning( void ) { return m_turn_rate != 0.0f; }
		bool			InMotion( void ) { return IsMoving() || IsTurning(); }
		void			GetTo( nvPosition &pos, nvHeading *heading, nvDistance *distance );
        float           WheelBaseAdjust( void ) { return m_wheel_base_adjust - 1.0f; }
        float           WheelDistAdjust( void ) { return m_wheel_dist_adjust - 1.0f; }
        float           LRWheelRatioAdjust( void ) { return m_lr_wheel_adjust - 1.0f; }

		// setters
		void            SetStartPose( const nvPose &pose) { m_init_pose.position = pose.position; m_init_pose.heading = nvClipHeading( pose.heading); }
		void            SetStartPosition( const nvPosition &pos) { m_init_pose.position = pos; }
		void            SetStartPosition( nvCoord x, nvCoord y) { m_init_pose.position.x = x; m_init_pose.position.y = y; }
		void            SetStartHeading( nvHeading heading ) { m_init_pose.heading = nvClipHeading(heading); }
		void            SetLRWheelRatioAdjust( float lr_adjust) { m_lr_wheel_adjust = 1.0f + lr_adjust; }
		void            SetWheelDistAdjust( float wd_adjust) { m_wheel_dist_adjust = 1.0f + wd_adjust; }
		void            SetWheelBaseAdjust( float b_adjust) { m_wheel_base_adjust = 1.0f + b_adjust; }
		void			SetMinInterval( nvTime min ) { m_min_dt = min; }

		// helpers
		nvPosition		NewPosition( nvDistance distance );	
		nvPosition		NewPositionByHeading( nvHeading, nvDistance distance );	
        nvDegrees       HeadingAdjust( nvHeading target ); 
		nvPosition		NewPosition( nvDistance x_offset, nvDistance y_offset );	

    protected:

		// spacial properties
        nvPose          m_pose;					// current pose
        nvRate          m_speed;                // current mm per second
        nvRate          m_turn_rate;            // current degrees per second

		// settings/config
        nvPose          m_init_pose;			// starting pose
		uint16_t		m_ticks_per_rev;		// encoder ticks per wheel revolution
        nvDistance      m_wheel_diam;		    // wheel diameter
        nvDistance      m_dist_per_tick;		// dist travelled per tick
        nvDistance      m_base_dist;            // nominal wheel base
        nvDistance      m_wheel_base;           // mm width of wheel base
        float           m_lr_wheel_adjust;	    // lleft/right wheel ration adjustment
        float           m_wheel_dist_adjust;	// wheel linear distance adjustment
        float           m_wheel_base_adjust;	// wheel base adjustment
		nvTime			m_min_dt;   			// minimum time delta unit

		// state data
        nvTime          m_last_ticks_time;		// time of last ticks update
		nvTime			m_dt;					// current time delta
		int16_t			m_lticks;				// current ticks
		int16_t			m_rticks;				// current ticks
		nvRadians		m_heading;				// current heading in radians

};


#endif // __NAVIGATOR_H
