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

#define nvMAX_TIME      0xffffffff

//----------------------------------------
// Base Types
//----------------------------------------

typedef float           nvCoord;        // millimeters
typedef float           nvDegrees;      // degrees 
typedef float           nvRadians;      // radians 
typedef nvDegrees       nvHeading;      // degrees from North
typedef float           nvRate;         // change per second
typedef float           nvDistance;     // millimeters
typedef uint32_t        nvTime;         // time in milliseconds

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
inline nvTime           nvDeltaTime( nvTime last, nvTime now)   { return now >= last ?  now - last : nvMAX_TIME - last + now + 1; }

// radians <-> degrees
inline nvDegrees        nvRadToDeg( nvRadians rad ) { return (rad*180.0f)/M_PI; }
inline nvRadians        nvDegToRad( nvDegrees deg ) { return (deg*M_PI)/180.0f; }
inline nvDegrees        nvClipDegrees( nvDegrees deg ) { return deg - ( ((int32_t)deg/360L)*360.0f); }
inline nvDegrees        nvClipHeading( nvDegrees deg ) { deg = nvClipDegrees(deg); return deg < 0.00f ? deg + 360.0f : deg; }
inline nvRadians        nvClipRadians( nvRadians rad ) { rad -= ( ((int32_t)(rad/(2.0f*M_PI)))*2*M_PI); return rad < 0.00f ? rad + 2.0f*M_PI : rad;}

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

   // initialization
        void            InitEncoder( nvDistance wheel_diameter, nvDistance wheel_base, uint16_t ticks_per_revolution );

   // service
        void            Reset( nvTime now );
        bool            UpdateTicks( int16_t lticks, int16_t rticks, nvTime now );

   // getters
        
        // location
        nvPose          Pose( void ) { return m_pose; }
        nvPosition      Position( void ) { return m_pose.position; }
        nvHeading       Heading( void ) { return m_pose.heading; }

        // movement
        nvRate          Speed( void ) { return m_speed; }
        nvRate          TurnRate( void ) { return m_turn_rate; }
        bool            IsMoving( void ) { return m_speed != 0.0f; }
        bool            IsTurning( void ) { return m_turn_rate != 0.0f; }
        bool            InMotion( void ) { return IsMoving() || IsTurning(); }

        // systematic odometry error correction
        float           DistanceScaler( void) { return m_dist_scaler; }
        float           WheelRLScaler( void ) { return m_wheel_rl_scaler; }
        float           WheelbaseScaler( void ) { return m_wheelbase_scaler; }

   // setters
        // location
        void            SetStartPose( const nvPose &pose) { m_init_pose.position = pose.position; m_init_pose.heading = nvClipHeading( pose.heading); }
        void            SetStartPosition( const nvPosition &pos) { m_init_pose.position = pos; }
        void            SetStartPosition( nvCoord x, nvCoord y) { m_init_pose.position.x = x; m_init_pose.position.y = y; }
        void            SetStartHeading( nvHeading heading ) { m_init_pose.heading = nvClipHeading(heading); }

        // systematic odometry error correction
        void            SetWheelRLScaler( float rl_scaler) { m_wheel_rl_scaler = rl_scaler;}
        void            SetWheelbaseScaler( float scaler ) { m_wheelbase_scaler = scaler;}
        void            SetDistanceScaler( float scaler) { m_dist_scaler = scaler; }

        // config
        void            SetMinInterval( nvTime min ) { m_min_dt = min; }


   // navigation
        nvPosition      NewPosition( nvDistance distance ); 
        nvPosition      NewPositionByHeading( nvPosition &pos, nvHeading heading, nvDistance dist );    
        nvPosition      NewPositionByHeading( nvHeading heading, nvDistance distance ) { return NewPositionByHeading( m_pose.position, heading, distance); }
        nvDegrees       HeadingAdjust( nvHeading target ); 
        nvPosition      NewPosition( nvDistance x_offset, nvDistance y_offset );
        void            GetTo( nvPosition &pos, nvHeading *heading, nvDistance *distance );

    protected:

        // spacial properties
        nvPose          m_pose;                             // current pose
        nvRate          m_speed;                            // current mm per second
        nvRate          m_turn_rate;                        // current degrees per second

        // settings/config                                  
        nvPose          m_init_pose;                        // starting pose
        nvTime          m_min_dt;                           // minimum time delta unit
        uint16_t        m_ticks_per_rev;                    // encoder ticks per wheel revolution
        nvDistance      m_nominal_wheel_diam;               // nominal wheel diameter
        nvDistance      m_nominal_wheelbase;                // nominal width of wheel base
        float           m_wheelbase_scaler;                 // scaling factor to correct nominal wheel base
        float           m_wheel_rl_scaler;                  // ratio of right and left wheel diameters - used to calculate real diameters
        float           m_dist_scaler;                      // scaling factor to correct for distance calcs due to effective wheel diamteres 

        // intermediate 
        float           m_rticks_to_dist;
        float           m_lticks_to_dist;
        nvDistance      m_effective_wheelbase; 
        nvDistance      m_effective_wheel_diam; 

        // state data
        nvTime          m_last_ticks_time;                  // time of last ticks update
        nvTime          m_dt;                               // current time delta
        int16_t         m_lticks;                           // current ticks
        int16_t         m_rticks;                           // current ticks
        nvRadians       m_heading;                          // current heading in radians

};


#endif // __NAVIGATOR_H
