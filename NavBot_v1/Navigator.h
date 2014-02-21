// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder 

#ifndef __NAVIGATOR_H
#define __NAVIGATOR_H

#include <stdint.h>

//----------------------------------------
//
//----------------------------------------

typedef float       nvCoord;        // millimeters
typedef float       nvDegrees;      // degrees 
typedef nvDegrees   nvHeading;      // degrees from North
typedef float       nvRate;         // change per second
typedef float       nvDistance;     // millimeters
typedef uint32_t    nvTime;         // time in milliseconds

//----------------------------------------
//
//----------------------------------------

#define nvMM(D)         ((nvDistance)(D))
#define nvMETERS(D)     ((nvDistance)((D)*1000))
#define nvMS(D)         ((nvTime)(D))
#define nvSECONDS(D)    ((nvTime)((D)*1000))
#define nvDEGREES(D)    ((nvDegrees)(D))

#define nvNORTH         nvDEGREES(0)
#define nvNORTHEAST     nvDEGREES(45)
#define nvEAST          nvDEGREES(90)
#define nvSOUTHEAST     nvDEGREES(135)
#define nvSOUTH         nvDEGREES(180)
#define nvSOUTHWEST     nvDEGREES(225)
#define nvWEST          nvDEGREES(270)
#define nvNORTHWEST     nvDEGREES(315)

//----------------------------------------
//
//----------------------------------------

struct nvPosition
{
    nvCoord     x;              // mm
    nvCoord     y;              // mm
};

//----------------------------------------
//
//----------------------------------------

struct nvPose
{
    nvPosition  position;       // mm from (0, 0)
    nvHeading   heading;        // degrees from North
};

//----------------------------------------
//
//----------------------------------------

class Navigator
{
    public:

        Navigator();

        // methods
        void            Init( nvDistance wheel_diameter, nvDistance wheel_base, uint16_t ticks_per_revolution );
        void            Reset( nvTime now );
        void            UpdateTicks( int16_t lticks, int16_t rticks, nvTime now );

        // setters and getters
        void            SetPose( const nvPose &pose) { m_pose = pose; }
        nvPose          Pose( void ) { return m_pose; }
        void            SetPosition( const nvPosition &pos) { m_pose.position = pos; }
        void            SetPosition( nvCoord x, nvCoord y) { m_pose.position.x = x; m_pose.position.y = y; }
        nvPosition      Position( void ) { return m_pose.position; }
        void            SetHeading( nvHeading heading ) { m_pose.heading = heading; }
        nvHeading       Heading( void ) { return m_pose.heading; }
        void            SetSpeed( nvRate speed ) { m_speed = speed; }
        nvRate          Speed( void ) { return m_speed; }
        void            SetTurnRate( nvRate rate ) { m_turn_rate = rate; }
        nvRate          TurnRate( void ) { return m_turn_rate; }
        void            SetEncoderBias( float bias) { m_encoder_bias = bias; }
        float           EncoderBias( void ) { return m_encoder_bias; }

    private:

        nvDistance      m_dist_per_tick;
        nvPose          m_pose;
        nvRate          m_speed;                // mm per second
        nvRate          m_turn_rate;            // degrees per second
        nvDistance      m_wheel_base;           // mm
        float           m_encoder_bias;
        nvTime          m_last_time;
};


#endif // __NAVIGATOR_H
