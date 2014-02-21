// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder 

#include "Navigator.h"
#include <math.h>

//----------------------------------------
//
//----------------------------------------

Navigator::Navigator()
: m_dist_per_tick(0)
{
    Reset( 0 );
}

//----------------------------------------
//
//----------------------------------------

void Navigator::Init( nvDistance wheel_diameter, nvDistance wheel_base, uint16_t ticks_per_rev )
{
    m_dist_per_tick = ticks_per_rev > 0 ? wheel_diameter*M_PI/ticks_per_rev : 0.0f;
    m_wheel_base = wheel_base > 1.0f ? wheel_base : 1.0f; 

}

//----------------------------------------
//
//----------------------------------------

void Navigator::Reset( uint32_t now )
{
    m_last_time = now;
    SetPosition( nvMM(0.0), nvMM(0.0));
    SetHeading( nvDEGREES(0.0) );
    SetSpeed( nvMM(0.0));
    SetTurnRate( nvDEGREES(0.0));
}

//----------------------------------------
//
//----------------------------------------

void Navigator::UpdateTicks( int16_t lticks, int16_t rticks, uint32_t now )
{


}

//----------------------------------------
//
//----------------------------------------


