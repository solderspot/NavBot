// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder 

#include "Navigator.h"
#include <math.h>
#include <Arduino.h>

//----------------------------------------
//
//----------------------------------------

Navigator::Navigator()
: m_min_dt(nvMS(10))
{
    m_init_pose.position.x = 0.0f;
    m_init_pose.position.y = 0.0f;
    m_init_pose.heading = 0.0f;
    m_dist_scaler = m_wheel_rl_scaler = m_wheelbase_scaler = 1.0f;
}

//----------------------------------------
//
//----------------------------------------

void Navigator::InitEncoder( nvDistance wheel_diameter, nvDistance wheelbase, uint16_t ticks_per_rev )
{
    m_nominal_wheel_diam = wheel_diameter;
    m_nominal_wheelbase = wheelbase;
    m_ticks_per_rev = ticks_per_rev;
    Reset(0);
}

//----------------------------------------
//
//----------------------------------------

void Navigator::Reset( nvTime now )
{
    // reset state
    m_last_ticks_time = now;
    m_dt = 0;
    m_lticks = m_rticks = 0;
    m_pose = m_init_pose;
    m_heading = nvDegToRad(m_pose.heading);
    m_speed = nvMM(0.0);
    m_turn_rate = nvDEGREES(0.0);

    // calc odomerty terms
    m_effective_wheelbase = m_nominal_wheelbase*m_wheelbase_scaler;
    m_effective_wheel_diam = m_nominal_wheel_diam*m_dist_scaler;
    float dist_per_tick = m_ticks_per_rev > 0 ? (m_effective_wheel_diam*M_PI/m_ticks_per_rev) : 0.0f;
    float invE = 1.0f/m_wheel_rl_scaler;
    m_rticks_to_dist = dist_per_tick*( 2.0f/(invE+1.0f));
    m_lticks_to_dist = dist_per_tick*( 2.0f/(m_wheel_rl_scaler + 1.0f));
}

//----------------------------------------
//
//----------------------------------------

bool Navigator::UpdateTicks( int16_t lticks, int16_t rticks, nvTime now )
{
    // update delta values
    m_dt +=  nvDeltaTime( m_last_ticks_time, now );
    m_lticks += lticks;
    m_rticks += rticks;

    // remember time for next call
    m_last_ticks_time = now;

    // see if we have accumulated min time delta 
    if ( m_dt < m_min_dt )
    {
        // no, so wait
        return false;
    }

    nvDistance sr = ((nvDistance)m_rticks)*m_rticks_to_dist;
    nvDistance sl = ((nvDistance)m_lticks)*m_lticks_to_dist;
    nvDistance s = (sr + sl)*0.5f;

    // calc and update change in heading
    nvRadians theta = (sl - sr)/m_effective_wheelbase;
    m_heading = nvClipRadians( m_heading + theta);

    // update velocities (per sec)
    m_speed = (s*1000.0f)/m_dt;
    m_turn_rate = (nvRadToDeg(theta)*1000.0f)/m_dt;

    // update pose
    m_pose.heading = nvRadToDeg(m_heading);
    m_pose.position.x += s*sin(m_heading);
    m_pose.position.y += s*cos(m_heading);

        // reset delta values
    m_dt = 0;
    m_lticks = 0;
    m_rticks = 0;

    return true;
}

//----------------------------------------
//
//----------------------------------------

nvPosition Navigator::NewPosition( nvDistance distance )
{
    nvPosition pos;

    pos.x = m_pose.position.x + distance*sin(m_heading);
    pos.y = m_pose.position.y + distance*cos(m_heading);

    return pos;
}

//----------------------------------------
//
//----------------------------------------

nvPosition Navigator::NewPosition( nvDistance x_offset, nvDistance y_offset )
{
    nvPosition pos;

    pos.x = m_pose.position.x + x_offset;
    pos.y = m_pose.position.y + y_offset;

    return pos;
}

//----------------------------------------
//
//----------------------------------------

nvPosition Navigator::NewPositionByHeading( nvPosition &pos, nvHeading heading, nvDistance distance )
{
    nvPosition new_pos;
    nvRadians h = nvDegToRad(nvClipHeading(heading));
    new_pos.x = pos.x + distance*sin(h);
    new_pos.y = pos.y + distance*cos(h);

    return new_pos;
}

//----------------------------------------
//
//----------------------------------------

void Navigator::GetTo( nvPosition &pos, nvHeading *heading, nvDistance *distance )
{

    nvDistance dx = pos.x - m_pose.position.x;
    nvDistance dy = pos.y - m_pose.position.y;

    *distance = sqrt( dx*dx + dy*dy );
    *heading = nvClipHeading(nvRadToDeg(atan2(dx, dy)+2*M_PI));
}

//----------------------------------------
//
//----------------------------------------

nvDegrees Navigator::HeadingAdjust( nvHeading target )
{
    nvDegrees adjust = nvClipDegrees(target - m_pose.heading);
    if (adjust < -180.0f)
    {
        return adjust + 360.0f;
    }
    else if ( adjust > 180.0f )
    {
        return adjust - 360.0f;
    }
    return adjust; 
}


