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
:m_dist_per_tick(0),
 m_min_dt(nvMS(10))
{
	m_init_pose.position.x = 0.0f;
	m_init_pose.position.y = 0.0f;
	m_init_pose.heading = 0.0f;
	for (int i=0; i<nvNUM_ADJUSTS; i++)
	{
		m_dist_adjust[i] = m_heading_adjust[i] = 1.0f;
	}
	Reset(0); 
}

//----------------------------------------
//
//----------------------------------------

void Navigator::InitEncoder( nvDistance wheel_diameter, nvDistance wheel_base, uint16_t ticks_per_rev )
{
	m_wheel_diam = wheel_diameter;
	m_wheel_base = wheel_base;
	m_ticks_per_rev = ticks_per_rev;
}

//----------------------------------------
//
//----------------------------------------

void Navigator::Reset( nvTime now )
{
    m_last_ticks_time = now;
	m_dt = 0;
	m_lticks = m_rticks = 0;
	m_pose = m_init_pose;
	m_heading = nvDegToRad(m_pose.heading);
    m_speed = nvMM(0.0);
    m_turn_rate = nvDEGREES(0.0);
    m_dist_per_tick = m_ticks_per_rev > 0 ? (m_wheel_diam*M_PI/m_ticks_per_rev) : 0.0f;
	m_base_dist = m_wheel_base;
    m_base_dist = m_base_dist > 1.0f ? m_base_dist : 1.0f; 
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

	int type;
	// use ticks and time delta to update position
	if( m_lticks < 0 )
	{
		type = m_rticks < 0 ? nvADJUST_BACKWARD : nvADJUST_LEFT ;
	}
	else
	{
		type = m_rticks < 0 ? nvADJUST_RIGHT : nvADJUST_FORWARD ;
	}

	nvDistance dr = ((nvDistance)m_rticks)*m_dist_per_tick*m_heading_adjust[type];
	nvDistance dl = ((nvDistance)m_lticks)*m_dist_per_tick;
	nvDistance dd = (dr + dl)/m_dist_adjust[type]/2;

    // calc and update change in heading
    nvRadians dh = (dl - dr)/m_base_dist;
	m_heading = nvClipRadians( m_heading + dh);
	nvRadians xyh = nvClipRadians( m_heading + (type<=nvADJUST_BACKWARD ? dh/2 : dh));

	// update velocities
	m_speed = (dd*1000.0f)/m_dt;
    m_turn_rate = (nvRadToDeg(dh)*1000.0f)/m_dt;

    // update pose
	m_pose.heading = nvRadToDeg(m_heading);
	m_pose.position.x += dd*sin(xyh);
	m_pose.position.y += dd*cos(xyh);

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
	#if 0
	Serial.print("GetTo heading = ");
	   Serial.print(pos.x);
	   Serial.print(F(", "));
	   Serial.print(pos.y);
	   Serial.print(F(" from "));
	   Serial.print(m_pose.position.x);
	   Serial.print(F(", "));
	   Serial.print(m_pose.position.y);
	   Serial.print(F(" -> heading: "));
	   Serial.print(*heading);
	   Serial.print(F(" - dist: "));
	   Serial.println(*distance);
	#endif
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


