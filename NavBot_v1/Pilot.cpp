// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder 

#include "Pilot.h"

#include <Arduino.h>

//----------------------------------------
//
//----------------------------------------

Pilot::Pilot()
: m_nav(NULL),
m_time_func(NULL),
m_ticks_handler(NULL),
m_ticks_handler_data(NULL),
m_motor_handler(NULL),
m_motor_handler_data(NULL),
m_max_move_speed(20.0f),
m_max_turn_speed(30.0f),
m_min_update_interval(100.0f),
m_task(PLT_TASK_DONE),
m_state(PLT_STATE_STOPPED),
m_mpower(0),
m_ladjust(0),
m_radjust(0),
m_mp_start(0),
m_mp_stop(0),
m_was_in_motion(false)
{
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::Reset( void )
{
	// ensure we are at a complete stop
	//Serial.println(F("Pilot::Reset"));
	Stop();
	while ( !IsDone() )
	{
		Service();
	}
	m_last_time = getTime();
	m_nav->SetMinInterval( m_min_update_interval / 2);
	m_nav->Reset(m_last_time);
	m_target_heading = m_nav->Heading();
	m_target_pos = m_nav->Position();
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::Service( void )
{
	nvTime  now  = getTime();
	m_dt  = now - m_last_time;

	if ( m_dt < m_min_update_interval )
	{
		return;
	}
	//Serial.println(F("Pilot::Service"));

	// first service the Navigator
	{
		int16_t lticks;
		int16_t rticks;

		if (!m_ticks_handler( this,  &lticks, &rticks ))
		{
			return;
		}

		m_nav->UpdateTicks( lticks, rticks, now );
	}

	if ( !m_was_in_motion && m_nav->InMotion())
	{
		// we just started moving
		m_mp_start - m_mpower;
		m_was_in_motion = true;
	}

	if ( m_state == PLT_STATE_STOPPING)
	{
		// stopping is a special state. we need to ignore
		// any tasks until stopping has completed
		if ( m_nav->InMotion())
		{
			adjust_mpower( -10 );
			m_mp_stop = m_mpower;
			// need to wait
			return;
		}
		// we have actually stopped.
		m_mpower = m_ladjust = m_radjust = 0;
		update_motors();
		m_was_in_motion = false;
		m_state = PLT_STATE_STOPPED;
		if ( m_end_task_on_stop )
		{
			m_task = PLT_TASK_DONE; 
		}
	}

	switch ( m_task )
	{
		default:
		case PLT_TASK_DONE: 
		{
			// nothing to do
			return;
		}
		case PLT_TASK_TURN:
		{
			if ( m_state == PLT_STATE_MOVING )
			{
				// need to stop first
				full_stop();
				return;
			}

			m_state = PLT_STATE_TURNING;
			break;
		}
		case PLT_TASK_MOVE:
		{
			// calculate the distance and heading to target
			m_nav->GetTo( m_target_pos, &m_target_heading, &m_target_dist);

			nvDegrees dh = m_target_heading - m_nav->Heading();

			if ( abs(dh) > 5.0f )
			{
				// we need to turn the bot first

				if ( m_state == PLT_STATE_MOVING )
				{
					// need to stop first
					full_stop();
					return;
				}
				m_state = PLT_STATE_TURNING;
			}
			else if ( m_state == PLT_STATE_STOPPED )
			{
				m_state = PLT_STATE_MOVING;
			}
			break;
		}
	}

	switch ( m_state )
	{
		case PLT_STATE_TURNING:
		{
			update_turn();
			break;
		}
		case PLT_STATE_MOVING:
		{
			update_move();
			break;
		}
		default: break;
	}


}

//----------------------------------------
//
//----------------------------------------

void Pilot::update_turn( void )
{
	nvDegrees dh = m_nav->HeadingAdjust( m_target_heading );
	//Serial.print(F("Pilot::update_turn - dh = "));
	//Serial.println( dh );
	nvRate turning = m_nav->TurnRate();
	int16_t dir = dh < 0.0f ? -1 : 1;
	nvDegrees adh = dh < 0.0f ? dh*-1.0f : dh;

	if ( adh < 2.0f )
	{
		full_stop();
		if ( m_task == PLT_TASK_TURN )
		{
			m_end_task_on_stop = true;
		}
		return;
	}

	// if the motors are not turn in the direction
	// we want then we need to slow them down first

	if ( m_mpower && (m_ldir != dir || m_rdir != -dir) )
	{
		adjust_mpower( -10 );
		return;
	}

	m_ldir = dir;
	m_rdir = -dir;

	if ( turning )
	{
		nvRate max_rate = adh < 20.0f ? nvDEGREES(20) : adh < 40.0f ? nvDEGREES(30) : nvDEGREES( 45);
		m_tPID.SetTarget( max_rate );
		float adjust = m_tPID.CalcAdjustment( turning, m_dt );
		adjust_mpower( (int16_t) adjust );
	}
	else
	{
		// start the turn
		adjust_mpower( m_mpower == 0 && m_mp_start ? m_mp_start : 10 ); 
	}
}

//----------------------------------------
//
//----------------------------------------

void Pilot::update_move( void )
{

}

//----------------------------------------
//
//----------------------------------------

void    Pilot::full_stop( void )
{
	adjust_mpower( -10 );
	m_state = PLT_STATE_STOPPING;
	m_end_task_on_stop = false;
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::Stop( void )
{
	// stop is a special action that
	// immediately overrides everything else
	full_stop();
	m_task = PLT_TASK_STOP;
	m_end_task_on_stop = true;
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::Move( nvDistance distance )
{
	m_task = PLT_TASK_MOVE;
	m_target_pos = m_nav->NewPosition( distance );
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::Turn( nvHeading degrees )
{
	//Serial.print(F("Pilot::Turn("));
	//Serial.print(degrees);
	//Serial.print(F(") -> "));
	m_task = PLT_TASK_TURN;
	m_target_heading = nvClipHeading( m_target_heading + degrees);
	//Serial.println(m_target_heading);
}

//----------------------------------------
//
//----------------------------------------

void Pilot::update_motors( void )
{
	int16_t l = m_mpower + (m_mpower*m_ladjust)/100L;
	int16_t r = m_mpower + (m_mpower*m_radjust)/100L;
	l = (l > 1024) ? 1024 : (l < 0 ? 0 : l); 
	r = (r > 1024) ? 1024 : (r < 0 ? 0 : r); 

	m_motor_handler( this, l*m_ldir, r*m_rdir );
}


//----------------------------------------
//
//----------------------------------------

void Pilot::adjust_mpower( int16_t delta )
{
	m_mpower += delta;
	m_mpower = (m_mpower > 1024) ? 1024 : (m_mpower < 0 ? 0 : m_mpower); 
	update_motors();
}


//----------------------------------------
// PID Code
//----------------------------------------

Pilot::PIDController::PIDController()
: Kp(0.0f),
  Ki(0.0f),
  Kd(0.0f)
{
	Reset();
}

//----------------------------------------
//
//----------------------------------------

void Pilot::PIDController::Reset( void )
{
	sumErrs = 0.0f;
	lastErr = 0.0f;
}

//----------------------------------------
//
//----------------------------------------

float Pilot::PIDController::CalcAdjustment( float err, nvTime dt )
{
	float derr = ((err - lastErr)*minDelta)/dt;
	sumErrs += err;
	return Kp*err + Ki*sumErrs +Kd*derr;
}


