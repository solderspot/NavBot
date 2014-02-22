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
m_lmotor(0),
m_rmotor(0)
{
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::Reset( void )
{
	// ensure we are at a complete stop
	Stop();
	while ( !IsDone() )
	{
		Service();
	}
	m_last_time = getTime();
	m_nav->SetMinInterval( m_min_update_interval / 2);
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::Service( void )
{
	nvTime  now  = getTime();
	nvTime  dt  = now - m_last_time;

	if ( dt < m_min_update_interval )
	{
		return;
	}


	// first service the Navigator

	int16_t lticks;
	int16_t rticks;

	if (!m_ticks_handler( this,  &lticks, &rticks ))
    {
		return;
    }

	m_nav->UpdateTicks( lticks, rticks, now );


	switch ( m_task )
	{
		case PLT_TASK_STOP:
		Serial.println("stop");
		m_task = PLT_TASK_DONE;
		break;
	}
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::Stop( void )
{
	m_task = PLT_TASK_STOP;
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
	m_task = PLT_TASK_TURN;
	m_target_heading = m_nav->Heading() + degrees;
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


