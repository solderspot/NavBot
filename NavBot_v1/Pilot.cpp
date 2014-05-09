// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder 

#include "Pilot.h"

#include <Arduino.h>

#define PLT_GRAPH_PID (PLT_GRAPH_HEADING_PID|PLT_GRAPH_SPEED_PID|PLT_GRAPH_WHEEL_PID)

//----------------------------------------
//
//----------------------------------------

#if PLT_DEBUG_STATE
const char *state_name[Pilot::PLT_NUM_STATES] = 
{
    "STOPPED",
    "TURNING",
    "MOVING",
    "STOPPING",
    "SPINNING"
};
#endif

#if PLT_DEBUG_TASK
const char *task_name[Pilot::PLT_NUM_TASKS] = 
{
    "NONE",
    "DONE",
    "TURN",
    "MOVE",
    "STOP",
    "SPIN"
};
#endif

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
#if PLT_SHOW_TASK
m_last_task(PLT_TASK_DONE),
#endif
#if PLT_DEBUG_STATE
m_last_state(PLT_STATE_STOPPED),
#endif
m_encoder_errors(0),
#if PLT_DEBUG_ENCODER
m_last_encoder_errors(0),
#endif
m_target_radius(nvMM(10)),
m_mpower(0),
m_ladjust(0),
m_radjust(0),
m_mp_start(0),
m_mp_stop(0),
m_drticks(0),
m_dlticks(0),
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
    m_nav->Reset(m_last_time);
    m_target_heading = m_nav->Heading();
    m_target_pos = m_nav->Position();
    m_encoder_errors = 0;
    m_dlticks = m_drticks = 0;
    m_hPID.Reset(m_min_update_interval);
    m_wPID.Reset(m_min_update_interval);
    m_sPID.Reset(m_min_update_interval);
}

//----------------------------------------
//
//----------------------------------------

#if PLT_GRAPH_PID
static void pid_graph_header( void )
{
    Serial.println (F("err, sum, output"));
}

static void pid_graph_data( float err, float sum, float output)
{
    Serial.print(err);
    Serial.print(F(", "));
    Serial.print(sum);
    Serial.print(F(", "));
    Serial.println(output);
}
#endif

//----------------------------------------
//
//----------------------------------------

void    Pilot::Service( void )
{
    nvTime  now  = getTime();
    m_dt  = now - m_last_time;

    // first service the Navigator
    {
        int16_t lticks = 0;
        int16_t rticks = 0;

        if( !m_ticks_handler( this,  &lticks, &rticks ))
        {
            m_encoder_errors++;
            #if PLT_SHOW_ERRORS 
            Serial.println(F("Pilot: Encoder error"));
            #endif
        }

        m_nav->UpdateTicks( lticks, rticks, now );

        m_dlticks += lticks;
        m_drticks += rticks;
    }


    if ( m_dt < m_min_update_interval )
    {
        return;
    }

    m_last_time = now;
    m_lticks = m_dlticks;
    m_rticks = m_drticks;
    m_dlticks = m_drticks = 0;

    //Serial.print(F("Pilot::Service - m_dt = "));
    //Serial.println(m_dt);

    #if PLT_OUTPUT_DEBUG
        output_debug();
    #endif

    if ( !m_was_in_motion && m_nav->InMotion())
    {
        // we just started moving
        m_mp_start = m_mpower;
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
            if ( m_state != PLT_STATE_TURNING)
            {
                #if PLT_GRAPH_PID
                    pid_graph_header();
                #endif
                m_state = PLT_STATE_TURNING; 
            }
            break;
        }
        case PLT_TASK_SPIN:
        {
            if ( m_state == PLT_STATE_MOVING )
            {
                // need to stop first
                full_stop();
                return;
            }
            if ( m_state != PLT_STATE_SPINNING)
            {
                #if PLT_GRAPH_WHEEL_PID
                pid_graph_header();
                #endif
                m_wPID.Reset(m_min_update_interval);
                m_state = PLT_STATE_SPINNING; 
                m_spin_last_heading = m_nav->Heading();
            }
            break;
        }
        case PLT_TASK_MOVE:
        {
            // calculate the distance and heading to target
            m_nav->GetTo( m_target_pos, &m_move_heading, &m_target_dist);
            nvDegrees dh = m_nav->HeadingAdjust( m_move_heading );
            nvPosition pos = m_nav->Position();

            if ( abs(dh) > 10.0f && abs(m_target_dist) > m_target_radius )
            {
                // we need to turn the bot first
                //Serial.print("dh = ");
                //Serial.println(dh);

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
                #if PLT_GRAPH_PID
                    pid_graph_header();
                #endif
                m_state = PLT_STATE_MOVING;
            }
            break;
        }
    }

    switch ( m_state )
    {
        case PLT_STATE_TURNING:
        {
            update_turn( m_nav->HeadingAdjust( m_task == PLT_TASK_MOVE ? m_move_heading : m_target_heading ));
            break;
        }
        case PLT_STATE_SPINNING:
        {
            nvHeading change = m_nav->HeadingAdjust(m_spin_last_heading);
            m_total_spin +=  change ;
            m_spin_last_heading = m_nav->Heading();

            update_turn( m_total_spin );
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

void Pilot::update_turn( nvDegrees dh )
{
    
    #if PLT_TURN_INFO
        Serial.print(F("Pilot::update_turn - dh = "));
        Serial.println( dh );
    #endif
    nvRate turning = abs(m_nav->TurnRate());
    int16_t dir = dh < 0.0f ? -1 : 1;
    nvDegrees adh = dh < 0.0f ? dh*-1.0f : dh;

    if ( adh < 2.0f )
    {
        full_stop();
        if ( m_task == PLT_TASK_TURN || m_task == PLT_TASK_SPIN )
        {
            m_end_task_on_stop = true;
        }
        return;
    }

    // if the motors are not turning in the direction
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
        // adjust for difference in wheel ticks
        int16_t lt = m_lticks < 0 ? -m_lticks : m_lticks;
        int16_t rt = m_rticks < 0 ? -m_rticks : m_rticks;
        int16_t err = rt-lt;
        int16_t tadjust = m_wPID.CalcAdjustment( err, m_dt );

        #if PLT_GRAPH_WHEEL_PID
            pid_graph_data( err, m_wPID.sumErrs, tadjust);
        #endif


        m_ladjust += tadjust;
        m_radjust -= tadjust;

        // adjust turn speed
        nvRate max_rate = adh < 20.0f ? nvDEGREES(20) : adh < 40.0f ? nvDEGREES(30) : m_max_turn_speed;
        max_rate = max_rate > m_max_turn_speed ? m_max_turn_speed : max_rate;

        int16_t adjust = m_sPID.CalcAdjustment( max_rate - turning, m_dt); 

        adjust = adjust > 100 ? 100 : adjust < -100 ? -100 : adjust;

        adjust_mpower( adjust );


        #if PLT_SHOW_TURN_ADJUST
        Serial.print(F("turn: err = "));
        Serial.print(err);
        Serial.print(F(" tadjust = "));
        Serial.print(tadjust);
        Serial.print(F(" m_ladjust = "));
        Serial.print(m_ladjust);
        Serial.print(F(" m_radjust = "));
        Serial.println(m_radjust);
        #endif
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
    nvDegrees dh = m_nav->HeadingAdjust( m_move_heading );
    #if PLT_MOVE_INFO
    Serial.print(F("Pilot::update_move - dh = "));
    Serial.print( dh );
    Serial.print(F(" dist = "));
    Serial.println( m_target_dist );
    #endif
    nvRate speed = m_nav->Speed();
    int16_t dir = m_target_dist < 0.0f ? -1 : 1;
    nvDistance dist = abs(m_target_dist);

    if ( dist < m_target_radius/2.0f )
    {
        full_stop();
        if ( m_task == PLT_TASK_MOVE )
        {
            m_end_task_on_stop = true;
        }
        return;
    }

    // if the motors are not turning in the direction
    // we want then we need to slow them down first

    if ( m_mpower && (m_ldir != dir || m_rdir != dir) )
    {
        adjust_mpower( -10 );
        return;
    }

    m_ldir = dir;
    m_rdir = dir;

    if ( speed )
    {
        {
            // adjust heading
            float hadj = m_hPID.CalcAdjustment( dh, m_dt);
            m_ladjust = (int16_t)hadj;
            m_radjust = -(int16_t)hadj;
            #if PLT_GRAPH_HEADING_PID
                pid_graph_data( dh, m_hPID.sumErrs, hadj);
            #endif
        }

        #if PLT_SHOW_HEADING_ADJUST
        Serial.print(F("move: dh = "));
        Serial.print(dh);
        Serial.print(F(" hadjust = "));
        Serial.print(hadj);
        Serial.print(F(" m_ladjust = "));
        Serial.print(m_ladjust);
        Serial.print(F(" m_radjust = "));
        Serial.println(m_radjust);
        #endif

        // adjust speed
        nvRate max_rate = dist < 30.0f ? nvMM(10) : dist < 50.0f ? nvMM(40) : m_max_move_speed ;
        max_rate = max_rate > m_max_move_speed ? m_max_move_speed : max_rate;

        int16_t adjust = m_sPID.CalcAdjustment( max_rate - speed, m_dt); 

        adjust = adjust > 100 ? 100 : adjust < -100 ? -100 : adjust;

        adjust_mpower( adjust );
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

void    Pilot::MoveBy( nvDistance distance )
{
    m_task = PLT_TASK_MOVE;
    m_target_pos = m_nav->NewPositionByHeading(m_target_pos, m_target_heading, distance); 
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::MoveTo( nvPosition &pos )
{
    m_task = PLT_TASK_MOVE;
    m_target_pos = pos;
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::TurnBy( nvDegrees degrees )
{
    m_task = PLT_TASK_TURN;
    m_target_heading = nvClipHeading(m_target_heading + degrees); 
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::SpinBy( nvDegrees degrees )
{
    m_task = PLT_TASK_SPIN;
    m_target_heading = nvClipHeading( m_target_heading + degrees);
    m_total_spin = degrees;
}

//----------------------------------------
//
//----------------------------------------

void    Pilot::TurnTo( nvHeading heading )
{
    m_task = PLT_TASK_TURN;
    m_target_heading = nvClipHeading( heading );
}

//----------------------------------------
//
//----------------------------------------

#define MAX_M_ADJUST        20

void Pilot::update_motors( void )
{
    m_ladjust =  m_ladjust > MAX_M_ADJUST ? MAX_M_ADJUST : m_ladjust < -MAX_M_ADJUST ? -MAX_M_ADJUST : m_ladjust; 
    m_radjust =  m_radjust > MAX_M_ADJUST ? MAX_M_ADJUST : m_radjust < -MAX_M_ADJUST ? -MAX_M_ADJUST : m_radjust; 
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
//
//----------------------------------------

#if PLT_OUTPUT_DEBUG
void Pilot::output_debug( void )
{
    #if PLT_DEBUG_STATE
    if ( m_state != m_last_state )
    {
        Serial.print(F("Pilot: state "));
        Serial.print(state_name[m_last_state]);
        Serial.print(F(" -> "));
        Serial.println(state_name[m_state]);
        m_last_state = m_state;
    }
    #endif

    #if PLT_DEBUG_TASK
    if ( m_task != m_last_task )
    {
        Serial.print(F("Pilot: TASK - "));
        Serial.print(task_name[m_last_task]);
        Serial.print(F(" -> "));
        Serial.println(task_name[m_task]);
        m_last_task = m_task;
    }
    #endif

    #if PLT_DEBUG_ENCODER
    if ( m_encoder_errors != m_last_encoder_errors )
    {
        Serial.print(F("Pilot: encoder errors "));
        Serial.print(m_encoder_errors);
        m_last_encoder_errors = m_encoder_errors;
    }
    #endif
}
#endif

//----------------------------------------
// PID Code
//----------------------------------------

Pilot::PIDController::PIDController()
: Kp(0.0f),
  Ki(0.0f),
  Kd(0.0f)
{
}

//----------------------------------------
//
//----------------------------------------

void Pilot::PIDController::Reset( nvTime mindt)
{
    sumErrs = 0.0f;
    lastErr = 0.0f;
    minDelta = mindt;
}

//----------------------------------------
//
//----------------------------------------

float Pilot::PIDController::CalcAdjustment( float err, nvTime dt )
{
    float derr = ((err - lastErr)*(float)minDelta)/(float)dt;
    sumErrs += err;
    lastErr = err;

    float result = Kp*err + Ki*sumErrs +Kd*derr;

    return result;
}


