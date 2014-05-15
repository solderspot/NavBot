// Copyright (c) 2014, Solder Spot
// All rights reserved.
// See LICENSE.txt in root folder 

//----------------------------------------
// Defines
//----------------------------------------

#define LEFT            1
#define RIGHT           0
#define FORWARDS        1
#define BACKWARDS      -1
#define PTH_END         0
#define PTH_MOVE        1
#define PTH_TURN        2

#define USE_SERIAL    (MOTOR_INFO|CFG_TEST_ENCODERS \
                       |PLT_USE_SERIAL|BUTTON_INFO \
                       |NAV_INFO|TARGET_INFO \
                       |MEM_REPORT|PLT_USE_SERIAL \
                       |CFG_TEST_MOTORS)


//----------------------------------------
// Forward reference
//----------------------------------------

Pilot::TicksHandler ticks_handler;
Pilot::MotorHandler motor_handler;

//----------------------------------------
// Instance classes
//----------------------------------------

Navigator     navigator;
Pilot         pilot;

