#NavBot

## Development of an Autonomous Navigation Robot

![image](NavBot.jpg "NavBot")


##Overview

This project snapshots the development of an autonomous ground-based navigation robot.

The aim is to implement, experiment with and evaluate various techniques used to enable robots to independently navigate their environs.

The initial design is for robots using differential steering.

The ultimate goal is to create a collection of useful components that can be applied to real world applications.

The project is mainly targeted for 8 bit microcontrollers using C.

The current snapshots are for the Arduino platform but can easily be deployed to others as most of the code is device independent.

##The Snapshots

The project is broken up into subfolders containing snapshots at different stages of the development along with accompainging documentation.

Current snapshots are: 

 * [**NavBot_v1**](NavBot_v1) - Initial version of the **Navigator** and **Pilot** that utilises wheel encoders to perform simple dead reckoning navigation.
 
##Project Setup
 
The project snapshots are currently setup for the Arduino 1.05 IDE. Each subfolder contains an "ino" file with the same name as its containing folder. This is the main code file and contains the **setup()** and **loop()** entry points.
 
Files named **"<XXX>Bot.h"** contain robot specific code for a particular robot config. For example the **NavBot_v1** project has **BlackBot.h** which is the code belonging to the bot pictured above. Each folder also contains a **MyBot.h** that is a template available file for adding your own robot to the project. Each folder contains instructions on how to integrate your bot using the template.

The remaining **.cpp** and **.h** files are for various navigation systems, such as the **Navigator** and the **Pilot**.
 
 
##Planned Features/Support

 * Compass
 * Beacons/triangulation
 * Mapping
 * GPS
 * IMU/Inertia
 * Waypoints
 * Obstacle avoidance

##Further Reading

You can read more about this project on my journal: [Solder Spot NavBot Posts](http://solderspot.wordpress.com/tag/navbot/)
