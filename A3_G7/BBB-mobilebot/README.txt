/*******************************************************************************
*                 Mobilebot Template Code for MBot/MBot-Mini
*                           pgaskell@umich.edu
*       
*    This code is a template for completing a fully featured
*    mobile robot controller
*   
*    Functions that need completing are marked with "TODO:"
*
*******************************************************************************/

bin/			      : Binaries folder
mobilebot/mobilebot.c/.h      : Main setup and threads
test_motors/test_motors.c/.h  : Program to test motor implementation
meas..params/meas..params.c/.h: Program to measure motor parameters
common/mb_controller.c/.h     : Contoller for manual and autonomous nav
common/mb_defs.h              : Define hardware config
common/mb_odometry.c/.h	      : Odometry and dead reckoning 
lcmtypes/                     : lcmtypes for Mobilebot
java/                         : java build folder for lcmtypes for lcm-spy
setenv.sh                     : sets up java PATH variables for lcm-spy (run with: source setenv.sh)

*******************************************************************************************************
Instructions:
The mobilebot.c is modified to be able to receive the <simple_motor_command_t> lcm messages sent from RPi and publish the <mbot_imu_t> lcm messages to send imu data.

You need to run "make" as you did in A0.

Before running the mobilebot, you should first calibrate the imu data. Go to mobilebot/bin and run:
./calibrate_accel
./calibrate_gyro

Then run (change the arguments as you did for drive_simple):
sudo ./mobilebot 1 -1 1 -1

Now if you send <simple_motor_command_t> lcm messages on RPi, then you will be able to control the robot by keyboards. If you get the lcm listener code done on PC, then you should receive <mbot_imu_t> lcm messages there.
