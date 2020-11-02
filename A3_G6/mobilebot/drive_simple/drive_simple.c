/*******************************************************************************
* drive_simple.c
*
* 
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <getopt.h>

#include <lcm/lcm.h>
#include "../lcmtypes/mbot_encoder_t.h"
#include "../lcmtypes/simple_motor_command_t.h"
#include "../common/mb_defs.h"

#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/motor.h>

//LCM
lcm_t * lcm;
#define MBOT_ENCODER_CHANNEL                "MBOT_ENCODERS"
#define MBOT_MOTOR_COMMAND_SIMPLE_CHANNEL   "MBOT_MOTOR_COMMAND_SIMPLE"

#define PI 3.14159

//global watchdog_timer to cut off motors if no lcm messages recieved
float watchdog_timer;

//functions
void publish_encoder_msg();
void print_answers();
void simple_motor_command_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                          const simple_motor_command_t *msg, void *user);

int mot_l_pol; //motor left polarity
int mot_r_pol;
int enc_l_pol;
int enc_r_pol; //encoder right polarity

//My input (with the current robot): 1 -1 -1 1

const double wheel_radius = 0.042; //meters
const double wheelbase = 0.11; //meters
const double gear_ratio = 62.5;

mbot_encoder_t last_msg;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char *argv[]){
    //check args
    if( argc != 5 ) {
        printf("Usage: test_simple {left motor polarity} {right motor polarity} {left encoder polarity} {right encoder polarity}\n");
        printf("Example: test_simple -1 1 -1 1\n");
        return 0;
    }
    
    mot_l_pol = atoi(argv[1]);
    mot_r_pol = atoi(argv[2]);
    enc_l_pol = atoi(argv[3]);
    enc_r_pol = atoi(argv[4]);

    if( ((mot_l_pol != 1)&(mot_l_pol != -1)) |
        ((mot_r_pol != 1)&(mot_r_pol != -1)) |
        ((enc_l_pol != 1)&(enc_l_pol != -1)) |
        ((enc_r_pol != 1)&(enc_r_pol != -1))){
        printf("Usage: polarities must be -1 or 1\n");
        return 0;
      }

	// make sure another instance isn't running
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
                fprintf(stderr,"ERROR: failed to start signal handler\n");
                return -1;
    }

	if(rc_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze motors\n");
        return -1;
    }

    lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	// rc_make_pid_file();

	// done initializing so set state to RUNNING
    rc_encoder_eqep_init();
	rc_set_state(RUNNING);

    simple_motor_command_t_subscribe(lcm, 
    							   MBOT_MOTOR_COMMAND_SIMPLE_CHANNEL, 
    							   simple_motor_command_handler,
    							   NULL);

    last_msg.utime = rc_nanos_since_epoch();
    last_msg.left_delta = 0;
    last_msg.right_delta = 0;
    last_msg.leftticks = enc_l_pol * rc_encoder_eqep_read(1);
    last_msg.rightticks = enc_r_pol * rc_encoder_eqep_read(2);
    
    watchdog_timer = 0.0;
    printf("Running...\n");
	while(rc_get_state()==RUNNING){
        watchdog_timer += 0.01;
        if(watchdog_timer >= 0.25)
        {
            rc_motor_set(1,0.0);
            rc_motor_set(2,0.0);
            printf("timeout...\r");
        }
        else {
            publish_encoder_msg();
        }
		// define a timeout (for erroring out) and the delay time
        lcm_handle_timeout(lcm, 1);
        rc_nanosleep(1E9 / 100); //handle at 10Hz
	}
    rc_motor_cleanup();
    rc_encoder_eqep_cleanup();
    lcm_destroy(lcm);
	// rc_remove_pid_file();   // remove pid file LAST

    print_answers();
	return 0;
}

/*******************************************************************************
*  simple_motor_command_handler()
*
*  sets motor PWMS from incoming lcm message
*
*******************************************************************************/
//////////////////////////////////////////////////////////////////////////////
/// TODO: Create a handler that receives lcm message simple_motor_command_t and
/// sets motor PWM according to the recieved message.
/// command the motor using the command: rc_motor_set(channel, polarity * pwm);
/// for now the pwm value should be proportional to the velocity you send, 
/// the sign of the velocity should indicate direction, and angular velocity 
//  indicates turning rate. 
//////////////////////////////////////////////////////////////////////////////
// void simple_motor_command_handler...
void simple_motor_command_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                          const simple_motor_command_t *msg, void *user) {
    watchdog_timer = 0.0;
    double v_r = msg->forward_velocity + (wheelbase * msg->angular_velocity / 2);
    double v_l = msg->forward_velocity - (wheelbase * msg->angular_velocity / 2);

    double pwm_r = v_r;
    if (pwm_r > 0.99)
        pwm_r = 0.99;
    double pwm_l = v_l;
    if (pwm_l > 0.99)
        pwm_l = 0.99;
    rc_motor_set(RIGHT_MOTOR, mot_r_pol*pwm_r);
    rc_motor_set(LEFT_MOTOR, mot_l_pol*pwm_l);
}

/*******************************************************************************
* void publish_encoder_msg()
*
* publishes LCM message of encoder reading
* 
*******************************************************************************/
void publish_encoder_msg(){
    //////////////////////////////////////////////////////////////////////////////
    /// TODO: update this fuction by calculating and printing the forward speeds(v) 
    ///     and angular speeds (w).
    //////////////////////////////////////////////////////////////////////////////
    mbot_encoder_t encoder_msg;
    encoder_msg.utime = rc_nanos_since_epoch();
    encoder_msg.leftticks = enc_l_pol * rc_encoder_eqep_read(1);
    encoder_msg.rightticks = enc_r_pol * rc_encoder_eqep_read(2);
    encoder_msg.left_delta = encoder_msg.leftticks - last_msg.leftticks;
    encoder_msg.right_delta = encoder_msg.rightticks - last_msg.rightticks;

    double delta_t = encoder_msg.utime - last_msg.utime;
    delta_t /= 1000000000; //seconds

    double forward_motion = (2 * PI * wheel_radius) / (20 * gear_ratio) / 2;
    forward_motion *= (encoder_msg.left_delta + encoder_msg.right_delta); //meters
    double velocity = forward_motion / delta_t; // m/s

    double angular_motion = (2 * PI * wheel_radius) / (20 * gear_ratio) / wheelbase;
    angular_motion *= (encoder_msg.right_delta - encoder_msg.left_delta); //radians
    double angular_velocity = angular_motion / delta_t; // radians/second

    printf(" ENC: %lld | %lld  - v: %f | w: %f \r", encoder_msg.leftticks, encoder_msg.rightticks, velocity,
        angular_velocity);
    mbot_encoder_t_publish(lcm, MBOT_ENCODER_CHANNEL, &encoder_msg);

    last_msg.leftticks = encoder_msg.leftticks;
    last_msg.rightticks = encoder_msg.rightticks;
    last_msg.utime = encoder_msg.utime;
}

void print_answers()
{
    /// Question 1:
    /// When you give your motors the same PWM command does the robot 
    /// move straight? Why not? What could be the reason for that? 
    /// How can we fix that?
    printf("\n");
    printf("Answer 1: \n When the robot is given the same PWM command, it does not quite drive straight. There could be a few reasons for this.  One is slight inconsistencies in the manufacturing process.  Another would be different levels of friction between the gear shafts and the wheels resulting in different applied forces, or different weights on each side of the robot having the same affect. Further, surfaces with more varied levels of frictions between places might snag up the wheels more or less, which could also lead to the wheels catching more or less for the same power input, which would change how much they rotate, and thus their forward speed.  If the wheels have different forward speeds, the vehicle will turn. The last cause I can easily think of is electrical interference in the system. \nAlso, I notice that the axels are not actually parallel to the bed, so different angles of lean could have an effect, too. \n There are two decent ways to fix the issue (that I can think of).  One would be to hardcode a conversion factor so that the two wheels rotate at equivalent speeds by changing the pwm signal of one to match.  The other would be to use a tuner to correct for drift by increasing or decreasing one of the pwm values. \n");

    /// Question 2:
    /// What could be some uses of logs in our project? why would we 
    /// want to play logs at different speeds?
    printf("Answer 2: \n Logs, in general, are useful.  In this project, logs could allow us to confirm that the motors are receiving the outputs that we are intending that our code give them (or prove otherwise, as may be the case) which is useful for debugging.  They can also be used to replay sensor data at a later point in time and simulate what our code would do under varying conditions.  (This would mostly be useful with the IMU data, since we could watch how the tuner responds.)  Camera logs can be used to train computer vision systems without needing to constantly run the robot, which is more efficient.  Further, if we know that the robot is behaving incorrectly, we can extract the logs and replay them to watch for where the output went wrong. \n We would want to play logs at different speeds for different cases.  If we are trying to watch a log in real time at a later date, we would want to be able to play at a normal frame.  If we are looking for specific blips, such as the output of a computer vision system at a particular moment in time, we would want to be able to slow the log down so that we do not miss the result we are trying to see.  On the other hand, if we have lots of log data and are just trying to see the average output of the log over time and look for major faults to key in on after, we would want to be able to play the log back at an increased speed so that we do not have to rewatch a massive data dump. \n");

}