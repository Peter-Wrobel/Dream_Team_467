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

#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/motor.h>

//LCM
lcm_t * lcm;
#define MBOT_ENCODER_CHANNEL                "MBOT_ENCODERS"
#define MBOT_MOTOR_COMMAND_SIMPLE_CHANNEL   "MBOT_MOTOR_COMMAND_SIMPLE"
#define M_PI                                3.14159265358979323846

//global watchdog_timer to cut off motors if no lcm messages recieved
float watchdog_timer;

//functions
void publish_encoder_msg();
void print_answers();
void simple_motor_command_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
                                const simple_motor_command_t * msg, void * user);

int mot_l_pol;
int mot_r_pol;
int enc_l_pol;
int enc_r_pol;

int64_t last_rightticks = 0;
int64_t last_leftticks = 0;
int64_t last_time = 0;

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

    simple_motor_command_t_subscribe(lcm, "MBOT_MOTOR_COMMAND_SIMPLE", &simple_motor_command_handler, NULL);
    
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
void simple_motor_command_handler(const lcm_recv_buf_t *rbuf, const char * channel, 
                                const simple_motor_command_t * msg, void * user){
    watchdog_timer = 0;
    float v = msg->forward_velocity;
    float w = msg->angular_velocity;
    float right_pwm = v + 12*w / 2;
    float left_pwm = v - 12*w / 2;
    rc_motor_set(1, mot_l_pol * left_pwm);
    rc_motor_set(2, mot_r_pol * right_pwm);
    publish_encoder_msg();
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
    encoder_msg.left_delta = encoder_msg.leftticks - last_leftticks;
    encoder_msg.right_delta = encoder_msg.rightticks - last_rightticks;
    int64_t time_delta = encoder_msg.utime - last_time;
    last_leftticks = encoder_msg.leftticks;
    last_rightticks = encoder_msg.rightticks;
    last_time = encoder_msg.utime;

    float v = (((encoder_msg.left_delta + encoder_msg.right_delta)/2) * ((2*M_PI*0.042) / (20*78))) / time_delta * 1e9;
    float w = (((encoder_msg.right_delta - encoder_msg.left_delta) / 0.11) * ((2*M_PI*0.042) / (20*78))) / time_delta * 1e9;
    printf(" ENC: %lld | %lld  - v: %f | w: %f \r", encoder_msg.leftticks, encoder_msg.rightticks, v, w);
    mbot_encoder_t_publish(lcm, MBOT_ENCODER_CHANNEL, &encoder_msg);
}

void print_answers()
{
    /// Question 1:
    /// When you give your motors the same PWM command does the robot 
    /// move straight? Why not? What could be the reason for that? 
    /// How can we fix that?
    printf("Answer 1:\n No, the robot doesn't move straight when I give the motors the same PWM. It's because the frictional forces on the two wheels are not symmetrical, which make the robot deviate from the straight course. Also, since the two motors might not be completly identical, they might move at slightly different speed when we give the same PWM to them. One solution to fix that is to read the ticks from the encoder and try to adjust the speed of two motors. Another solution is to let the robot have a sense of its location, and get some feedback from the environment to adjust the motors. \n");

    /// Question 2:
    /// What could be some uses of logs in our project? why would we 
    /// want to play logs at different speeds?
    printf("Answer 2:\n In our project, logs can help us to replay and further investigate some interesting scenarios that might have caused unintended behavior so that we can identify the cause of some specific errors. Playing logs at different speeds enable us to speed up some unimportant parts and also to slow down the critical commands to help investigation. \n");

}
