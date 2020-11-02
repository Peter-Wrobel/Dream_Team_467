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
#include "../lcmtypes/mbot_imu_t.h"

#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/motor.h>
#include <rc/mpu.h>

#define M_PI 3.14159265358979323846

//LCM
lcm_t * lcm;
#define MBOT_ENCODER_CHANNEL                "MBOT_ENCODERS"
#define MBOT_IMU_CHANNEL                    "MBOT_IMU"
#define MBOT_MOTOR_COMMAND_SIMPLE_CHANNEL   "MBOT_MOTOR_COMMAND_SIMPLE"
#define LEFT_MOTOR              1     // id of left motor
#define RIGHT_MOTOR             2     // id of right motor

//global watchdog_timer to cut off motors if no lcm messages recieved
float watchdog_timer;

//functions
void publish_encoder_msg();
void simple_motor_command_handler(const lcm_recv_buf_t *rbuf, const char *channel,
    const simple_motor_command_t *msg, void *user);
void publish_imu_msg();

typedef struct mb_state mb_state_t;
struct mb_state{
    // raw sensor inputs
    float tb_angles[3]; // DMP filtered angles, tb_angles[3] is heading
    float accel[3]; // units of m/s^2
    float gyro[3];  // units of degrees/s
    float mag[3];   // units of uT
    float temp;     // units of degrees Celsius
    float last_yaw;
    
    int     left_encoder_delta;      // left encoder counts since last reading
    int     right_encoder_delta;     // right encoder counts since last reading

    uint64_t left_encoder_total;  //total encoder ticks since running
    uint64_t right_encoder_total;
    
    float fwd_velocity;
    float turn_velocity;
    float left_velocity;
    float right_velocity;

    float opti_x;               // Optitrack coordinates 
    float opti_y;               // (if using optitrack for ground truth)
    float opti_theta;           // Optitrack heading

    //outputs
    float   left_cmd;  //left wheel command [-1..1]
    float   right_cmd; //right wheel command [-1..1]

    //TODO: Add more variables to this state as needed
};

const double wheel_radius = 0.042; //meters
const double wheelbase = 0.11; //meters
float gear_ratio;
int mot_l_pol;
int mot_r_pol;
int enc_l_pol;
int enc_r_pol;

int64_t prev_utime = 0;
float cur_left_pwm = 0;
float cur_right_pwm = 0;

rc_mpu_data_t imu_data;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char *argv[]){
    //check args
    if( argc != 6 ) {
        printf("Usage: test_simple {gear ratio} {left motor polarity} {right motor polarity} {left encoder polarity} {right encoder polarity}\n");
        printf("Example: test_simple 78 -1 1 -1 1\n");
        return 0;
    }
    
    gear_ratio = atof(argv[1]);
    mot_l_pol = atoi(argv[2]);
    mot_r_pol = atoi(argv[3]);
    enc_l_pol = atoi(argv[4]);
    enc_r_pol = atoi(argv[5]);

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

    simple_motor_command_t_subscribe(lcm, 
    							   MBOT_MOTOR_COMMAND_SIMPLE_CHANNEL, 
    							   simple_motor_command_handler, 
    							   NULL);

    // set up IMU configuration
    // see RCL documentation for other parameters
    printf("initializing imu... \n");
    rc_mpu_config_t imu_config = rc_mpu_default_config();
    imu_config.dmp_sample_rate = 50;
    imu_config.dmp_fetch_accel_gyro = 1;
    //imu_config.dmp_interrupt_sched_policy = SCHED_FIFO;
    //imu_config.dmp_interrupt_priority = 90;

    if(rc_mpu_initialize_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU! Exiting.\n");
		return -1;
    }
	
	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&publish_imu_msg);

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	// rc_make_pid_file();

	// done initializing so set state to RUNNING
    rc_encoder_eqep_init();
	rc_set_state(RUNNING);

    // Set initial utime
    prev_utime = rc_nanos_since_epoch();
    
    watchdog_timer = 0.0;
    printf("Running...\n");
	while(rc_get_state()==RUNNING){
        watchdog_timer += 0.01;
        if (watchdog_timer >= 0.25)
        {
            rc_motor_set(1,0.0);
            rc_motor_set(2,0.0);
            printf("timeout...\r");
        }
        publish_encoder_msg();
        
		// define a timeout (for erroring out) and the delay time
        lcm_handle_timeout(lcm, 1);
        rc_nanosleep(1E9 / 100); //handle at 100Hz
	}
    rc_motor_cleanup();
    rc_encoder_eqep_cleanup();
    lcm_destroy(lcm);
	// rc_remove_pid_file();   // remove pid file LAST
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
void simple_motor_command_handler(const lcm_recv_buf_t *rbuf, const char *channel,
    const simple_motor_command_t *msg, void *user)
{
    watchdog_timer = 0;
	
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
    static int64_t prev_leftticks = 0;
    static int64_t prev_rightticks = 0;

    mbot_encoder_t encoder_msg;
    encoder_msg.utime = rc_nanos_since_epoch();
    encoder_msg.leftticks = enc_l_pol * rc_encoder_eqep_read(1);
    encoder_msg.rightticks = enc_r_pol * rc_encoder_eqep_read(2);
    encoder_msg.left_delta = encoder_msg.leftticks - prev_leftticks;
    encoder_msg.right_delta = encoder_msg.rightticks - prev_rightticks;

    prev_utime = encoder_msg.utime;
    prev_leftticks = encoder_msg.leftticks;
    prev_rightticks = encoder_msg.rightticks;

    // Lecture 1 slides explain the hardcoded values
    // Gear ratio of 78 manually confirmed
    float forward_velocity = ((encoder_msg.left_delta + encoder_msg.right_delta) / 2) * ((2 * M_PI * 0.042) / (20 * gear_ratio));
    float angular_velocity = ((encoder_msg.right_delta - encoder_msg.left_delta) / 0.11) * ((2 * M_PI * 0.042) / (20 * gear_ratio));

    printf(" ENC: %*lld | %*lld - v: %4.3f | w: %4.3f - l: %4.3f r: %4.3f \r",
        5, encoder_msg.leftticks, 5, encoder_msg.rightticks, forward_velocity, angular_velocity, cur_left_pwm, cur_right_pwm);
    mbot_encoder_t_publish(lcm, MBOT_ENCODER_CHANNEL, &encoder_msg);
}

void publish_imu_msg(){
    mb_state_t mb_state;
    // Read IMU
    mb_state.tb_angles[0] = imu_data.dmp_TaitBryan[TB_PITCH_X];
    mb_state.tb_angles[1] = imu_data.dmp_TaitBryan[TB_ROLL_Y];
    mb_state.last_yaw = mb_state.tb_angles[2];
    mb_state.tb_angles[2] = imu_data.dmp_TaitBryan[TB_YAW_Z];
    mb_state.temp = imu_data.temp;

    for(int i=0;i<3;i++){
        mb_state.accel[i] = imu_data.accel[i];
        mb_state.gyro[i] = imu_data.gyro[i];
        mb_state.mag[i] = imu_data.mag[i];
    }

    mbot_imu_t imu_msg;

    //Create IMU LCM Message
    imu_msg.utime = rc_nanos_since_epoch();
    imu_msg.temp = mb_state.temp;
    for(int i=0;i<3;i++){
        imu_msg.tb_angles[i] = mb_state.tb_angles[i];
        imu_msg.accel[i] = mb_state.accel[i];
        imu_msg.gyro[i] = mb_state.gyro[i];
    }

    //publish IMU Data to LCM
    mbot_imu_t_publish(lcm, MBOT_IMU_CHANNEL, &imu_msg);
}
