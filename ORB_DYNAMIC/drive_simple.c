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
// #include "../lcmtypes/longitudinal_t.h"
#include "../lcmtypes/motion_vec_t.h"
#include "../lcmtypes/state_t.h"
// #include "../lcmtypes/bbb_state_t.h"
#include "../lcmtypes/mbot_encoder_t.h"

#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/motor.h>

//LCM
lcm_t * lcm;
#define MBOT_ENCODER_CHANNEL                "MBOT_ENCODERS"
#define MBOT_MOTOR_COMMAND_SIMPLE_CHANNEL   "MBOT_MOTOR_COMMAND_SIMPLE"
#define M_PI 3.14159265358979323846
#define SPEED 0.1
#define CLAMP 0.3

enum Mode {Explore = 1, Follow = 2};
// enum State {Forward, Backward, Right, Left};

//global watchdog_timer to cut off motors if no lcm messages recieved
float watchdog_timer;

//functions
void publish_encoder_msg();
void print_answers();

int mot_l_pol;
int mot_r_pol;
int enc_l_pol;
int enc_r_pol;

// setpoints
float theta_des = 0;
float v_des = 0;

float theta_curr = 0;
float v_curr = 0;

// float last_p_v_term = 0;
// float p_v_term = 0;
// float d_v_term = 0;

float l_pwm = 0;
float r_pwm = 0;

// Task II: w control
// float p_w_term = 0;
// float d_w_term = 0;

// Task IV: turn control
// float p_turn = 0;
// float d_turn = 0;

float vel = 0;
float ang = 0;

int64_t t_prev = 0;
int64_t t_prev_v = 0;
int64_t left_prev = 0;
int64_t right_prev = 0;

float obj_x = 0;
float obj_y = 0;
float obj_theta = 0;
float obj_vel = 0;
float dist = 0;
float selfx = 0;
float selfy = 0;

enum Mode mode;
enum Mode prev_mode;

bool stopped = true;
bool clear_ahead = true;

int64_t curr_left_enc = 0;
int64_t curr_right_enc = 0;
int64_t base_left_enc = 0;
int64_t base_right_enc = 0;

void state_handler(const lcm_recv_buf_t* rbuf,
                   const char* channel,
                   const state_t* msg,
                   void* user);
void motion_vec_handler(const lcm_recv_buf_t* rbuf,
                          const char* channel,
                          const motion_vec_t* msg,
                          void* user);
// void lateral_handler(const lcm_recv_buf_t* rbuf,
//                           const char* channel,
//                           const lateral_t* msg,
//                           void* user);
void calculate_setpoints();
void longitudinal_controller();
void stop_controller();
void lateral_controller();
void clamp_pwm();
void clear_encoder_history();

/*******************************************************************************
* int main() 
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
        fprintf(stderr,"ERROR: failed to initialize motors\n");
        return -1;
    }

    lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");

    // simple_motor_command_t_subscribe(lcm, "MBOT_MOTOR_COMMAND_SIMPLE", &simple_motor_command_handler, NULL);
    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	// rc_make_pid_file();

	// done initializing so set state to RUNNING
    rc_encoder_eqep_init();
    rc_set_state(RUNNING);
    
    watchdog_timer = 0.0;
    printf("Running...\n");
    state_t_subscribe(lcm, "STATE", &state_handler, NULL);
    longitudinal_t_subscribe(lcm, "LONGITUDINAL", &longitudinal_handler, NULL);
    lateral_t_subscribe(lcm, "LATERAL", &lateral_handler, NULL);
  
    while(rc_get_state()==RUNNING){
        watchdog_timer += 0.01;

        printf("STATE: %d", mode);
        // define a timeout (for erroring out) and the delay time
        if(watchdog_timer >= 1)
        {
            rc_motor_set(1,0.0);
            rc_motor_set(2,0.0);
            printf("timeout...\r");
        }
	
	if (mode != prev_mode){
        stopped = false;
        while(!stopped){
            stop_controller();

            rc_motor_set(1, l_pwm);
            rc_motor_set(2, r_pwm);

            rc_nanosleep(1E9 / 10);
        }
    }
    else if (mode == Explore) {
        if (clear_ahead){
            forward_controller();
        }
        else{
            right_controller();
        }
		// if (mode == Forward) {forward_controller()}
        // if (mode == Backward) {backward_controller()}
        // if (mode == Right) {right_controller()}
        // if (mode == Left) {left_controller()}
	}
	else if (mode == Follow) {
        update_self_odom();
        longitudinal_controller();
        lateral_controller();

        rc_motor_set(1, l_pwm);
	    rc_motor_set(2, r_pwm);
		// if (mode == Forward) {forward_controller()}
        // if (mode == Backward) {backward_controller()}
        // if (mode == Right) {right_controller()}
        // if (mode == Left) {left_controller()}
	}
	else {
        // no mode selected 
		rc_motor_set(1,0.0);
        rc_motor_set(2,0.0);
	}
	// rc_motor_set(2, l_pwm);
	// rc_motor_set(1, r_pwm);
	lcm_handle_timeout(lcm, 1);
	publish_encoder_msg();
	rc_nanosleep(1E9 / 10); //handle at 10Hz
		
	// last_p_v_term = p_v_term;
	float t = (float) (t_prev - t_prev_v) / 1E9;
	// p_v_term = v_goal - vel;
	// d_v_term = (p_v_term - last_p_v_term)/t;
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

void state_handler(const lcm_recv_buf_t* rbuf,
                   const char* channel,
                   const state_t* msg,
                   void* user) {
	watchdog_timer = 0.0;
    prev_mode = mode;    
	mode = msg->state;
}

void motion_vec_handler(const lcm_recv_buf_t* rbuf,
                        const char* channel,
                        const motion_vec_t* msg,
                        void* user) {
    watchdog_timer = 0.0;
    obj_x = msg->obj_x;
    obj_y = msg->obj_y;
    obj_theta = msg->obj_theta;
    obj_vel = msg->obj_vel;
	if(msg->dynamic_obj){
        mode = Follow;
        calculate_setpoints();
        clear_encoder_history();
    }
    else{
        mode = Explore;
    }
    base_left_enc = enc_l_pol * rc_encoder_eqep_read(1);
    base_right_enc = enc_r_pol * rc_encoder_eqep_read(2);
}

// void longitudinal_handler(const lcm_recv_buf_t* rbuf,
//                            const char* channel,
//                            const longitudinal_t* msg,
//                            void* user) {
// 	watchdog_timer = 0.0;
// 	p_w_term = msg->p_term;
// 	d_w_term = msg->d_term;
// }

// void lateral_handler(const lcm_recv_buf_t* rbuf,
//                            const char* channel,
//                            const lateral_t* msg,
//                            void* user) {
// 	watchdog_timer = 0.0;
// 	p_w_term = msg->p_term;
// 	d_w_term = msg->d_term;
// }

void forward(){
    rc_motor_set(1,SPEED;
    rc_motor_set(2,SPEED);
}

void backward(){
    rc_motor_set(1,-SPEED);
    rc_motor_set(2,-SPEED);
}

void right(){
    rc_motor_set(1,-SPEED);
    rc_motor_set(2,SPEED);
}
// right and left may be accidentally REVERSED
void left(){
    rc_motor_set(1,SPEED);
    rc_motor_set(2,-SPEED);
}

void calculate_setpoints(){
    //given our x,y,z and object x,y,z, we calculate the angle we need to face
    //we also calculate an L2 norm for (x1,y1) -> (x2,y2) and make the velocity
    //  setpoint proportional to the distance

    //convert theta to degrees
    obj_theta = obj_theta * (180/PI);

    //calc dist to object and apply scaling w/ offset to get vel command
    dist = sqrt(pow(obj_x,2)+pow(obj_y,2));
    
    //move these two vals to a better spot in the code
    float offset = -.5; //neg half meter offset to keep robot behind object
    float scale = 1/5; //20% scaling factor on meters to make it dist/5 m/s
    
    dist = (dist + offset) * scale;
}

void longitudinal_controller() {
	watchdog_timer = 0.0;
	float k_p = 0.5;
	// float k_d = 0; // 0.1;
    
    // printf("e: %f\n", p_v_term);
    // printf("e_dot: %f\n\n", d_v_term);
	// l_pwm = l_pwm + k_p*p_v_term + k_d*d_v_term;
	// r_pwm = r_pwm + k_p*p_v_term + k_d*d_v_term;
	float error = dist - covered_dist;
    l_pwm = k_p*error;
	r_pwm = k_p*error;

	// float zero_w = (l_pwm + r_pwm) / 2.;
	// l_pwm = zero_w + k_p_w*p_w_term + k_d_w*d_w_term;
	// r_pwm = zero_w - k_p_w*p_w_term - k_d_w*d_w_term;
	//l_pwm = l_pwm + k_p_w*p_w_term + k_d_w*d_w_term;
	//r_pwm = r_pwm - k_p_w*p_w_term - k_d_w*d_w_term;

	clamp_pwm();
    if(l_pwm < 0.05) l_pwm = 0;
    if(r_pwm < 0.05) r_pwm = 0;
}

void stop_controller() {
	watchdog_timer = 0.0;
	float zero_w = (l_pwm + r_pwm) / 2.;
	l_pwm = zero_w;
	r_pwm = zero_w;

    if(l_pwm < 0.1) l_pwm = 0;
    if(r_pwm < 0.1) r_pwm = 0;
    if(l_pwm < 0.1 && r_pwm < 0.1) stopped = true;
}

void lateral_controller() {
	watchdog_timer = 0.0;
	float k_p = 0.5;
	// float k_d = 0; // 0.1;
	// float k_p_w = 0.001;
	// float k_d_w = 0; //0.00001;
    // printf("e: %f\n", p_v_term);
    // printf("e_dot: %f\n\n", d_v_term);
	// l_pwm = l_pwm + k_p*p_v_term + k_d*d_v_term;
	// r_pwm = r_pwm + k_p*p_v_term + k_d*d_v_term;
	
    float error = obj_theta - self_angle;
    l_pwm = l_pwm + k_p*error;
    r_pwm = r_pwm - k_p*error;

	// float zero_w = (l_pwm + r_pwm) / 2.;
	// l_pwm = zero_w + k_p_w*p_w_term + k_d_w*d_w_term;
	// r_pwm = zero_w - k_p_w*p_w_term - k_d_w*d_w_term;
	// l_pwm = l_pwm + k_p_w*p_w_term + k_d_w*d_w_term;
	// r_pwm = r_pwm - k_p_w*p_w_term - k_d_w*d_w_term;

	clamp_pwm();
    if(l_pwm < 0.05) l_pwm = 0;
    if(r_pwm < 0.05) r_pwm = 0;
}

void clamp_pwm(){
    if(l_pwm > CLAMP) l_pwm = CLAMP;
	else if(l_pwm < -CLAMP) l_pwm = -CLAMP;
	if(r_pwm > CLAMP) r_pwm = CLAMP;
	else if(r_pwm < -CLAMP) r_pwm = -CLAMP;
}

void clear_encoder_history(){
    curr_left_enc = 0;
    curr_right_enc = 0;

}

void update_self_odom(){
    curr_left_enc = (enc_l_pol * rc_encoder_eqep_read(1)) - base_left_enc;
    curr_left_enc = curr_left_enc*(2*M_PI*0.042)/(20*78);
    curr_right_enc = (enc_r_pol * rc_encoder_eqep_read(2)) - base_right_enc;
    curr_right_enc = curr_right_enc*(2*M_PI*0.042)/(20*78);

    covered_dist = min(curr_left_enc, curr_right_enc);
    self_angle = ((curr_left_enc - curr_right_enc)*(2*M_PI*0.042)/(0.11*20*78)) % 360;
}

int64_t min(int64_t left, int64_t right){
    if(abs(left) < abs(right)){
        return left;
    }
    else{
        return right;
    }
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
    t_prev_v = t_prev;

    int64_t curr_left = enc_l_pol * rc_encoder_eqep_read(1);
    int64_t curr_right = enc_r_pol * rc_encoder_eqep_read(2);
   
    encoder_msg.left_delta = curr_left - left_prev;
    encoder_msg.right_delta = curr_right - right_prev;
    encoder_msg.leftticks = curr_left;
    encoder_msg.rightticks = curr_right;
    
    float t_passed = (float) (encoder_msg.utime - t_prev) / 1E9;

    vel = (encoder_msg.left_delta+encoder_msg.right_delta)*(2*M_PI*0.042)/(2.0*20*78*t_passed);
    ang = (-encoder_msg.left_delta+encoder_msg.right_delta)*(2*M_PI*0.042)/(0.11*20*78*t_passed);
    printf(" ENC: %lld | %lld  - v: %0.3f | w: %0.3f | p-term: %0.3f | l-pwm: %0.3f | r-pwm: %0.3f |- t_pass: %0.3f | t: %lld\n", encoder_msg.leftticks, encoder_msg.rightticks, vel, ang, p_w_term, l_pwm, r_pwm, t_passed, encoder_msg.utime);

    t_prev = encoder_msg.utime;
    left_prev = curr_left;
    right_prev = curr_right;

    mbot_encoder_t_publish(lcm, MBOT_ENCODER_CHANNEL, &encoder_msg);
}

