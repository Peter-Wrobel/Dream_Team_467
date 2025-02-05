/*******************************************************************************
*    mobilebot.c
*    Template Code
*    
*    This code is a template for completing a fully featured
*    mobile robot controller
*   
*    Functions that need completing are marked with "TODO:"
* 
*******************************************************************************/
#include "mobilebot.h"

#define MBOT_PWM_CHANNEL "MBOT_MOTOR_PWM"

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int mot_l_pol;
int mot_r_pol;
int enc_l_pol;
int enc_r_pol;
int main(int argc, char* argv[]){
    
    //check args
    if( argc != 5 ) {
        printf("Usage: test_simple  {left motor polarity} {right motor polarity} {left encoder polarity} {right encoder polarity}\n");
        printf("Example: test_simple  -1 1 -1 1\n");
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

    rc_led_set(RC_LED_GREEN, LED_OFF);
    rc_led_set(RC_LED_RED, LED_ON);
	//set cpu freq to max performance
	rc_cpu_set_governor(RC_GOV_PERFORMANCE);
    // start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
        fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	// start lcm handle thread
	printf("starting lcm thread... \n");
	lcm = lcm_create(LCM_ADDRESS);
	pthread_t lcm_subscribe_thread;
    rc_pthread_create(&lcm_subscribe_thread, lcm_subscribe_loop, (void*) NULL, SCHED_FIFO, LCM_PRIORITY);

	// start control thread
	printf("starting dsm_radio thread... \n");
	pthread_t  dsm_radio_thread;
	rc_pthread_create(&dsm_radio_thread, dsm_radio_control_loop, (void*) NULL, SCHED_OTHER, 0);

    // start printf_thread 
    printf("starting print thread... \n");
    pthread_t  printf_thread;
    rc_pthread_create(&printf_thread, printf_loop, (void*) NULL, SCHED_OTHER, 0);
    
    //wait for threads to set up
    rc_nanosleep(1E5);

	// set up IMU configuration
    // see RCL documentation for other parameters
	printf("initializing imu... \n");
	rc_mpu_config_t imu_config = rc_mpu_default_config();

	imu_config.i2c_bus = 2;
	if(rc_mpu_calibrate_gyro_routine(imu_config) < 0) {
		printf("Failed calibrate\n");
		return -1;
	}

	imu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
    imu_config.dmp_fetch_accel_gyro=1;
    imu_config.dmp_interrupt_sched_policy = SCHED_FIFO;
    imu_config.dmp_interrupt_priority = CONTROLLER_PRIORITY;

	if(rc_mpu_initialize_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU! Exiting.\n");
		return -1;
	}

	//initialize state mutex
    pthread_mutex_init(&state_mutex, NULL);

	//attach controller function to IMU interrupt
	printf("initializing controller...\n");
	mb_initialize_controller();

	printf("initializing motors...\n");
#if defined(MRC_VERSION_1v3) || defined(MRC_VERSION_2v1)
	mb_motor_init();
    mb_motor_brake(1);
    mb_motor_set(1,0);
    mb_motor_set(2,0);
#endif

#if defined(BEAGLEBONE_BLUE)
    rc_motor_init_freq(DEFAULT_PWM_FREQ);
    rc_motor_standby(0);
    rc_motor_set(1,0);
    rc_motor_set(2,0);
#endif

	printf("initializing odometry...\n");
    rc_encoder_init();
    rc_encoder_write(1, 0);
    rc_encoder_write(2, 0);
	mb_initialize_odometry(&mb_odometry, 0.0,0.0,0.0);

	printf("attaching imu interupt...\n");
	rc_mpu_set_dmp_callback(&mobilebot_controller);

	printf("we are running!!!\n");
	// done initializing so set state to RUNNING
	rc_set_state(RUNNING); 
    rc_led_set(RC_LED_RED, LED_OFF);

	// Keep looping until state changes to EXITING
	while(rc_get_state()!=EXITING){
		// other functions are handled in other threads
		// there is no need to do anything here but sleep
        led_heartbeat();
        rc_nanosleep(7E8);
	}
	rc_led_set(RC_LED_RED, LED_ON);
	// exit cleanly
    rc_pthread_timed_join(lcm_subscribe_thread, NULL, 1.5);
    rc_pthread_timed_join(printf_thread, NULL, 1.5);
    rc_pthread_timed_join(dsm_radio_thread, NULL, 1.5);
    rc_led_set(RC_LED_GREEN, LED_OFF);
    rc_led_set(RC_LED_RED, LED_OFF);
    rc_mpu_power_off();
#if defined(MRC_VERSION_1v3) || defined(MRC_VERSION_2v1)
    mb_motor_cleanup();
#endif
#if defined(BEAGLEBONE_BLUE)
    rc_motor_cleanup();
#endif
    rc_encoder_cleanup();
    rc_remove_pid_file();
	return 0;
}

/*******************************************************************************
* void read_mb_sensors()
*
* Reads all the sensors on the mobilebot
* TODO: modify this function to read other sensors
* 
*******************************************************************************/
void read_mb_sensors(){
    pthread_mutex_lock(&state_mutex); 
    // Read IMU
    mb_state.tb_angles[0] = imu_data.dmp_TaitBryan[TB_PITCH_X];
    mb_state.tb_angles[1] = imu_data.dmp_TaitBryan[TB_ROLL_Y];
    mb_state.last_yaw = mb_state.tb_angles[2];
    mb_state.tb_angles[2] = imu_data.dmp_TaitBryan[TB_YAW_Z];
    mb_state.temp = imu_data.temp;

    int i;
    for(i=0;i<3;i++){
        mb_state.accel[i] = imu_data.accel[i];
        mb_state.gyro[i] = imu_data.gyro[i];
        mb_state.mag[i] = imu_data.mag[i];
    }

    // Read encoders    
    mb_state.left_encoder_delta = rc_encoder_read(LEFT_MOTOR);
    mb_state.right_encoder_delta = rc_encoder_read(RIGHT_MOTOR);
    mb_state.left_encoder_total += mb_state.left_encoder_delta;
    mb_state.right_encoder_total += mb_state.right_encoder_delta;
    rc_encoder_write(LEFT_MOTOR,0);
    rc_encoder_write(RIGHT_MOTOR,0);

    //unlock state mutex
    pthread_mutex_unlock(&state_mutex);

}
/*******************************************************************************
* void publish_mb_msgs()
*
* publishes LCM messages from the mobilbot code
* TODO: modify this function to add other messages
* 
*******************************************************************************/
void publish_mb_msgs(){
    mbot_imu_t imu_msg;
    mbot_encoder_t encoder_msg;
//  odometry_t odo_msg;

    //Create IMU LCM Message
    imu_msg.utime = now;
    imu_msg.temp = mb_state.temp;
    int i;
    for(i=0;i<3;i++){
        imu_msg.tb_angles[i] = mb_state.tb_angles[i];
        imu_msg.accel[i] = mb_state.accel[i];
        imu_msg.gyro[i] = mb_state.gyro[i];
    }

    //Create Encoder LCM message
    encoder_msg.utime = now;
    encoder_msg.left_delta = mb_state.left_encoder_delta;
    encoder_msg.right_delta = mb_state.right_encoder_delta;
    encoder_msg.leftticks = mb_state.left_encoder_total;
    encoder_msg.rightticks = mb_state.right_encoder_total;

    //TODO: Create Odometry LCM message

    //publish IMU & Encoder Data to LCM
    mbot_imu_t_publish(lcm, MBOT_IMU_CHANNEL, &imu_msg);
    mbot_encoder_t_publish(lcm, MBOT_ENCODER_CHANNEL, &encoder_msg);
//  odometry_t_publish(lcm, ODOMETRY_CHANNEL, &odo_msg);
}

/*******************************************************************************
* void mobilebot_controller()
*
* discrete-time mobile controller operated off IMU interrupt
* Called at SAMPLE_RATE_HZ
*
* TODO: You must add to this function so it:
*           1. calculates odometry
*           2. calculates controller ouputs
*           3. outputs commands to the motor drivers
* 
*
*******************************************************************************/
void mobilebot_controller(){
    update_now();
    read_mb_sensors();
    publish_mb_msgs();

}


/*******************************************************************************
*  update_now()
*
*  updates the now global variable with the current time
*
*******************************************************************************/
void update_now(){
	now = rc_nanos_since_epoch()/1000 + time_offset;
}


/*******************************************************************************
*  timesync_handler()
*
*  set time_offset based of difference 
*  between the Pi time and the local time
*
*******************************************************************************/
void timesync_handler(const lcm_recv_buf_t * rbuf, const char *channel,
				const timestamp_t *timestamp, void *_user){

	if(!time_offset_initialized) time_offset_initialized = 1;
	time_offset = timestamp->utime - rc_nanos_since_epoch()/1000;
}


/*******************************************************************************
*  motor_command_handler()
*
* sets motor velocity setpoints from incoming lcm message
*
*******************************************************************************/
void mbot_motor_pwm_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                          const mbot_motor_pwm_t *msg, void *user){
	float l_pwm = msg->left_motor_pwm;
	float r_pwm = msg->right_motor_pwm;
	
	rc_motor_set(1, mot_l_pol * l_pwm);
	rc_motor_set(2, mot_r_pol * r_pwm);

}


/*******************************************************************************
*  reset_odometry_handler()
*
* sets the initial odometry position
*
*******************************************************************************/
void reset_odometry_handler(const lcm_recv_buf_t *rbuf, const char *channel,
                          const reset_odometry_t *msg, void *user){
    mb_odometry.x = msg->x;
    mb_odometry.y = msg->y;
    mb_odometry.theta = msg->theta;
}

/*******************************************************************************
*  dsm_radio_control_loop()
*
*  sets current setpoints based on dsm radio data, odometry
*
*  TODO: Use this thread to handle changing setpoints to your controller
*
*******************************************************************************/
void* dsm_radio_control_loop(void* ptr){

	// start dsm listener for radio control
	rc_dsm_init();

	while(1){
		if (rc_dsm_is_new_data()) {
	 		
			// TODO: Handle the DSM data from the Spektrum radio reciever
			// You may also implement switching between manual and autonomous mode
			// using channel 5 of the DSM data.

		    if(rc_dsm_ch_normalized(5) > 0.0){
			    mb_setpoints.manual_ctl = 1;
			    mb_setpoints.fwd_velocity = rc_dsm_ch_normalized(3);
			    mb_setpoints.turn_velocity = rc_dsm_ch_normalized(2);
		    }

		    else{
			    mb_setpoints.manual_ctl = 0;
		    }

	 	}
	 	rc_nanosleep(1E9 / RC_CTL_HZ);
	}
}


/*******************************************************************************
* lcm_subscribe_loop() 
*
* thread subscribes to lcm channels and sets handler functions
* then handles lcm messages in a non-blocking fashion
*
* TODO: Add other subscriptions as needed
*******************************************************************************/
void *lcm_subscribe_loop(void *data){
    // pass in lcm object instance, channel from which to read from
    // function to call when data receiver over the channel,
    // and the lcm instance again?
    mbot_motor_pwm_t_subscribe(lcm, 
				MBOT_PWM_CHANNEL, 
				mbot_motor_pwm_handler, 
				NULL);

    timestamp_t_subscribe(lcm, 
			    MBOT_TIMESYNC_CHANNEL, 
			    timesync_handler, 
			    NULL);

    reset_odometry_t_subscribe(lcm, 
                          RESET_ODOMETRY_CHANNEL, 
                          reset_odometry_handler, 
                          NULL);

    while(1){
        // define a timeout (for erroring out) and the delay time
        lcm_handle_timeout(lcm, 1);
        rc_nanosleep(1E9 / LCM_HZ);
    }
    lcm_destroy(lcm);
    return 0;
}

/*******************************************************************************
* printf_loop() 
*
* prints diagnostics to console
* this only gets started if executing from terminal
*
* TODO: Add other data to help you tune/debug your code
*******************************************************************************/
void* printf_loop(void* ptr){
	rc_state_t last_state, new_state; // keep track of last state
	while(rc_get_state()!=EXITING){
		new_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_state==RUNNING && last_state!=RUNNING){
			printf("\nRUNNING...\n");
			printf("           SENSORS           |           ODOMETRY          |     SETPOINTS     |");
			printf("\n");
			printf("  IMU θ  |");
			printf("  L_ENC  |");
			printf("  R_ENC  |");
			printf("    X    |");
			printf("    Y    |");
			printf("    θ    |");
			printf("   FWD   |");
            printf("   TURN  |");

			printf("\n");
		}
		else if(new_state==PAUSED && last_state!=PAUSED){
			printf("\nPAUSED\n");
		}
		last_state = new_state;
		
		if(new_state == RUNNING){
			printf("\r");
			//Add Print stattements here, do not follow with /n
			printf("%7.3f  |", mb_state.tb_angles[2]);
			printf("%7lld  |", mb_state.left_encoder_total);
			printf("%7lld  |", mb_state.right_encoder_total);
			printf("%7.3f  |", mb_odometry.x);
			printf("%7.3f  |", mb_odometry.y);
			printf("%7.3f  |", mb_odometry.theta);
			printf("%7.3f  |", mb_setpoints.fwd_velocity);
            printf("%7.3f  |", mb_setpoints.turn_velocity);

			fflush(stdout);
		}
		rc_nanosleep(1E9 / PRINTF_HZ);
	}
	return NULL;
} 


void led_heartbeat(){
        rc_led_set(RC_LED_GREEN, LED_ON);
        rc_nanosleep(1E8);
        rc_led_set(RC_LED_GREEN, LED_OFF);
        rc_nanosleep(1E8);
        rc_led_set(RC_LED_GREEN, LED_ON);
        rc_nanosleep(1E8);
        rc_led_set(RC_LED_GREEN, LED_OFF);

}
