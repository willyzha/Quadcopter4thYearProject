#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_InertialNav.h>  
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>     
#include <AP_Baro_Glitch.h> 
#include <AP_GPS.h>
#include <AP_GPS_Glitch.h> 
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <Filter.h>
#include <SITL.h>
#include <AP_Buffer.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <RC_Channel.h>
#include <AP_Motors.h>
#include <AP_Curve.h>
#include <AC_AttitudeControl.h>
#include <AC_PosControl.h>
#include <AP_Scheduler.h>
#include <AC_PID.h>
#include <AC_P.h>

//////////////////////////////////////////////////////////////////////////////////////////////////
#define RATE_ROLL_P     0.11
#define RATE_ROLL_I     0.1
#define RATE_ROLL_D     0.004
#define RATE_ROLL_IMAX  100

#define RATE_PITCH_P    0.11
#define RATE_PITCH_I    0.1
#define RATE_PITCH_D    0.004
#define RATE_PITCH_IMAX 100

#define RATE_YAW_P      0.2
#define RATE_YAW_I      0.02
#define RATE_YAW_D      0.004
#define RATE_YAW_IMAX   100

#define STABILIZE_ROLL_P    4.5
#define STABILIZE_PITCH_P   4.5
#define STABILIZE_YAW_P     4.5

#define ALT_HOLD_P         1.0
#define THROTTLE_RATE_P    5.0

#define THROTTLE_ACCEL_P      0.50
#define THROTTLE_ACCEL_I      1.0
#define THROTTLE_ACCEL_D      0.0
#define THROTTLE_ACCEL_IMAX   10

#define LOITER_POS_P       1.0
#define LOITER_RATE_P      1.0
#define LOITER_RATE_I      0.5
#define LOITER_RATE_D      0.0
#define LOITER_RATE_IMAX   1000        // maximum acceleration from I term build-up in cm/s/s
//////////////////////////////////////////////////////////////////////////////////////////////////
#define rc_feel_rp          25   // controls vehicle response to user input 0 is extremely soft and 100 is extremely crisp
#define ROLL_PITCH_INPUT_MAX 4500
#define acro_yaw_p          4.5f
#define RC_FAST_SPEED       490
#define MAIN_LOOP_SECONDS   0.01
#define MAIN_LOOP_MICROS    10000
//////////////////////////////////////////////////////////////////////////////////////////////////
#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            10  // called at 10hz so 1 second
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    15  // called at 1hz so 15 seconds
static uint8_t auto_disarming_counter;
//////////////////////////////////////////////////////////////////////////////////////////////////
#define PILOT_VELZ_MAX     10     // maximum vertical velocity in cm/s
#define PILOT_ACCEL_Z      10     // vertical acceleration in cm/s/s while altitude is under pilot control

#define LAND_DETECTOR_TRIGGER 50    // number of 50hz iterations with near zero climb rate and low throttle that triggers landing complete.
#define LAND_DETECTOR_MAYBE_TRIGGER   10  // number of 50hz iterations with near zero climb rate and low throttle that means we might be landed (used to reset horizontal position targets to prevent tipping over)
#define LAND_DETECTOR_CLIMBRATE_MAX    30  // vehicle climb rate must be between -30 and +30 cm/s
#define LAND_DETECTOR_BARO_CLIMBRATE_MAX   150  // barometer climb rate must be between -150cm/s ~ +150cm/s
#define LAND_DETECTOR_DESIRED_CLIMBRATE_MAX    -20    // vehicle desired climb rate must be below -20cm/s
#define LAND_DETECTOR_ROTATION_MAX 0.50f // vehicle rotation under 0.5 rad/sec is param to be considered landed

#define THROTTLE_IN_MIDDLE 500.0          // the throttle mid point
//////////////////////////////////////////////////////////////////////////////////////////////////
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;

RC_Channel rc_1(CH_1); 
RC_Channel rc_2(CH_2); 
RC_Channel rc_3(CH_3); 
RC_Channel rc_4(CH_4);

int pi_rc_1;
int pi_rc_2;
int pi_rc_3;
int pi_rc_4;

AC_PID pid_rate_roll(RATE_ROLL_P, RATE_ROLL_I,  RATE_ROLL_D, RATE_ROLL_IMAX);
AC_PID pid_rate_pitch(RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_PITCH_IMAX);
AC_PID pid_rate_yaw(RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, RATE_YAW_IMAX);

AC_P p_stabilize_roll(STABILIZE_ROLL_P);
AC_P p_stabilize_pitch(STABILIZE_PITCH_P);
AC_P p_stabilize_yaw(STABILIZE_YAW_P);

AC_P p_alt_hold(ALT_HOLD_P);
AC_P p_throttle_rate(THROTTLE_RATE_P);
AC_PID pid_throttle_accel(THROTTLE_ACCEL_P, THROTTLE_ACCEL_I, THROTTLE_ACCEL_D, THROTTLE_ACCEL_IMAX);

AC_P p_loiter_pos(LOITER_POS_P);
AC_PID pid_loiter_rate_lat(LOITER_RATE_P, LOITER_RATE_I, LOITER_RATE_D, LOITER_RATE_IMAX);
AC_PID pid_loiter_rate_lon(LOITER_RATE_P, LOITER_RATE_I, LOITER_RATE_D, LOITER_RATE_IMAX);

AP_InertialSensor ins;
AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
static Baro_Glitch baro_glitch(barometer);
AP_Compass_HMC5843 compass;
AP_GPS gps;
static GPS_Glitch gps_glitch(gps);

AP_AHRS_DCM  ahrs(ins, barometer, gps);

static AP_Vehicle::MultiCopter aparm;
static AP_MotorsQuad motors(rc_1, rc_2, rc_3, rc_4);
AC_AttitudeControl attitude_control(ahrs, aparm, motors, p_stabilize_roll, p_stabilize_pitch, p_stabilize_yaw,
                        pid_rate_roll, pid_rate_pitch, pid_rate_yaw);

static AP_InertialNav inertial_nav(ahrs, barometer, gps_glitch, baro_glitch);

AC_PosControl pos_control(ahrs, inertial_nav, motors, attitude_control,
                        p_alt_hold, p_throttle_rate, pid_throttle_accel,
                        p_loiter_pos, pid_loiter_rate_lat, pid_loiter_rate_lon);

static float G_Dt = 0.02;
static uint32_t fast_loopTimer;

static int16_t climb_rate;
static int32_t baro_alt;            // barometer altitude in cm above home
static float baro_climbrate;        // barometer climbrate in cm/s

static int16_t sonar_alt;
static uint8_t sonar_alt_health;   // true if we can trust the altitude from the sonar
static float target_sonar_alt;      // desired altitude in cm above the ground
static struct   Location current_loc; // current location of the copter

static uint16_t land_detector = LAND_DETECTOR_TRIGGER;  // we assume we are landed
bool land_complete = true;
//////////////////////////////////////////////////////////////////////////////////////////////////
static AP_Scheduler scheduler;
/*
  Frequency the task should be called at and the maximum time it's expected to take (microseconds)
  1    = 100hz
  2    = 50hz
  4    = 25hz
  10   = 10hz
  20   = 5hz
  33   = 3hz
  50   = 2hz
  100  = 1hz
  1000 = 0.1hz
 */
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { rc_loop,               1,     100 },  
    { throttle_loop,         2,     450 },
    { arm_motors_check,     10,      10 },    
    { update_altitude,      10,    1000 },    
    { barometer_accumulate,  2,     250 },
    { update_compass,       10,     720 },
    { compass_accumulate,    2,     420 }
  };

//////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    // setup storage layout for copter
    StorageManager::set_layout_copter();
    
//init_ardupilot();
  hal.scheduler->set_timer_speed(500);
    barometer.init();

    althold_init();

  //init_rc_in();               // sets up rc channels from radio
    rc_1.set_angle(ROLL_PITCH_INPUT_MAX);
    rc_2.set_angle(ROLL_PITCH_INPUT_MAX);
    rc_3.set_range(0, AP_MOTORS_DEFAULT_MAX_THROTTLE);
    rc_4.set_angle(4500);

    rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

    if (compass.init() && compass.read()) {
      ahrs.set_compass(&compass);
    }

    attitude_control.set_dt(MAIN_LOOP_SECONDS);

    inertial_nav.init();

    //init_baromter(true);  
      barometer.calibrate();
  
  //startup_ground(true);
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    ins.init(AP_InertialSensor::COLD_START, ins_sample_rate);
    ins.init_accel();

    ahrs.reset_gyro_drift();
    ahrs.set_fast_gains(true);
    land_complete=true;

   //init_rc_out();              // sets up motors and output to escs
    motors.set_update_rate(RC_FAST_SPEED);
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.Init();
    motors.set_min_throttle(AP_MOTORS_DEFAULT_MIN_THROTTLE);

    read_radio();
    // we want the input to be scaled correctly
    rc_3.set_range_out(0,1000);

    // enable output to motors
    motors.enable();
    motors.output_min();
    
// initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], sizeof(scheduler_tasks)/sizeof(scheduler_tasks[0]));
}


void loop() 
{
    ins.wait_for_sample();
    uint32_t timer = hal.scheduler->micros();

    // used by PI Loops
    G_Dt                    = (float)(timer - fast_loopTimer) / 1000000.f;
    fast_loopTimer          = timer;

/////////////////////////////////////////////////////////////////////////////////////////////////////    
//fast_loop()
  // IMU DCM Algorithm
  // read_AHRS();
    ahrs.update();

  // Run low level rate controllers that only require IMU data
    attitude_control.rate_controller_run();

  // Write out the servo PWM values
  // set_servos_4();
    motors.output();

  // Inertial Nav
  // read_inertia();
    inertial_nav.update(G_Dt);

  // Run the attitude controllers    
  // update_flight_mode()
    if(1){
        stabilize_run();
    }else{
        althold_run();
    }
     
///////////////////////////////////////////////////////////////////////////////////////////////////////////

    scheduler.tick();

    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - hal.scheduler->micros();
    scheduler.run(time_available);

}
static void stabilize_run(){
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    if(!motors.armed() || rc_3.control_in <= 0) { 
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);   
    }
    else{
      // Mix pid outputs and send to the motors

      // get pilot's desired lean angles   
      get_pilot_desired_lean_angles(rc_1.control_in, rc_2.control_in, target_roll, target_pitch);
      // get pilot's desired yaw rate
      target_yaw_rate = get_pilot_desired_yaw_rate(rc_4.control_in);
      // get pilot's desired throttle
      pilot_throttle_scaled = get_pilot_desired_throttle(rc_3.control_in);    

      // call attitude controller
      attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
      // output pilot's throttle
      attitude_control.set_throttle_out(pilot_throttle_scaled, true); 
    }
}
static void althold_run(){
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t target_climb_rate; 

    if(!motors.armed()){
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);   
        pos_control.set_alt_target_to_current_alt();
    }
    else{
      // Mix pid outputs and send to the motors.  
      
      // get pilot's desired lean angles 
      get_pilot_desired_lean_angles(rc_1.control_in, rc_2.control_in, target_roll, target_pitch);
      // get pilot's desired yaw rate
      target_yaw_rate = get_pilot_desired_yaw_rate(rc_4.control_in);

      // get pilot desired climb rate 
      target_climb_rate = get_pilot_desired_climb_rate(rc_3.control_in);

      if(land_complete && target_climb_rate > 0){
          land_complete = false;

          //set_throttle_takeoff()
            // tell position controller to reset alt target and reset I terms
            pos_control.init_takeoff();

            // tell motors to do a slow start
            motors.slow_start(true);
      }

      if(land_complete){
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);   
        pos_control.set_alt_target_to_current_alt();
      }else{
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // call throttle controller
        //if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
                // if sonar is ok, use surface tracking
                //target_climb_rate = get_throttle_surface_tracking(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        //}
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
      }
    }

           hal.console->printf_P(
                PSTR("r:%5.1f  p:%5.1f y:%5.1f comp=%.1f "
                    "alt:%3.0fcm        climbrate:%6dcm/s   alttarget:%3.0fcm         rc1:%6d rc2:%6d rc3:%6d rc4:%6d\n"),
                        ToDeg(ahrs.roll),
                        ToDeg(ahrs.pitch),
                        ToDeg(ahrs.yaw),
                        ToDeg(compass.calculate_heading(ahrs.get_dcm_matrix())),
                        baro_alt/1.0,
                        target_climb_rate,
                        pos_control.get_alt_target(),
                        motors.armed(),
                        rc_2.control_in,
                        rc_3.control_in,
                        rc_4.control_in);
}

static void rc_loop()
{
 // Read radio and 3-position switch on radio
 read_radio();
 //read_control_switch();
 
 // Update the channel using values obtained from pi
 pi_channel_update();
}
// throttle_loop - should be run at 50 hz
static void throttle_loop()
{
    // get altitude and climb rate from inertial lib
    read_inertial_altitude();

    // check if we've landed
    update_land_detector();
}

static void arm_motors_check()
{
    static int16_t arming_counter;

    // ensure throttle is down
    if (rc_3.control_in > 0) {
        arming_counter = 0;
        return;
    }

    int16_t tmp = rc_4.control_in;

    if (tmp > 4000) { // full right
        // increase the arming counter
        arming_counter++;

        // arm the motors and configure for flight after 2 seconds
        if (arming_counter == ARM_DELAY && !motors.armed()) {
            if (!init_arm_motors()) {
                // reset arming counter if arming fail
                arming_counter = 0;
            }
        }
    }else if (tmp < -4000) {    // full left
        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors.armed()) {
            init_disarm_motors();
        }
    }else {    // Yaw is centered so reset arming counter
        arming_counter = 0;
    }
}
static bool init_arm_motors()
{
    static bool did_ground_start = false;
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // disable inertial nav errors temporarily
    inertial_nav.ignore_next_error();

    if(did_ground_start == false) {
        //startup_ground(true);
          ahrs.init();
          ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

          ins.init(AP_InertialSensor::COLD_START, ins_sample_rate);

          ahrs.reset_gyro_drift();
          ahrs.set_fast_gains(true);
          land_complete=true;
        // final check that gyros calibrated successfully
        if (!ins.gyro_calibrated_ok_all()) {
            in_arm_motors = false;
            return false;
        }
        did_ground_start = true;
    }

    // fast baro calibration to reset ground pressure
    //init_barometer(false);
      barometer.update_calibration();

    // reset inertial nav alt to zero
    inertial_nav.set_altitude(0.0f);

    // go back to normal AHRS gains
    ahrs.set_fast_gains(false);

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    ahrs.set_armed(true);

    // Cancel arming if throttle is raised too high so that copter does not suddenly take off
    read_radio();
    if (rc_3.control_in > 100) {
        motors.output_min();
        in_arm_motors = false;
        return false;
    }

    // short delay to allow reading of rc inputs    
	  hal.scheduler->delay(30);

    // enable output to motors
    //output_min();
      motors.enable();
      motors.output_min();

    // finally actually arm the motors
    motors.armed(true);

    // flag exiting this function
    in_arm_motors = false;

    // return success
    return true;
}
static void init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors.armed()) {
        return;
    }
    motors.armed(false);

    land_complete = true;

    // disable inertial nav errors temporarily
    inertial_nav.ignore_next_error();

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    ahrs.set_armed(false);
}

// read baro and sonar altitude at 10hz
static void update_altitude()
{
    // read in baro altitude
    //read_barometer();
        barometer.read();
        baro_alt = barometer.get_altitude() * 100.0f;
        baro_climbrate = barometer.get_climb_rate() * 100.0f;

    // read in sonar altitude
    //sonar_alt           = read_sonar();

}
static void barometer_accumulate(void)
{
    barometer.accumulate();
}

static void update_compass(void)
{
    compass.set_throttle((float)rc_3.servo_out/1000.0f);
    compass.read();    
}
static void compass_accumulate(void)
{
    compass.accumulate();    
}

static void read_radio()
{
    rc_1.set_pwm(hal.rcin->read(CH_1));
    rc_2.set_pwm(hal.rcin->read(CH_2));
    rc_3.set_pwm(hal.rcin->read(CH_3));
    rc_4.set_pwm(hal.rcin->read(CH_4));
}

static void althold_init()
{
    // initialize vertical speeds and leash lengths
    pos_control.set_speed_z(-PILOT_VELZ_MAX, PILOT_VELZ_MAX);
    pos_control.set_accel_z(PILOT_ACCEL_Z);

    // initialise altitude target to stopping point
    pos_control.set_target_to_stopping_point_z();
}

static void read_inertial_altitude()
{
    // with inertial nav we can update the altitude and climb rate at 50hz
    current_loc.alt = inertial_nav.get_altitude();
    current_loc.flags.relative_alt = true;
    climb_rate = inertial_nav.get_velocity_z();
}
// Checks if we have landed and updates land_complete
static void update_land_detector()
{
    bool climb_rate_low = (abs(climb_rate) < LAND_DETECTOR_CLIMBRATE_MAX) && (abs(baro_climbrate) < LAND_DETECTOR_BARO_CLIMBRATE_MAX);
    bool target_climb_rate_low = !pos_control.is_active_z() || (pos_control.get_desired_velocity().z <= LAND_DETECTOR_DESIRED_CLIMBRATE_MAX);
    bool motor_at_lower_limit = motors.limit.throttle_lower;
    bool throttle_low = (motors.get_throttle_out() < (THROTTLE_IN_MIDDLE/2.0f));
    bool not_rotating_fast = (ahrs.get_gyro().length() < LAND_DETECTOR_ROTATION_MAX);

    if (climb_rate_low && target_climb_rate_low && motor_at_lower_limit && throttle_low && not_rotating_fast) {
        if (!land_complete) {
            // increase counter until we hit the trigger then set land complete flag
            if(land_detector < LAND_DETECTOR_TRIGGER) {
                land_detector++;
            }else{
                land_complete = true;
                land_detector = LAND_DETECTOR_TRIGGER;
            }
        }
    } else {
        // we've sensed movement up or down so reset land_detector
        land_detector = 0;
        // if throttle output is high then clear landing flag
        if (motors.get_throttle_out() > (THROTTLE_IN_MIDDLE/2.0f)) {
            land_complete = false;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Checksum and updating channel values from the pi tracking system
///////////////////////////////////////////////////////////////////////////////////////////////////////////

int checkSum(char *str)
{
  int sum = 0;
  int len = strlen(str);
  for(int i=0;i<len;i++)
  {
    sum += str[i];
  }
  return sum;
}

static void pi_channel_update() 
{
  char buf[255];
  char out[255];
  int chs;
  int compareSum;
  long val[4];
  int counter;
  int buf_offset = 0;
  int lastCheck;
  
  int totalBytes = hal.console->available();
  if(totalBytes > 0) // check to see if data is present
  {
    counter = 0;
    while(totalBytes > 0) // start loop with number of bytes
    {
      char c = (char)hal.console->read(); // read next byte    
      if(c == '\n') // when \n is reached
      {
        buf[buf_offset] = '\0'; // null terminator
        // process data
        char *chk = strtok(buf," *,"); //obtain checkSum
        char *str = strtok(NULL," *,"); //str = roll,pitch,throttle,yaw
        
        while (str != NULL) // loop to go through each token
        {
          strcat(out,str);
          strcat(out," ");
          val[counter++] = strtol(str,NULL,10); //saving values of each token as long
          str = strtok(NULL," ,");
        }

        //Set string endings
        out[strlen(out)-1] = '\0';
        
        //calculate checksum and convert char chk into int
        chs = checkSum(out);
        compareSum = strtol(chk,NULL,10);
        
        //compare checksum value with value from python
        if (chs == compareSum)
        {
          hal.console->printf("Flag: %s\n", out);
          //set channel values using val[0] to val[3]
          pi_rc_1 = val[0]; // roll
          pi_rc_2 = val[1]; // pitch
          pi_rc_3 = val[2]; // throttle
          pi_rc_4 = val[3]; // yaw
        }
        else
        {
          hal.console->printf("Flag: CheckSum Fail\n");
        }
        buf_offset = 0; //reset buf_offset
      }
      else //c is not \n
      {
        buf[buf_offset++] = c; // store in buffer and continue until newline is found
      }
      
      //decrease totalBytes for loop
      totalBytes--;
      
      //reset out and full char arrays
      out[0] = '\0';
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//Checksum and updating channel values from the pi tracking system
///////////////////////////////////////////////////////////////////////////////////////////////////////////

AP_HAL_MAIN();

