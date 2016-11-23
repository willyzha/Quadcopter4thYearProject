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
#include <AP_Scheduler.h>
#include <AC_PID.h>
#include <AC_P.h>

//////////////////////////////////////////////////////////////////////////////////////////////////
#define RATE_ROLL_P 0.15
#define RATE_ROLL_I 0.1
#define RATE_ROLL_D 0.004
#define RATE_ROLL_IMAX 100

#define RATE_PITCH_P 0.15
#define RATE_PITCH_I 0.1
#define RATE_PITCH_D 0.004
#define RATE_PITCH_IMAX 100

#define RATE_YAW_P 0.2
#define RATE_YAW_I 0.02
#define RATE_YAW_D 0.004
#define RATE_YAW_IMAX 100

#define STABILIZE_ROLL_P 4.5
#define STABILIZE_PITCH_P 4.5
#define STABILIZE_YAW_P 4.5
//////////////////////////////////////////////////////////////////////////////////////////////////
#define rc_feel_rp 25   // controls vehicle response to user input 0 is extremely soft and 100 is extremely crisp
#define ROLL_PITCH_INPUT_MAX 4500
#define acro_yaw_p 4.5f
#define RC_FAST_SPEED 490
#define MAIN_LOOP_SECONDS 0.01
#define MAIN_LOOP_MICROS  10000
//////////////////////////////////////////////////////////////////////////////////////////////////
#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            10  // called at 10hz so 1 second
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define AUTO_DISARMING_DELAY    15  // called at 1hz so 15 seconds
static uint8_t auto_disarming_counter;
//////////////////////////////////////////////////////////////////////////////////////////////////
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
static const AP_InertialSensor::Sample_rate ins_sample_rate = AP_InertialSensor::RATE_100HZ;

RC_Channel rc_1(CH_1); 
RC_Channel rc_2(CH_2); 
RC_Channel rc_3(CH_3); 
RC_Channel rc_4(CH_4); 

AC_PID pid_rate_roll(RATE_ROLL_P, RATE_ROLL_I,  RATE_ROLL_D, RATE_ROLL_IMAX);
AC_PID pid_rate_pitch(RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_PITCH_IMAX);
AC_PID pid_rate_yaw (RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, RATE_YAW_IMAX);

AC_P p_stabilize_roll(STABILIZE_ROLL_P);
AC_P p_stabilize_pitch(STABILIZE_PITCH_P);
AC_P p_stabilize_yaw(STABILIZE_YAW_P);

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

static float G_Dt = 0.02;
static uint32_t fast_loopTimer;

static uint8_t auto_trim_counter;
//////////////////////////////////////////////////////////////////////////////////////////////////
static AP_Scheduler scheduler;
static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
    { rc_loop,               1,     100 },  
    { arm_motors_check,     10,      10 },
    { auto_trim,            10,     140 },
    { update_altitude,      10,    1000 },    
    { barometer_accumulate,  2,     250 }
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

  //init_rc_in();               // sets up rc channels from radio
    rc_1.set_angle(ROLL_PITCH_INPUT_MAX);
    rc_2.set_angle(ROLL_PITCH_INPUT_MAX);
    rc_3.set_range(0, AP_MOTORS_DEFAULT_MAX_THROTTLE);
    rc_4.set_angle(4500);

    rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

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

    ahrs.reset_gyro_drift();
    ahrs.set_fast_gains(true);
    
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
    // stabilize_run()
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    if(!motors.armed() || rc_3.control_in <= 0) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);        
    }
    else{
      // mix pid outputs and send to the motors.   
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////

    scheduler.tick();

    uint32_t time_available = (timer + MAIN_LOOP_MICROS) - hal.scheduler->micros();
    scheduler.run(time_available);

}

static void rc_loop()
{
 // Read radio and 3-position switch on radio
 read_radio();

  //read_control_switch();
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
        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors.armed()) {
            if (!init_arm_motors()) {
                // reset arming counter if arming fail
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors.armed()) {
            auto_trim_counter = 250;
            // ensure auto-disarm doesn't trigger immediately
            auto_disarming_counter = 0;
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
    }else{    // Yaw is centered so reset arming counter
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

    // disable inertial nav errors temporarily
    inertial_nav.ignore_next_error();

    // setup fast AHRS gains to get right attitude
    ahrs.set_fast_gains(true);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    ahrs.set_armed(false);
}

static void auto_trim()
{
    if(auto_trim_counter > 0) {
        auto_trim_counter--;

        float roll_trim_adjustment = ToRad((float)rc_1.control_in / 4000.0f);
        float pitch_trim_adjustment = ToRad((float)rc_2.control_in / 4000.0f);

        // make sure accelerometer values impact attitude quickly
        ahrs.set_fast_gains(true);

        // add trim to ahrs object
        ahrs.add_trim(roll_trim_adjustment, pitch_trim_adjustment, (auto_trim_counter == 0));

        // on last iteration restore accel gains to normal
        if(auto_trim_counter == 0) {
            ahrs.set_fast_gains(false);
        }
    }
}

// read baro and sonar altitude at 10hz
static void update_altitude()
{
    // read in baro altitude
    //read_barometer();

    // read in sonar altitude
    //sonar_alt           = read_sonar();

}
static void barometer_accumulate(void)
{
    barometer.accumulate();
}

static void read_radio()
{
    rc_1.set_pwm(hal.rcin->read(CH_1));
    rc_2.set_pwm(hal.rcin->read(CH_2));
    rc_3.set_pwm(hal.rcin->read(CH_3));
    rc_4.set_pwm(hal.rcin->read(CH_4));
}
AP_HAL_MAIN();
