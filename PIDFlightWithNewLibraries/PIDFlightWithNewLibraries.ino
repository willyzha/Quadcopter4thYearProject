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

#include <AC_PID.h>
#include <AC_P.h>

//////////////////////////////////////////////////////////////////////////////////////////////////
#define RATE_ROLL_P 0.26
#define RATE_ROLL_I 0.003
#define RATE_ROLL_D 0
#define RATE_ROLL_IMAX 50

#define RATE_PITCH_P 0.26
#define RATE_PITCH_I 0.003
#define RATE_PITCH_D 0
#define RATE_PITCH_IMAX 50

#define RATE_YAW_P 0.26
#define RATE_YAW_I 0.003
#define RATE_YAW_D 0.2
#define RATE_YAW_IMAX 50

#define STABILIZE_ROLL_P 4.5
#define STABILIZE_PITCH_P 4.5
#define STABILIZE_YAW_P 10
//////////////////////////////////////////////////////////////////////////////////////////////////
#define rc_feel_rp 50   // controls vehicle response to user input 0 is extremely soft and 100 is extremely crisp
#define ROLL_PITCH_INPUT_MAX 4500
#define acro_yaw_p 4.5f
#define RC_FAST_SPEED 490
#define MAIN_LOOP_SECONDS 0.01
#define MAIN_LOOP_MICROS  10000
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
//////////////////////////////////////////////////////////////////////////////////////////////////


void setup() 
{
//init_ardupilot();
    barometer.init();

  //init_rc_in();               // sets up rc channels from radio
    rc_1.set_angle(ROLL_PITCH_INPUT_MAX);
    rc_2.set_angle(ROLL_PITCH_INPUT_MAX);
    rc_3.set_range(AP_MOTORS_DEFAULT_MIN_THROTTLE, AP_MOTORS_DEFAULT_MAX_THROTTLE);
    rc_4.set_angle(4500);

    rc_1.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    rc_2.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);
    rc_4.set_type(RC_CHANNEL_TYPE_ANGLE_RAW);

  //init_rc_out();              // sets up motors and output to escs
    motors.set_update_rate(RC_FAST_SPEED);
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
    motors.Init();
    motors.set_min_throttle(AP_MOTORS_DEFAULT_MIN_THROTTLE);

    rc_1.set_pwm(hal.rcin->read(CH_1));
    rc_2.set_pwm(hal.rcin->read(CH_2));
    rc_3.set_pwm(hal.rcin->read(CH_3));
    rc_4.set_pwm(hal.rcin->read(CH_4));

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
    barometer.calibrate();
  
  //startup_ground(true);
    ahrs.init();
    ahrs.set_vehicle_class(AHRS_VEHICLE_COPTER);

    ins.init(AP_InertialSensor::COLD_START, ins_sample_rate);

    ahrs.reset_gyro_drift();
    ahrs.set_fast_gains(true);
    
}


void loop() 
{
    ins.wait_for_sample();
    
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

}

AP_HAL_MAIN();

