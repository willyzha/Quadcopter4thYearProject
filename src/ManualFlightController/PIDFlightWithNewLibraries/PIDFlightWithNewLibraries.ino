#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            
#include <AP_GPS.h>
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

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

#include <AC_PID.h>
#include <AC_P.h>

//////////////////////////////////////////////////////////////////////////////////////////////////
// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   912
#define RC_THR_MAX   1985
#define RC_YAW_MIN   1000
#define RC_YAW_MAX   2000
#define RC_PIT_MIN   1020
#define RC_PIT_MAX   2001
#define RC_ROL_MIN   1000
#define RC_ROL_MAX   2000

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

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

//////////////////////////////////////////////////////////////////////////////////////////////////
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

RC_Channel rc_fr(0); 
RC_Channel rc_bl(1); 
RC_Channel rc_fl(2); 
RC_Channel rc_br(3); 

AP_InertialSensor ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
AP_Compass_HMC5843 compass;
AP_GPS gps;
AP_AHRS_DCM  ahrs(ins, baro, gps);

AC_PID pid_rate_roll;
AC_PID pid_rate_pitch;
AC_PID pid_rate_yaw;

AC_P p_stabilize_roll;
AC_P p_stabilize_pitch;
AC_P p_stabilize_yaw;

static AP_Vehicle::MultiCopter aparm;
static AP_MotorsQuad motors(rc_fr, rc_bl, rc_fl, rc_br);
AC_AttitudeControl attitude_control(ahrs, aparm, motors, p_stabilize_roll, p_stabilize_pitch, p_stabilize_yaw,
                        pid_rate_roll, pid_rate_pitch, pid_rate_yaw);
//////////////////////////////////////////////////////////////////////////////////////////////////



void setup() 
{
  // Enable the motors
  hal.rcout->set_freq(0xFF, 490);
  hal.rcout->enable_ch(0xFF);
 
  // PID Configuration 
  pid_rate_roll(RATE_ROLL_P, RATE_ROLL_I,  RATE_ROLL_D, RATE_ROLL_IMAX);
  pid_rate_pitch(RATE_PITCH_P, RATE_PITCH_I, RATE_PITCH_D, RATE_PITCH_IMAX);
  pid_rate_yaw (RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, RATE_YAW_IMAX);
  
  p_stabilize_roll(STABILIZE_ROLL_P);
  p_stabilize_pitch(STABILIZE_PITCH_P);
  p_stabilize_yaw(STABILIZE_YAW_P);

  // Turn off Barometer to avoid bus collisions
  hal.gpio->pinMode(40, HAL_GPIO_OUTPUT);
  hal.gpio->write(40, 1);
  
  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ);
  hal.scheduler->suspend_timer_procs();  // stop bus collisions

  ins.init_accel();
  ahrs.init();  
  if( compass.init() ) {
        hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
  hal.scheduler->resume_timer_procs();
}



void loop() 
{
  static float yaw_target = 0;  
  uint16_t channels[8];

  // Read RC transmitter and map to sensible values  
  hal.rcin->read(channels, 8);

  long rcthr, rcyaw, rcpit, rcroll;  // Variables to store radio in
  
  rcthr = channels[2];
  rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
  rcpit = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, -45, 45);
  rcroll = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, -45, 45);

  // Get orientation angles
  ahrs.update();
  float roll,pitch,yaw;  
  roll = ahrs.roll;
  pitch = ahrs.pitch;
  yaw = ahrs.yaw ;
  
  // Get gyro data
  Vector3f gyro = ahrs.get_gyro();
  float gyroPitch = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);
  


// Throttle raised, turn on stablisation
  if(rcthr > RC_THR_MIN + 100) {  
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;
    
    // mix pid outputs and send to the motors.   
    get_pilot_desired_lean_angles(rc_fr.control_in, rc_bl.control_in, target_roll, target_pitch);
    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(rc_br.control_in);
    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(rc_fl.control_in);
    // call attitude controller
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);
    attitude_control.rate_controller_run();
    
  } else {
    // motors off
       
    // reset yaw target so we maintain this on takeoff
    yaw_target = yaw;
    
    // reset PID integrals whilst on the ground
    pid_rate_roll.reset_I();    
    pid_rate_pitch.reset_I();
    pid_rate_yaw.reset_I(); 
  }
}

// Arduino map function
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

AP_HAL_MAIN();

