#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <PID.h>

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

// MPU6050 accel/gyro chip
AP_InertialSensor_MPU6000 ins;

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   923
#define RC_YAW_MIN   1003
#define RC_YAW_MAX   1991
#define RC_PIT_MIN   1001
#define RC_PIT_MAX   1992
#define RC_ROL_MIN   1006
#define RC_ROL_MAX   2001

// Motor numbers definitions
#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

// Arduino map function
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// PID array (6 pids, two for each axis)
PID pids[6];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

void setup() 
{
  // Enable the motors and set at 490Hz update
  hal.rcout->set_freq(0xF, 490);
  hal.rcout->enable_mask(0xFF);

  // PID Configuration
  pids[PID_PITCH_RATE].kP(0.7);
  pids[PID_PITCH_RATE].kI(1);
  pids[PID_PITCH_RATE].imax(50);
  
  pids[PID_ROLL_RATE].kP(0.7);
  pids[PID_ROLL_RATE].kI(1);
  pids[PID_ROLL_RATE].imax(50);
  
  pids[PID_YAW_RATE].kP(2.7);
  pids[PID_YAW_RATE].kI(1);
  pids[PID_YAW_RATE].imax(50);
  
  pids[PID_PITCH_STAB].kP(4.5);
  pids[PID_ROLL_STAB].kP(4.5);
  pids[PID_YAW_STAB].kP(10);

  // Turn off Barometer to avoid bus collisions
  hal.gpio->pinMode(40, GPIO_OUTPUT);
  hal.gpio->write(40, 1);
  
  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ,
                        NULL);

  // initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
  hal.scheduler->suspend_timer_procs();  // stop bus collisions
  ins.dmp_init();
  hal.scheduler->resume_timer_procs();
  
  // We're ready to go! Now over to loop()
}

void loop() 
{
  static float yaw_target = 0;  
  // Wait until new orientation data (normally 5ms max)
  while (ins.num_samples_available() == 0);
 
  uint16_t channels[8];

  // Read RC transmitter and map to sensible values  
  hal.rcin->read(channels, 8);

//  while(!armed) {
//    if (channels[2] > 1900) {
//      while (channels[2] < 1000) {
//        armed = true;
//      }
//      
//      write (
//    }
//  }

  long rcthr, rcyaw, rcpit, rcroll;  // Variables to store radio in
  
  rcthr = channels[2];
  rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
  rcpit = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, -45, 45);
  rcroll = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, -45, 45);

  // Ask MPU6050 for orientation
  ins.update();
  float roll,pitch,yaw;  
  ins.quaternion.to_euler(&roll, &pitch, &yaw);
  roll = ToDeg(roll) ;
  pitch = ToDeg(pitch) ;
  yaw = ToDeg(yaw) ;
  
  // Ask MPU6050 for gyro data
  Vector3f gyro = ins.get_gyro();
  float gyroPitch = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);
  
  // Do the magic
  if(rcthr > RC_THR_MIN + 100) {  // Throttle raised, turn on stablisation.
    // Stablise PIDS
   float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
    float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
    float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
  
    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5) {
      yaw_stab_output = rcyaw;
      yaw_target = yaw;   // remember this yaw for when pilot stops
    }
    
    // rate PIDS
    long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 500);  
    long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
    long yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  

    // mix pid outputs and send to the motors.
    
    hal.console->printf_P(
          PSTR("individual read THR %ld YAW %ld PIT %ld ROLL %ld\r\n"),
          rcthr + roll_output + pitch_output - yaw_output, 
          rcthr + roll_output - pitch_output + yaw_output, 
          rcthr - roll_output + pitch_output + yaw_output, 
          rcthr - roll_output - pitch_output - yaw_output); 
    
    hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
    hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
    hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
    hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
  } else {
    // motors off
    hal.rcout->write(MOTOR_FL, RC_THR_MIN);
    hal.rcout->write(MOTOR_BL, RC_THR_MIN);
    hal.rcout->write(MOTOR_FR, RC_THR_MIN);
    hal.rcout->write(MOTOR_BR, RC_THR_MIN);
       
    // reset yaw target so we maintain this on takeoff
    yaw_target = yaw;
    
    // reset PID integrals whilst on the ground
    for(int i=0; i<6; i++)
      pids[i].reset_I();

  }
}

AP_HAL_MAIN();

