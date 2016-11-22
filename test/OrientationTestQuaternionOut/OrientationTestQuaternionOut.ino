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

void setup()
{
  
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
  
}

void loop()
{
  
  // Wait until new orientation data (normally 5ms max)
  while (ins.num_samples_available() == 0);
  
  // Ask MPU6050 (fusion sensor) for orientation
  ins.update();
  float roll,pitch,yaw;
  float q1,q2,q3,q4;
  //ins.quaternion.to_euler(&roll, &pitch, &yaw);
  q1=ins.quaternion.q1;
  q2=ins.quaternion.q2;
  q3=ins.quaternion.q3;
  q4=ins.quaternion.q4;
  Quaternion quat;
  quat.q1=q1;
  quat.q2=q2;
  quat.q3=q3+0.06;
  quat.q4=q4;
  quat.to_euler(&roll, &pitch, &yaw);
  
  roll = ToDeg(roll) ;
  pitch = ToDeg(pitch) ;
  yaw = ToDeg(yaw) ;
  float p;
  p=asin(2*(-q1*quat.q3-q4*q2))/0.017;
  hal.console->printf_P(
	  PSTR("P:%4.1f  p2: %4.1f q0:%4.2f  q2:%4.2f   q3:%4.2f  q1:%4.2f \n"),
			  pitch,p,
			  quat.q1,
			  quat.q3,quat.q4,quat.q2);


}

AP_HAL_MAIN();
