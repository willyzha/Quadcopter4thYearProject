//#include <AP_Common.h>
//#include <AP_Math.h>
//#include <AP_Param.h>
//#include <AP_Progmem.h>
//#include <AP_ADC.h>
//#include <AP_InertialSensor.h>
//
//#include <AP_HAL.h>
//#include <AP_HAL_AVR.h>
//
//#include <PID.h>

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
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

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>


// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

// MPU6050 accel/gyro chip
AP_InertialSensor ins;
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
AP_Compass_HMC5843 compass;
AP_GPS gps;
AP_AHRS_DCM  ahrs(ins, baro, gps);

void setup()
{
  
  // Turn off Barometer to avoid bus collisions
  hal.gpio->pinMode(40, HAL_GPIO_OUTPUT);
  hal.gpio->write(40, 1);
  
  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ);

  ins.init_accel();

  ahrs.init();
  
  hal.console->printf("Enabling compass\n");
  ahrs.set_compass(&compass);
}

void loop()
{
  static uint32_t resetTime;
  uint32_t currentTime = hal.scheduler->micros();
  
  if (resetTime == 0) {
    resetTime = currentTime;
  }
  
  ahrs.update();
  
  

                        
  if (currentTime - resetTime > 100000) {
    loop10Hz();
  }
}

void loop10Hz()
{
    Vector3f drift  = ahrs.get_gyro_drift();
    
    float heading = 0;
    heading = compass.calculate_heading(ahrs.get_dcm_matrix());
    
    hal.console->printf_P(
                PSTR("r:%4.1f  p:%4.1f y:%4.1f "
                    "drift=(%5.1f %5.1f %5.1f) hdg=%.1f \n"),
                        ToDeg(ahrs.roll),
                        ToDeg(ahrs.pitch),
                        ToDeg(ahrs.yaw),
                        ToDeg(drift.x),
                        ToDeg(drift.y),
                        ToDeg(drift.z),
                        compass.use_for_yaw() ? ToDeg(heading) : 0.0);
}

AP_HAL_MAIN();
