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
  

}

void loop()
{
  // Turn off Barometer to avoid bus collisions
  hal.gpio->pinMode(54, HAL_GPIO_OUTPUT);
  hal.gpio->write(54,0);
  hal.scheduler->delay_microseconds(2);
  hal.gpio->write(54,1);
  hal.scheduler->delay_microseconds(5);
  hal.gpio->write(54,0);
  
  hal.gpio->pinMode(54, HAL_GPIO_INPUT);
  uint8_t t = hal.gpio->read(54);
  
  
  while (!t) {
    t = hal.gpio->read(54);
        //hal.console->printf_P(
         //     PSTR("t:%d \n"),
          //            t);
  }
  uint32_t start = hal.scheduler->micros();  
   while (t) {
     t = hal.gpio->read(54);
        //hal.console->printf_P(
         //     PSTR("t:%d \n"),
          //            t);
  }
  uint32_t duration = hal.scheduler->micros() - start;
  float distance = duration / 29 / 2;

        hal.console->printf_P(
              PSTR("distance:%4.0f \n"),
                      distance);

  //hal.scheduler->delay(2000);
}


AP_HAL_MAIN();
