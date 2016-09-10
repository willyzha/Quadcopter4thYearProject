#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>


#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right


const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  // Hardware abstraction layer
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//long rcthr, rcyaw, rcpit, rcroll;
int rcthr;
int increment = 100;

void setup() 
{
  hal.rcout->set_freq(0xF, 50);
  
  hal.rcout->enable_mask(0xFF);
  
  //hal.scheduler->delay(1000);
  hal.rcout->write(MOTOR_FR, 900);
  hal.rcout->write(MOTOR_FL, 900);
  hal.rcout->write(MOTOR_BR, 900);
  hal.rcout->write(MOTOR_BL, 900);
  hal.scheduler->delay(1000);
  
  hal.rcout->write(MOTOR_FR, 2000);
  hal.rcout->write(MOTOR_FL, 2000);
  hal.rcout->write(MOTOR_BR, 2000);
  hal.rcout->write(MOTOR_BL, 2000);
  hal.scheduler->delay(1500);
  
  hal.rcout->write(MOTOR_FR, 900);
  hal.rcout->write(MOTOR_FL, 900);
  hal.rcout->write(MOTOR_BR, 900);
  hal.rcout->write(MOTOR_BL, 900);
  hal.scheduler->delay(10000);
  
  rcthr = 900;
}

void loop() 
{ 
  if (rcthr < 900) {
    increment = 100;
  } else if (rcthr > 2000) {
    increment = -100;
  }
  hal.scheduler->delay(1000);
  rcthr = rcthr + increment;
  
  hal.rcout->write(MOTOR_FL, rcthr);
  hal.rcout->write(MOTOR_FR, rcthr);
  hal.rcout->write(MOTOR_BL, rcthr);
  hal.rcout->write(MOTOR_BR, rcthr);

  hal.console->printf_P(
            PSTR("individual read THR %d\n"),
            rcthr); 
  /*uint16_t channels[8];  // array for raw channel values
  
  // Read RC channels and store in channels array
  hal.rcin->read(channels, 4);
  
  // Copy from channels array to something human readable - array entry 0 = input 1, etc.
  
  rcthr = channels[2];
  rcyaw = map(channels[3], 1003, 1991, -150, 150);
  rcpit = -map(channels[1], 1001, 1992, -45, 45);
  rcroll = map(channels[0], 1006, 2001, -45, 45);
  
  // Variables to store rc input
  hal.rcout->write(MOTOR_FL, rcthr);
  hal.rcout->write(MOTOR_FR, 900);
  hal.rcout->write(MOTOR_BL, rcthr);
  hal.rcout->write(MOTOR_BR, rcthr);

  hal.console->printf_P(
            PSTR("individual read THR %ld YAW %ld PIT %ld ROLL %ld\r\n"),
            rcthr, rcyaw, rcpit, rcroll);*/
}

AP_HAL_MAIN();    // speci
