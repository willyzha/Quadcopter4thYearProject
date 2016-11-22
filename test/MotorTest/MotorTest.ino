#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  // Hardware abstraction layer
long rcthr, rcyaw, rcpit, rcroll;

void setup() 
{
  hal.rcout->set_freq(0xFF, 490);
  hal.rcout->enable_ch(0xFF);

  hal.rcout->write(MOTOR_FR, 1000);
  hal.rcout->write(MOTOR_FL, 1000);
  hal.rcout->write(MOTOR_BR, 1000);
  hal.rcout->write(MOTOR_BL, 1000);
  hal.scheduler->delay(1000);
  
  hal.rcout->write(MOTOR_FR, 2000);
  hal.rcout->write(MOTOR_FL, 2000);
  hal.rcout->write(MOTOR_BR, 2000);
  hal.rcout->write(MOTOR_BL, 2000);
  hal.scheduler->delay(2000);
  
  hal.rcout->write(MOTOR_FR, 1000);
  hal.rcout->write(MOTOR_FL, 1000);
  hal.rcout->write(MOTOR_BR, 1000);
  hal.rcout->write(MOTOR_BL, 1000);
  hal.scheduler->delay(5000);
}



void loop() 
{ 
  uint16_t channels[8];  // array for raw channel values
  
  // Read RC channels and store in channels array
  hal.rcin->read(channels, 4);
  
  // Copy from channels array to something human readable - array entry 0 = input 1, etc.
  
  rcthr = channels[2];
  rcyaw = map(channels[3], 1003, 1991, -150, 150);
  rcpit = -map(channels[1], 1001, 1992, -45, 45);
  rcroll = map(channels[0], 1006, 2001, -45, 45);
  
  // Variables to store rc input
  hal.rcout->write(MOTOR_FL, rcthr-rcpit+rcroll);
  hal.rcout->write(MOTOR_FR, rcthr-rcpit-rcroll);
  hal.rcout->write(MOTOR_BL, rcthr+rcpit+rcroll);
  hal.rcout->write(MOTOR_BR, rcthr+rcpit-rcroll);

  hal.console->printf_P(
            PSTR("individual read THR %ld YAW %ld PIT %ld ROLL %ld\r\n"),
            rcthr, rcyaw, rcpit, rcroll); 
}

AP_HAL_MAIN();    // speci
