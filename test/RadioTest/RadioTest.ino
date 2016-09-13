#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <AP_InertialSensor.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#define MAX_SIGNAL 1983
#define MIN_SIGNAL 109

#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   917
#define RC_THR_MAX   1985
#define RC_YAW_MIN   1000
#define RC_YAW_MAX   1984
#define RC_PIT_MIN   1019
#define RC_PIT_MAX   2000
#define RC_ROL_MIN   1000
#define RC_ROL_MAX   1997

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  // Hardware abstraction layer

void setup() {
//  Serial.begin(9600);
//  hal.console->printf_P("Program begin...");
//  hal.console->printf_P("This program will calibrate the ESC.");
//
//  hal.rcout->set_freq(0xF, 50);
//  hal.rcout->enable_mask(0xFF);
//
//  hal.console->printf_P("Now writing maximum output.");
//  hal.console->printf_P("Turn on power source, then wait 2 seconds and press any key.");
//  hal.rcout->write(MOTOR_FR, MAX_SIGNAL);

  // Wait for input
//  while (!Serial.available());
//  Serial.read();
//
//  // Send min output
//  hal.console->printf_P("Sending minimum output");
//  hal.rcout->write(MOTOR_FR, MIN_SIGNAL);

}

void loop() {  
  uint16_t channels[8];

  // Read RC transmitter and map to sensible values  
  hal.rcin->read(channels, 8);
  
  long rcthr, rcyaw, rcpit, rcroll;  // Variables to store radio in
  
  rcthr = channels[2];
  rcyaw = channels[3];
  rcpit = channels[1];
  rcroll = channels[0];
  
  hal.console->printf_P(
            PSTR("individual read THR %ld YAW %ld PIT %ld ROLL %ld\r\n"),
            rcthr, rcyaw, rcpit, rcroll);
}

AP_HAL_MAIN();    // speci

