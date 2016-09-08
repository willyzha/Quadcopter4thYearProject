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

const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  // Hardware abstraction layer

void setup() {
//  Serial.begin(9600);
  hal.console->printf_P("Program begin...");
  hal.console->printf_P("This program will calibrate the ESC.");

  hal.rcout->set_freq(0xF, 490);
  hal.rcout->enable_mask(0xFF);

  hal.console->printf_P("Now writing maximum output.");
  hal.console->printf_P("Turn on power source, then wait 2 seconds and press any key.");
  hal.rcout->write(MOTOR_FR, MAX_SIGNAL);

  // Wait for input
//  while (!Serial.available());
//  Serial.read();
//
//  // Send min output
//  hal.console->printf_P("Sending minimum output");
//  hal.rcout->write(MOTOR_FR, MIN_SIGNAL);

}

void loop() {  

}

AP_HAL_MAIN();    // speci
