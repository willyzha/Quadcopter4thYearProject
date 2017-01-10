// Returns smoothing gain to be passed into attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth
//      result is a number from 2 to 12 with 2 being very sluggish and 12 being very crisp
float get_smoothing_gain()
{
    return (2.0f + (float)rc_feel_rp/10.0f);
}

// Transform pilot's roll or pitch input into a desired lean angle
// returns desired angle in centi-degrees
static void get_pilot_desired_lean_angles(int16_t roll_in, int16_t pitch_in, int16_t &roll_out, int16_t &pitch_out)
{
    static float _scaler = 1.0;
    static int16_t _angle_max = 0;

    // range check the input
    roll_in = constrain_int16(roll_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);
    pitch_in = constrain_int16(pitch_in, -ROLL_PITCH_INPUT_MAX, ROLL_PITCH_INPUT_MAX);

    // return filtered roll if no scaling required
    if (aparm.angle_max == ROLL_PITCH_INPUT_MAX) {
        roll_out = roll_in;
        pitch_out = pitch_in;
        return;
    }

    // check if angle_max has been updated and redo scaler
    if (aparm.angle_max != _angle_max) {
        _angle_max = aparm.angle_max;
        _scaler = (float)aparm.angle_max/(float)ROLL_PITCH_INPUT_MAX;
    }

    // convert pilot input to lean angle
    roll_out = (int16_t)((float)roll_in * _scaler);
    pitch_out = (int16_t)((float)pitch_in * _scaler);
}

// Transform pilot's yaw input into a desired heading
// returns desired angle in centi-degrees
static float get_pilot_desired_yaw_rate(int16_t stick_angle)
{
    // convert pilot input to the desired yaw rate
    return stick_angle * acro_yaw_p;
}

// stabilisty mode
// get_pilot_desired_throttle - transform pilot's throttle input to make cruise throttle mid stick
// used only for manual throttle modes
// returns throttle output 0 to 1000
//#define THR_MID_DEFAULT 500 //
static int16_t get_pilot_desired_throttle(int16_t throttle_control)
{
    //int16_t throttle_out;

    // exit immediately in the simple cases
    //if( throttle_control == 0 || g.throttle_mid == 500) {
        return throttle_control;
    //}
/*
    // ensure reasonable throttle values
    throttle_control = constrain_int16(throttle_control,0,1000);
    g.throttle_mid = constrain_int16(g.throttle_mid,300,700);

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_MIDDLE) {
        // below the deadband
        throttle_out = g.throttle_min + ((float)(throttle_control-g.throttle_min))*((float)(g.throttle_mid - g.throttle_min))/((float)(500-g.throttle_min));
    }else if(throttle_control > THROTTLE_IN_MIDDLE) {
        // above the deadband
        throttle_out = g.throttle_mid + ((float)(throttle_control-500))*(float)(1000-g.throttle_mid)/500.0f;
    }else{
        // must be in the deadband
        throttle_out = g.throttle_mid;
    }

    return throttle_out;
*/
}

// althold mode
// get_pilot_desired_climb_rate - transform pilot's throttle input to
// climb rate in cm/s.  we use radio_in instead of control_in to get the full range
// without any deadzone at the bottom
#define THR_DZ_DEFAULT         100.0 
#define THROTTLE_IN_DEADBAND_TOP (THROTTLE_IN_MIDDLE+THR_DZ_DEFAULT)  // top of the deadband
#define THROTTLE_IN_DEADBAND_BOTTOM (THROTTLE_IN_MIDDLE-THR_DZ_DEFAULT)  // bottom of the deadband
static int16_t get_pilot_desired_climb_rate(int16_t throttle_control)
{
    int16_t desired_rate = 0;

    // check throttle is above, below or in the deadband
    if (throttle_control < THROTTLE_IN_DEADBAND_BOTTOM) {
        // below the deadband
        desired_rate = (int32_t) PILOT_VELZ_MAX * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THR_DZ_DEFAULT);
    }else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_rate = (int32_t) PILOT_VELZ_MAX * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THR_DZ_DEFAULT);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    return desired_rate;
}


// get_throttle_surface_tracking - hold copter at the desired distance above the ground
//      returns climb rate (in cm/s) which should be passed to the position controller
# define SONAR_GAIN_DEFAULT 0.8
# define THR_SURFACE_TRACKING_VELZ_MAX 150
static float get_throttle_surface_tracking(int16_t target_rate, float current_alt_target, float dt)
{
    static uint32_t last_call_ms = 0;
    float distance_error;
    float velocity_correction;

    // uint32_t now = millis();

    // // reset target altitude if this controller has just been engaged
    // if (now - last_call_ms > SONAR_TIMEOUT_MS) {
    //     target_sonar_alt = sonar_alt + current_alt_target - current_loc.alt;
    // }
    // last_call_ms = now;

    // adjust sonar target alt if motors have not hit their limits
    if ((target_rate<0 && !motors.limit.throttle_lower) || (target_rate>0 && !motors.limit.throttle_upper)) {
        target_sonar_alt += target_rate * dt;
    }

    // do not let target altitude get too far from current altitude above ground
    // Note: the 750cm limit is perhaps too wide but is consistent with the regular althold limits and helps ensure a smooth transition
    target_sonar_alt = constrain_float(target_sonar_alt,sonar_alt-pos_control.get_leash_down_z(),sonar_alt+pos_control.get_leash_up_z());

    // calc desired velocity correction from target sonar alt vs actual sonar alt (remove the error already passed to Altitude controller to avoid oscillations)
    distance_error = (target_sonar_alt - sonar_alt) - (current_alt_target - current_loc.alt);
    velocity_correction = distance_error * SONAR_GAIN_DEFAULT;
    velocity_correction = constrain_float(velocity_correction, -THR_SURFACE_TRACKING_VELZ_MAX, THR_SURFACE_TRACKING_VELZ_MAX);

    // return combined pilot climb rate + rate to correct sonar alt error
    return (target_rate + velocity_correction);
}
