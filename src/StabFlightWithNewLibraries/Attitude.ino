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
        desired_rate = (int32_t) 10*PILOT_VELZ_MAX * (throttle_control-THROTTLE_IN_DEADBAND_BOTTOM) / (THROTTLE_IN_MIDDLE - THR_DZ_DEFAULT);
    }else if (throttle_control > THROTTLE_IN_DEADBAND_TOP) {
        // above the deadband
        desired_rate = (int32_t) PILOT_VELZ_MAX * (throttle_control-THROTTLE_IN_DEADBAND_TOP) / (THROTTLE_IN_MIDDLE - THR_DZ_DEFAULT);
    }else{
        // must be in the deadband
        desired_rate = 0;
    }

    return desired_rate;
}

