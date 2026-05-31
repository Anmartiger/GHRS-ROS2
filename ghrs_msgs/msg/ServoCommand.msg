# TSR-1 Servo Command Message
std_msgs/Header header

# Servo channel identifiers (matching PCA9685 channels)
# ch0 = turret_pan, ch1 = turret_tilt, ch2 = trigger
# ch3 = hose_pan,   ch4 = hose_tilt

int32 channel           # PCA9685 channel number 0-15
float32 pulse_us        # pulse width in microseconds
float32 angle_deg       # alternative: angle in degrees (-90 to 90 relative to centre)
bool use_angle          # if true, use angle_deg; else use pulse_us
