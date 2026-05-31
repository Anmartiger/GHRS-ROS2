# TSR-1 Rover Status Message
std_msgs/Header header

# Operational state
string state          # IDLE, PATROLLING, TRACKING, SUPPRESSING, EMERGENCY
bool enabled
bool autonomous_mode

# Drive status
float32 left_speed    # -1.0 to 1.0
float32 right_speed   # -1.0 to 1.0
float32 heading       # degrees, 0-360

# System health
float32 cpu_temp      # Celsius
float32 cpu_usage     # 0-100%
bool pump_active
bool led_active
string error_msg
