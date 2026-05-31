# TSR-1 Plant Health / Disease Detection Message
std_msgs/Header header

string disease_label      # e.g. "Tomato Late Blight", "Healthy", "Unknown"
float32 confidence        # 0.0 - 1.0
float32 severity          # 0.0 (none) - 1.0 (critical)
string recommendations    # comma-separated action list
sensor_msgs/NavSatFix location   # where the detection was made
string image_path         # path to saved evidence image
string[] secondary_labels # runner-up detections
float32[] secondary_conf
