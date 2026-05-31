# TSR-1 Fire Detection Message
std_msgs/Header header

bool fire_detected
float32 confidence        # 0.0 - 1.0
float32 pixel_ratio       # fraction of frame that is fire-colored
int32 centroid_x          # pixel coords in camera frame
int32 centroid_y
float32 bbox_x            # normalized 0-1
float32 bbox_y
float32 bbox_w
float32 bbox_h
string source             # "webcam" | "ip_cam" | "thermal"
float32 area_m2           # estimated area (if range available)
