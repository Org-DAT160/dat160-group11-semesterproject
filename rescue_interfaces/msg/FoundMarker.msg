std_msgs/Header header        # stamp = image time, frame_id = "map"
int8 marker_id                # 0-4
geometry_msgs/Point position  # plate centre in the map frame
float32 distance              # camera-to-marker distance in metres
string robot                  # "tb3_0" or "tb3_1"
