# Raw Leishen LIDAR packet.

#Header header           #standard ros message header
time stamp
uint8[1206] data        # packet contents
float32[4] prism_angle
float32 prism_offset_angle

