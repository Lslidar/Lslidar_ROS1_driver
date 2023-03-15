# Raw Leishen LIDAR packet.
Header header           #standard ros message header
time stamp              # packet timestamp
float64[4] prism_angle
uint8[1206] data        # packet contents

