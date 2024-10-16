# difop data


# 40 0 正常 1 异常
string scan_system_status

# 44 0 gps 1 ptp
string  clock_source_selection

# 86 0 正常 1 异常 未锁定
string ptp_status

# 92  0 正常  1 异常，丢失
string gps_status

# 93  0 正常  1 异常，丢失
string pps_status

# 100 0=标称帧率  1=50%帧率 2= 25%帧率
string frame_rate

# 102 0=close  1=open
string phase_lock_switch

#103  0= unlock 1= lock
string phase_lock_status

# 114 115  data *  503.975 / 4096 - 273.15
float32 FPGA_temp

# 116 117    data / 65535 * 175 - 45
float32 main_board_temp

# 118 119 data / 65535
float32 main_board_humidity

# 120-123 单位分钟
uint32 total_lidar_operation_time

# 124-126  单位分钟  低于零下40度运行时间
uint32  less_than_minus_40_temp_operation_time

# 127-129  单位分钟  低于零下10度运行时间
uint32  less_than_minus_10_temp_operation_time

# 130-132  单位分钟  低于30度运行时间
uint32  less_than_30_temp_operation_time

# 133-135  单位分钟  低于70度运行时间
uint32  less_than_70_temp_operation_time

# 136-138  单位分钟  低于100度运行时间
uint32  less_than_100_temp_operation_time



