#lslidar Ch scan packets

Header           header         #standard ros message header
uint64           basetime       #time to the top of hour
LslidarChPacket[] packets        #vector of raw packets
