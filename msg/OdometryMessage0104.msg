# Odometry message type 1 version 4
std_msgs/Header header
uint64 telegram_count     # 8 byte TelegramCount uint64
uint64 timestamp          # 8 byte Timestamp uint64 [microseconds]
int16 x_velocity          # 2 byte X - velocity int16 [mm/s]
int16 y_velocity          # 2 byte Y - velocity int16 [mm/s]
int32 angular_velocity    # 4 byte angular velocity int32 [mdeg/s]

uint32 sync_timestamp_sec   # seconds part of synchronized timestamp in system time calculated by Software-PLL
uint32 sync_timestamp_nsec  # nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
uint32 sync_timestamp_valid # 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
