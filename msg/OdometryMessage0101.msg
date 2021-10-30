# Odometry message type 1 version 1 (localization controller input message)
std_msgs/Header header
uint64 telegram_count     # 8 byte TelegramCount uint64
uint64 timestamp          # 8 byte Timestamp uint64 [milliseconds]
int32 x_velocity          # 4 byte X - velocity int16 [mm/s]
int32 y_velocity          # 4 byte Y - velocity int16 [mm/s]
int64 angular_velocity    # 8 byte angular velocity int32 [mdeg/s]
