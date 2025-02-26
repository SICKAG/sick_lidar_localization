# Odometry message type 1 version 5
std_msgs/Header header
uint64 telegram_count     # 8 byte TelegramCount uint64
uint64 timestamp          # 8 byte Timestamp uint64 [microseconds]
int32 source_id           # 4 byte source id
int64 x_position          # 8 byte X-position int64 [mm]
int64 y_position          # 8 byte Y-position int64 [mm]
int64 heading             # 8 byte heading int32 [mdeg]

uint32 sync_timestamp_sec   # seconds part of synchronized timestamp in system time calculated by Software-PLL
uint32 sync_timestamp_nsec  # nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
uint32 sync_timestamp_valid # 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
