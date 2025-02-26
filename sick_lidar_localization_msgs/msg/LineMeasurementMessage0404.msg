# Line measurement message type 4 version 3
std_msgs/Header header
uint64 telegram_count     # 8 byte TelegramCount uint64
uint64 timestamp          # 8 byte Timestamp uint64 [microseconds]
int32 source_id           # 4 byte source id
int16 lcp1                # 2 byte lcp1 int16 [mm]
int16 lcp2                # 2 byte lcp2 int16 [mm]
int16 lcp3                # 2 byte lcp3 int16 [mm]
uint8 cnt_lpc             # 1 byte cnt_lpc uint8

uint32 sync_timestamp_sec   # seconds part of synchronized timestamp in system time calculated by Software-PLL
uint32 sync_timestamp_nsec  # nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
uint32 sync_timestamp_valid # 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
