# Code measurement message type 3 version 4
std_msgs/Header header
uint64 telegram_count     # 8 byte TelegramCount uint64
uint64 timestamp          # 8 byte Timestamp uint64 [microseconds]
int32 code                # 4 byte Code int32
int32 distance            # 4 byte Distance int32 [mm]

uint32 sync_timestamp_sec   # seconds part of synchronized timestamp in system time calculated by Software-PLL
uint32 sync_timestamp_nsec  # nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
uint32 sync_timestamp_valid # 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
