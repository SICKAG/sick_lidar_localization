# Localization result message type 5 version 2
std_msgs/Header header
uint64 telegram_count     # 8 byte TelegramCount uint64
uint64 timestamp          # 8 byte Timestamp uint64 [microseconds]
int64 x                   # 8 byte X int64 [mm]
int64 y                   # 8 byte Y int64 [mm]
int32 heading             # 4 byte Heading int32 [mdeg]
uint8 loc_status          # 1 byte LocalizationStatus [0...100, 10: OK, 20: Warning, 30: Not localized, 40: System error]
uint8 map_match_status    # 1 byte MapMatchingStatus [0...100, 90: Good, 60: Medium, 30: Low, 0; None]

uint32 sync_timestamp_sec   # seconds part of synchronized timestamp in system time calculated by Software-PLL
uint32 sync_timestamp_nsec  # nanoseconds part of synchronized timestamp in system time calculated by Software-PLL
uint32 sync_timestamp_valid # 1: sync_timestamp successfully calculated by Software-PLL, 0: Software-PLL not yet synchronized
