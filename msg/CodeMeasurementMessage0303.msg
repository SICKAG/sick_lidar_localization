# Code measurement message type 3 version 3 (localization controller input message)
std_msgs/Header header
uint64 telegram_count     # 8 byte TelegramCount uint64
uint64 timestamp          # 8 byte Timestamp uint64 [microseconds]
int64  code               # 8 byte Code int32
