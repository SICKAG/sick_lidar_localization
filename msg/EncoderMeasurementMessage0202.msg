# Encoder measurement message type 2 version 2 (localization controller input message)
std_msgs/Header header
uint64 telegram_count     # 8 byte TelegramCount uint64
uint64 timestamp          # 8 byte Timestamp uint64 [microseconds]
int64 encoder_value       # 8 byte EncoderValue int64 in tics
