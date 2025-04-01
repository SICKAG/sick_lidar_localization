# Encoder measurement message type 2 version 2 (LLS device input message)
std_msgs/Header header
uint64 telegram_count     # 8 byte TelegramCount uint64
uint64 timestamp          # 8 byte Timestamp uint64 [microseconds]
int32 source_id           # 4 byte source id
int64 encoder_value       # 8 byte EncoderValue int64 in tics
