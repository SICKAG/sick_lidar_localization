# Code measurement message type 7 version 1 (LLS device input message for 2D codes)
std_msgs/Header header
uint64 telegram_count     # 8 byte TelegramCount uint64
uint64 timestamp          # 8 byte Timestamp uint64 [microseconds]
int32 source_id           # 4 byte source id
string code               # variable length string # udp message: 2 byte codelength (uint16) + <codelength> byte string
int32 x_position          # 4 byte relative pose along the x-axis of the sensor frame [mm]
int32 y_position          # 4 byte relative pose along the y-axis of the sensor frame [mm]
int32 heading             # 4 byte relative orientation of the code in the sensor frame [mdeg]
