# Definition of ros message SickLocResultPortTelegramMsg.
# SickLocResultPortTelegramMsg publishes result port telegrams
# for sick localization. See chapter 5.9 ("About result port telegrams")
# of the operation manual for further details.

Header header                                 # ROS Header with sequence id, timestamp and frame id

SickLocResultPortHeaderMsg   telegram_header  # 52 byte header of a result port telegram
SickLocResultPortPayloadMsg  telegram_payload # 52 byte payload of a result port telegram
SickLocResultPortCrcMsg      telegram_trailer #  2 byte CRC trailer of a result port telegram

