idf_component_register(
    SRCS bmp180emH.c
    INCLUDE_DIRS .
    REQUIRES bmp180 linked_list i2c i2cdev client progresser timesync logger
)