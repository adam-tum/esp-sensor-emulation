idf_component_register(
    SRCS device_manager.c
    INCLUDE_DIRS .
    REQUIRES i2c_bitbang bmp180em timesync
)