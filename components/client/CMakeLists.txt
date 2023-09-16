idf_component_register(
    SRCS client.c
    INCLUDE_DIRS .
    REQUIRES i2c_bitbang device_manager timesync connect logger
)