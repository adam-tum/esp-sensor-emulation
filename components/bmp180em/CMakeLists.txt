idf_component_register(
    SRCS bmp180em.c
    INCLUDE_DIRS .
    REQUIRES bmp180 i2c_bitbang linked_list client progresser timesync logger
)