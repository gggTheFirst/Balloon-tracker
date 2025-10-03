idf_component_register(SRCS "gps.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                       REQUIRES nvs_flash)
