idf_component_register(SRCS "temp.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver
                       PRIV_REQUIRES esp_adc
                       )
