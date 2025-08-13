idf_component_register(SRCS "esp32_serial_transport.c" "main.c"
                    PRIV_REQUIRES spi_flash
                    INCLUDE_DIRS ""
                    REQUIRES driver micro_ros_espidf_component)
