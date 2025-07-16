idf_component_register(SRCS "hid_host_example.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES usb esp_driver_gpio esp_wifi esp_netif nvs_flash
                       )
