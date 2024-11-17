idf_component_register(SRCS "usb_cdc_example_main.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES usb usb_host_cdc_acm)
