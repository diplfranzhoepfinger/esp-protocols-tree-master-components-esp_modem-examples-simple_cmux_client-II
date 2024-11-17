#include <stdio.h>
#include "modem_pwkey.h"

extern void simple_cmux_client_main(void);

extern void usb_cdc_example_main(void);


void app_main(void)
{



    init_modem_pwkey();

    // power Up does work now. .
    power_up_modem_pwkey();

    usb_cdc_example_main();

    simple_cmux_client_main();



}
