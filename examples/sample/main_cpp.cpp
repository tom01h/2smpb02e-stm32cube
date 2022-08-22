#include <stdio.h>
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "Omron2SMPB02E.hpp"

extern "C" void main_cpp(I2C_HandleTypeDef * hi2c1)
{
    uint8_t SDO = 1;
    Omron2SMPB02E prs(hi2c1, SDO);

    u_int8_t buffer[256];
    
    prs.begin();
    prs.set_mode(MODE_NORMAL);
    prs.set_average(AVG_8, AVG_32);
    prs.set_filter(FILTER_8);

    while( 1 ){
        float height = prs.read_height();
        float temp = prs.get_temp();
        float press = prs.get_pressure();

        sprintf((char *)buffer, "temperature: %5.2f C\tpressure: %7.2f hP\theight: %6.2f m\r\n", temp, press, height);
        CDC_Transmit_FS(buffer,strlen((char *)buffer));

        //float temp = prs.read_temp();
        //sprintf((char *)buffer, "temperature: %5.2f C\t", temp);
        //CDC_Transmit_FS(buffer,strlen((char *)buffer));

        //float press = prs.read_pressure();
        //sprintf((char *)buffer, "pressure: %8.2f hP\t", press);
        //CDC_Transmit_FS(buffer,strlen((char *)buffer));

        //float height = prs.read_height();
        //sprintf((char *)buffer, "height: %6.2f m\r\n", height);
        //CDC_Transmit_FS(buffer,strlen((char *)buffer));

	    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	    HAL_Delay(200);
    }
}
