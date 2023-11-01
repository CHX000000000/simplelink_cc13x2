/*
 * Copyright (c) 2016-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *    ======== i2cbme280.c ========
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
/* POSIX Header files */
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/drivers/apps/Button.h>
#include <ti/drivers/utils/RingBuf.h>

#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* Module Header */
//#include <ti/sail/bme280/bme280.h>
#include "bmp180.h"

/* Driver configuration */
#include "ti_drivers_config.h"

#ifndef CONFIG_BUTTONCOUNT
#define CONFIG_BUTTONCOUNT     2
#endif

#define BLINKCOUNT            3
#define FASTBLINK             500
#define SLOWBLINK             1000
#define FIFTYMS               50000
#define EVENTBUFSIZE          10

s32    g_s32ActualTemp   = 0;
u32    g_u32ActualPress  = 0;
u32    g_u32ActualHumity = 0;

I2C_Handle      i2c;
I2C_Params      i2cParams;


static Display_Handle display;


typedef struct buttonStats
{
    unsigned int pressed;
    unsigned int clicked;
    unsigned int released;
    unsigned int longPress;
    unsigned int longClicked;
    unsigned int doubleclicked;
    unsigned int lastpressedduration;
} buttonStats;

Button_Handle    buttonHandle[CONFIG_BUTTONCOUNT];
buttonStats      bStats;
RingBuf_Object   ringObj;
uint8_t          eventBuf[EVENTBUFSIZE];
Bool Index;

extern s32 bmp180_data_readout_template(I2C_Handle i2cHndl);


/*
 *  ======== mainThread ========
 */


//void BME280_delay_msek(BMP180_MDELAY_DATA_TYPE msek)
//{
//    usleep(msek*1000);
//}

void handleButtonCallback(Button_Handle handle, Button_EventMask events)
{
    Index = (buttonHandle[CONFIG_BUTTON_0] == handle) ? 0 : 1;


    if(Button_EV_PRESSED == (events & Button_EV_PRESSED))
    {
        bStats.pressed++;
    }

    if(Button_EV_RELEASED == (events & Button_EV_RELEASED))
    {
        bStats.released++;
    }

    if(Button_EV_CLICKED == (events & Button_EV_CLICKED))
    {
        bStats.clicked++;
        bStats.lastpressedduration =
        Button_getLastPressedDuration(handle);

        /* Put event in ring buffer for printing */
        RingBuf_put(&ringObj, events);


    }

    if(Button_EV_LONGPRESSED == (events & Button_EV_LONGPRESSED))
    {
        bStats.longPress++;

        /* Put event in ring buffer for printing */
        RingBuf_put(&ringObj, events);
    }

    if(Button_EV_LONGCLICKED == (events & Button_EV_LONGCLICKED))
    {
        bStats.longClicked++;
        bStats.lastpressedduration = Button_getLastPressedDuration(handle);
    }

    if(Button_EV_DOUBLECLICKED == (events & Button_EV_DOUBLECLICKED))
    {
        bStats.doubleclicked++;

        /* Put event in ring buffer for printing */
        RingBuf_put(&ringObj, events);

    }
}

void *mainThread(void *arg0)
{
    Button_Params  buttonParams;
    /* Call driver init functions */
    I2C_init();
    GPIO_init();
    Button_init();

    float pressure;
    float  temp;

    /* Create ring buffer to store button events */
    RingBuf_construct(&ringObj, eventBuf, EVENTBUFSIZE);

    // bmp180.delay_msec = BME280_delay_msek;

    /* Open the HOST display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        while (1);
    }
    Display_print0(display, 0, 0, "Starting the i2cbmp180 sensor example...\n\n");

    Button_Params_init(&buttonParams);
    buttonHandle[CONFIG_BUTTON_0] = Button_open(CONFIG_BUTTON_0,
                                                  handleButtonCallback,
                                                  &buttonParams);
    buttonHandle[CONFIG_BUTTON_1] = Button_open(CONFIG_BUTTON_1,
                                                  handleButtonCallback,
                                                  &buttonParams);

    /* Check if the button open is successful */
    if((buttonHandle[CONFIG_BUTTON_1]  == NULL) ||
        (buttonHandle[CONFIG_BUTTON_0]  == NULL))
    {
        Display_print0(display, 0, 0, "Button Open Failed!");
    }

    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.transferCallbackFxn = NULL;


    i2c = I2C_open(CONFIG_I2C_BME, &i2cParams);
    if (i2c == NULL) {
        Display_print0(display, 0, 0, "Error Initializing I2C\n");
    }
    else {
        Display_print0(display, 0, 0, "I2C Initialized!\n");
    }

    Display_print1(display, 0, 0, "bmp180_data_readout_template :%u  \n", bmp180_data_readout_template(i2c));

    if(BMP180_INIT_VALUE != bmp180_data_readout_template(i2c))
    {
        Display_print0(display, 0, 0, "Error Initializing bme180\n");
    }
    /* Initialize the BME Sensor */

    while(1)
    {
            pressure = bmp180_get_pressure(bmp180_get_uncomp_pressure());

            temp     = bmp180_get_temperature(bmp180_get_uncomp_temperature());
            //           temp     = bmp180_get_uncomp_temperature();
            
            uint8_t event;
            while(RingBuf_get(&ringObj, &event) >= 0)
            {
                if(event & Button_EV_CLICKED)
                {
                    if(Index) Display_print1(display, 0, 0, "%5.2f KPa(Pressure)\n",pressure/1000);
                    else Display_print1(display, 0, 0, "%4.2f DegC(Temp) \n", temp/10 );
                }
            }
            usleep(FIFTYMS);
    }
}

