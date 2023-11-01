/*
 * Copyright (c) 2019, Texas Instruments Incorporated
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

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <mqueue.h>
#include <stddef.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/ADCBuf.h>
/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "ti_drivers_config.h"
#include <ti_radio_config.h>
#include <ti/display/Display.h>
/***** Defines *****/

/* Do power measurement */
//#define POWER_MEASUREMENT

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      30
#ifdef POWER_MEASUREMENT
#define PACKET_INTERVAL     5  /* For power measurement set packet interval to 5s */
#else
#define PACKET_INTERVAL     500000  /* Set packet interval to 500000us or 500ms */
#endif

#define ADCSAMPLESIZE    (5)
#define MAX_QUEUED_ADC_CONVERSIONS (4)
#define QUEUED_ADC_MESSAGE_SIZE (sizeof(uint32_t) * ADCSAMPLESIZE)
uint16_t sampleBufferOne[ADCSAMPLESIZE];
uint16_t sampleBufferTwo[ADCSAMPLESIZE];
uint32_t microVoltBuffer[ADCSAMPLESIZE];
uint32_t outputBuffer[ADCSAMPLESIZE];
uint32_t buffersCompletedCounter = 0;

/***** Prototypes *****/

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static uint8_t packet[PAYLOAD_LENGTH];
static uint16_t seqNumber=8224; // 0x2020
/* Display Driver Handle */
Display_Handle displayHandle;
/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
/* Used to pass ADC data between callback and main task */
static mqd_t queueReceive;
static mqd_t queueSend;

PIN_Config pinTable[] =
{
    CONFIG_PIN_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion,
    void *completedADCBuffer, uint32_t completedChannel, int_fast16_t status)
{
    /* Adjust raw ADC values and convert them to microvolts */
    ADCBuf_adjustRawValues(handle, completedADCBuffer, ADCSAMPLESIZE,
        completedChannel);
    ADCBuf_convertAdjustedToMicroVolts(handle, completedChannel,
        completedADCBuffer, microVoltBuffer, ADCSAMPLESIZE);

    /* Send ADC data to main thread using message queue */
    mq_send(queueSend, (char *) microVoltBuffer, QUEUED_ADC_MESSAGE_SIZE, 0);
}
/***** Function definitions *****/

void *mainThread(void *arg0)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    Display_Params displayParams;
    Display_Params_init(&displayParams);
    displayParams.lineClearMode = DISPLAY_CLEAR_BOTH;
    displayHandle = Display_open(Display_Type_UART, &displayParams);
    if (displayHandle == NULL) {
         Display_printf(displayHandle, 0, 0, "Error creating displayHandle\n");
         while (1);
    }

    Display_printf(displayHandle, 0, 0,
                            "Starting the TX example");

    ADCBuf_Handle adcBuf;
    ADCBuf_Params adcBufParams;
    ADCBuf_Conversion continuousConversion;
    struct mq_attr attr;
    uint32_t average;
    uint_fast16_t i = 0;
    attr.mq_flags = 0;
    attr.mq_maxmsg = MAX_QUEUED_ADC_CONVERSIONS;
    attr.mq_msgsize = QUEUED_ADC_MESSAGE_SIZE;
    attr.mq_curmsgs = 0;

    queueReceive = mq_open("ADCBuf", O_RDWR | O_CREAT, 0664, &attr);
    queueSend = mq_open("ADCBuf", O_RDWR | O_CREAT | O_NONBLOCK, 0664,
                                  &attr);
    if ((queueReceive == (mqd_t)-1) || (queueSend == (mqd_t)-1)) {
                /* Failed to open message queue */
        while (1);
    }

            /* Call driver init functions */
     ADCBuf_init();

     /* Set up an ADCBuf peripheral in ADCBuf_RECURRENCE_MODE_CONTINUOUS */
     ADCBuf_Params_init(&adcBufParams);
     adcBufParams.callbackFxn = adcBufCallback;
     adcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_CONTINUOUS;
     adcBufParams.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
     adcBufParams.samplingFrequency = 200;
     adcBuf = ADCBuf_open(CONFIG_ADCBUF_0, &adcBufParams);

      /* Configure the conversion struct */
     continuousConversion.arg = NULL;
     continuousConversion.adcChannel = CONFIG_ADCBUF_0_CHANNEL_0;
     continuousConversion.sampleBuffer = sampleBufferOne;
     continuousConversion.sampleBufferTwo = sampleBufferTwo;
     continuousConversion.samplesRequestedCount = ADCSAMPLESIZE;

     if (adcBuf == NULL){
             /* ADCBuf failed to open. */
         while(1);
     }

         /* Start converting. */
     if (ADCBuf_convert(adcBuf, &continuousConversion, 1) !=
         ADCBuf_STATUS_SUCCESS) {
             /* Did not start conversion process correctly. */
         while(1);
     }
    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTx.syncWord = 0x930B51D5;

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    while(1)
    {
        if (mq_receive(queueReceive, (char *) outputBuffer,
                              QUEUED_ADC_MESSAGE_SIZE, NULL) == -1)
        {
              Display_printf(displayHandle, 0, 0, "Error receiving ADC message");
              while(1);
        }

        Display_printf(displayHandle, 0, 0, "Buffer %u finished:",
                           (unsigned int)buffersCompletedCounter++);

               /* Calculate average ADC data value in uV */
        average = 0;
        for (i = 0; i < ADCSAMPLESIZE; i++) {
             average += outputBuffer[i];
        }
        average = average/ADCSAMPLESIZE;

        /* Print average ADCBuf value */
        Display_printf(displayHandle, 0, 0, "  Average: %u uV", average);\
        char text[10];
        sprintf(text, "%d", average);
        /* Create packet with incrementing sequence number and random payload */
        packet[0] = (uint8_t)(seqNumber >> 8);
        packet[1] = (uint8_t)(seqNumber);
        uint8_t i;
        for (i = 2; i < PAYLOAD_LENGTH; i++)
        {
            packet[i] = text[i-2];
        }

        /* Send packet */
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                                   RF_PriorityNormal, NULL, 0);

        switch(terminationReason)
        {
            case RF_EventLastCmdDone:
                // A stand-alone radio operation command or the last radio
                // operation command in a chain finished.
                break;
            case RF_EventCmdCancelled:
                // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
                break;
            case RF_EventCmdAborted:
                // Abrupt command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            case RF_EventCmdStopped:
                // Graceful command termination caused by RF_cancelCmd() or
                // RF_flushCmd().
                break;
            default:
                // Uncaught error event
                while(1);
        }

        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
        switch(cmdStatus)
        {
            case PROP_DONE_OK:
                // Packet transmitted successfully
                break;
            case PROP_DONE_STOPPED:
                // received CMD_STOP while transmitting packet and finished
                // transmitting packet
                break;
            case PROP_DONE_ABORT:
                // Received CMD_ABORT while transmitting packet
                break;
            case PROP_ERROR_PAR:
                // Observed illegal parameter
                break;
            case PROP_ERROR_NO_SETUP:
                // Command sent without setting up the radio in a supported
                // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                break;
            case PROP_ERROR_NO_FS:
                // Command sent without the synthesizer being programmed
                break;
            case PROP_ERROR_TXUNF:
                // TX underflow observed during operation
                break;
            default:
                // Uncaught error event - these could come from the
                // pool of states defined in rf_mailbox.h
                while(1);
        }

#ifndef POWER_MEASUREMENT
        PIN_setOutputValue(ledPinHandle, CONFIG_PIN_GLED,!PIN_getOutputValue(CONFIG_PIN_GLED));
#endif
        /* Power down the radio */
        RF_yield(rfHandle);

#ifdef POWER_MEASUREMENT
        /* Sleep for PACKET_INTERVAL s */
        sleep(PACKET_INTERVAL);
#else
        /* Sleep for PACKET_INTERVAL us */
        usleep(PACKET_INTERVAL);
#endif

    }
}
