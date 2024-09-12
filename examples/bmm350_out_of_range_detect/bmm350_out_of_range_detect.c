/**
* Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file  bmm350_out_of_range_detect.c
*
* @brief This file contains an example to robustly detect if the sensor is out of range, even in presence
* of high-magnetic disturbances
*
*/

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "bmm350.h"
#include "bmm350_oor.h"
#include "common.h"
#include "coines.h"

/******************************************************************************/
/*!           Static Variable Declaration                                     */

/* Flag to track when to trigger the magnetic reset */
static bool trigger_magnetic_reset = false;

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program */
int main(void)
{
    /* Status of api are returned to this variable */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bmm350_dev dev = { 0 };

    /* Structure instance of magnetometer and temperature data */
    struct bmm350_mag_temp_data mag_temp_data = { 0 };

    /* Structure instance of self-test data */
    struct bmm350_self_test st_data = { 0 };

    /* Structure instance of out of range detect */
    struct bmm350_oor_params oor = { 0 };

    enum coines_pin_direction btn_dir = COINES_PIN_DIRECTION_IN;
    enum coines_pin_value btn_value = COINES_PIN_VALUE_HIGH;

    bool out_of_range = false;
    float field_str = 0.0f;
    uint16_t delay_in_us;

    /* Variable to store last measurement time */
    uint32_t last_meas_time = 0;

    /* Update device structure */
    rslt = bmm350_interface_init(&dev);
    bmm350_error_codes_print_result("bmm350_interface_selection", rslt);

    (void)coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF);

    /* Initialize BMM350 */
    rslt = bmm350_init(&dev);
    bmm350_error_codes_print_result("bmm350_init", rslt);

    printf("Read : 0x00 : BMM350 Chip ID : 0x%X\n", dev.chip_id);

    /* Enable all axis */
    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
    bmm350_error_codes_print_result("bmm350_enable_axes", rslt);

    rslt = bmm350_perform_self_test(&st_data, &dev);
    bmm350_error_codes_print_result("bmm350_perform_self_test", rslt);
    if ((st_data.out_ust_x < BMM350_FULL_ST_THRESHOLD) || (st_data.out_ust_y < BMM350_FULL_ST_THRESHOLD))
    {
        out_of_range = true;
    }

    /* Set ODR to 200hz for best performance */
    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_200HZ, BMM350_AVERAGING_2, &dev);
    bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

    rslt = bmm350_set_powermode(BMM350_FORCED_MODE_FAST, &dev);
    bmm350_error_codes_print_result("bmm350_set_powermode", rslt);

    last_meas_time = coines_get_realtime_usec();

    /* For 200Hz ODR, poll at 5ms */
    delay_in_us = 5000;

    printf("Measuring data from the sensor\n");
    printf(
        "\nMx, My, Mz, field_str, OutOfRange, Xfailed, Yfailed, ST DX, ST DY, Reset, Reset counter, ST active, ST counter\n");

    /* Read data indefinitely */
    while (rslt == BMM350_OK)
    {
        /* Read data when ready */
        if ((coines_get_realtime_usec() - last_meas_time) >= delay_in_us)
        {
            last_meas_time = coines_get_realtime_usec();

            if (oor.trigger_reset)
            {
                rslt = bmm350_oor_perform_reset_sequence_forced(&oor, &dev);
            }
            else
            {
                rslt = bmm350_oor_read(&out_of_range, &mag_temp_data, &oor, &dev);
            }

            /* Light up the LED Green or Red to indicate out of range */
            if (out_of_range)
            {
                (void)coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON);
                (void)coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_OFF);
                (void)coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_1_4, COINES_PIN_DIRECTION_OUT,
                                            COINES_PIN_VALUE_HIGH);
            }
            else
            {
                (void)coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF);
                (void)coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_ON);
                (void)coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_1_4, COINES_PIN_DIRECTION_OUT,
                                            COINES_PIN_VALUE_LOW);
            }

            field_str =
                sqrtf((mag_temp_data.x * mag_temp_data.x) + (mag_temp_data.y * mag_temp_data.y) +
                      (mag_temp_data.z * mag_temp_data.z));

            printf("%.2f\t%.2f\t%.2f\t%.2f\t%d\t%u\t%u\t%.2f\t%.2f\t%u\t%u\t%u\t%u\n",
                   mag_temp_data.x,
                   mag_temp_data.y,
                   mag_temp_data.z,
                   field_str,
                   (uint8_t)out_of_range,
                   oor.x_failed,
                   oor.y_failed,
                   oor.mag_xp - oor.mag_xn,
                   oor.mag_yp - oor.mag_yn,
                   oor.trigger_reset,
                   oor.reset_counter,
                   oor.enable_selftest,
                   oor.st_counter);
        }

        /* Press the Button 1 to manually trigger a magnetic reset */
        btn_dir = COINES_PIN_DIRECTION_IN; /* Input */
        btn_value = COINES_PIN_VALUE_HIGH; /* Pull-up */
        bmm350_coines_get_button_state(BUTTON_1, &btn_dir, &btn_value);

        if ((btn_value == COINES_PIN_VALUE_LOW) || (out_of_range))
        {
            trigger_magnetic_reset = true;
        }

        /* Trigger the magnetic reset once the self tests have passed and the button is not pressed (active-low) */
        if (trigger_magnetic_reset && !out_of_range && (btn_value == COINES_PIN_VALUE_HIGH))
        {
            oor.trigger_reset = true;
            trigger_magnetic_reset = false;
        }
    }

    bmm350_coines_deinit();

    return rslt;
}
