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
* @file  bmm350_sensor_time.c
*
* @brief This file contains reading of sensortime data with respect to power modes, ODR and average.
*
*/

#include <stdio.h>
#include "bmm350.h"
#include "common.h"
#include "coines.h"

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program */
int main(void)
{
    /* Status of api are returned to this variable */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bmm350_dev dev = { 0 };

    uint8_t loop = 20;
    uint32_t secs, nano_secs = 0;

    /* Update device structure */
    rslt = bmm350_interface_init(&dev);
    bmm350_error_codes_print_result("bmm350_interface_selection", rslt);

    /* Initialize BMM350 */
    rslt = bmm350_init(&dev);
    bmm350_error_codes_print_result("bmm350_init", rslt);

    printf("Read : 0x00 : BMM350 Chip ID : 0x%X\n", dev.chip_id);

    /* Set ODR and performance */
    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_100HZ, BMM350_AVERAGING_4, &dev);
    bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

    /* Enable all axis */
    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
    bmm350_error_codes_print_result("bmm350_enable_axes", rslt);

    if (rslt == BMM350_OK)
    {
        rslt = bmm350_set_powermode(BMM350_SUSPEND_MODE, &dev);
        bmm350_error_codes_print_result("bmm350_set_powermode", rslt);

        printf("\nSensortime in suspend mode\n");
        printf("Time(secs)\n");

        while (loop > 0)
        {
            rslt = bmm350_read_sensortime(&secs, &nano_secs, &dev);
            bmm350_error_codes_print_result("bmm350_read_sensortime", rslt);

            printf("%lu.%09lu\n", (long unsigned int)secs, (long unsigned int)nano_secs);

            loop--;
        }

        rslt = bmm350_set_ctrl_user(BMM350_CFG_SENS_TIM_AON_EN, &dev);
        bmm350_error_codes_print_result("bmm350_set_ctrl_user", rslt);

        loop = 20;

        printf("\nSensortime in forced mode\n");

        printf("Time(secs)\n");

        while (loop > 0)
        {
            rslt = bmm350_set_powermode(BMM350_FORCED_MODE, &dev);
            bmm350_error_codes_print_result("bmm350_set_powermode", rslt);

            rslt = bmm350_delay_us(40000, &dev);
            bmm350_error_codes_print_result("bmm350_delay_us", rslt);

            rslt = bmm350_read_sensortime(&secs, &nano_secs, &dev);
            bmm350_error_codes_print_result("bmm350_read_sensortime", rslt);

            printf("\n%lu.%09lu\n", (long unsigned int)secs, (long unsigned int)nano_secs);

            rslt = bmm350_read_sensortime(&secs, &nano_secs, &dev);
            bmm350_error_codes_print_result("bmm350_read_sensortime", rslt);

            printf("%lu.%09lu\n", (long unsigned int)secs, (long unsigned int)nano_secs);

            rslt = bmm350_read_sensortime(&secs, &nano_secs, &dev);
            bmm350_error_codes_print_result("bmm350_read_sensortime", rslt);

            printf("%lu.%09lu\n", (long unsigned int)secs, (long unsigned int)nano_secs);

            loop--;
        }

        rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
        bmm350_error_codes_print_result("bmm350_set_powermode", rslt);

        /* Set ODR and performance */
        rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_100HZ, BMM350_AVERAGING_2, &dev);
        bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

        loop = 20;

        printf("\nChange in ODR\n");

        printf("Time(secs)\n");

        while (loop > 0)
        {
            rslt = bmm350_delay_us(11000, &dev);
            bmm350_error_codes_print_result("bmm350_delay_us", rslt);

            rslt = bmm350_read_sensortime(&secs, &nano_secs, &dev);
            bmm350_error_codes_print_result("bmm350_read_sensortime", rslt);

            printf("%lu.%09lu\n", (long unsigned int)secs, (long unsigned int)nano_secs);

            loop--;
        }
    }

    bmm350_coines_deinit();

    return rslt;
}
