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
* @file  bmm350_self_test.c
*
* @brief This file contains reading of magnetometer data in normal mode by performing before and after self-test.
*/

#include <stdio.h>
#include "coines.h"
#include "common.h"
#include "bmm350.h"

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program */
int main(void)
{
    /* Status of api are returned to this variable */
    int8_t rslt;

    /* Sensor initialization configuration */
    struct bmm350_dev dev = { 0 };

    struct bmm350_self_test out_data;

    uint8_t err_reg_data = 0;
    uint8_t loop = 10;
    uint32_t time_ms = 0;

    struct bmm350_mag_temp_data mag_temp_data;
    struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0;

    /* Update device structure */
    rslt = bmm350_interface_init(&dev);
    bmm350_error_codes_print_result("bmm350_interface_selection", rslt);

    /* Initialize BMM350 */
    rslt = bmm350_init(&dev);
    bmm350_error_codes_print_result("bmm350_init", rslt);

    printf("Read : 0x00 : BMM350 Chip ID : 0x%X\n", dev.chip_id);

    /* Check PMU busy */
    rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &dev);
    bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);

    printf("Expected : 0x07 : PMU cmd busy : 0x0\n");
    printf("Read : 0x07 : PMU cmd busy : 0x%X\n", pmu_cmd_stat_0.pmu_cmd_busy);

    /* Get error data */
    rslt = bmm350_get_regs(BMM350_REG_ERR_REG, &err_reg_data, 1, &dev);
    bmm350_error_codes_print_result("bmm350_get_error_reg_data", rslt);

    printf("Expected : 0x02 : Error Register : 0x0\n");
    printf("Read : 0x02 : Error Register : 0x%X\n", err_reg_data);

    /* Set ODR and performance */
    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_100HZ, BMM350_AVERAGING_4, &dev);
    bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

    rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
    bmm350_error_codes_print_result("bmm350_set_powermode", rslt);

    rslt = bmm350_delay_us(10000, &dev);
    bmm350_error_codes_print_result("bmm350_delay_us", rslt);

    /* Enable all axis */
    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &dev);
    bmm350_error_codes_print_result("bmm350_enable_axes", rslt);

    if (rslt == BMM350_OK)
    {
        printf("\n**********BEFORE SELFTEST**********\n");

        printf("\nPower mode is set to normal mode\n");

        printf("\nCompensated Magnetometer and temperature data with delay\n");

        printf("Timestamp(ms), Mag_X(uT), Mag_Y(uT), Mag_Z(uT), Temperature(degC)\n");

        /* Time in milliseconds */
        time_ms = coines_get_millis();

        while (loop)
        {
            rslt = bmm350_delay_us(100000, &dev);
            bmm350_error_codes_print_result("bmm350_delay_us", rslt);

            rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &dev);
            bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data", rslt);

            printf("%lu, %f, %f, %f, %f\n",
                   (long unsigned int)(coines_get_millis() - time_ms),
                   mag_temp_data.x,
                   mag_temp_data.y,
                   mag_temp_data.z,
                   mag_temp_data.temperature);

            loop--;
        }

        printf("\nSelf-test data in suspend mode\n");

        printf("\nIteration, OUT_UST_X, OUT_UST_Y\n");

        while (loop < 20)
        {
            rslt = bmm350_perform_self_test(&out_data, &dev);
            bmm350_error_codes_print_result("bmm350_perform_self_test", rslt);

            printf("%d, %f, %f\n", loop, out_data.out_ust_x, out_data.out_ust_y);

            loop++;
        }

        printf("\n**********AFTER SELFTEST**********\n");

        loop = 20;

        printf("\nCompensated Magnetometer and temperature data\n");

        printf("Timestamp(ms), Mag_X(uT), Mag_Y(uT), Mag_Z(uT), Temperature(degC)\n");

        /* Time in milliseconds */
        time_ms = coines_get_millis();

        while (loop)
        {
            rslt = bmm350_delay_us(10000, &dev);
            bmm350_error_codes_print_result("bmm350_delay_us", rslt);

            rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &dev);
            bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data", rslt);

            printf("%lu, %f, %f, %f, %f\n",
                   (long unsigned int)(coines_get_millis() - time_ms),
                   mag_temp_data.x,
                   mag_temp_data.y,
                   mag_temp_data.z,
                   mag_temp_data.temperature);

            loop--;
        }

        printf("\nSelf-test data in normal mode\n");

        rslt = bmm350_perform_self_test(&out_data, &dev);
        bmm350_error_codes_print_result("bmm350_perform_self_test", rslt);

        printf("Self-test data\n");

        printf("\nOUT_UST_X, OUT_UST_Y\n");

        printf("%f, %f\n", out_data.out_ust_x, out_data.out_ust_y);

        loop = 20;

        printf("\nCompensated Magnetometer and temperature data\n");

        printf("Timestamp(ms), Mag_X(uT), Mag_Y(uT), Mag_Z(uT), Temperature(degC)\n");

        /* Time in milliseconds */
        time_ms = coines_get_millis();

        while (loop)
        {
            rslt = bmm350_delay_us(10000, &dev);
            bmm350_error_codes_print_result("bmm350_delay_us", rslt);

            rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_temp_data, &dev);
            bmm350_error_codes_print_result("bmm350_get_compensated_mag_xyz_temp_data", rslt);

            printf("%lu, %f, %f, %f, %f\n",
                   (long unsigned int)(coines_get_millis() - time_ms),
                   mag_temp_data.x,
                   mag_temp_data.y,
                   mag_temp_data.z,
                   mag_temp_data.temperature);

            loop--;
        }
    }

    bmm350_coines_deinit();

    return rslt;
}
