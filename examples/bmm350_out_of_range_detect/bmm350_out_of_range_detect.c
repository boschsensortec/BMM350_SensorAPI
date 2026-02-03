/**
* Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
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

/* Enable this Macro to enable the Out of Range Detection Application in Normal Mode */
/* By default, the Out of Range Detection Application is operated in Forced Mode */
/*#define BMM350_OOR_NORMAL_MODE */

/******************************************************************************/
/*!           Static Variable Declaration                                     */

/* Flag to track when to trigger the magnetic reset */
static bool trigger_magnetic_reset = false;

#ifdef BMM350_OOR_NORMAL_MODE
int8_t bmm350_normal_read(struct bmm350_mag_temp_data *data, struct bmm350_oor_params *oor, struct bmm350_dev *dev);

bool get_interrupt_status_normal(struct bmm350_dev *dev);

#else
int8_t bmm350_forced_read(struct bmm350_mag_temp_data *data, struct bmm350_oor_params *oor, struct bmm350_dev *dev);

#endif

void out_of_range_status(const struct bmm350_mag_temp_data *data, const struct bmm350_oor_params *oor);

bool get_interrupt_status_forced(uint32_t last_meas_time, struct bmm350_oor_reset_delay resetDelay);

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

    /* Variable to store last measurement time */
    uint32_t last_meas_time = 0;

#ifdef BMM350_OOR_NORMAL_MODE
    struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0;
#endif

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
        oor.out_of_range = true;
    }

    /* Set ODR to 200hz for best performance */
    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_200HZ, BMM350_AVERAGING_2, &dev);
    bmm350_error_codes_print_result("bmm350_set_odr_performance", rslt);

#ifdef BMM350_OOR_NORMAL_MODE

    /* Configure interrupt settings */
    rslt = bmm350_configure_interrupt(BMM350_PULSED,
                                      BMM350_ACTIVE_HIGH,
                                      BMM350_INTR_PUSH_PULL,
                                      BMM350_UNMAP_FROM_PIN,
                                      &dev);
    bmm350_error_codes_print_result("bmm350_configure_interrupt", rslt);

    /* Enable data ready interrupt */
    rslt = bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, &dev);
    bmm350_error_codes_print_result("bmm350_enable_interrupt", rslt);

    rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
    bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
#else
    rslt = bmm350_set_powermode(BMM350_FORCED_MODE_FAST, &dev);
    bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
#endif

    last_meas_time = coines_get_realtime_usec();

    struct bmm350_oor_reset_delay resetDelay = { 0 };
    bmm350_oor_compute_delay_setting(BMM350_DATA_RATE_200HZ, &resetDelay);

    printf("Measuring data from the sensor\n");
    printf(
        "\nMx, My, Mz, field_str, OutOfRange, Xfailed, Yfailed, ST DX, ST DY, Reset, Reset counter, ST active, ST counter\n");

    /* Read data indefinitely */
    while (rslt == BMM350_OK)
    {
        /* Read data when ready */
        if (oor.out_of_range || oor.trigger_reset)
        {
            if (get_interrupt_status_forced(last_meas_time, resetDelay))
            {
                last_meas_time = coines_get_realtime_usec();

                if (oor.trigger_reset)
                {
                    rslt = bmm350_oor_perform_reset_sequence(&resetDelay, &oor, &dev);
                }
                else
                {
#ifdef BMM350_OOR_NORMAL_MODE
                    rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &dev);
                    bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);

                    if (pmu_cmd_stat_0.pwr_mode_is_normal == BMM350_ENABLE)
                    {
                        rslt = bmm350_set_powermode(BMM350_SUSPEND_MODE, &dev);
                        bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
                    }

#endif
                    rslt = bmm350_oor_read(&resetDelay, &mag_temp_data, &oor, &dev);
                }

                out_of_range_status(&mag_temp_data, &oor);
            }
        }
        else
        {

#ifdef BMM350_OOR_NORMAL_MODE
            rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &dev);
            bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);

            if (pmu_cmd_stat_0.pwr_mode_is_normal != BMM350_ENABLE)
            {
                rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
                bmm350_error_codes_print_result("bmm350_set_powermode", rslt);
            }

            if (get_interrupt_status_normal(&dev))
            {
                last_meas_time = coines_get_realtime_usec();

                rslt = bmm350_normal_read(&mag_temp_data, &oor, &dev);

                out_of_range_status(&mag_temp_data, &oor);
            }

#else
            if (get_interrupt_status_forced(last_meas_time, resetDelay))
            {
                last_meas_time = coines_get_realtime_usec();

                rslt = bmm350_forced_read(&mag_temp_data, &oor, &dev);

                out_of_range_status(&mag_temp_data, &oor);
            }

#endif
        }

        /* Press the Button 1 to manually trigger a magnetic reset */
        btn_dir = COINES_PIN_DIRECTION_IN; /* Input */
        btn_value = COINES_PIN_VALUE_HIGH; /* Pull-up */
        bmm350_coines_get_button_state(BUTTON_1, &btn_dir, &btn_value);

        if ((btn_value == COINES_PIN_VALUE_LOW) || (oor.out_of_range))
        {
            trigger_magnetic_reset = true;
        }

        /* Trigger the magnetic reset once the self tests have passed and the button is not pressed (active-low) */
        if (trigger_magnetic_reset && !oor.out_of_range && (btn_value == COINES_PIN_VALUE_HIGH))
        {
            oor.trigger_reset = true;
            trigger_magnetic_reset = false;
        }
    }

    bmm350_coines_deinit();

    return rslt;
}

#ifdef BMM350_OOR_NORMAL_MODE
int8_t bmm350_normal_read(struct bmm350_mag_temp_data *data, struct bmm350_oor_params *oor, struct bmm350_dev *dev)
{
    int8_t rslt = 0;

#ifdef BMM350_USE_FIXED_POINT
    rslt = bmm350_get_compensated_mag_xyz_temp_data_fixed(data, dev);
#else
    rslt = bmm350_get_compensated_mag_xyz_temp_data(data, dev);
#endif

    bmm350_oor_validate_out_of_range(data, oor);

    return rslt;
}

#else
int8_t bmm350_forced_read(struct bmm350_mag_temp_data *data, struct bmm350_oor_params *oor, struct bmm350_dev *dev)
{
    int8_t rslt = 0;
    uint8_t pmu_cmd = BMM350_PMU_CMD_FM_FAST;

    rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &pmu_cmd, 1, dev);

    if (rslt == BMM350_OK)
    {
#ifdef BMM350_USE_FIXED_POINT
        rslt = bmm350_get_compensated_mag_xyz_temp_data_fixed(data, dev);
#else
        rslt = bmm350_get_compensated_mag_xyz_temp_data(data, dev);
#endif
    }

    bmm350_oor_validate_out_of_range(data, oor);

    return rslt;
}
#endif

void out_of_range_status(const struct bmm350_mag_temp_data *data, const struct bmm350_oor_params *oor)
{
    /* Light up the LED Green or Red to indicate out of range */
    if (oor->out_of_range)
    {
        (void)coines_set_led(COINES_LED_RED, COINES_LED_STATE_ON);
        (void)coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_OFF);
        (void)coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_1_4, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
    }
    else
    {
        (void)coines_set_led(COINES_LED_RED, COINES_LED_STATE_OFF);
        (void)coines_set_led(COINES_LED_GREEN, COINES_LED_STATE_ON);
        (void)coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_1_4, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
    }

#ifdef BMM350_USE_FIXED_POINT
    print_A48_16(data->x);
    printf("\t");
    print_A48_16(data->y);
    printf("\t");
    print_A48_16(data->z);
    printf("\t");
    printf("%lu\t%d\t%u\t%u\t ", oor->field_strength, (uint8_t)oor->out_of_range, oor->x_failed, oor->y_failed);
    print_A48_16(oor->mag_x_st_en - oor->mag_x_st_dis);
    printf("\t");
    print_A48_16(oor->mag_y_st_en - oor->mag_y_st_dis);
    printf("\t");
    printf("%u\t%d\t%d\t%d\n", oor->trigger_reset, oor->reset_counter, oor->enable_selftest, oor->st_counter);
#else
    printf("%.2f\t%.2f\t%.2f\t%.2f\t%d\t%u\t%u\t%.2f\t%.2f\t%u\t%u\t%u\t%u\n",
           data->x,
           data->y,
           data->z,
           oor->field_strength,
           (uint8_t)oor->out_of_range,
           oor->x_failed,
           oor->y_failed,
           oor->mag_x_st_en - oor->mag_x_st_dis,
           oor->mag_y_st_en - oor->mag_y_st_dis,
           oor->trigger_reset,
           oor->reset_counter,
           oor->enable_selftest,
           oor->st_counter);
#endif
}

bool get_interrupt_status_forced(uint32_t last_meas_time, struct bmm350_oor_reset_delay resetDelay)
{
    if ((coines_get_realtime_usec() - last_meas_time) >= resetDelay.delay_step)
    {
        return true;
    }

    return false;
}

#ifdef BMM350_OOR_NORMAL_MODE
bool get_interrupt_status_normal(struct bmm350_dev *dev)
{
    int8_t rslt = 0;
    uint8_t int_status = 0;

    /* Get data ready interrupt status */
    rslt = bmm350_get_regs(BMM350_REG_INT_STATUS, &int_status, 1, dev);
    bmm350_error_codes_print_result("bmm350_get_regs", rslt);

    /* Check if data ready interrupt occurred */
    if (int_status & BMM350_DRDY_DATA_REG_MSK)
    {
        return true;
    }

    return false;
}
#endif
