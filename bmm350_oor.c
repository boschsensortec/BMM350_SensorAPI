/**
* Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
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
* @file       bmm350_oor.c
* @date       2025-10-30
* @version    v1.10.0
*
*/

#include "bmm350.h"
#include "bmm350_oor.h"

#ifndef BMM350_USE_FIXED_POINT
#include "math.h"

#else

#ifndef __KERNEL__
#include "stdlib.h"
#endif

#endif

/********************** Static function declarations ************************/

/*!
 * @brief This internal API is used to execute delay operation for the reset functions.
 */
static int8_t execute_delay_in_steps(uint32_t delay, uint32_t delay_step, const struct bmm350_dev *dev);

#ifdef BMM350_OOR_HALF_SELF_TEST

/*!
 * @brief This internal API is used to trigger half self-test
 * @param[out] oor             : Structure that stores the state of the out of range detector
 * @param[in,out] dev          : Device structure of the BMM350
 */
static int8_t trigger_half_selftest(const struct bmm350_oor_reset_delay *rdelay,
                                    struct bmm350_oor_params *oor,
                                    struct bmm350_dev *dev)
{
    int8_t rslt = BMM350_OK;

    uint8_t pmu_cmd = BMM350_PMU_CMD_BR;

    oor->st_cmd = BMM350_SELF_TEST_DISABLE;

    /* Trigger a self-test on every alternate measurement if needed */
    if (oor->enable_selftest)
    {
        oor->st_counter++;

        switch (oor->st_counter)
        {
            case 1:
                oor->st_cmd = BMM350_SELF_TEST_POS_X;
                break;
            case 2:
                oor->st_cmd = BMM350_SELF_TEST_DISABLE;
                break;
            case 3:
                oor->st_cmd = BMM350_SELF_TEST_POS_Y;
                break;
            case 4:
                oor->st_cmd = BMM350_SELF_TEST_DISABLE;
                break;

            case 5:
                if (dev->enable_auto_br == BMM350_DISABLE)
                {
                    /* Trigger the Bit Reset */
                    pmu_cmd = BMM350_PMU_CMD_BR;
                    rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &pmu_cmd, 1, dev);
                    if (rslt == BMM350_OK)
                    {
                        /* Bit Reset delay*/
                        (void)execute_delay_in_steps(rdelay->br_delay, rdelay->delay_step, dev);
                    }
                }
                else
                {
                    oor->st_counter = 0;
                    oor->st_cmd = BMM350_SELF_TEST_DISABLE;
                }

                break;

            default:
                oor->st_counter = 0;
                oor->st_cmd = BMM350_SELF_TEST_DISABLE;
                break;
        }

        rslt = bmm350_set_regs(BMM350_REG_TMR_SELFTEST_USER, &(oor->st_cmd), 1, dev);
    }
    else
    {
        if (oor->last_st_cmd != BMM350_SELF_TEST_DISABLE)
        {
            rslt = bmm350_set_regs(BMM350_REG_TMR_SELFTEST_USER, &(oor->st_cmd), 1, dev);
            oor->st_counter = 0;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to validate half self-test
 * @param[out] data            : Sensor data
 * @param[out] oor             : Structure that stores the state of the out of range detector
 */
static void validate_half_selftest(const struct bmm350_mag_temp_data *data, struct bmm350_oor_params *oor)
{
    switch (oor->last_st_cmd)
    {
        case BMM350_SELF_TEST_DISABLE:
            oor->mag_x_st_dis = data->x;
            oor->mag_y_st_dis = data->y;
            break;

        case BMM350_SELF_TEST_POS_X:
            oor->mag_x_st_en = data->x;
            oor->x_failed = (oor->mag_x_st_en - oor->mag_x_st_dis) < BMM350_HALF_ST_THRESHOLD ? true : false;
            break;

        case BMM350_SELF_TEST_POS_Y:
            oor->mag_y_st_en = data->y;
            oor->y_failed = (oor->mag_y_st_en - oor->mag_y_st_dis) < BMM350_HALF_ST_THRESHOLD ? true : false;
            break;

        default:
            break;
    }
}
#else

/*!
 * @brief This internal API is used to trigger self-test
 * @param[in] rdelay           : Structure that stores the delay settings for the various magnetic reset sequences
 * @param[out] oor             : Structure that stores the state of the out of range detector
 * @param[in,out] dev          : Device structure of the BMM350
 *
 *  @return Result of API execution status
 *  @retval = 0 -> Success
 *  @retval < 0 -> Error
 */
static int8_t trigger_selftest(const struct bmm350_oor_reset_delay *rdelay,
                               struct bmm350_oor_params *oor,
                               struct bmm350_dev *dev)
{
    int8_t rslt = BMM350_OK;
    uint8_t pmu_cmd = BMM350_PMU_CMD_BR;

    oor->st_cmd = BMM350_SELF_TEST_DISABLE;

    /* Trigger a self-test on every alternate measurement if needed */
    if (oor->enable_selftest)
    {
        oor->st_counter++;

        switch (oor->st_counter)
        {
            case 1:
                oor->st_cmd = BMM350_SELF_TEST_POS_X;
                break;
            case 2:
                oor->st_cmd = BMM350_SELF_TEST_NEG_X;
                break;
            case 3:
                oor->st_cmd = BMM350_SELF_TEST_POS_Y;
                break;
            case 4:
                oor->st_cmd = BMM350_SELF_TEST_NEG_Y;
                break;

            case 5:
                if (dev->enable_auto_br == BMM350_DISABLE)
                {
                    /* Trigger the Bit Reset */
                    pmu_cmd = BMM350_PMU_CMD_BR;
                    rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &pmu_cmd, 1, dev);
                    if (rslt == BMM350_OK)
                    {
                        /* Bit Reset delay*/
                        (void)execute_delay_in_steps(rdelay->br_delay, rdelay->delay_step, dev);
                    }
                }
                else
                {
                    oor->st_counter = 0;
                    oor->st_cmd = BMM350_SELF_TEST_DISABLE;
                }

                break;
            default:
                oor->st_counter = 0;
                oor->st_cmd = BMM350_SELF_TEST_DISABLE;
                break;
        }

        rslt = bmm350_set_regs(BMM350_REG_TMR_SELFTEST_USER, &(oor->st_cmd), 1, dev);
    }
    else
    {
        if (oor->last_st_cmd != BMM350_SELF_TEST_DISABLE)
        {
            rslt = bmm350_set_regs(BMM350_REG_TMR_SELFTEST_USER, &(oor->st_cmd), 1, dev);
            oor->st_counter = 0;
        }
    }

    return rslt;
}

/*!
 * @brief This internal API is used to validate self-test during the self-test window
 * @param[in] data            : Sensor data
 * @param[out] oor             : Structure that stores the state of the out of range detector
 */
static void validate_selftest_window(const struct bmm350_mag_temp_data *data, struct bmm350_oor_params *oor)
{
    switch (oor->last_st_cmd)
    {
        case BMM350_SELF_TEST_POS_X:
            oor->mag_x_st_en = data->x;
            break;

        case BMM350_SELF_TEST_NEG_X:
            oor->mag_x_st_dis = data->x;
            oor->x_failed = (oor->mag_x_st_en - oor->mag_x_st_dis) < BMM350_FULL_ST_THRESHOLD ? true : false;
            break;

        case BMM350_SELF_TEST_POS_Y:
            oor->mag_y_st_en = data->y;
            break;

        case BMM350_SELF_TEST_NEG_Y:
            oor->mag_y_st_dis = data->y;
            oor->y_failed = (oor->mag_y_st_en - oor->mag_y_st_dis) < BMM350_FULL_ST_THRESHOLD ? true : false;
            break;

        default:
            break;
    }
}
#endif

/********************** Global function definitions ************************/

/*!
 * @brief This internal API is used to validate out of range.
 * @param[in] data            : Sensor data
 * @param[out] oor             : Structure that stores the state of the out of range detector
 */
void bmm350_oor_validate_out_of_range(const struct bmm350_mag_temp_data *data, struct bmm350_oor_params *oor)
{
#ifdef BMM350_USE_FIXED_POINT

    /* Threshold to start out of range detection */
    fixed_t threshold = BMM350_OUT_OF_RANGE_THRESHOLD;

    /* Threshold to start self-tests */
    fixed_t st_threshold = BMM350_SELF_TEST_THRESHOLD;

    /* Variable to compute the magnitude square */
    fixed_t magnitude_square = 0;
#else

    /* Threshold to start out of range detection */
    float threshold = BMM350_OUT_OF_RANGE_THRESHOLD;

    /* Threshold to start self-tests */
    float st_threshold = BMM350_SELF_TEST_THRESHOLD;
#endif

#ifdef BMM350_USE_FIXED_POINT

    /* Compute the Field Strength */
    magnitude_square =
        (uint32_t)((fixed_mul_A48_16(data->x,
                                     data->x) +
                    fixed_mul_A48_16(data->y, data->y) + fixed_mul_A48_16(data->z, data->z)) >> F16_FRAC_BITS);
    oor->field_strength = (uint32_t)bmm350_fixed_point_sqrt(magnitude_square);
#else

    /* Compute the Field Strength */
    oor->field_strength = sqrtf((data->x * data->x) + (data->y * data->y) + (data->z * data->z));
#endif

    /* If either self-test failed, alert that the sensor is out of range and continue self-tests */
    if (oor->x_failed || oor->y_failed)
    {
        oor->out_of_range = true;
        oor->enable_selftest = true;
    }
    else
    {

        /* Check for the self-test threshold and perform self-tests to catch if the sensor is out of range */
#ifdef BMM350_USE_FIXED_POINT
        if ((abs(data->x) >= st_threshold) || (abs(data->y) >= st_threshold) || (abs(data->z) >= st_threshold) ||
            (oor->field_strength >= (uint32_t)(st_threshold >> F16_FRAC_BITS)))
#else
        if ((fabsf(data->x) >= st_threshold) || (fabsf(data->y) >= st_threshold) || (fabsf(data->z) >= st_threshold) ||
            (oor->field_strength >= st_threshold))
#endif
        {
            oor->enable_selftest = true;
        }
        else if (oor->st_counter == 0) /* If a self-test procedure has started, wait for it to complete */
        {
            oor->enable_selftest = false;
        }

        /* If out of range was previously detected, reduce the threshold to get back in range,
         * effectively preventing hysteresis. Selecting 400uT */
        if (oor->out_of_range)
        {
            threshold = BMM350_IN_RANGE_THRESHOLD;
        }

        /* Check if X or Y or Z > the threshold or the magnitude of all 3 is greater */
#ifdef BMM350_USE_FIXED_POINT
        if ((abs(data->x) >= threshold) || (abs(data->y) >= threshold) || (abs(data->z) >= threshold) ||
            (oor->field_strength >= (uint32_t)(threshold >> F16_FRAC_BITS)))
#else
        if ((fabsf(data->x) >= threshold) || (fabsf(data->y) >= threshold) || (fabsf(data->z) >= threshold) ||
            (oor->field_strength >= threshold))
#endif
        {
            oor->out_of_range = true;
        }
        else if (oor->st_counter == 0) /* If a self-test procedure has started, wait for it to complete */
        {
            if (oor->out_of_range)
            {
                oor->trigger_reset = true;
            }

            oor->out_of_range = false;
        }
    }
}

/*!
 * @brief This API is used to perform magnetic reset sequence.
 * @param[in] rdelay           : Structure that stores the delay settings for the various magnetic reset sequences
 * @param[out] oor             : Structure that stores the state of the out of range detector
 * @param[in,out] dev          : Device structure of the BMM350
 *
 *  @return Result of API execution status
 *  @retval = 0 -> Success
 *  @retval < 0 -> Error
 */
int8_t bmm350_oor_perform_reset_sequence(const struct bmm350_oor_reset_delay *rdelay,
                                         struct bmm350_oor_params *oor,
                                         struct bmm350_dev *dev)
{
    int8_t rslt = 0;
    uint8_t pmu_cmd = 0;

    oor->reset_counter++;

    switch (oor->reset_counter)
    {
        case 1: /* Trigger the Bit reset fast */
            pmu_cmd = BMM350_PMU_CMD_BR_FAST;
            rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &pmu_cmd, 1, dev);
            if (rslt == BMM350_OK)
            {
                /* Bit Reset Window synchronization delay*/
                (void)execute_delay_in_steps(rdelay->br_delay, rdelay->delay_step, dev);
            }

            break;

        case 2: /* Trigger Flux Guide reset */
            pmu_cmd = BMM350_PMU_CMD_FGR;
            rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &pmu_cmd, 1, dev);
            if (rslt == BMM350_OK)
            {
                /* Flux Guide Reset Window synchronization delay*/
                (void)execute_delay_in_steps(rdelay->fgr_delay, rdelay->delay_step, dev);
            }

            break;

        case 3: /* Flux Guide dummy */
            break;

        default: /* Default acts like the Flux guide reset dummy */
            oor->reset_counter = 0;
            oor->trigger_reset = false;
            break;
    }

    return rslt;
}

/*!
 * @brief This API is used to read out of range in during self-test.
 * @param[in] rdelay           : Structure that stores the delay settings for the various magnetic reset sequences
 * @param[out] data            : Sensor data
 * @param[out] oor             : Structure that stores the state of the out of range detector
 * @param[in,out] dev          : Device structure of the BMM350
 *
 *  @return Result of API execution status
 *  @retval = 0 -> Success
 *  @retval < 0 -> Error
 */
int8_t bmm350_oor_read(const struct bmm350_oor_reset_delay *rdelay,
                       struct bmm350_mag_temp_data *data,
                       struct bmm350_oor_params *oor,
                       struct bmm350_dev *dev)
{
    int8_t rslt = 0;
    uint8_t pmu_cmd = BMM350_PMU_CMD_SUS;

#ifdef BMM350_OOR_HALF_SELF_TEST
    rslt = trigger_half_selftest(rdelay, oor, dev);
#else
    rslt = trigger_selftest(rdelay, oor, dev);
#endif

    if (rslt == BMM350_OK)
    {
        pmu_cmd = BMM350_PMU_CMD_FM_FAST;
        rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &pmu_cmd, 1, dev);

        if (rslt == BMM350_OK)
        {
#ifdef BMM350_USE_FIXED_POINT
            rslt = bmm350_get_compensated_mag_xyz_temp_data_fixed(data, dev);
#else
            rslt = bmm350_get_compensated_mag_xyz_temp_data(data, dev);
#endif
        }
    }

#ifdef BMM350_OOR_HALF_SELF_TEST
    validate_half_selftest(data, oor);
#else
    validate_selftest_window(data, oor);
#endif

    bmm350_oor_validate_out_of_range(data, oor);

    oor->last_st_cmd = oor->st_cmd;

    return rslt;
}

/*!
 * @brief This API is used to computing the reset delay settings for Magnetic Reset Sequence.
 * @param[in] odr_config            : ODR Configuration
 * @param[out] rdelay           : Structure that stores the delay settings for the various magnetic reset sequences
 */
void bmm350_oor_compute_delay_setting(uint8_t odr_config, struct bmm350_oor_reset_delay *rdelay)
{
    uint16_t min_dealy_unit = 2500; /* corresponds to 400Hz -> (1/400) * (10^6) us */

#ifdef BMM350_USE_FIXED_POINT
    uint32_t power_scale = ((uint32_t)1 << (odr_config - BMM350_ODR_400HZ));
    rdelay->delay_step = (uint32_t)(min_dealy_unit * power_scale);
#else
    rdelay->delay_step = (uint32_t)(min_dealy_unit * (pow(2.0, (double)(odr_config - BMM350_ODR_400HZ))));
#endif

    rdelay->br_delay = ((rdelay->delay_step * 3) - 1000);
    rdelay->fgr_delay = ((rdelay->delay_step * 4) - 2000);
}

/********************** Static function definitions ************************/

/*!
 * @brief This internal API is used to execute delay operation for the reset functions.
 * @param[in] delay           : ODR Window Delay
 * @param[in] delay_step      : Delay Time Resolution
 * @param[in,out] dev         : Device structure of the BMM350
 *
 *  @return Result of API execution status
 *  @retval = 0 -> Success
 *  @retval < 0 -> Error
 */
static int8_t execute_delay_in_steps(uint32_t delay, uint32_t delay_step, const struct bmm350_dev *dev)
{
    int8_t rslt = BMM350_OK;

    if ((delay > 0) && (delay_step > 0) && (delay >= delay_step))
    {
        while (delay >= delay_step)
        {
            rslt = bmm350_delay_us(delay_step, dev);
            if (rslt == BMM350_OK)
            {
                delay = delay - delay_step;
            }
            else
            {
                break;
            }
        }
    }
    else
    {
        rslt = BMM350_E_INVALID_INPUT;
    }

    return rslt;
}
