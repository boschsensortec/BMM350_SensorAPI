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
* @file       bmm350_oor.h
* @date       2025-10-30
* @version    v1.10.0
*
*/

#ifndef _BMM350_OOR_H
#define _BMM350_OOR_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdbool.h>
#ifdef BMM350_USE_FIXED_POINT
#include <math.h>
#endif
#endif

#include "bmm350.h"

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*! @name        General Macro Definitions                                    */
/******************************************************************************/

/*! Macro to define half self-test for out of range
 *  NOTE: Comment this to use generic self test */

/*#define BMM350_OOR_HALF_SELF_TEST */

/*! Macro to synchronize the reset window with the measurement window */
#define BMM350_OOR_WINDOW_SYNC_DELAY   UINT16_C(9000)

/*! Macro to define mag data minimum and maximum range in uT */
#ifdef BMM350_USE_FIXED_POINT
#define BMM350_HALF_ST_THRESHOLD       (130 << F16_FRAC_BITS)
#define BMM350_FULL_ST_THRESHOLD       (300 << F16_FRAC_BITS)
#else
#define BMM350_HALF_ST_THRESHOLD       (130.0f)
#define BMM350_FULL_ST_THRESHOLD       (300.0f)
#endif

/*! Macro to define threshold values of in range, out of range and self-test */
#ifdef BMM350_USE_FIXED_POINT
#define BMM350_IN_RANGE_THRESHOLD      (2000 << F16_FRAC_BITS)
#define BMM350_OUT_OF_RANGE_THRESHOLD  (2400 << F16_FRAC_BITS)
#define BMM350_SELF_TEST_THRESHOLD     (2600 << F16_FRAC_BITS)
#else
#define BMM350_IN_RANGE_THRESHOLD      (2000.0f)
#define BMM350_OUT_OF_RANGE_THRESHOLD  (2400.0f)
#define BMM350_SELF_TEST_THRESHOLD     (2600.0f)
#endif

/************************* Structure definitions *************************/

/*!
 * @brief Structure to define bmm350 out of range parameters
 */
struct bmm350_oor_params
{
    /*! Field Strength */
#ifdef BMM350_USE_FIXED_POINT
    uint32_t field_strength;
#else
    float field_strength;
#endif

    /*! Flags to enable Out of Range */
    bool out_of_range;

    /*! Counter to track what self test to trigger */
    uint8_t st_counter;

    /*! Current self-test command */
    uint8_t st_cmd;

    /*! Stores the last applied self test configuration */
    uint8_t last_st_cmd;

    /*! Self test enabled/disabled measurements for X and Y */
#ifdef BMM350_USE_FIXED_POINT
    fixed_t mag_x_st_en, mag_x_st_dis, mag_y_st_en, mag_y_st_dis, mag_x_st_dis_avg, mag_y_st_dis_avg;
#else
    float mag_x_st_en, mag_x_st_dis, mag_y_st_en, mag_y_st_dis, mag_x_st_dis_avg, mag_y_st_dis_avg;
#endif

    /*! Flags to track if the test failed to redo it */
    bool x_failed, y_failed;

    /*! Flags to enable self-test */
    bool enable_selftest;

    /*! Flags to trigger reset */
    bool trigger_reset;

    /*! Variable to store reset counter value */
    uint8_t reset_counter;
};

/*!
 * @brief Structure to define bmm350 out of range reset delay parameters
 */
struct bmm350_oor_reset_delay
{
    uint32_t delay_step;
    uint32_t br_delay;
    uint32_t fgr_delay;
};

/******************* Function prototype declarations ********************/

/*!
 * @brief Function to read data and validate if the sensor is out of range
 *
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
                       struct bmm350_dev *dev);

/*!
 * @brief Function to perform reset sequence in forced mode.
 *
 * @param[in] rdelay           : Structure that stores the delay settings for the various magnetic reset sequences
 * @param[in,out] oor          : Structure that stores the state of the out of range detector
 * @param[in,out]  dev         : Structure instance of bmm350_dev.
 *
 * @return Result of API execution status
 *  @retval = 0 -> Success
 *  @retval < 0 -> Error
 */
int8_t bmm350_oor_perform_reset_sequence(const struct bmm350_oor_reset_delay *rdelay,
                                         struct bmm350_oor_params *oor,
                                         struct bmm350_dev *dev);

/*!
 * @brief Function to compute the delay settings for the magnetic reset sequence.
 *
 * @param[in] odr_config       : Configured Output Data Rate
 * @param[in] rdelay           : Structure that stores the delay settings for the various magnetic reset sequences
 * @param[in,out]  dev         : Structure instance of bmm350_dev.
 */
void bmm350_oor_compute_delay_setting(uint8_t odr_config, struct bmm350_oor_reset_delay *rdelay);

/*!
 * @brief Function to check for the Out of Range condition.
 *
 * @param[out] data            : Sensor data
 * @param[in,out] oor          : Structure that stores the state of the out of range detector
 */
void bmm350_oor_validate_out_of_range(const struct bmm350_mag_temp_data *data, struct bmm350_oor_params *oor);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _BMM350_OOR_H */
