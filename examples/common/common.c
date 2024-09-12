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
* @file  common.c
*
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "coines.h"
#include "bmm350.h"
#include "common.h"

/******************************************************************************/
/*!                Structure definition                                       */

#define BMM350_SHUTTLE_ID  UINT16_C(0x27)

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address selection */
static uint8_t dev_addr;

/*! Variable to store coines I2C bus selection */
static enum coines_i2c_bus i2c_bus;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BMM350_INTF_RET_TYPE bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    return coines_read_i2c(i2c_bus, device_addr, reg_addr, reg_data, (uint16_t)length);
}

/*!
 * I2C write function map to COINES platform
 */
BMM350_INTF_RET_TYPE bmm350_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;

    (void)intf_ptr;

    return coines_write_i2c(i2c_bus, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)length);
}

/*!
 * Delay function map to COINES platform
 */
void bmm350_delay(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

#if defined(MCU_APP30)
    coines_delay_realtime_usec(period);
#else
    coines_delay_usec(period);
#endif
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmm350_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMM350_OK:
            break;

        case BMM350_E_NULL_PTR:
            printf("%s Error [%d] : Null pointer\r\n", api_name, rslt);
            break;
        case BMM350_E_COM_FAIL:
            printf("%s Error [%d] : Communication fail\r\n", api_name, rslt);
            break;
        case BMM350_E_DEV_NOT_FOUND:
            printf("%s Error [%d] : Device not found\r\n", api_name, rslt);
            break;
        case BMM350_E_INVALID_CONFIG:
            printf("%s Error [%d] : Invalid configuration\r\n", api_name, rslt);
            break;
        case BMM350_E_BAD_PAD_DRIVE:
            printf("%s Error [%d] : Bad pad drive\r\n", api_name, rslt);
            break;
        case BMM350_E_RESET_UNFINISHED:
            printf("%s Error [%d] : Reset unfinished\r\n", api_name, rslt);
            break;
        case BMM350_E_INVALID_INPUT:
            printf("%s Error [%d] : Invalid input\r\n", api_name, rslt);
            break;
        case BMM350_E_SELF_TEST_INVALID_AXIS:
            printf("%s Error [%d] : Self-test invalid axis selection\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_BOOT:
            printf("%s Error [%d] : OTP boot\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_PAGE_RD:
            printf("%s Error [%d] : OTP page read\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_PAGE_PRG:
            printf("%s Error [%d] : OTP page prog\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_SIGN:
            printf("%s Error [%d] : OTP sign\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_INV_CMD:
            printf("%s Error [%d] : OTP invalid command\r\n", api_name, rslt);
            break;
        case BMM350_E_OTP_UNDEFINED:
            printf("%s Error [%d] : OTP undefined\r\n", api_name, rslt);
            break;
        case BMM350_E_ALL_AXIS_DISABLED:
            printf("%s Error [%d] : All axis are disabled\r\n", api_name, rslt);
            break;
        case BMM350_E_PMU_CMD_VALUE:
            printf("%s Error [%d] : Unexpected PMU CMD value\r\n", api_name, rslt);
            break;
        default:
            printf("%s Error [%d] : Unknown error code\r\n", api_name, rslt);
            break;
    }
}

/*!
 *  @brief Function to select the interface.
 */
int8_t bmm350_interface_init(struct bmm350_dev *dev)
{
    int8_t rslt = BMM350_OK;
    struct coines_board_info board_info;

    if (dev != NULL)
    {
        int16_t result = coines_open_comm_intf(COINES_COMM_INTF_USB, NULL);
        if (result < COINES_SUCCESS)
        {
            printf(
                "\n Unable to connect with Application Board ! \n" " 1. Check if the board is connected and powered on. \n" " 2. Check if Application Board USB driver is installed. \n"
                " 3. Check if board is in use by another application. (Insufficient permissions to access USB) \n");
            exit(result);
        }

        (void)coines_get_board_info(&board_info);

        if (board_info.shuttle_id == BMM350_SHUTTLE_ID)
        {
            i2c_bus = COINES_I2C_BUS_0;
        }
        else
        {
            i2c_bus = COINES_I2C_BUS_1;
        }

        dev_addr = BMM350_I2C_ADSEL_SET_LOW;
        dev->intf_ptr = &dev_addr;
        dev->read = bmm350_i2c_read;
        dev->write = bmm350_i2c_write;
        dev->delay_us = bmm350_delay;

#if !defined(MCU_APP20)
        (void)coines_set_pin_config(COINES_MINI_SHUTTLE_PIN_1_4, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif

        (void)coines_config_i2c_bus(i2c_bus, COINES_I2C_STANDARD_MODE);

        (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);

        coines_delay_msec(100);

        (void)coines_set_shuttleboard_vdd_vddio_config(1800, 1800);

        coines_delay_msec(100);
    }
    else
    {
        rslt = BMM350_E_NULL_PTR;
    }

    return rslt;
}

void bmm350_coines_get_button_state(enum coines_multi_io_pin button_id,
                                    enum coines_pin_direction *btn_dir,
                                    enum coines_pin_value *btn_value)
{

    (void)coines_get_pin_config(button_id, btn_dir, btn_value);

}

void bmm350_coines_deinit(void)
{
    (void)fflush(stdout);

    (void)coines_set_shuttleboard_vdd_vddio_config(0, 0);

    coines_delay_msec(2000);

    coines_soft_reset();

    coines_delay_msec(100);

    (void)coines_close_comm_intf(COINES_COMM_INTF_USB, NULL);
}
