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
* @file  bmm350_illegal_command.c
*
* @brief This file contains set get operations of illegal commands.
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

    uint8_t pmu_cmd;
    struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0;

    /* Update device structure */
    rslt = bmm350_interface_init(&dev);
    bmm350_error_codes_print_result("bmm350_interface_selection", rslt);

    /* Initialize BMM350 */
    rslt = bmm350_init(&dev);
    bmm350_error_codes_print_result("bmm350_init", rslt);

    printf("Read : 0x00 : BMM350 Chip ID : 0x%X\n", dev.chip_id);

    rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &dev);
    bmm350_error_codes_print_result("bmm350_set_powermode", rslt);

    printf("Set legal PMU command to the register \n");

    pmu_cmd = 2;

    rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &pmu_cmd, 1, &dev);
    bmm350_error_codes_print_result("bmm350_set_regs", rslt);

    printf("Write : 0x06 : PMU CMD : 0x%X\n", pmu_cmd);

    /* Check PMU cmd illegal */
    rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &dev);
    bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);

    printf("Read : 0x07 : PMU command illegal status : 0x%X\n", pmu_cmd_stat_0.cmd_is_illegal);

    rslt = bmm350_delay_us(40000, &dev);
    bmm350_error_codes_print_result("bmm350_delay_us", rslt);

    printf("Set illegal PMU command to the register \n");

    pmu_cmd = 12;

    rslt = bmm350_set_regs(BMM350_REG_PMU_CMD, &pmu_cmd, 1, &dev);
    bmm350_error_codes_print_result("bmm350_set_regs", rslt);

    printf("Write : 0x06 : PMU CMD : 0x%X\n", pmu_cmd);

    /* Check PMU cmd illegal */
    rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, &dev);
    bmm350_error_codes_print_result("bmm350_get_pmu_cmd_status_0", rslt);

    printf("Read : 0x07 : PMU command illegal status : 0x%X\n", pmu_cmd_stat_0.cmd_is_illegal);

    bmm350_coines_deinit();

    return rslt;
}
