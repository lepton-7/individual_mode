/*
 * Copyright (c) 2022 - 2024, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <nrfx_example.h>
#include <nrfx_pwm.h>

#define NRFX_LOG_MODULE EXAMPLE
#define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
#define NRFX_EXAMPLE_CONFIG_LOG_LEVEL 3
#include <nrfx_log.h>

/**
 * @defgroup nrfx_pwm_grouped_example Grouped mode PWM example
 * @{
 * @ingroup nrfx_pwm_examples
 *
 * @brief Example showing basic functionality of nrfx_pwm driver for sequences loaded in grouped
 *        mode.
 *
 * @details Application initializes nrfx_pmw driver. It starts two-sequence playback on LEDs and
 *          replays this sequence @ref NUM_OF_LOOPS times. The @ref pwm_handler() is executed with
 *          relevant log message after every loop. Additionally, it changes SEQ1 each time it is
 *          called.
 */

/** @brief Symbol specifying PWM instance to be used. */
#define PWM_INST_IDX 0

/**
 * @brief Symbol specifying number of times that each duty cycle is to be repeated (after being
 *        played once) and is strictly correlated with the speed of LEDs brightness change.
 */
#define VALUE_REPEATS 0UL

/**
 * @brief Symbol specifying number of loops to be performed (one loop means to complete
 *        SEQ0 and SEQ1 @ref PLAYBACK_COUNT times).
 */
#define NUM_OF_LOOPS 6UL

/**
 * @brief Symbol specifying number of playbacks to be performed. In this example couple of
 *        playbacks might be considered as one loop.
 */
#define PLAYBACK_COUNT 1UL

/**
 * @brief Sequence default configuration in NRF_PWM_LOAD_GROUPED mode.
 *
 * This configuration sets up sequence with the following options:
 * - end delay: 0 PWM periods
 * - length: actual number of 16-bit values in the array pointed by @p _pwm_val
 * - repeats: VALUE_REPEATS
 *
 * @param[in] _pwm_val pointer to an array with duty cycle values.
 */
#define SEQ_CONFIG(_pwm_val)                     \
    {                                            \
        .values.p_individual = _pwm_val,         \
        .length = 4 * NRFX_ARRAY_SIZE(_pwm_val), \
        .repeats = VALUE_REPEATS,                \
        .end_delay = 0}

#define DEAD_TIME_US 2
uint16_t pw_us = 50;
uint16_t ip_us = 40;
uint16_t freq = 50;

static nrf_pwm_values_individual_t biphasic_vals[] =
    {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
};

/** @brief Array containing sequences to be used in this example. */
static nrf_pwm_sequence_t seq[] =
    {
        SEQ_CONFIG(biphasic_vals),
};

static void update_seqs(nrf_pwm_values_individual_t *arr, int arr_len)
{
    seq[0] = (nrf_pwm_sequence_t)SEQ_CONFIG(arr);
    seq[0].length = 4 * arr_len;
}

static uint16_t get_count_us(nrfx_pwm_config_t *conf, uint16_t us)
{
    uint16_t ctop;
    switch (conf->base_clock)
    {
    case NRF_PWM_CLK_16MHz:
        ctop = 16 * us;
        break;
    case NRF_PWM_CLK_8MHz:
        ctop = 8 * us;
        break;
    case NRF_PWM_CLK_4MHz:
        ctop = 4 * us;
        break;
    case NRF_PWM_CLK_2MHz:
        ctop = 2 * us;
        break;
    case NRF_PWM_CLK_1MHz:
        ctop = 1 * us;
        break;
    case NRF_PWM_CLK_500kHz:
        ctop = us / 2;
        break;
    case NRF_PWM_CLK_250kHz:
        ctop = us / 4;
        break;
    case NRF_PWM_CLK_125kHz:
        ctop = us / 8;
        break;

    default:
        break;
    }

    switch (conf->count_mode)
    {
    case NRF_PWM_MODE_UP:
        return ctop;
    case NRF_PWM_MODE_UP_AND_DOWN:
        return ctop / 2;
    default:
        break;
    }
}

#define INV(_COMP) (_COMP | 1 << 16)

static void set_biphasic_sequence(nrfx_pwm_config_t *conf, nrf_pwm_values_individual_t *arr, uint16_t pw)
{
    uint16_t comp_on = conf->top_value - get_count_us(conf, pw);
    NRFX_LOG_INFO("On time COMP: %d", comp_on);
    NRFX_LOG_INFO("On time Inverted COMP: %d", INV(comp_on));
    uint16_t comp_on_dead = conf->top_value - get_count_us(conf, pw + 2 * DEAD_TIME_US);
    NRFX_LOG_INFO("On time COMP with deadband: %d", comp_on_dead);

    uint16_t max = conf->top_value;
    // arr[0] = (nrf_pwm_values_individual_t){comp_on, comp_on_dead, max, max};
    // arr[1] = (nrf_pwm_values_individual_t){max, max, comp_on, comp_on_dead};
    // arr[2] = (nrf_pwm_values_individual_t){max, max, max, max};

    arr[0] = (nrf_pwm_values_individual_t){comp_on, INV(comp_on_dead), max, INV(max)};
    arr[1] = (nrf_pwm_values_individual_t){max, INV(max), comp_on, INV(comp_on_dead)};
    arr[2] = (nrf_pwm_values_individual_t){max, INV(max), max, INV(max)};
}

static void set_seq_countertop(nrfx_pwm_config_t *conf, uint16_t period_us)
{
    conf->top_value = get_count_us(conf, period_us);
}

/**
 * @brief Function for handling PWM driver events.
 *
 * @param[in] event_type PWM event.
 * @param[in] p_context  General purpose parameter set during initialization of
 *                       the timer. This parameter can be used to pass
 *                       additional information to the handler function.
 */
static void pwm_handler(nrfx_pwm_evt_type_t event_type, void *p_context)
{
    nrfx_pwm_t *inst = p_context;
    static uint32_t m_curr_loop = 1;

    if (m_curr_loop == NUM_OF_LOOPS)
    {
        NRFX_LOG_INFO("PWM finished");
        nrfx_pwm_uninit(inst);
    }

    m_curr_loop++;
}

/**
 * @brief Function for application main entry.
 *
 * @return Nothing.
 */
int main(void)
{
    nrfx_err_t status;
    (void)status;

#if defined(__ZEPHYR__)
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM_INST_GET(PWM_INST_IDX)), IRQ_PRIO_LOWEST,
                NRFX_PWM_INST_HANDLER_GET(PWM_INST_IDX), 0, 0);
#endif

    NRFX_EXAMPLE_LOG_INIT();

    NRFX_LOG_INFO("Starting nrfx_pwm example for sequences loaded in grouped mode.");
    NRFX_EXAMPLE_LOG_PROCESS();

    nrfx_pwm_t pwm_instance = NRFX_PWM_INSTANCE(PWM_INST_IDX);
    nrfx_pwm_config_t config = NRFX_PWM_DEFAULT_CONFIG(17, 18, 19, 20);
    // config.pin_inverted[1] = true;
    // config.pin_inverted[3] = true;
    config.load_mode = NRF_PWM_LOAD_INDIVIDUAL;
    config.base_clock = NRF_PWM_CLK_4MHz;
    config.count_mode = NRF_PWM_MODE_UP_AND_DOWN;

    set_seq_countertop(&config, pw_us + ip_us);
    set_biphasic_sequence(&config, biphasic_vals, pw_us);
    update_seqs(biphasic_vals, 3);

    NRFX_LOG_INFO("CTOP set to %d", config.top_value);

    status = nrfx_pwm_init(&pwm_instance, &config, pwm_handler, &pwm_instance);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_pwm_simple_playback(&pwm_instance, &seq[0], PLAYBACK_COUNT, NRFX_PWM_FLAG_LOOP);

    while (1)
    {
        NRFX_EXAMPLE_LOG_PROCESS();
    }
}

/** @} */
