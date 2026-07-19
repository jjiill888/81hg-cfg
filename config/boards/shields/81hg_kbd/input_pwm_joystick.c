/*
 * Copyright (c) 2025 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * PWM duty-cycle joystick driver for GP9101F1K-based analog sticks.
 * Uses burst-sampling (polling GPIO levels over one PWM period) WITHOUT
 * disabling interrupts, so BLE stack is never blocked.
 */

#define DT_DRV_COMPAT pwm_joystick

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

/* nRF GPIO register access for complete pin shutdown */
#include <hal/nrf_gpio.h>

LOG_MODULE_REGISTER(pwm_joystick, CONFIG_ZMK_LOG_LEVEL);

/*
 * GP9101 outputs 1 kHz PWM
 * Period = 1 ms = 1000 us
 * Center position ~ 50% duty cycle
 * Range: approximately 20%-80%
 *
 * Burst-sampling: continuously read GPIO level in a tight loop for ~1.1 ms,
 * count high-level samples as a fraction of total samples to get duty cycle.
 * Interrupts remain ENABLED so BLE is never blocked.
 * BLE ISRs (~10-50 μs) cause minor sampling gaps that the filter handles.
 */

#define DUTY_SCALE      10000   /* 0.01% precision */
#define CALIBRATION_SAMPLES 50
/* Sampling duration, slightly over one PWM period to cover a full cycle */
#define SAMPLE_DURATION_US 1100

/*
 * EMA filter coefficient, alpha = EMA_ALPHA/256. Burst-sampling already
 * averages a full PWM period, so a light filter is enough; keeping alpha
 * high keeps direction changes near-instant (TrackPoint-like).
 */
#define EMA_ALPHA 115

/*
 * Movement is computed in 1/256-subpixel units and accumulated across
 * polls, so slow-zone speed can go well below 1 px/poll (fine aiming).
 */
#define MV_SCALE 256
#define MV_MAX   (127 * MV_SCALE)

/*
 * Dynamic polling: after the stick has been centered for a while, back off
 * to a slow poll rate to cut CPU busy-sampling from ~22% to ~2%.
 * Wake-up is detected on RAW samples (not the filtered value), so the only
 * added latency is at most one slow poll period.
 */
#define IDLE_POLLS_THRESHOLD 250            /* ~2.5 s of no movement at fast rate */
#define IDLE_POLL_PERIOD_MS  100

struct pwm_joystick_config {
    struct gpio_dt_spec x_gpio;
    struct gpio_dt_spec y_gpio;
    uint16_t poll_period_ms;
    uint8_t  deadzone_pct;
    uint16_t slow_gain;
    uint16_t knee;
    uint16_t fast_gain_div;
    bool     invert_x;
    bool     invert_y;
    bool     swap_xy;
};

struct axis_state {
    /* EMA filter state, duty * 256 */
    int32_t  filt256;

    /* Sub-pixel movement accumulator, px * 256 */
    int32_t  accum256;

    /* Calibration */
    uint16_t center_duty;
    uint32_t cal_sum;
    uint8_t  cal_count;
};

struct pwm_joystick_data {
    const struct device *dev;

    struct axis_state x_axis;
    struct axis_state y_axis;

    bool calibrated;
    bool slow_mode;
    uint16_t idle_polls;
    struct k_work_delayable poll_work;
};

/* ---- Exponential moving average filter ---- */
static uint16_t filter_duty(struct axis_state *axis, uint16_t new_duty)
{
    axis->filt256 += (((int32_t)new_duty << 8) - axis->filt256) * EMA_ALPHA / 256;
    return (uint16_t)(axis->filt256 >> 8);
}

static void seed_filter(struct axis_state *axis, uint16_t duty)
{
    axis->filt256 = (int32_t)duty << 8;
    axis->accum256 = 0;
}

/* ---- Burst-sampling: measure single axis duty cycle ---- */
static uint16_t measure_duty(const struct gpio_dt_spec *gpio)
{
    uint32_t high_count = 0;
    uint32_t total_count = 0;

    /* NO irq_lock() — BLE interrupts stay enabled */

    uint32_t start = k_cycle_get_32();
    uint32_t duration_cycles = (uint32_t)(
        (uint64_t)sys_clock_hw_cycles_per_sec() * SAMPLE_DURATION_US / 1000000ULL);
    uint32_t end = start + duration_cycles;

    while ((int32_t)(end - k_cycle_get_32()) > 0) {
        if (gpio_pin_get_dt(gpio)) {
            high_count++;
        }
        total_count++;
    }

    if (total_count == 0) {
        return 5000; /* fallback: 50% */
    }

    return (uint16_t)((high_count * (uint32_t)DUTY_SCALE) / total_count);
}

/* ---- TrackPoint-style two-zone curve ----
 * Returns movement in px*256 per poll:
 *   slow zone (e <= knee): linear, sub-pixel speeds for fine aiming
 *   fast zone (e > knee):  adds a quadratic term for cross-screen flicks
 */
static int32_t apply_curve(int32_t offset, int32_t deadzone,
                           const struct pwm_joystick_config *cfg)
{
    int32_t sign = (offset >= 0) ? 1 : -1;
    int32_t e = ((offset >= 0) ? offset : -offset) - deadzone;

    if (e <= 0) {
        return 0;
    }

    int32_t mv = (e * cfg->slow_gain) / 100;

    if (e > cfg->knee) {
        int32_t f = e - cfg->knee;
        mv += (f * f) / cfg->fast_gain_div;
    }

    if (mv > MV_MAX) {
        mv = MV_MAX;
    }
    return sign * mv;
}

/* ---- Periodic poll handler ---- */
static void poll_work_handler(struct k_work *work)
{
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct pwm_joystick_data *data =
        CONTAINER_OF(dwork, struct pwm_joystick_data, poll_work);
    const struct pwm_joystick_config *cfg = data->dev->config;

    /* Burst-sample X and Y sequentially (~1.1 ms each, ~2.2 ms total) */
    uint16_t raw_x = measure_duty(&cfg->x_gpio);
    uint16_t raw_y = measure_duty(&cfg->y_gpio);

    static uint8_t dbg_cnt;
    if (++dbg_cnt >= (1000 / cfg->poll_period_ms)) {
        dbg_cnt = 0;
        LOG_INF("raw X=%u Y=%u  center X=%u Y=%u",
                raw_x, raw_y,
                data->x_axis.center_duty, data->y_axis.center_duty);
    }

    /* Calibration phase */
    if (!data->calibrated) {
        data->x_axis.cal_sum += raw_x;
        data->y_axis.cal_sum += raw_y;
        data->x_axis.cal_count++;
        data->y_axis.cal_count++;

        if (data->x_axis.cal_count >= CALIBRATION_SAMPLES) {
            data->x_axis.center_duty = data->x_axis.cal_sum / CALIBRATION_SAMPLES;
            data->y_axis.center_duty = data->y_axis.cal_sum / CALIBRATION_SAMPLES;
            data->calibrated = true;

            /* Initialize filters with center value */
            seed_filter(&data->x_axis, data->x_axis.center_duty);
            seed_filter(&data->y_axis, data->y_axis.center_duty);

            LOG_INF("Calibrated: X_center=%u Y_center=%u",
                    data->x_axis.center_duty, data->y_axis.center_duty);
        }

        k_work_schedule(&data->poll_work, K_MSEC(cfg->poll_period_ms));
        return;
    }

    int32_t deadzone = (cfg->deadzone_pct * DUTY_SCALE) / 100;

    /* Slow mode: check RAW offsets against the deadzone so wake-up isn't
     * delayed by the sliding filter ramping up from center. */
    if (data->slow_mode) {
        int32_t roff_x = (int32_t)raw_x - (int32_t)data->x_axis.center_duty;
        int32_t roff_y = (int32_t)raw_y - (int32_t)data->y_axis.center_duty;

        if (roff_x > -deadzone && roff_x < deadzone &&
            roff_y > -deadzone && roff_y < deadzone) {
            /* Still centered: keep the filter tracking, stay slow */
            uint16_t xf = filter_duty(&data->x_axis, raw_x);
            uint16_t yf = filter_duty(&data->y_axis, raw_y);

            /* Drift compensation: slowly re-center while idle so the
             * deadzone can stay small without cursor creep */
            data->x_axis.center_duty +=
                ((int32_t)xf - (int32_t)data->x_axis.center_duty) / 32;
            data->y_axis.center_duty +=
                ((int32_t)yf - (int32_t)data->y_axis.center_duty) / 32;

            k_work_schedule(&data->poll_work, K_MSEC(IDLE_POLL_PERIOD_MS));
            return;
        }

        /* Movement detected: seed filters with current position so this
         * very poll already reports it, and resume fast polling */
        seed_filter(&data->x_axis, raw_x);
        seed_filter(&data->y_axis, raw_y);
        data->slow_mode = false;
        data->idle_polls = 0;
    }

    /* Filter */
    uint16_t x_duty = filter_duty(&data->x_axis, raw_x);
    uint16_t y_duty = filter_duty(&data->y_axis, raw_y);

    /* Offset from center */
    int32_t off_x = (int32_t)x_duty - (int32_t)data->x_axis.center_duty;
    int32_t off_y = (int32_t)y_duty - (int32_t)data->y_axis.center_duty;

    /* Invert axes */
    if (cfg->invert_x) { off_x = -off_x; }
    if (cfg->invert_y) { off_y = -off_y; }

    /* Apply curve (px*256 per poll) */
    int32_t mvx = apply_curve(off_x, deadzone, cfg);
    int32_t mvy = apply_curve(off_y, deadzone, cfg);

    /* Swap axes */
    if (cfg->swap_xy) {
        int32_t tmp = mvx;
        mvx = mvy;
        mvy = tmp;
    }

    /* Sub-pixel accumulation: emit whole pixels, carry the remainder */
    data->x_axis.accum256 += mvx;
    data->y_axis.accum256 += mvy;
    int32_t dx = data->x_axis.accum256 / MV_SCALE;
    int32_t dy = data->y_axis.accum256 / MV_SCALE;
    data->x_axis.accum256 -= dx * MV_SCALE;
    data->y_axis.accum256 -= dy * MV_SCALE;

    /* Report movement */
    if (dx != 0 || dy != 0) {
        input_report_rel(data->dev, INPUT_REL_X, dx, false, K_NO_WAIT);
        input_report_rel(data->dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
    }

    /* Idle tracking: any sub-pixel drive counts as activity */
    if (mvx != 0 || mvy != 0) {
        data->idle_polls = 0;
    } else if (data->idle_polls < IDLE_POLLS_THRESHOLD) {
        data->idle_polls++;
        if (data->idle_polls == IDLE_POLLS_THRESHOLD) {
            data->slow_mode = true;
            data->x_axis.accum256 = 0;
            data->y_axis.accum256 = 0;
            LOG_INF("Joystick idle, slow polling (%d ms)", IDLE_POLL_PERIOD_MS);
        }
    }

    k_work_schedule(&data->poll_work,
                    K_MSEC(data->slow_mode ? IDLE_POLL_PERIOD_MS : cfg->poll_period_ms));
}

/* ---- Device initialization ---- */
static int pwm_joystick_init(const struct device *dev)
{
    struct pwm_joystick_data *data = dev->data;
    const struct pwm_joystick_config *cfg = dev->config;
    int ret;

    data->dev = dev;
    data->calibrated = false;
    data->slow_mode = false;
    data->idle_polls = 0;
    memset(&data->x_axis, 0, sizeof(struct axis_state));
    memset(&data->y_axis, 0, sizeof(struct axis_state));

    /* Configure as input only, no interrupts needed */
    if (!gpio_is_ready_dt(&cfg->x_gpio)) {
        LOG_ERR("X GPIO not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&cfg->x_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure X GPIO: %d", ret);
        return ret;
    }

    if (!gpio_is_ready_dt(&cfg->y_gpio)) {
        LOG_ERR("Y GPIO not ready");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&cfg->y_gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("Failed to configure Y GPIO: %d", ret);
        return ret;
    }

    k_work_init_delayable(&data->poll_work, poll_work_handler);
    k_work_schedule(&data->poll_work, K_MSEC(500)); /* Startup delay */

    LOG_INF("PWM joystick initialized (burst-sampling, interrupts enabled)");
    return 0;
}

/* ---- Power management: suspend/resume for deep sleep ---- */

static int pwm_joystick_pm_action(const struct device *dev, enum pm_device_action action)
{
    struct pwm_joystick_data *data = dev->data;
    const struct pwm_joystick_config *cfg = dev->config;

    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        k_work_cancel_delayable(&data->poll_work);
        /* Simply disconnect GPIOs using Zephyr API.
         * Avoid complex nRF HAL calls that might cause issues. */
        gpio_pin_configure_dt(&cfg->x_gpio, GPIO_DISCONNECTED);
        gpio_pin_configure_dt(&cfg->y_gpio, GPIO_DISCONNECTED);
        LOG_INF("PWM joystick suspended");
        return 0;
    case PM_DEVICE_ACTION_RESUME:
        /* Reconfigure GPIO pins as inputs before restarting polling */
        gpio_pin_configure_dt(&cfg->x_gpio, GPIO_INPUT);
        gpio_pin_configure_dt(&cfg->y_gpio, GPIO_INPUT);
        data->calibrated = false;
        data->slow_mode = false;
        data->idle_polls = 0;
        memset(&data->x_axis, 0, sizeof(struct axis_state));
        memset(&data->y_axis, 0, sizeof(struct axis_state));
        k_work_schedule(&data->poll_work, K_MSEC(500));
        LOG_INF("PWM joystick resumed");
        return 0;
    default:
        return -ENOTSUP;
    }
}

#define PWM_JOYSTICK_INST(n)                                                    \
    static struct pwm_joystick_data pwm_joystick_data_##n = {};                 \
    static const struct pwm_joystick_config pwm_joystick_cfg_##n = {            \
        .x_gpio = GPIO_DT_SPEC_INST_GET(n, x_gpios),                           \
        .y_gpio = GPIO_DT_SPEC_INST_GET(n, y_gpios),                           \
        .poll_period_ms = DT_INST_PROP(n, poll_period_ms),                      \
        .deadzone_pct = DT_INST_PROP(n, deadzone_pct),                          \
        .slow_gain = DT_INST_PROP(n, slow_gain),                                \
        .knee = DT_INST_PROP(n, knee),                                          \
        .fast_gain_div = DT_INST_PROP(n, fast_gain_div),                        \
        .invert_x = DT_INST_PROP(n, invert_x),                                 \
        .invert_y = DT_INST_PROP(n, invert_y),                                 \
        .swap_xy = DT_INST_PROP(n, swap_xy),                                    \
    };                                                                          \
    /* PM disabled - enabling causes wake-up failure */                           \
    DEVICE_DT_INST_DEFINE(n, pwm_joystick_init,                                 \
                          NULL,                                                  \
                          &pwm_joystick_data_##n, &pwm_joystick_cfg_##n,        \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PWM_JOYSTICK_INST)
