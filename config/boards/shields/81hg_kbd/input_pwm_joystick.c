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
#define FILTER_SIZE     8
#define CALIBRATION_SAMPLES 50
/* Sampling duration, slightly over one PWM period to cover a full cycle */
#define SAMPLE_DURATION_US 1100

struct pwm_joystick_config {
    struct gpio_dt_spec x_gpio;
    struct gpio_dt_spec y_gpio;
    uint16_t poll_period_ms;
    uint8_t  deadzone_pct;
    uint8_t  speed_scale;
    bool     invert_x;
    bool     invert_y;
    bool     swap_xy;
};

struct axis_state {
    /* Filter */
    uint16_t filter_buf[FILTER_SIZE];
    uint8_t  filter_idx;

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
    struct k_work_delayable poll_work;
};

/* ---- Sliding average filter ---- */
static uint16_t filter_duty(struct axis_state *axis, uint16_t new_duty)
{
    axis->filter_buf[axis->filter_idx] = new_duty;
    axis->filter_idx = (axis->filter_idx + 1) % FILTER_SIZE;

    uint32_t sum = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        sum += axis->filter_buf[i];
    }
    return (uint16_t)(sum / FILTER_SIZE);
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

/* ---- Deadzone + linear scaling ---- */
static int32_t apply_curve(int32_t offset, int32_t deadzone, int32_t scale)
{
    if (offset > -deadzone && offset < deadzone) {
        return 0;
    }

    int32_t sign = (offset >= 0) ? 1 : -1;
    int32_t abs_off = (offset >= 0) ? offset : -offset;
    int32_t effective = abs_off - deadzone;

    if (effective < 0) {
        return 0;
    }

    int32_t movement = (effective * scale) / 400;
    if (movement > 127) {
        movement = 127;
    }
    return sign * movement;
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

            /* Initialize filter buffers with center value */
            for (int i = 0; i < FILTER_SIZE; i++) {
                data->x_axis.filter_buf[i] = data->x_axis.center_duty;
                data->y_axis.filter_buf[i] = data->y_axis.center_duty;
            }

            LOG_INF("Calibrated: X_center=%u Y_center=%u",
                    data->x_axis.center_duty, data->y_axis.center_duty);
        }

        k_work_schedule(&data->poll_work, K_MSEC(cfg->poll_period_ms));
        return;
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

    /* Deadzone in duty-cycle units */
    int32_t deadzone = (cfg->deadzone_pct * DUTY_SCALE) / 100;

    /* Apply curve */
    int32_t dx = apply_curve(off_x, deadzone, cfg->speed_scale);
    int32_t dy = apply_curve(off_y, deadzone, cfg->speed_scale);

    /* Swap axes */
    if (cfg->swap_xy) {
        int32_t tmp = dx;
        dx = dy;
        dy = tmp;
    }

    /* Report movement */
    if (dx != 0 || dy != 0) {
        input_report_rel(data->dev, INPUT_REL_X, dx, false, K_NO_WAIT);
        input_report_rel(data->dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
    }

    k_work_schedule(&data->poll_work, K_MSEC(cfg->poll_period_ms));
}

/* ---- Device initialization ---- */
static int pwm_joystick_init(const struct device *dev)
{
    struct pwm_joystick_data *data = dev->data;
    const struct pwm_joystick_config *cfg = dev->config;
    int ret;

    data->dev = dev;
    data->calibrated = false;
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
        .speed_scale = DT_INST_PROP(n, speed_scale),                            \
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
