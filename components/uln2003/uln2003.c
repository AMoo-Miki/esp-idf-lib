#include <string.h>
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include <math.h>
#include <freertos/task.h>
#include "driver/gpio.h"

#include "uln2003.h"

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

static const char *TAG = "ULN2003";

static const uint16_t step_gap[] = {
        [ULN2003_MODE_FULL_STEPS] = 3000,
        [ULN2003_MODE_HALF_STEPS] = 660,
};

esp_err_t uln2003_set_mode(uln2003_dev_t *dev, uln2003_mode_t mode) {
    CHECK_ARG(dev);

    if (dev->mode != mode) {
        if (mode == ULN2003_MODE_FULL_STEPS) {
            int8_t remainder = (int8_t) (dev->_position % 2);
            if (remainder == 1)
                dev->_position--;
            else if (remainder == -1)
                dev->_position++;
        } else {
            dev->_position = dev->position;
        }
    }

    dev->mode = mode;

    return ESP_OK;
}

esp_err_t uln2003_get_mode(uln2003_dev_t *dev, uln2003_mode_t *mode) {
    CHECK_ARG(dev && mode);

    *mode = dev->mode;

    return ESP_OK;
}

esp_err_t uln2003_init_desc(uln2003_dev_t *dev, gpio_num_t pin1, gpio_num_t pin2, gpio_num_t pin3, gpio_num_t pin4) {
    CHECK_ARG(dev);

    dev->pin1 = pin1;
    dev->pin2 = pin2;
    dev->pin3 = pin3;
    dev->pin4 = pin4;

    dev->direction = ULN2003_DIRECTION_NONE;
    dev->mode = ULN2003_MODE_FULL_STEPS;
    dev->position = 0;
    dev->_position = 0;

    return ESP_OK;
}

esp_err_t uln2003_stop(uln2003_dev_t *dev) {
    CHECK(gpio_set_level(dev->pin1, 0));
    CHECK(gpio_set_level(dev->pin2, 0));
    CHECK(gpio_set_level(dev->pin3, 0));
    CHECK(gpio_set_level(dev->pin4, 0));

    return ESP_OK;
}

esp_err_t uln2003_update_timer(uln2003_dev_t *dev) {
    if (dev->targeted && dev->position == dev->target) {
        dev->direction = ULN2003_DIRECTION_NONE;
    }

    if (dev->direction == ULN2003_DIRECTION_NONE) {
        CHECK(uln2003_stop(dev));
        return esp_timer_stop(dev->timer);
    }

    if (!esp_timer_is_active(dev->timer)) {
        esp_timer_start_periodic(dev->timer, step_gap[dev->mode]);
    }

    return ESP_OK;
}

static void uln2003_step_timer(void *arg) {
    uln2003_dev_t *dev = (uln2003_dev_t *) arg;

    uln2003_update_timer(dev);

    if (dev->direction == ULN2003_DIRECTION_NONE) return;

    int step = (dev->mode == ULN2003_MODE_FULL_STEPS ? 2 : 1) * (dev->direction == ULN2003_DIRECTION_CCW ? -1 : 1);
    uint8_t nextStep = ((dev->_position + step) / (dev->mode == ULN2003_MODE_FULL_STEPS ? 2 : 1)) % dev->mode;
    const uint8_t *step_def = dev->mode == ULN2003_MODE_FULL_STEPS
                              ? ULN2003_SEQUENCE_FULL_STEPS[nextStep]
                              : ULN2003_SEQUENCE_HALF_STEPS[nextStep];

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(dev->pin1, (uint32_t) step_def[0]));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(dev->pin2, (uint32_t) step_def[1]));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(dev->pin3, (uint32_t) step_def[2]));
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(dev->pin4, (uint32_t) step_def[3]));

    dev->position += step;
    dev->_position += step;
}

esp_err_t uln2003_init(uln2003_dev_t *dev) {
    CHECK_ARG(dev);

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << dev->pin1;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = 1ULL << dev->pin2;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = 1ULL << dev->pin3;
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = 1ULL << dev->pin4;
    gpio_config(&io_conf);

    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &uln2003_step_timer,
            .arg = (void *) dev
    };
    CHECK(esp_timer_create(&periodic_timer_args, &dev->timer));
    CHECK(esp_timer_start_periodic(dev->timer, step_gap[dev->mode]));

    return ESP_OK;
}

esp_err_t uln2003_set_direction(uln2003_dev_t *dev, uln2003_direction_t direction) {
    CHECK_ARG(dev);
    dev->targeted = false;
    dev->direction = direction;

    return uln2003_update_timer(dev);
}

esp_err_t uln2003_get_direction(uln2003_dev_t *dev, uln2003_direction_t *direction) {
    CHECK_ARG(dev && direction);
    *direction = dev->direction;

    return ESP_OK;
}

esp_err_t uln2003_reset_position(uln2003_dev_t *dev) {
    CHECK_ARG(dev);
    dev->position = 0;

    return ESP_OK;
}

esp_err_t uln2003_get_position(uln2003_dev_t *dev, int *position) {
    CHECK_ARG(dev && position);
    *position = dev->position;

    return ESP_OK;
}

esp_err_t uln2003_goto_position(uln2003_dev_t *dev, int position) {
    CHECK_ARG(dev);

    dev->targeted = true;
    dev->target = position;

    return uln2003_update_timer(dev);
}


esp_err_t uln2003_free_desc(uln2003_dev_t *dev) {
    CHECK_ARG(dev);

    void *d = dev;
    free(d);

    return ESP_OK;
}