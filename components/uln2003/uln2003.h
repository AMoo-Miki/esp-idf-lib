#ifndef ESP_IDF_LIB_ULN2003_H
#define ESP_IDF_LIB_ULN2003_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <esp_timer.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ULN2003_DIRECTION_NONE = 0,
    ULN2003_DIRECTION_CW,
    ULN2003_DIRECTION_CCW,
} uln2003_direction_t;

typedef enum {
    ULN2003_MODE_FULL_STEPS = 4,
    ULN2003_MODE_HALF_STEPS = 8,
} uln2003_mode_t;

typedef struct {
    gpio_num_t pin1;
    gpio_num_t pin2;
    gpio_num_t pin3;
    gpio_num_t pin4;

    int position;
    int _position;
    int target;
    bool targeted;

    uln2003_direction_t direction;
    uln2003_mode_t mode;

    esp_timer_handle_t timer;
} uln2003_dev_t;

static const uint8_t ULN2003_SEQUENCE_FULL_STEPS[ULN2003_MODE_FULL_STEPS][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
};

static const uint8_t ULN2003_SEQUENCE_HALF_STEPS[ULN2003_MODE_HALF_STEPS][4] = {
        {1, 0, 0, 0},
        {1, 1, 0, 0},
        {0, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 1},
        {0, 0, 0, 1},
        {1, 0, 0, 1},
};

/**
 * @brief Initialize device descriptor
 *
 * @param dev I2C device descriptor
 * @param addr ULN2003 address
 * @param bootloader Boot into bootloader after initialization
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return ESP_OK to indicate success
 */
esp_err_t uln2003_init_desc(uln2003_dev_t *dev, gpio_num_t pin1, gpio_num_t pin2, gpio_num_t pin3, gpio_num_t pin4);

/**
 * @brief Free device descriptor
 *
 * @param dev I2C device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t uln2003_free_desc(uln2003_dev_t *dev);

/**
 * @brief Initialize device
 *
 * @param dev I2C device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t uln2003_init(uln2003_dev_t *dev);

/**
 * @brief Set mode
 * @param dev I2C device descriptor
 * @param mode mode in which sensor will be running
 * @return
 */
esp_err_t uln2003_set_mode(uln2003_dev_t *dev, uln2003_mode_t mode);

esp_err_t uln2003_get_mode(uln2003_dev_t *dev, uln2003_mode_t *mode);

esp_err_t uln2003_set_direction(uln2003_dev_t *dev, uln2003_direction_t direction);

esp_err_t uln2003_get_direction(uln2003_dev_t *dev, uln2003_direction_t *direction);

esp_err_t uln2003_reset_position(uln2003_dev_t *dev);

esp_err_t uln2003_get_position(uln2003_dev_t *dev, int *position);

esp_err_t uln2003_goto_position(uln2003_dev_t *dev, int position);

#ifdef __cplusplus
}
#endif

#endif // ESP_IDF_LIB_ULN2003_H
