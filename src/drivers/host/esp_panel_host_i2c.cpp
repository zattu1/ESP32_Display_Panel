/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "utils/esp_panel_utils_log.h"
#include "esp_panel_host_i2c.hpp"

// Arduino-ESP32 driver_ng I2C HAL symbols (provided by esp32-hal-i2c-ng.c)
extern "C" {
bool i2cInit(uint8_t i2c_num, int sda, int scl, uint32_t frequency);
bool i2cDeinit(uint8_t i2c_num);
void *i2cBusHandle(uint8_t i2c_num);
}

namespace esp_panel::drivers {

HostI2C::~HostI2C()
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();

    if (isOverState(State::BEGIN)) {
        int id = getID();
        if (owns_bus) {
            (void)i2cDeinit(static_cast<uint8_t>(id));
            ESP_UTILS_LOGD("Deinit I2C host(%d) via driver_ng", id);
        }
        setState(State::DEINIT);
    }

    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();
}

bool HostI2C::begin()
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();

    if (isOverState(State::BEGIN)) {
        goto end;
    }

    {
        int id = getID();
        owns_bus = false;
        const uint8_t port = static_cast<uint8_t>(id);
        const int sda = config.sda_io_num;
        const int scl = config.scl_io_num;
        const uint32_t freq = config.master.clk_speed;

        bool ok = i2cInit(port, sda, scl, freq);
        if (!ok) {
            // If another component already initialized the bus, Arduino may report failure
            // but still provide a valid bus handle.
            host_handle = i2cBusHandle(port);
            if (host_handle != nullptr) {
                ESP_UTILS_LOGW("I2C init reported failure, but bus handle exists; continue (host=%d)", id);
                goto host_ready;
            }

            // Retry once after deinit to recover from stale state.
            (void)i2cDeinit(port);
            ok = i2cInit(port, sda, scl, freq);
            ESP_UTILS_CHECK_FALSE_RETURN(ok, false, "I2C init (driver_ng) failed");
        }

        host_handle = i2cBusHandle(port);
        ESP_UTILS_CHECK_FALSE_RETURN(host_handle != nullptr, false, "I2C bus handle is null");
        owns_bus = true;
        ESP_UTILS_LOGD("Initialize I2C host(%d) via driver_ng", id);
    }

host_ready:

    setState(State::BEGIN);

end:
    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();

    return true;
}

bool HostI2C::calibrateConfig(const i2c_config_t &config)
{
    if (memcmp(&config, &this->config, sizeof(i2c_config_t))) {
        ESP_UTILS_LOGI(
            "Original config: mode(%d), sda_io_num(%d), scl_io_num(%d), sda_pullup_en(%d), scl_pullup_en(%d),"
            "clk_speed(%d)", this->config.mode, this->config.sda_io_num, this->config.scl_io_num,
            this->config.sda_pullup_en, this->config.scl_pullup_en, static_cast<int>(this->config.master.clk_speed)
        );
        ESP_UTILS_LOGI(
            "New config: mode(%d), sda_io_num(%d), scl_io_num(%d), sda_pullup_en(%d), scl_pullup_en(%d), clk_speed(%d)",
            config.mode, config.sda_io_num, config.scl_io_num, config.sda_pullup_en, config.scl_pullup_en,
            static_cast<int>(config.master.clk_speed)
        );
        ESP_UTILS_CHECK_FALSE_RETURN(false, false, "Config mismatch");
    }

    return true;
}

} // namespace esp_panel::drivers
