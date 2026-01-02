/*
 * This file was an experiment to implement a custom esp_lcd panel IO over I2C
 * using driver_ng. It can't be implemented here because `struct esp_lcd_panel_io_t`
 * is opaque in the public headers shipped with this environment.
 *
 * We now use `esp_lcd_new_panel_io_i2c_v2()` from ESP-IDF instead.
 */

#if 0
#endif
