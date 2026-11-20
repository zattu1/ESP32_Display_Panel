/*
 * This file was an experiment to implement a custom esp_lcd panel IO over I2C
 * using driver_ng. It can't be implemented here because `struct esp_lcd_panel_io_t`
 * is opaque in the public headers shipped with this environment.
 *
 * We now use ESP-IDF's panel IO on the new I2C master bus:
 * `esp_lcd_new_panel_io_i2c_v2()` on IDF 5.5–5.x, and `esp_lcd_new_panel_io_i2c()` on IDF 6.
 */

#if 0
#endif
