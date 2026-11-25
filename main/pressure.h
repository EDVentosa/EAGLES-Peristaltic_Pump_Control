
#ifndef PRESSURE_H
#define PRESSURE_H

#include <stdint.h>
#include "esp_err.h"

// Initialize I2C master bus
void i2c_master_init(void);

// Initialize ADS1115
void ads1115_init(void);

// ADS1115 register operations
esp_err_t ads1115_write_register(uint8_t reg, uint16_t value);
esp_err_t ads1115_read_register(uint8_t reg, uint16_t *value);

// Read ADC voltage from AIN0
esp_err_t ads1115_read_voltage(float *voltage);

// Pressure sensor conversions
float pressure_sensor_get_voltage(float adc_voltage);
float pressure_sensor_get_mpa(float sensor_voltage);

#endif // PRESSURE_H
