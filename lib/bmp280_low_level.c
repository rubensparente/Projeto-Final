#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "bmp280.h"
#include <math.h>

// I2C port para BMP280
#define BMP_I2C_PORT i2c0

void bmp280_i2c_init() {
    // O I2C já é inicializado na função main
}

void bmp280_read_array(uint8_t deviceAddress, uint8_t startRegisterAddress, uint8_t *data, uint8_t dataLength) {
    // Primeiro escreve o endereço do registro
    uint8_t reg = startRegisterAddress;
    i2c_write_blocking(BMP_I2C_PORT, deviceAddress, &reg, 1, true);
    // Depois lê os dados
    i2c_read_blocking(BMP_I2C_PORT, deviceAddress, data, dataLength, false);
}

void bmp280_write_array(uint8_t deviceAddress, uint8_t startRegisterAddress, uint8_t *data, uint8_t dataLength) {
    uint8_t buffer[dataLength + 1];
    buffer[0] = startRegisterAddress;
    
    // Copia os dados para o buffer
    for(uint8_t i = 0; i < dataLength; i++) {
        buffer[i + 1] = data[i];
    }
    
    // Escreve tudo de uma vez
    i2c_write_blocking(BMP_I2C_PORT, deviceAddress, buffer, dataLength + 1, false);
}

void delay_function(uint32_t delayMS) {
    sleep_ms(delayMS);
}

float power_function(float x, float y) {
    // Usa a função powf da math library
    return powf(x, y);
}