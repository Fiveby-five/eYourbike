// Wire.h - STM32 HAL兼容层
#pragma once
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;  // 假设使用I2C1

class TwoWire {
public:
    TwoWire(I2C_HandleTypeDef* i2c_handle) : hi2c(i2c_handle) {}

    void begin() {
        // STM32无需初始化，在CubeMX中配置
    }

    void beginTransmission(uint8_t address) {
        txBufferIndex = 0;
        devAddress = address << 1;  // 转换为HAL要求的7位地址格式
    }

    size_t write(uint8_t data) {
        if (txBufferIndex < sizeof(txBuffer)) {
            txBuffer[txBufferIndex++] = data;
            return 1;
        }
        return 0;
    }

    int endTransmission(bool sendStop = true) {
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, devAddress, txBuffer, txBufferIndex, HAL_MAX_DELAY);
        txBufferIndex = 0;
        return (status == HAL_OK) ? 0 : 1;
    }

    int requestFrom(uint8_t address, size_t size, bool sendStop = true) {
        rxBufferIndex = 0;
        rxBufferSize = 0;
        devAddress = address << 1;
        
        HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, devAddress, &regAddress, 1, HAL_MAX_DELAY);
        if (status != HAL_OK) return 0;
        
        status = HAL_I2C_Master_Receive(hi2c, devAddress, rxBuffer, size, HAL_MAX_DELAY);
        if (status == HAL_OK) {
            rxBufferSize = size;
            return size;
        }
        return 0;
    }

    int read() {
        if (rxBufferIndex < rxBufferSize) {
            return rxBuffer[rxBufferIndex++];
        }
        return -1;
    }

    int available() {
        return rxBufferSize - rxBufferIndex;
    }

    // 用于读写寄存器地址的特殊处理
    void sendRegisterAddress(uint8_t reg) {
        regAddress = reg;
    }

private:
    I2C_HandleTypeDef* hi2c;
    uint8_t devAddress;
    uint8_t txBuffer[32];
    uint8_t txBufferIndex = 0;
    uint8_t rxBuffer[32];
    uint8_t rxBufferIndex = 0;
    uint8_t rxBufferSize = 0;
    uint8_t regAddress;
};

// 创建全局实例
extern TwoWire Wire;