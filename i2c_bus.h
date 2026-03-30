#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stddef.h>
#include <stdint.h>

#include <Wire.h>

struct I2cBusConfig {
  int sda_pin;
  int scl_pin;
  uint32_t clock_hz;
  uint8_t retry_count;
  uint16_t retry_delay_ms;
  uint16_t recovery_delay_ms;
  uint16_t transaction_gap_us;
  uint16_t failure_cooldown_ms;
  uint8_t max_consecutive_failures;
};

struct I2cBusContext {
  TwoWire *wire;
  I2cBusConfig config;
  bool initialized;
  uint8_t consecutive_failures;
  uint32_t cooldown_until_ms;
  uint32_t last_transaction_us;
};

bool i2cBusInit(I2cBusContext *ctx, TwoWire *wire, const I2cBusConfig *config);
bool i2cBusRecover(I2cBusContext *ctx);
bool i2cBusProbe(I2cBusContext *ctx, uint8_t address);
bool i2cBusWrite(I2cBusContext *ctx, uint8_t address, const uint8_t *data, size_t length);
bool i2cBusWriteRegister(I2cBusContext *ctx,
                         uint8_t address,
                         uint8_t reg,
                         const uint8_t *data,
                         size_t length);
bool i2cBusReadRegister(I2cBusContext *ctx,
                        uint8_t address,
                        uint8_t reg,
                        uint8_t *data,
                        size_t length);

#endif
