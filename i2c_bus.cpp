#include "i2c_bus.h"

#include <Arduino.h>
#include <string.h>

namespace {

bool isCooldownActive(const I2cBusContext *ctx) {
  if (ctx == nullptr || ctx->cooldown_until_ms == 0U) {
    return false;
  }

  return static_cast<int32_t>(millis() - ctx->cooldown_until_ms) < 0;
}

void clearBusFaultState(I2cBusContext *ctx) {
  if (ctx == nullptr) {
    return;
  }

  ctx->consecutive_failures = 0U;
  ctx->cooldown_until_ms = 0U;
}

void markTransactionBoundary(I2cBusContext *ctx) {
  if (ctx != nullptr) {
    ctx->last_transaction_us = micros();
  }
}

void enforceTransactionGap(I2cBusContext *ctx) {
  if (ctx == nullptr || ctx->config.transaction_gap_us == 0U) {
    return;
  }

  const uint32_t now_us = micros();
  if (ctx->last_transaction_us != 0U) {
    const uint32_t elapsed_us = now_us - ctx->last_transaction_us;
    if (elapsed_us < ctx->config.transaction_gap_us) {
      delayMicroseconds(ctx->config.transaction_gap_us - elapsed_us);
    }
  }

  ctx->last_transaction_us = micros();
}

bool i2cWriteRaw(TwoWire *wire, uint8_t address, const uint8_t *data, size_t length) {
  if (wire == nullptr) {
    return false;
  }

  wire->beginTransmission(address);
  if (data != nullptr && length > 0U) {
    wire->write(data, length);
  }
  return (wire->endTransmission() == 0);
}

bool i2cReadWithRegister(TwoWire *wire,
                         uint8_t address,
                         uint8_t reg,
                         uint8_t *data,
                         size_t length) {
  if (wire == nullptr || data == nullptr || length == 0U) {
    return false;
  }

  wire->beginTransmission(address);
  wire->write(reg);
  if (wire->endTransmission(false) != 0) {
    return false;
  }

  const int requested = wire->requestFrom(static_cast<int>(address), static_cast<int>(length));
  if (requested != static_cast<int>(length)) {
    return false;
  }

  for (size_t i = 0; i < length; ++i) {
    data[i] = wire->read();
  }
  return true;
}

template <typename Operation>
bool runWithRecovery(I2cBusContext *ctx, Operation operation) {
  if (ctx == nullptr || ctx->wire == nullptr || !ctx->initialized) {
    return false;
  }

  if (isCooldownActive(ctx)) {
    return false;
  }

  const uint8_t attempts = (ctx->config.retry_count > 0U) ? ctx->config.retry_count : 1U;
  for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
    enforceTransactionGap(ctx);
    if (operation()) {
      clearBusFaultState(ctx);
      markTransactionBoundary(ctx);
      return true;
    }

    markTransactionBoundary(ctx);

    if (attempt + 1U >= attempts) {
      break;
    }

    i2cBusRecover(ctx);
    if (ctx->config.retry_delay_ms > 0U) {
      delay(ctx->config.retry_delay_ms);
    }
  }

  if (ctx->consecutive_failures < 0xFFU) {
    ++ctx->consecutive_failures;
  }

  if (ctx->config.max_consecutive_failures > 0U &&
      ctx->consecutive_failures >= ctx->config.max_consecutive_failures) {
    (void)i2cBusRecover(ctx);
    if (ctx->config.failure_cooldown_ms > 0U) {
      ctx->cooldown_until_ms = millis() + ctx->config.failure_cooldown_ms;
    }
  }

  return false;
}

}  // namespace

bool i2cBusInit(I2cBusContext *ctx, TwoWire *wire, const I2cBusConfig *config) {
  if (ctx == nullptr || wire == nullptr || config == nullptr) {
    return false;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->wire = wire;
  ctx->config = *config;
  ctx->wire->begin(config->sda_pin, config->scl_pin);
  ctx->wire->setClock(config->clock_hz);
  if (config->recovery_delay_ms > 0U) {
    delay(config->recovery_delay_ms);
  }
  ctx->initialized = true;
  ctx->last_transaction_us = micros();
  clearBusFaultState(ctx);
  return true;
}

bool i2cBusRecover(I2cBusContext *ctx) {
  if (ctx == nullptr || ctx->wire == nullptr) {
    return false;
  }

  if (ctx->initialized) {
    ctx->wire->end();
  }

  ctx->wire->begin(ctx->config.sda_pin, ctx->config.scl_pin);
  ctx->wire->setClock(ctx->config.clock_hz);
  if (ctx->config.recovery_delay_ms > 0U) {
    delay(ctx->config.recovery_delay_ms);
  }
  ctx->initialized = true;
  ctx->last_transaction_us = micros();
  return true;
}

bool i2cBusProbe(I2cBusContext *ctx, uint8_t address) {
  return runWithRecovery(ctx, [ctx, address]() {
    return i2cWriteRaw(ctx->wire, address, nullptr, 0U);
  });
}

bool i2cBusWrite(I2cBusContext *ctx, uint8_t address, const uint8_t *data, size_t length) {
  return runWithRecovery(ctx, [ctx, address, data, length]() {
    return i2cWriteRaw(ctx->wire, address, data, length);
  });
}

bool i2cBusWriteRegister(I2cBusContext *ctx,
                         uint8_t address,
                         uint8_t reg,
                         const uint8_t *data,
                         size_t length) {
  return runWithRecovery(ctx, [ctx, address, reg, data, length]() {
    uint8_t buffer[40];
    if (length > (sizeof(buffer) - 1U)) {
      return false;
    }

    buffer[0] = reg;
    if (data != nullptr && length > 0U) {
      memcpy(&buffer[1], data, length);
    }
    return i2cWriteRaw(ctx->wire, address, buffer, length + 1U);
  });
}

bool i2cBusReadRegister(I2cBusContext *ctx,
                        uint8_t address,
                        uint8_t reg,
                        uint8_t *data,
                        size_t length) {
  return runWithRecovery(ctx, [ctx, address, reg, data, length]() {
    return i2cReadWithRegister(ctx->wire, address, reg, data, length);
  });
}
