#ifndef BOT_STATE_H
#define BOT_STATE_H

#include <stdint.h>

enum BotState {
  BOT_IDLE = 0,
  BOT_CALIBRATING,
  BOT_TRACKING,
  BOT_ROTATE,
  BOT_ERROR
};

struct BotStateConfig {
  uint8_t tracking_confirm_count;
  uint8_t rotate_confirm_count;
};

struct BotStateInputs {
  bool init_ok;
  bool calibration_done;
  bool line_present;
  bool line_strong;
  uint32_t now_ms;
};

struct BotStateMachine {
  BotState current_state;
  uint32_t state_entry_ms;
  uint8_t tracking_count;
  uint8_t rotate_count;
};

void botStateInit(BotStateMachine *machine, uint32_t now_ms);
void botStateTransition(BotStateMachine *machine, BotState next_state, uint32_t now_ms);
BotState botStateUpdate(BotStateMachine *machine, const BotStateConfig *config, const BotStateInputs *inputs);
const char *botStateName(BotState state);

#endif
