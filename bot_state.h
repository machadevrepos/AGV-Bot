#ifndef BOT_STATE_H
#define BOT_STATE_H

#include <stdint.h>

enum BotState {
  BOT_IDLE = 0,
  BOT_CALIBRATING,
  BOT_TRACKING,
  BOT_EDGE,
  BOT_RECOVER,
  BOT_ERROR
};

struct BotStateConfig {
  uint32_t edge_to_recover_ms;
};

struct BotStateInputs {
  bool init_ok;
  bool calibration_done;
  bool line_present;
  bool line_strong;
  float position;
  float error;
  uint32_t now_ms;
};

struct BotStateMachine {
  BotState current_state;
  uint32_t state_entry_ms;
  uint32_t last_valid_line_ms;
  float last_valid_position;
  float last_error_sign;
};

void botStateInit(BotStateMachine *machine, uint32_t now_ms);
void botStateTransition(BotStateMachine *machine, BotState next_state, uint32_t now_ms);
BotState botStateUpdate(BotStateMachine *machine, const BotStateConfig *config, const BotStateInputs *inputs);
const char *botStateName(BotState state);

#endif
