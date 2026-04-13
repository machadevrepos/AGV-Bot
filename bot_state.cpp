#include "bot_state.h"

void botStateInit(BotStateMachine *machine, uint32_t now_ms) {
  if (machine == nullptr) {
    return;
  }

  machine->current_state = BOT_IDLE;
  machine->state_entry_ms = now_ms;
  machine->tracking_count = 0U;
  machine->rotate_count = 0U;
}

void botStateTransition(BotStateMachine *machine, BotState next_state, uint32_t now_ms) {
  if (machine == nullptr) {
    return;
  }

  machine->current_state = next_state;
  machine->state_entry_ms = now_ms;
}

BotState botStateUpdate(BotStateMachine *machine, const BotStateConfig *config, const BotStateInputs *inputs) {
  if (machine == nullptr || config == nullptr || inputs == nullptr) {
    return BOT_ERROR;
  }

  if (!inputs->init_ok) {
    botStateTransition(machine, BOT_ERROR, inputs->now_ms);
    return machine->current_state;
  }

  switch (machine->current_state) {
    case BOT_IDLE:
      break;

    case BOT_CALIBRATING:
      if (inputs->calibration_done) {
        botStateTransition(machine, BOT_ROTATE, inputs->now_ms);
        machine->tracking_count = 0U;
        machine->rotate_count = 0U;
      }
      break;

    case BOT_TRACKING:
      if (inputs->line_present) {
        machine->rotate_count = 0U;
      } else {
        if (machine->rotate_count < 255U) {
          ++machine->rotate_count;
        }
      }

      if (machine->rotate_count >= config->rotate_confirm_count) {
        botStateTransition(machine, BOT_ROTATE, inputs->now_ms);
        machine->tracking_count = 0U;
        machine->rotate_count = 0U;
      }
      break;

    case BOT_ROTATE:
      if (inputs->line_strong) {
        if (machine->tracking_count < 255U) {
          ++machine->tracking_count;
        }
      } else {
        machine->tracking_count = 0U;
      }

      if (machine->tracking_count >= config->tracking_confirm_count) {
        botStateTransition(machine, BOT_TRACKING, inputs->now_ms);
        machine->tracking_count = 0U;
        machine->rotate_count = 0U;
      }
      break;

    case BOT_ERROR:
      break;
  }

  return machine->current_state;
}

const char *botStateName(BotState state) {
  switch (state) {
    case BOT_IDLE:
      return "IDLE";
    case BOT_CALIBRATING:
      return "CALIBRATING";
    case BOT_TRACKING:
      return "TRACKING";
    case BOT_ROTATE:
      return "ROTATE";
    case BOT_ERROR:
    default:
      return "ERROR";
  }
}
