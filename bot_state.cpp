#include "bot_state.h"

namespace {

float signOrLast(float value, float last_sign) {
  if (value > 0.001f) {
    return 1.0f;
  }
  if (value < -0.001f) {
    return -1.0f;
  }
  return (last_sign != 0.0f) ? last_sign : 1.0f;
}

}  // namespace

void botStateInit(BotStateMachine *machine, uint32_t now_ms) {
  if (machine == nullptr) {
    return;
  }

  machine->current_state = BOT_IDLE;
  machine->state_entry_ms = now_ms;
  machine->last_valid_line_ms = now_ms;
  machine->last_valid_position = 0.0f;
  machine->last_error_sign = 1.0f;
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

  if (inputs->line_strong) {
    machine->last_valid_line_ms = inputs->now_ms;
    machine->last_valid_position = inputs->position;
    machine->last_error_sign = signOrLast(inputs->error, machine->last_error_sign);
  }

  switch (machine->current_state) {
    case BOT_IDLE:
      break;

    case BOT_CALIBRATING:
      if (inputs->calibration_done) {
        botStateTransition(machine, BOT_TRACKING, inputs->now_ms);
      }
      break;

    case BOT_TRACKING:
      if (inputs->line_strong) {
        break;
      }

      if (inputs->line_present) {
        botStateTransition(machine, BOT_EDGE, inputs->now_ms);
      } else if ((inputs->now_ms - machine->last_valid_line_ms) >= config->edge_to_recover_ms) {
        botStateTransition(machine, BOT_RECOVER, inputs->now_ms);
      } else {
        botStateTransition(machine, BOT_EDGE, inputs->now_ms);
      }
      break;

    case BOT_EDGE:
      if (inputs->line_strong) {
        botStateTransition(machine, BOT_TRACKING, inputs->now_ms);
      } else if (!inputs->line_present &&
                 (inputs->now_ms - machine->last_valid_line_ms) >= config->edge_to_recover_ms) {
        botStateTransition(machine, BOT_RECOVER, inputs->now_ms);
      }
      break;

    case BOT_RECOVER:
      if (inputs->line_strong) {
        botStateTransition(machine, BOT_TRACKING, inputs->now_ms);
      } else if (inputs->line_present) {
        botStateTransition(machine, BOT_EDGE, inputs->now_ms);
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
    case BOT_EDGE:
      return "EDGE";
    case BOT_RECOVER:
      return "RECOVER";
    case BOT_ERROR:
    default:
      return "ERROR";
  }
}
