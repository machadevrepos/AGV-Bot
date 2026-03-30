#include "bot_state.h"

void botStateInit(BotStateMachine *machine, uint32_t now_ms) {
  if (machine == nullptr) {
    return;
  }

  machine->current_state = BOT_IDLE;
  machine->state_entry_ms = now_ms;
  machine->line_present_count = 0U;
  machine->line_lost_count = 0U;
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
        botStateTransition(machine, BOT_LINE_LOST, inputs->now_ms);
        machine->line_present_count = 0U;
        machine->line_lost_count = 0U;
      }
      break;

    case BOT_LINE_PRESENT:
      if (inputs->line_present) {
        machine->line_lost_count = 0U;
      } else {
        if (machine->line_lost_count < 255U) {
          ++machine->line_lost_count;
        }
      }

      if (machine->line_lost_count >= config->line_lost_confirm_count) {
        botStateTransition(machine, BOT_LINE_LOST, inputs->now_ms);
        machine->line_present_count = 0U;
        machine->line_lost_count = 0U;
      }
      break;

    case BOT_LINE_LOST:
      if (inputs->line_strong) {
        if (machine->line_present_count < 255U) {
          ++machine->line_present_count;
        }
      } else {
        machine->line_present_count = 0U;
      }

      if (machine->line_present_count >= config->line_present_confirm_count) {
        botStateTransition(machine, BOT_LINE_PRESENT, inputs->now_ms);
        machine->line_present_count = 0U;
        machine->line_lost_count = 0U;
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
    case BOT_LINE_PRESENT:
      return "LINE_PRESENT";
    case BOT_LINE_LOST:
      return "LINE_LOST";
    case BOT_ERROR:
    default:
      return "ERROR";
  }
}
