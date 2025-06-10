#include "States.h"

void Recovery::initialize_impl() {
  ctx->xbeeLoggingDelay = 500;
}

State *Recovery::loop_impl() { return nullptr; }
