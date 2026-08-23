#pragma once
#include "rds_state.h"

// Populates state with the project's standard startup values. Called once at boot, before any
// UECP or web-interface input has arrived.
void rdsTxStateSetDefaults(RdsTxState& state);
