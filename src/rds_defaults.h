#pragma once
#include "rds_state.h"

// Populates state with the project's standard startup values: every data set (DSN 1-6) gets a single
// main service, PSN 1, with the same generic placeholder content and no EON services; DSN 1 is
// active. Called once at boot, before any UECP or web-interface input has arrived.
void rdsEncoderStateSetDefaults(RdsEncoderState& state);

// Copies data set 1 (main service, EON services and DSN-level content - group sequence, SLC etc.)
// over every other data set, and captures DSN 1's main service as state.serviceDefaults (the
// starting content for services a UECP MEC 0x28 PSN list creates later). Lets a sketch apply its
// own boot overrides (station PI/PS, group sequence...) once, to rdsActiveMain()/rdsActiveDataSet()
// straight after rdsEncoderStateSetDefaults(), and then have all six data sets - and every service
// defined afterwards - start out identical. Call once in setup(), after those overrides.
void rdsCopyDataSet1ToAll(RdsEncoderState& state);

// Result of rdsDefinePsnList() - every value except RDS_PSN_LIST_OK means nothing was changed.
enum RdsPsnListResult {
  RDS_PSN_LIST_OK,
  RDS_PSN_LIST_BAD_DSN,     // not 1..RDS_DSN_COUNT
  RDS_PSN_LIST_ACTIVE_DSN,  // the active data set's list can't be redefined while it's on air
  RDS_PSN_LIST_BAD_COUNT,   // empty, or more than RDS_EON_PER_DSN_MAX + 1 PSNs
  RDS_PSN_LIST_BAD_PSN,     // contains PSN 0 (not a valid service number)
  RDS_PSN_LIST_DUPLICATE    // the same PSN appears twice
};

// UECP MEC 0x28 - (re)defines data set dsn's PSN list from scratch: psns[0] becomes its main
// service, psns[1..count-1] its EON services, in that order. Every service is freshly initialised
// from state.serviceDefaults - anything previously stored for that data set's services is wiped,
// even for PSNs that appear in both the old and new list. The data set's DSN-level content (group
// sequence, SLC, Group 1A/14A variant sequences) is left untouched. Validates everything first, so a
// rejected list leaves the data set exactly as it was.
RdsPsnListResult rdsDefinePsnList(RdsEncoderState& state, uint8_t dsn, const uint8_t* psns,
                                  uint8_t count);
