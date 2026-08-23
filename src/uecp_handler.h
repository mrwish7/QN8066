#pragma once
#include <Arduino.h>
#include "rds_state.h"
#include "free_format_groups.h"
#include "oda_directory.h"

// Maximum raw (stuffed) byte count accepted between a 0xFE/0xFF pair.
#define UECP_MAX_FRAME 512

// Validate, address-filter, and dispatch a single UECP frame.
// rawInner: the bytes captured between 0xFE and 0xFF (not including those framing bytes).
// rawLen:   number of bytes in rawInner.
// clientIp: used only for log messages.
// state:    RDS transmitter state updated by each handled MEC.
// ffPool:   free-format (MEC 0x24/0x30) per-group queues updated by those MECs - see
//           free_format_groups.h. Unrelated to state/RdsTxState, which only ever holds this
//           project's own fixed struct-backed groups (0A/1A/2A/3A-manual/4A).
// odaLiveDir: persistent AID -> group directory, updated whenever a live Group 3A MEC 0x24/0x40/
//           0x46 definition arrives - see oda_directory.h. Independent of ffPool's own Group 3A
//           queue lifecycle.
// pendingImmediateGroupIndex: set (to the group that should be built next, bypassing the normal
//           schedule) when a MEC 0x46 "immediate" priority message is applied - see
//           buildNextGroup()'s own check of this in rds_scheduler.cpp. FF_INDEX_NONE (its usual value)
//           means nothing is pending; the caller should leave it as FF_INDEX_NONE going in.
// ourSiteAddress/ourEncoderAddress: this device's own UECP address (0/0 = accept every frame -
// see processUecpFrame()'s address-filter step for the exact matching rule).
void processUecpFrame(const uint8_t* rawInner, uint16_t rawLen,
                      const String& clientIp, RdsTxState& state, FreeFormatPool& ffPool,
                      OdaLiveDirectory& odaLiveDir, uint8_t& pendingImmediateGroupIndex,
                      uint16_t ourSiteAddress, uint8_t ourEncoderAddress);
