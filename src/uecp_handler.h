#pragma once
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include "rds_state.h"
#include "free_format_groups.h"
#include "oda_directory.h"

// Maximum raw (stuffed) byte count accepted between a 0xFE/0xFF pair.
#define UECP_MAX_FRAME 512

// Per the UECP spec, an encoder is allowed more than one site address and more than one encoder
// address (e.g. a relay/repeater acting on behalf of several sites, or standing in for several
// encoder identities at once) - these caps just bound the fixed arrays below; raise them if a real
// deployment ever needs more.
#define UECP_MAX_SITE_ADDRESSES    4
#define UECP_MAX_ENCODER_ADDRESSES 8

// This device's own UECP address(es) - site address is 10 bits, encoder address 6 bits per spec.
// Default (the struct's own default member initializers below) is a single 0/0 entry, meaning
// "accept every frame regardless of its address", per the UECP spec's own "all sites"/"all
// encoders" convention plus this project's own default policy - the same default this project has
// always had, now just expressed as a 1-entry list instead of a single scalar. See
// processUecpFrame()'s address-filter step (uecpAddressMatches() in uecp_handler.cpp) for the exact
// matching rule once more than one address is configured. Every sketch's own UECP Site/Encoder
// Address CFG key or web form field(s) (and parseUecpAddressList() below, shared by all of them)
// populate this by replacing sites[]/siteCount or encoders[]/encoderCount wholesale, same "replace
// the whole list" convention this project's SEQ=/rds_seq group-sequence field already uses -
// resending the same value is always idempotent, and setting it back to a single 0 restores the
// "accept everything" default.
struct UecpAddressConfig {
  uint16_t sites[UECP_MAX_SITE_ADDRESSES]      = {0};
  uint8_t  siteCount                            = 1;
  uint8_t  encoders[UECP_MAX_ENCODER_ADDRESSES] = {0};
  uint8_t  encoderCount                         = 1;
};

// Parses a comma-separated list of numbers (e.g. "12,34" decimal or "01A,02B" hex - pass whichever
// `base` the caller already used for this field, per strtol()) into `out[]`, up to `maxCount`
// entries, masking each parsed value with `mask` (0x03FF for a site address, 0x3F for an encoder
// address, matching their own field widths above). Returns the number of values actually parsed (0
// for an empty string); entries beyond maxCount are silently dropped rather than rejected, same
// "capped, not rejected" precedent as this project's other list fields (e.g. MEC 0x16/SEQ=/
// rds_seq's own group sequence). Shared by every sketch's UECP Site/Encoder Address CFG key and web
// form field(s), so a list of addresses parses identically everywhere it's entered - the caller is
// responsible for actually replacing sites[]/siteCount or encoders[]/encoderCount with the result
// (see UecpAddressConfig's own comment above).
inline uint8_t parseUecpAddressList(const char* text, uint16_t* out, uint8_t maxCount,
                                     uint16_t mask, int base) {
  uint8_t count = 0;
  char buf[64];
  strncpy(buf, text, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  char* p = buf;
  while (*p != '\0' && count < maxCount) {
    while (*p == ' ' || *p == ',') p++;
    if (*p == '\0') break;
    char* tokenStart = p;
    while (*p != '\0' && *p != ',') p++;
    bool more = (*p != '\0');
    if (more) *p = '\0';
    out[count++] = (uint16_t)(strtol(tokenStart, nullptr, base) & mask);
    if (!more) break;
    p++;
  }
  return count;
}

// Validate, address-filter, and dispatch a single UECP frame.
// rawInner: the bytes captured between 0xFE and 0xFF (not including those framing bytes).
// rawLen:   number of bytes in rawInner.
// clientIp: used only for log messages.
// state:    RDS encoder state (all data sets) updated by each handled MEC - see
//           uecp_handler.cpp's uecpDsnTargets()/uecpForEachMainService() for how DSN/PSN pick
//           which data set(s)/service(s) an element lands in.
// ffPool:   free-format (MEC 0x24/0x30) per-group queues updated by those MECs - see
//           free_format_groups.h. Unrelated to state/RdsEncoderState, which only ever holds this
//           project's own fixed struct-backed groups (0A/1A/2A/3A-manual/4A).
// odaLiveDir: persistent AID -> group directory, updated whenever a live Group 3A MEC 0x24/0x40/
//           0x46 definition arrives - see oda_directory.h. Independent of ffPool's own Group 3A
//           queue lifecycle.
// pendingImmediateGroupIndex: set (to the group that should be built next, bypassing the normal
//           schedule) when a MEC 0x46 "immediate" priority message is applied - see
//           buildNextGroup()'s own check of this in rds_scheduler.cpp. FF_INDEX_NONE (its usual value)
//           means nothing is pending; the caller should leave it as FF_INDEX_NONE going in.
// ourAddress: this device's own UECP address(es) - see UecpAddressConfig's own comment above for
//           its default and how each sketch populates it; see processUecpFrame()'s address-filter
//           step (uecpAddressMatches() in uecp_handler.cpp) for the exact matching rule.
void processUecpFrame(const uint8_t* rawInner, uint16_t rawLen,
                      const String& clientIp, RdsEncoderState& state, FreeFormatPool& ffPool,
                      OdaLiveDirectory& odaLiveDir, uint8_t& pendingImmediateGroupIndex,
                      const UecpAddressConfig& ourAddress);
