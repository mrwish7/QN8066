#include "uecp_handler.h"
#include <string.h>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// CRC-16/CCITT with final inversion, matching crc16_ccitt() in uecp_mp2.py.
// Applied to destuffed[0..dLen-3]; the result must equal destuffed[dLen-2..dLen-1].
static uint16_t uecpCrc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    }
  }
  return 0xFFFF - crc;
}

// Byte de-stuffing (unescape_uecp in uecp_mp2.py).
// 0xFD is the escape byte; the byte that follows has 0xFD added back to recover
// the original value (0xFD→0xFD 0x00, 0xFE→0xFD 0x01, 0xFF→0xFD 0x02).
static uint16_t uecpDestuff(const uint8_t* in, uint16_t inLen, uint8_t* out) {
  uint16_t outLen = 0;
  bool escaped = false;
  for (uint16_t i = 0; i < inLen; i++) {
    uint8_t b = in[i];
    if (escaped) {
      escaped = false;
      out[outLen++] = ((int)b + 0xFD <= 0xFF) ? (b + 0xFD) : 0xFF;
    } else if (b == 0xFD) {
      escaped = true;
    } else {
      out[outLen++] = b;
    }
  }
  return outLen;
}

// Returns the fixed MED (data) length for the small set of MECs currently handled - all of them
// have a fixed MEC[1]+DSN[1]+PSN[1]+MED[dataLen] layout with no MEL (length) field, per their own
// entries in the UECP spec's Section 3 message catalog. Returns 0 for anything else: an
// unrecognized MEC's own layout (and even whether it has DSN/PSN/MEL fields at all) is
// MEC-specific, so its length can't be inferred generically - the walk in processUecpFrame() must
// stop there rather than guess and misparse whatever bytes follow (this is exactly the "MEL
// overflow" fragility that made per-element dispatch unsafe before this fixed-length subset).
static uint8_t uecpMecDataLen(uint8_t mec) {
  switch (mec) {
    case 0x02: // PS
      return RDS_PS_LEN;
    case 0x3E: // PTYN
      return RDS_PTYN_LEN;
    case 0x01: // PI
    case 0x06: // PIN
      return 2;
    case 0x03: // TA/TP
    case 0x04: // DI/PTYI
    case 0x05: // MS
    case 0x07: // PTY
      return 1;
    default:
      return 0;
  }
}

// UECP DSN address matching. 0 means "no DSN used" (a source that never partitions by dataset) and
// always matches, regardless of what this device's own state.dsn is configured as. 255 is the
// UECP spec's own "all datasets" wildcard (a source that does partition by dataset, but wants this
// particular element to reach every one of them at once) - also always matches. Two genuinely
// different source intents, but this project doesn't need to tell them apart: the effect ("apply
// regardless of state.dsn") is identical either way. Was missing the 255 case entirely until a real
// UECP source was observed sending it for exactly this "reaches our dataset too" purpose - every
// element addressed that way was silently dropped as a mismatch instead.
static inline bool uecpDsnMatches(uint8_t dsn, const RdsTxState& state) {
  return dsn == 0 || dsn == 0xFF || state.dsn == 0 || dsn == state.dsn;
}

// Applies one already DSN/PSN-matched element's MED bytes (MSB first, per the UECP spec's own
// format tables) to the matching RdsTxState field.
static void uecpApplyMec(uint8_t mec, const uint8_t* data, RdsTxState& state) {
  switch (mec) {
    case 0x01: state.pi[0]    = data[0]; state.pi[1]  = data[1]; break; // PI
    case 0x02: // PS - staged, not written to ps[] directly: sendRDS() swaps it in at the next
               // clean psSegment==0 wrap so a receiver never sees old/new characters mixed
               // mid-cycle (see rds_state.h's psPending/psPendingValid comment).
      memcpy(state.psPending, data, RDS_PS_LEN);
      state.psPendingValid = true;
      break;
    case 0x03: state.tatp     = data[0];                         break; // TA/TP
    case 0x04: state.diPtyi   = data[0];                         break; // DI/PTYI
    case 0x05: state.ms       = data[0];                         break; // MS
    case 0x06: state.pin[0]   = data[0]; state.pin[1] = data[1]; break; // PIN
    case 0x07: state.pty      = data[0];                         break; // PTY
    case 0x3E: // PTYN - applied immediately (see rds_state.h's ptyn comment), and the AB flag
               // flips unconditionally on every element, whether or not the text actually changed.
      memcpy(state.ptyn, data, RDS_PTYN_LEN);
      state.ptynABflag = !state.ptynABflag;
      break;
  }
}

// Applies one already DSN/PSN-matched RT (MEC 0x0A) element. Unless state.rtBufferMode is set, a
// clearing replace is applied immediately - no staging - since the RDS spec's own text-A/B flag
// already gives receivers a clean "content changed" signal, so a torn mid-cycle transition is the
// expected, spec-compliant way RT updates propagate (as long as the source toggles the A/B bit
// correctly). With rtBufferMode set, a clearing replace is staged into rtPending instead and
// swapped in later by rds_scheduler.cpp's buildGroup2A() - see rds_state.h's rtBufferMode comment.
//
// medByte layout, MSB(bit7) to LSB(bit0): bit7 unused; bits6-5 buffer config (00 = clear the whole
// RT_BUFFER then insert this as the sole message; anything else = append to the end, silently
// dropped if RDS_RT_BUFFER_SIZE would overflow - the spec defines further insert modes we don't
// yet distinguish, so they fall into the same "append" bucket rather than being rejected); bits4-1
// repeat count (0-15, maps directly to RtMessage::repeatCount); bit0 toggleAB.
// Returns false if the message was dropped (buffer full) rather than applied, so the caller can
// log accurately instead of claiming "applied" for a silent drop.
static bool uecpApplyRt(uint8_t medByte, const uint8_t* text, uint16_t textLen, RdsTxState& state) {
  if (textLen > RDS_RT_MAX_LEN) textLen = RDS_RT_MAX_LEN; // defensive cap; never trust length data blindly

  bool    clearBuffer = ((medByte >> 5) & 0x03) == 0x00;
  uint8_t repeatCount = (medByte >> 1) & 0x0F;
  bool    toggleAB    = (medByte & 0x01) != 0;

  // Which buffer this element actually lands in. A clearing replace starts a fresh buffer - live
  // if rtBufferMode is off, staged into rtPending if it's on (see rds_state.h's rtBufferMode
  // comment). An append continues whichever buffer is currently "the future": if a buffered clear
  // is already staged and waiting (rtPendingValid), an append has to land in rtPending too, or it
  // would be silently lost the moment that staged clear finally swaps in and overwrites the live
  // buffer wholesale - otherwise (rtBufferMode off, or on but nothing staged yet) it applies
  // straight to the live buffer as always, since an append never disturbs what's currently playing
  // regardless of the mode.
  bool       buffered = state.rtBufferMode && (clearBuffer || state.rtPendingValid);
  RtMessage* buf   = buffered ? state.rtPending       : state.rt;
  uint8_t*   count = buffered ? &state.rtPendingCount : &state.rtCount;

  uint8_t slot;
  if (clearBuffer) {
    memset(buf, 0, sizeof(RtMessage) * RDS_RT_BUFFER_SIZE); // rt[]/rtPending[] are the same size
    slot = 0;
    if (buffered) {
      state.rtPendingValid = true;
    } else {
      state.rtCurrent     = 0;
      state.rtSegment     = 0;
      state.rtRepeatsDone = 0;

      // Every real UECP arrival honors its own toggleAB bit unconditionally, even if the text is
      // byte-for-byte identical to what's already there - "don't toggle on a repeat" refers only to
      // the scheduler's own internal repeat/loop cycling (replaying the same stored message's
      // segments, or looping it repeatCount times), which never calls this function at all since no
      // new UECP frame arrived. The one exception is the very first real message ever: the boot
      // placeholder it's replacing was never part of a genuine broadcast toggle lineage, so nothing
      // meaningful exists to flip relative to - rtABFlag is left exactly as it already is (0, from
      // the boot default) rather than being derived from this first message's bit at all. A
      // buffered clear (above) goes through this same "first real message" exemption too, just
      // later - see buildGroup2A()'s swap-in, which checks rtSeeded/toggleAB the same way once the
      // staged content actually takes effect.
      if (!state.rtSeeded) {
        state.rtSeeded = true;
      } else if (toggleAB) {
        state.rtABFlag = !state.rtABFlag;
      }
    }
  } else {
    if (*count >= RDS_RT_BUFFER_SIZE) return false; // buffer full - drop, per the MED spec's own convention
    slot = *count;
    // Appending doesn't "start" transmission of anything yet, whether it lands live or staged - the
    // scheduler's own rotation (buildGroup2A()'s rtCurrent/rtRepeatsDone cycling) will reach this
    // slot in due course once the messages ahead of it have each been shown their configured
    // repeatCount times, so the live A/B flag isn't touched here either way.
  }

  RtMessage& msg = buf[slot];
  memcpy(msg.text, text, textLen);
  msg.textLen     = (uint8_t)textLen;
  msg.repeatCount = repeatCount;
  msg.toggleAB    = toggleAB;
  *count = slot + 1;
  return true;
}

// Applies one already DSN/PSN-matched AF (MEC 0x13) element. medBytes is the 2-byte "insert
// offset" field; afData/afDataLen is everything after it (the frequency bytes plus a mandatory
// trailing 0x00 terminator), i.e. afDataLen = MEL-2. Only a full, from-the-start AF list is
// supported for now: MED must be 0x0000 ("insert from offset 0"), and the data must end in a
// single 0x00 terminator marking a complete list - anything else (a non-zero insert offset, a
// missing terminator, or an odd frequency-byte count, since they're transmitted two at a time -
// see sendRDS()'s Group 0A branch) is rejected outright rather than guessed at. Applied
// immediately, like RT - no staging, since (unlike PS) there's no existing receiver convention
// for signalling "AF list is mid-update" that a torn transmission would violate.
// Returns false (state.af/afLen untouched) if rejected, so the caller can log why.
static bool uecpApplyAf(const uint8_t* medBytes, const uint8_t* afData, uint16_t afDataLen, RdsTxState& state) {
  if (medBytes[0] != 0x00 || medBytes[1] != 0x00) return false; // only "from the start" supported
  if (afDataLen == 0 || afData[afDataLen - 1] != 0x00) return false; // must end in the full-list terminator

  uint16_t freqLen = afDataLen - 1;
  if ((freqLen & 0x01) != 0) return false;   // sent two bytes at a time; an odd count can't cycle evenly
  if (freqLen > RDS_AF_MAX_LEN) return false; // defensive cap - state.af is fixed-size

  memcpy(state.af, afData, freqLen);
  state.afLen     = (uint8_t)freqLen;
  state.afSegment = 0;
  return true;
}

// Applies one already DSN/PSN-matched Long PS (MEC 0x21) element - dataLen bytes of raw UTF-8 text
// (no EBU conversion; unlike PS/RT/PTYN, long PS is transmitted as-is - see rds_state.h's longPs
// comment). Always staged into longPsPending/longPsPendingLen/longPsPendingValid and swapped into
// longPs/longPsLen later by rds_scheduler.cpp's buildGroup15A() once its segment counter wraps back
// to 0 - same clean-swap convention as PS's own psPending/psPendingValid, applied unconditionally
// (unlike RT's optional rtBufferMode): long PS has no A/B flag of its own to signal "content
// changed" to a receiver, so unlike RT there's no spec-sanctioned "torn transition is fine" case to
// default to - staging is the only sane behaviour here, not an opt-in.
static void uecpApplyLongPs(const uint8_t* data, uint8_t dataLen, RdsTxState& state) {
  if (dataLen > RDS_LONG_PS_MAX_LEN) dataLen = RDS_LONG_PS_MAX_LEN; // defensive cap; never trust length data blindly
  memcpy(state.longPsPending, data, dataLen);
  state.longPsPendingLen   = dataLen;
  state.longPsPendingValid = true;
}

// Applies MEC 0x0D (Time). Unlike every other MEC handled here, this one carries no DSN/PSN field
// at all - it's a single global "set the clock" message, not addressed to a specific
// dataset/service, so processUecpFrame()'s walk skips DSN/PSN matching for it entirely.
// data is the 8-byte MED (no MEL - this element is always exactly MEC+8 bytes, never more or
// less): year (last two digits as a plain binary value, e.g. 26 = 2026), month (1-12), day
// (1-31), hour (0-23), minute (0-59), second (0-59), centisecond (0-99), then a local time offset
// byte: bit5 = sign (0=+, 1=-), bits4-0 = magnitude in half-hours, or 0xFF meaning "leave the
// current offset unchanged". Date/time is UTC, matching the RDS Group 4A fields it ultimately
// feeds (see sendRDS()'s Group 4A branch) - the offset only fills in that group's otherwise-
// unknowable local-time-offset bits, it never affects the UTC value itself.
// Deliberately does NOT touch the system clock here - see rds_state.h's ctPending comment for why.
static void uecpApplyTime(const uint8_t* data, RdsTxState& state) {
  state.ctYear        = (uint16_t)(2000 + data[0]);
  state.ctMonth       = data[1];
  state.ctDay         = data[2];
  state.ctHour        = data[3];
  state.ctMinute      = data[4];
  state.ctSecond      = data[5];
  state.ctCentisecond = data[6];

  uint8_t offsetByte = data[7];
  if (offsetByte != 0xFF) state.ctOffset = offsetByte & 0x3F; // 0xFF = leave the current offset alone

  state.ctFromUecp = true;
  state.ctPending  = true;
}

// Applies MEC 0x1A (SLC - Slow Labelling Codes). Like MEC 0x0D, this one is DSN-only - no PSN
// field exists for it at all. data is the 2-byte MED: bits6-4 of the first byte give the SLC
// application's variant index (0-7), which doubled is the byte offset to store both bytes at in
// state.slc[] - stored verbatim (including those same variant bits), since sendRDS()'s Group 1A
// branch reads slc[idx]/slc[idx+1] straight into the transmitted block 3 unmodified.
static void uecpApplySlc(const uint8_t* data, RdsTxState& state) {
  uint8_t index = (uint8_t)(((data[0] & 0x70) >> 4) * 2);
  state.slc[index]     = data[0];
  state.slc[index + 1] = data[1];
}

// Applies MEC 0x29 (Group variant code sequence) for group 0x02 (RDS Group 1A) - group 0x1C
// (Group 14A) is a real, spec-defined option too, but this project doesn't implement Group 14A
// yet, so it's silently ignored rather than guessed at; any other group value is likewise
// ignored. data/dataLen is the sequence of variant indices (0-7, matching uecpApplySlc()'s slc[]
// slots) sendRDS() should cycle through for Group 1A - copied into state.slcSeq[], capped at its
// fixed capacity ("drop if the sequence is too long", per spec). slcCurrent resets to 0 so the
// new sequence always starts from its own beginning rather than wherever the old one left off.
static void uecpApplySlcSeq(const uint8_t* data, uint16_t dataLen, RdsTxState& state) {
  uint16_t copyLen = dataLen;
  if (copyLen > sizeof(state.slcSeq)) copyLen = sizeof(state.slcSeq);
  memcpy(state.slcSeq, data, copyLen);
  state.slcSeqLen  = (uint8_t)copyLen;
  state.slcCurrent = 0;
}

// Applies MEC 0x16 (Group sequence) - replaces this project's own RDS group scheduling order
// wholesale (rds_scheduler.cpp's buildNextGroup() dispatch - see rds_state.h's rdsSequence[] comment).
// data/dataLen is the new sequence itself, one ffGroupIndex()-packed byte per entry, copied into
// state.rdsSequence[] capped at its fixed capacity ("drop if the sequence is too long", per spec -
// MEC 0x16's own LEN field is already capped at RDS_SEQUENCE_MAX_LEN, so this never actually trims
// in practice). rdsSeqPos resets to 0 so the new sequence is read from its own beginning rather
// than wherever the old one left off - same reasoning as uecpApplySlcSeq()'s slcCurrent reset above.
static void uecpApplySequence(const uint8_t* data, uint16_t dataLen, RdsTxState& state) {
  uint16_t copyLen = dataLen;
  if (copyLen > RDS_SEQUENCE_MAX_LEN) copyLen = RDS_SEQUENCE_MAX_LEN;
  memcpy(state.rdsSequence, data, copyLen);
  state.rdsSequenceLen = (uint8_t)copyLen;
  state.rdsSeqPos      = 0;
}

// Applies one Group 3A ODA AID/group definition - shared by MEC 0x24 (when targeting Group 3A),
// MEC 0x40, and MEC 0x46's MEL==5 case, since all three ultimately mean the same thing: "AID
// aid's data is on targetGroupIndex, announce that on Group 3A with these message bytes in
// block3". The AID->group mapping is tracked in odaLiveDir regardless of what happens to the
// on-air queue entry below (even a duplicate, or one dropped for being over Group 3A's own slot
// cap, still refreshes the mapping) - a later MEC 0x46 for this AID must be able to find its
// group even if this exact definition never gets (re-)queued for on-air resend.
//
// The on-air queue is deduplicated on a full (targetGroupIndex, messageBytes, aid) match, but
// only when cyclic - a one-shot entry always gets (re-)queued regardless of any existing match.
// Cyclic entries never free themselves (they rotate forever until an explicit clear), so a
// periodically-resent unchanged definition would otherwise pile up indefinitely; a one-shot entry
// is naturally self-limiting (consumed and freed after its own single transmission), so a
// same-content resend arriving before the first is even sent isn't worth rejecting.
//
// flags/immediate carry MEC 0x46's priority bits through (0/false from MEC 0x24 and MEC 0x40,
// which have no priority concept of their own) - "immediate" additionally breaks Group 3A into
// the schedule on the very next group, like Group 4A does; see buildNextGroup()'s own
// pendingImmediateGroupIndex check. mecLabel is only for the log lines. state is only consulted
// for rdsSequenceHasGroup() - see that function's own comment for why, and why immediate skips it.
// Returns true if a new entry was actually queued (not deduplicated away or dropped).
static bool applyGroup3ADefinition(FreeFormatPool& ffPool, OdaLiveDirectory& odaLiveDir,
                                   uint8_t targetGroupIndex, uint16_t aid, uint16_t messageBytes,
                                   bool cyclic, uint8_t flags, bool immediate,
                                   uint8_t& pendingImmediateGroupIndex, const RdsTxState& state,
                                   const String& clientIp, const char* mecLabel) {
  uint8_t group3AIndex = ffGroupIndex(3, false);
  odaLiveDirectorySet(odaLiveDir, aid, targetGroupIndex);

  if (cyclic && ffContainsExact(ffPool, group3AIndex, targetGroupIndex, messageBytes, aid)) {
    Serial.printf("UECP %s (Group 3A, AID 0x%04X -> Group %u%c) from %s: duplicate of an "
                  "already-queued definition, not re-added (AID mapping still refreshed)\n",
                  mecLabel, aid, ffGroupType(targetGroupIndex),
                  ffGroupIsVersionB(targetGroupIndex) ? 'B' : 'A', clientIp.c_str());
    return false;
  }
  if (!immediate && !rdsSequenceHasGroup(state, group3AIndex)) {
    Serial.printf("UECP %s (Group 3A, AID 0x%04X) from %s dropped: Group 3A isn't in the current "
                  "group sequence, so it would never be sent (AID mapping still refreshed)\n",
                  mecLabel, aid, clientIp.c_str());
    return false;
  }
  FfEnqueueResult result = ffEnqueue(ffPool, group3AIndex, targetGroupIndex, messageBytes, aid,
                                     cyclic, flags);
  if (result != FF_ENQUEUE_FAILED) {
    if (result == FF_ENQUEUE_OK_CLEARED) {
      Serial.printf("UECP %s (Group 3A): buffer was full - cleared it to make room for this "
                    "definition instead of dropping it; consider giving Group 3A more airtime in "
                    "the group sequence if this keeps happening\n", mecLabel);
    }
    Serial.printf("UECP applied %s (Group 3A, AID 0x%04X -> Group %u%c, %s) from %s\n",
                  mecLabel, aid, ffGroupType(targetGroupIndex),
                  ffGroupIsVersionB(targetGroupIndex) ? 'B' : 'A',
                  cyclic ? "cyclic" : "one-shot", clientIp.c_str());
    if (immediate) pendingImmediateGroupIndex = group3AIndex;
    return true;
  }
  Serial.printf("UECP %s (Group 3A, AID 0x%04X) from %s dropped (free-format pool full; AID "
                "mapping still refreshed)\n", mecLabel, aid, clientIp.c_str());
  return false;
}

// Applies MEC 0x46's direct ODA data (MEL==8 for A groups, MEL==6 for B groups): the AID's data
// itself, targeting whichever group oda_directory says that AID is mapped to - not Group 3A.
// Drops silently if there's no known mapping for aid, or if the mapped group's A/B-ness doesn't
// match wantVersionB (this MEC 0x46 sub-format already commits to one or the other - MEL==8 is
// only ever for A groups, MEL==6 only ever for B). Unlike applyGroup3ADefinition(), this never
// deduplicates against the mapped group's queue - only Group 3A's own periodic-(re)announcement
// pattern motivated that restriction; nothing asked for the same treatment here. bufferConfig
// 0b11 (clear) is handled by the caller, not here - by the time this is called it's always 0b00
// or 0b10. state is only consulted for rdsSequenceHasGroup() - see that function's own comment for
// why, and why immediate skips it.
static void applyOdaDirectGroupData(FreeFormatPool& ffPool, const OdaLiveDirectory& odaLiveDir,
                                    uint16_t aid, uint8_t block2Last5, uint16_t block3,
                                    uint16_t block4, uint8_t bufferConfig, uint8_t flags,
                                    bool immediate, bool wantVersionB,
                                    uint8_t& pendingImmediateGroupIndex, const RdsTxState& state,
                                    const String& clientIp) {
  uint8_t mappedGroupIndex;
  if (!odaLiveDirectoryGet(odaLiveDir, aid, &mappedGroupIndex)) {
    Serial.printf("UECP MEC 0x46 (AID 0x%04X) from %s: no known group mapping, dropped\n",
                  aid, clientIp.c_str());
    return;
  }
  if (ffGroupIsVersionB(mappedGroupIndex) != wantVersionB) {
    Serial.printf("UECP MEC 0x46 (AID 0x%04X) from %s: mapped to Group %u%c, not a %s group as "
                  "this message expects, dropped\n", aid, clientIp.c_str(),
                  ffGroupType(mappedGroupIndex), ffGroupIsVersionB(mappedGroupIndex) ? 'B' : 'A',
                  wantVersionB ? "B" : "A");
    return;
  }
  if (!immediate && !rdsSequenceHasGroup(state, mappedGroupIndex)) {
    Serial.printf("UECP MEC 0x46 (AID 0x%04X -> Group %u%c) from %s dropped: that group isn't in "
                  "the current group sequence, so it would never be sent\n",
                  aid, ffGroupType(mappedGroupIndex), wantVersionB ? 'B' : 'A', clientIp.c_str());
    return;
  }

  bool cyclic = (bufferConfig == 0b10);
  FfEnqueueResult result = ffEnqueue(ffPool, mappedGroupIndex, block2Last5, block3, block4, cyclic,
                                     flags);
  if (result != FF_ENQUEUE_FAILED) {
    if (result == FF_ENQUEUE_OK_CLEARED) {
      Serial.printf("UECP MEC 0x46 (AID 0x%04X -> Group %u%c): buffer was full - cleared it to "
                    "make room for this message instead of dropping it; consider giving Group "
                    "%u%c more airtime in the group sequence if this keeps happening\n",
                    aid, ffGroupType(mappedGroupIndex), wantVersionB ? 'B' : 'A',
                    ffGroupType(mappedGroupIndex), wantVersionB ? 'B' : 'A');
    }
    Serial.printf("UECP applied MEC 0x46 (AID 0x%04X -> Group %u%c, %s) from %s\n",
                  aid, ffGroupType(mappedGroupIndex), wantVersionB ? 'B' : 'A',
                  cyclic ? "cyclic" : "one-shot", clientIp.c_str());
    if (immediate) pendingImmediateGroupIndex = mappedGroupIndex;
  } else {
    Serial.printf("UECP MEC 0x46 (AID 0x%04X -> Group %u%c) from %s dropped (free-format pool "
                  "full)\n", aid, ffGroupType(mappedGroupIndex), wantVersionB ? 'B' : 'A',
                  clientIp.c_str());
  }
}

// ---------------------------------------------------------------------------
// Public entry point
//
// Handles the fixed-layout MEC subset (PI/TA-TP/DI-PTYI/MS/PIN/PTY/PTYN) via a real per-element
// DSN/PSN-matched walk, plus RT (MEC 0x0A), AF (MEC 0x13), and Long PS (MEC 0x21) as special
// MEL-bearing cases, and Time (MEC 0x0D) / group sequence (MEC 0x16, DSN-only) / SLC (MEC 0x1A, DSN-only) / group
// variant sequence (MEC 0x29, DSN-only) /
// Free-Format Group (MEC 0x24) / MEC 0x40 (ODA AID/group definition, an alternate encoding of a
// Group 3A definition) / TMC (MEC 0x30) / MEC 0x46 (ODA data) as special no-DSN/PSN-at-all cases
// (0x24/0x40 fixed-length, 0x30/0x46 MEL-bearing like RT/AF); any other MEC encountered stops the
// walk (see uecpMecDataLen()'s comment). MEC 0x24 targeting Group 3A, MEC 0x40, and MEC 0x46's
// MEL==5 case all funnel into applyGroup3ADefinition(); MEC 0x46's MEL==8/6 cases funnel into
// applyOdaDirectGroupData() - see both functions' own comments and oda_directory.h. A frame that
// fails CRC gets a full hex dump so the failure is inspectable.
// ---------------------------------------------------------------------------

void processUecpFrame(const uint8_t* rawInner, uint16_t rawLen,
                      const String& clientIp, RdsTxState& state, FreeFormatPool& ffPool,
                      OdaLiveDirectory& odaLiveDir, uint8_t& pendingImmediateGroupIndex,
                      uint16_t ourSiteAddress, uint8_t ourEncoderAddress) {
  // 1. De-stuff
  uint8_t  destuffed[UECP_MAX_FRAME];
  uint16_t dLen = uecpDestuff(rawInner, rawLen, destuffed);

  // 2. CRC check — need at least 1 payload byte + 2 CRC bytes
  if (dLen < 3) {
    Serial.printf("UECP frame too short from %s\n", clientIp.c_str());
    return;
  }
  uint16_t computed = uecpCrc16(destuffed, dLen - 2);
  uint16_t stored   = ((uint16_t)destuffed[dLen - 2] << 8) | destuffed[dLen - 1];
  if (computed != stored) {
    Serial.printf("UECP CRC error from %s (got %04X, expected %04X), %u bytes:",
                  clientIp.c_str(), computed, stored, dLen);
    for (uint16_t i = 0; i < dLen; i++) Serial.printf(" %02X", destuffed[i]);
    Serial.printf("\n");
    return;
  }

  // 3. Address filter. ADD is the first 2 destuffed bytes (MSB first): site address (10 bits,
  // most significant) + encoder address (6 bits, least significant). Two independent wildcards,
  // per the UECP spec plus this project's own default policy: a frame addressed 0x0000 ("all
  // sites/all encoders") always matches, AND our own address defaulting to 0x0000 means we match
  // every frame regardless of its address. Otherwise the two must match exactly.
  uint16_t addr    = ((uint16_t)destuffed[0] << 8) | destuffed[1];
  uint16_t ourAddr = (uint16_t)(((ourSiteAddress & 0x03FF) << 6) | (ourEncoderAddress & 0x3F));
  if (addr != 0 && ourAddr != 0 && addr != ourAddr) {
    Serial.printf("UECP frame from %s ignored (address 0x%04X, ours is 0x%04X)\n",
                  clientIp.c_str(), addr, ourAddr);
    return;
  }

  // 4. Header is address[2] SQC[1] MFL[1]; need all 4 to walk elements.
  uint16_t msgLen = dLen - 2;  // strip the two CRC bytes
  if (msgLen < 4) {
    Serial.printf("UECP frame header too short from %s\n", clientIp.c_str());
    return;
  }
  uint8_t  sqc = destuffed[2];
  uint16_t mfl = destuffed[3];
  uint16_t msgEnd = 4 + mfl;
  if (msgEnd > msgLen) msgEnd = msgLen; // don't trust a lying MFL past what we actually received

  Serial.printf("UECP packet from %s: SQC=%u, %u message byte(s)\n", clientIp.c_str(), sqc,
                (unsigned)(msgEnd - 4));

  // 5. Walk message elements, applying any of the fixed-layout MECs whose DSN/PSN match ours.
  // DSN matching (uecpDsnMatches() above) handles the "no DSN used"/0 and "all datasets"/255
  // wildcards; PSN matching stays deliberately simple for now (either side being 0 means "anything
  // matches") - the UECP spec's own fuller PSN semantics (specific set / all-except-current / all)
  // are a later refinement, same as DSN's used to be before 255 turned up on a real source and
  // needed handling.
  uint16_t pos = 4;
  while (pos < msgEnd) {
    uint8_t mec = destuffed[pos];

    if (mec == 0x0A) {
      // RT - variable length via its own MEL byte (MEC[1] DSN[1] PSN[1] MEL[1] MED[1] text[N]).
      // Real-world quirk: some encoders set MEL to just the RT text length N instead of the
      // spec-correct 1(MED)+N, leaving exactly one text byte physically present beyond what MEL
      // declares. A dangling single byte can never be a legitimate next element (every element
      // needs at least 4 header bytes), so it's safe to unconditionally treat it as that missing
      // byte rather than a malformed frame - this is the one deliberate compromise here: a
      // genuine multi-message frame where a real second element is exactly 1 byte long would be
      // misread, but that shape can't occur (a header alone is 4 bytes), so nothing legitimate is
      // ever misinterpreted this way.
      if ((uint16_t)(pos + 4) > msgEnd) {
        Serial.printf("UECP RT element truncated (no MEL byte) from %s\n", clientIp.c_str());
        break;
      }
      uint8_t  dsn   = destuffed[pos + 1];
      uint8_t  psn   = destuffed[pos + 2];
      uint8_t  mel   = destuffed[pos + 3];
      uint16_t base  = pos + 4;
      if (mel == 0 || (uint16_t)(base + mel) > msgEnd) {
        Serial.printf("UECP RT element (MEL=%u) truncated from %s\n", mel, clientIp.c_str());
        break;
      }
      uint16_t declaredEnd = base + mel;
      uint16_t contentEnd  = declaredEnd;
      if (msgEnd - declaredEnd == 1) contentEnd = declaredEnd + 1; // absorb the miscounted-MEL byte

      uint8_t        medByte = destuffed[base];
      const uint8_t* text    = &destuffed[base + 1];
      uint16_t       textLen = contentEnd - (base + 1);

      bool dsnMatches = uecpDsnMatches(dsn, state);
      bool psnMatches = (psn == 0 || state.psn == 0 || psn == state.psn);
      if (dsnMatches && psnMatches) {
        if (uecpApplyRt(medByte, text, textLen, state)) {
          Serial.printf("UECP applied MEC 0x0A RT (DSN=%u PSN=%u, %u char(s)) from %s\n",
                        dsn, psn, (unsigned)textLen, clientIp.c_str());
        } else {
          Serial.printf("UECP MEC 0x0A RT from %s dropped (RT buffer full)\n", clientIp.c_str());
        }
      } else {
        Serial.printf("UECP MEC 0x0A from %s ignored (DSN=%u PSN=%u, ours is %u/%u)\n",
                      clientIp.c_str(), dsn, psn, state.dsn, state.psn);
      }

      pos = contentEnd;
      continue;
    }

    if (mec == 0x13) {
      // AF - variable length via its own MEL byte (MEC[1] DSN[1] PSN[1] MEL[1] MED[2] AFdata[MEL-2]).
      // MEL must cover at least the 2 MED bytes, or the subtraction below would underflow.
      if ((uint16_t)(pos + 4) > msgEnd) {
        Serial.printf("UECP AF element truncated (no MEL byte) from %s\n", clientIp.c_str());
        break;
      }
      uint8_t  dsn  = destuffed[pos + 1];
      uint8_t  psn  = destuffed[pos + 2];
      uint8_t  mel  = destuffed[pos + 3];
      uint16_t base = pos + 4;
      if (mel < 2 || (uint16_t)(base + mel) > msgEnd) {
        Serial.printf("UECP AF element (MEL=%u) truncated from %s\n", mel, clientIp.c_str());
        break;
      }

      const uint8_t* medBytes  = &destuffed[base];
      const uint8_t* afData    = &destuffed[base + 2];
      uint16_t       afDataLen = mel - 2;

      bool dsnMatches = uecpDsnMatches(dsn, state);
      bool psnMatches = (psn == 0 || state.psn == 0 || psn == state.psn);
      if (dsnMatches && psnMatches) {
        if (uecpApplyAf(medBytes, afData, afDataLen, state)) {
          Serial.printf("UECP applied MEC 0x13 AF (DSN=%u PSN=%u, %u freq byte(s)) from %s\n",
                        dsn, psn, (unsigned)state.afLen, clientIp.c_str());
        } else {
          Serial.printf("UECP MEC 0x13 AF from %s rejected (only a full, from-start, even-length "
                        "list is supported)\n", clientIp.c_str());
        }
      } else {
        Serial.printf("UECP MEC 0x13 from %s ignored (DSN=%u PSN=%u, ours is %u/%u)\n",
                      clientIp.c_str(), dsn, psn, state.dsn, state.psn);
      }

      pos = base + mel;
      continue;
    }

    if (mec == 0x21) {
      // Long PS - variable length via its own MEL byte (MEC[1] DSN[1] PSN[1] MEL[1] text[MEL]).
      // Unlike RT/AF there's no extra config/offset byte ahead of the data: MEL is exactly the
      // long PS text length (up to RDS_LONG_PS_MAX_LEN, defensively capped again inside
      // uecpApplyLongPs()).
      if ((uint16_t)(pos + 4) > msgEnd) {
        Serial.printf("UECP Long PS element truncated (no MEL byte) from %s\n", clientIp.c_str());
        break;
      }
      uint8_t  dsn  = destuffed[pos + 1];
      uint8_t  psn  = destuffed[pos + 2];
      uint8_t  mel  = destuffed[pos + 3];
      uint16_t base = pos + 4;
      if (mel == 0 || (uint16_t)(base + mel) > msgEnd) {
        Serial.printf("UECP Long PS element (MEL=%u) truncated from %s\n", mel, clientIp.c_str());
        break;
      }
      const uint8_t* text = &destuffed[base];

      bool dsnMatches = uecpDsnMatches(dsn, state);
      bool psnMatches = (psn == 0 || state.psn == 0 || psn == state.psn);
      if (dsnMatches && psnMatches) {
        uecpApplyLongPs(text, mel, state);
        Serial.printf("UECP applied MEC 0x21 Long PS (DSN=%u PSN=%u, %u char(s), staged) from %s\n",
                      dsn, psn, (unsigned)mel, clientIp.c_str());
      } else {
        Serial.printf("UECP MEC 0x21 from %s ignored (DSN=%u PSN=%u, ours is %u/%u)\n",
                      clientIp.c_str(), dsn, psn, state.dsn, state.psn);
      }

      pos = base + mel;
      continue;
    }

    if (mec == 0x0D) {
      // Time - no DSN/PSN, no MEL: always exactly MEC[1]+MED[8] with nothing else, per its own
      // "global, unaddressed" nature (see uecpApplyTime()'s comment).
      if ((uint16_t)(pos + 9) > msgEnd) {
        Serial.printf("UECP Time element truncated from %s\n", clientIp.c_str());
        break;
      }
      uecpApplyTime(&destuffed[pos + 1], state);
      Serial.printf("UECP applied MEC 0x0D Time (%04u-%02u-%02u %02u:%02u:%02u UTC, offset byte 0x%02X) from %s\n",
                    state.ctYear, state.ctMonth, state.ctDay, state.ctHour, state.ctMinute, state.ctSecond,
                    destuffed[pos + 8], clientIp.c_str());
      pos += 9;
      continue;
    }

    if (mec == 0x16) {
      // Group sequence - DSN only (no PSN), no MEL (it uses its own LEN field instead, capped at
      // 0xFC by spec rather than MEL's usual encoding): MEC[1]+DSN[1]+LEN[1]+Data[LEN].
      if ((uint16_t)(pos + 3) > msgEnd) {
        Serial.printf("UECP group-sequence element truncated (no LEN byte) from %s\n", clientIp.c_str());
        break;
      }
      uint8_t dsn = destuffed[pos + 1];
      uint8_t len = destuffed[pos + 2];
      if (len == 0 || (uint16_t)(pos + 3 + len) > msgEnd) {
        Serial.printf("UECP group-sequence element (LEN=%u) truncated from %s\n", len, clientIp.c_str());
        break;
      }
      const uint8_t* data = &destuffed[pos + 3];

      bool dsnMatches = uecpDsnMatches(dsn, state);
      if (dsnMatches) {
        uecpApplySequence(data, len, state);
        Serial.printf("UECP applied MEC 0x16 group sequence (DSN=%u, %u entries) from %s\n",
                      dsn, (unsigned)state.rdsSequenceLen, clientIp.c_str());
      } else {
        Serial.printf("UECP MEC 0x16 from %s ignored (DSN=%u, ours is %u)\n", clientIp.c_str(), dsn, state.dsn);
      }
      pos += 3 + len;
      continue;
    }

    if (mec == 0x1A) {
      // SLC - DSN only (no PSN), no MEL: always exactly MEC[1]+DSN[1]+MED[2].
      if ((uint16_t)(pos + 4) > msgEnd) {
        Serial.printf("UECP SLC element truncated from %s\n", clientIp.c_str());
        break;
      }
      uint8_t        dsn  = destuffed[pos + 1];
      const uint8_t* data = &destuffed[pos + 2];
      bool dsnMatches = uecpDsnMatches(dsn, state);
      if (dsnMatches) {
        uecpApplySlc(data, state);
        Serial.printf("UECP applied MEC 0x1A SLC (DSN=%u, variant %u) from %s\n",
                      dsn, (unsigned)((data[0] & 0x70) >> 4), clientIp.c_str());
      } else {
        Serial.printf("UECP MEC 0x1A from %s ignored (DSN=%u, ours is %u)\n", clientIp.c_str(), dsn, state.dsn);
      }
      pos += 4;
      continue;
    }

    if (mec == 0x29) {
      // Group variant code sequence - DSN only (no PSN). MEC[1]+DSN[1]+MEL[1]+Group[1]+Data[MEL-1].
      if ((uint16_t)(pos + 4) > msgEnd) {
        Serial.printf("UECP group-variant-sequence element truncated (no MEL/Group byte) from %s\n", clientIp.c_str());
        break;
      }
      uint8_t dsn   = destuffed[pos + 1];
      uint8_t mel   = destuffed[pos + 2];
      uint8_t group = destuffed[pos + 3];
      if (mel == 0 || (uint16_t)(pos + 3 + mel) > msgEnd) {
        Serial.printf("UECP group-variant-sequence element (MEL=%u) truncated from %s\n", mel, clientIp.c_str());
        break;
      }
      const uint8_t* data    = &destuffed[pos + 4];
      uint16_t       dataLen = (uint16_t)(mel - 1);

      bool dsnMatches = uecpDsnMatches(dsn, state);
      if (dsnMatches) {
        if (group == 0x02) { // RDS Group 1A; Group 14A (0x1C) isn't implemented yet, so it's ignored below
          uecpApplySlcSeq(data, dataLen, state);
          Serial.printf("UECP applied MEC 0x29 group-variant-sequence (DSN=%u, %u entries) from %s\n",
                        dsn, (unsigned)state.slcSeqLen, clientIp.c_str());
        } else {
          Serial.printf("UECP MEC 0x29 (DSN=%u, group=0x%02X) from %s ignored (only Group 1A/0x02 is supported)\n",
                        dsn, group, clientIp.c_str());
        }
      } else {
        Serial.printf("UECP MEC 0x29 from %s ignored (DSN=%u, ours is %u)\n", clientIp.c_str(), dsn, state.dsn);
      }
      pos = pos + 3 + mel;
      continue;
    }

    if (mec == 0x24) {
      // Free-Format Group - no DSN/PSN, no MEL: always exactly
      // MEC[1]+Group[1]+MED[1]+Data[4] = 7 bytes, per its own fixed-length layout (unlike RT/AF,
      // which declare their own length via a MEL byte).
      if ((uint16_t)(pos + 7) > msgEnd) {
        Serial.printf("UECP Free-Format Group element truncated from %s\n", clientIp.c_str());
        break;
      }
      uint8_t  groupByte = destuffed[pos + 1];
      uint8_t  medByte   = destuffed[pos + 2];
      uint16_t block3    = ((uint16_t)destuffed[pos + 3] << 8) | destuffed[pos + 4];
      uint16_t block4    = ((uint16_t)destuffed[pos + 5] << 8) | destuffed[pos + 6];

      uint8_t groupType  = groupByte >> 1;
      bool    versionB   = (groupByte & 0x01) != 0;
      uint8_t groupIndex = ffGroupIndex(groupType, versionB);
      uint8_t mode        = (medByte >> 5) & 0x03; // bits6-5; bit7 is unused/ignored per spec
      uint8_t block2Last5 = medByte & 0x1F;

      bool isGroup3A = (groupIndex == ffGroupIndex(3, false));

      if (mode == 0b11) {
        ffClearGroup(ffPool, groupIndex);
        if (isGroup3A) {
          // A Group 3A "wipe all" also resets whatever AID->group mappings live 3A traffic had
          // established - the only thing that ever clears odaLiveDir (see oda_directory.h).
          odaLiveDirectoryClear(odaLiveDir);
        }
        Serial.printf("UECP applied MEC 0x24 (Group %u%c) buffer cleared from %s\n",
                      groupType, versionB ? 'B' : 'A', clientIp.c_str());
      } else if (isGroup3A) {
        // Group 3A - ODA AID/group definitions. block2Last5 here isn't a generic 5-bit payload
        // like every other group's - it's itself an ffGroupIndex()-packed group (per the RDS
        // spec's own Group 3A layout: 4-bit group type + A/B bit), naming which group block4's
        // AID is being announced on. See applyGroup3ADefinition()'s own comment for the
        // AID-tracking/dedup rationale - shared with MEC 0x40, which reaches the same place.
        bool cyclic = (mode == 0b10);
        applyGroup3ADefinition(ffPool, odaLiveDir, block2Last5, block4, block3, cyclic,
                               /*flags=*/0, /*immediate=*/false, pendingImmediateGroupIndex, state,
                               clientIp, "MEC 0x24");
      } else if (!rdsSequenceHasGroup(state, groupIndex)) {
        // See rdsSequenceHasGroup()'s own comment - a group with no slot in the current sequence
        // can never be dequeued, so there's no point letting it fill up the pool.
        Serial.printf("UECP MEC 0x24 (Group %u%c) from %s dropped: that group isn't in the "
                      "current group sequence, so it would never be sent\n",
                      groupType, versionB ? 'B' : 'A', clientIp.c_str());
      } else {
        // mode 0b10 = cyclic; 0b00 = one-shot; the spec leaves 0b01 undefined - treated as
        // one-shot too rather than guessed at as anything fancier.
        bool cyclic = (mode == 0b10);
        FfEnqueueResult result = ffEnqueue(ffPool, groupIndex, block2Last5, block3, block4, cyclic);
        if (result != FF_ENQUEUE_FAILED) {
          if (result == FF_ENQUEUE_OK_CLEARED) {
            Serial.printf("UECP MEC 0x24 (Group %u%c): buffer was full - cleared it to make room "
                          "for this message instead of dropping it; consider giving Group %u%c "
                          "more airtime in the group sequence if this keeps happening\n",
                          groupType, versionB ? 'B' : 'A', groupType, versionB ? 'B' : 'A');
          }
          Serial.printf("UECP applied MEC 0x24 (Group %u%c, %s) from %s\n",
                        groupType, versionB ? 'B' : 'A', cyclic ? "cyclic" : "one-shot", clientIp.c_str());
        } else {
          Serial.printf("UECP MEC 0x24 (Group %u%c) from %s dropped (free-format pool full)\n",
                        groupType, versionB ? 'B' : 'A', clientIp.c_str());
        }
      }
      pos += 7;
      continue;
    }

    if (mec == 0x40) {
      // Another way to signal a Group 3A ODA AID/group definition (alongside MEC 0x24 sent
      // directly for Group 3A) - no DSN/PSN, no MEL: always exactly
      // MEC[1]+Group[1]+AID[2]+Config[1]+Msg[2]+Timeout[1] = 8 bytes.
      if ((uint16_t)(pos + 8) > msgEnd) {
        Serial.printf("UECP MEC 0x40 element truncated from %s\n", clientIp.c_str());
        break;
      }
      uint8_t  targetGroupIndex = destuffed[pos + 1] & 0x1F; // already ffGroupIndex()-packed
      uint16_t aid              = ((uint16_t)destuffed[pos + 2] << 8) | destuffed[pos + 3];
      uint8_t  configByte       = destuffed[pos + 4];
      uint16_t messageBytes     = ((uint16_t)destuffed[pos + 5] << 8) | destuffed[pos + 6];
      uint8_t  timeoutMinutes   = destuffed[pos + 7]; // 0 = none; not enforced yet, see below

      uint8_t bufferConfig = configByte & 0x03; // spec: "just final 2 bits used"

      if (bufferConfig == 0b11) {
        // Narrower than MEC 0x24's mode 0b11: removes only this AID's own queued Group 3A
        // entries, not the whole group's buffer, and (deliberately, per spec silence on this)
        // does NOT touch odaLiveDir - this clears the on-air announcement only, not the
        // AID->group mapping itself.
        ffClearMatchingBlock4(ffPool, ffGroupIndex(3, false), aid);
        Serial.printf("UECP applied MEC 0x40 (AID 0x%04X) queued Group 3A entries cleared from %s\n",
                      aid, clientIp.c_str());
      } else {
        // 0b10 = add to the buffer (cyclic, same rotating semantics as MEC 0x24's mode 0b10);
        // 0b00 = one-shot; the spec leaves 0b01 unused - treated as one-shot too, same "fall into
        // the nearest safe bucket" precedent as MEC 0x24/0x30's own undefined codes.
        bool cyclic = (bufferConfig == 0b10);
        applyGroup3ADefinition(ffPool, odaLiveDir, targetGroupIndex, aid, messageBytes, cyclic,
                               /*flags=*/0, /*immediate=*/false, pendingImmediateGroupIndex, state,
                               clientIp, "MEC 0x40");
      }

      if (timeoutMinutes != 0) {
        Serial.printf("UECP MEC 0x40 (AID 0x%04X) requested a %u-minute ODA data timeout - not "
                      "yet implemented, ignored\n", aid, timeoutMinutes);
      }
      pos += 8;
      continue;
    }

    if (mec == 0x46) {
      // ODA data - the application-specific payload for an AID that's already been announced on
      // Group 3A (via MEC 0x24/0x40, or resolved from oda_directory here). No DSN/PSN, no MEL-vs-
      // MED split like RT/AF: MEC[1] MEL[1] AID[2] Config[1] then (MEL-3) further bytes, whose
      // meaning depends on MEL: 5 = Group 3A short-message content for an already-known AID/group
      // mapping (2 final bytes, routed through applyGroup3ADefinition() same as MEC 0x24/0x40); 8
      // = direct data for the AID's mapped group, only valid if that mapping is an "A" group (5
      // final bytes: block2Last5+block3+block4); 6 = same, but only valid for a "B" mapping (3
      // final bytes: block2Last5+block4 - no block3 field at all, since buildGenericGroup()
      // already substitutes PI for B groups regardless of what's stored).
      if ((uint16_t)(pos + 2) > msgEnd) {
        Serial.printf("UECP MEC 0x46 element truncated (no MEL byte) from %s\n", clientIp.c_str());
        break;
      }
      uint8_t  mel  = destuffed[pos + 1];
      uint16_t base = pos + 2; // AID starts here
      if (mel < 3 || (uint16_t)(base + mel) > msgEnd) {
        Serial.printf("UECP MEC 0x46 element (MEL=%u) truncated from %s\n", mel, clientIp.c_str());
        break;
      }

      uint16_t      aid        = ((uint16_t)destuffed[base] << 8) | destuffed[base + 1];
      uint8_t       configByte = destuffed[base + 2];
      const uint8_t* finalBytes = &destuffed[base + 3];

      bool    shortMessageFlagSet = (configByte & 0x40) != 0; // bit6 - informational per spec;
                                                                 // mel below is authoritative
      uint8_t priority     = (configByte >> 4) & 0x03; // bits5-4: 0=normal,1=urgent,2=immediate,
                                                          // 3=unused (bits3-2 = mode select,
                                                          // ignored for now per spec)
      uint8_t bufferConfig = configByte & 0x03;         // bits1-0

      bool    immediate = (priority == 0b10);
      uint8_t flags     = (priority != 0) ? FF_FLAG_PRIORITY : 0;

      // Priority (urgent/immediate) is only meaningful paired with one-shot, per spec - reject
      // the combination outright rather than guessing an interpretation for anything else.
      if (priority != 0 && bufferConfig != 0b00) {
        Serial.printf("UECP MEC 0x46 (AID 0x%04X) from %s rejected: priority %u requires a "
                      "one-shot buffer config (got %u)\n", aid, clientIp.c_str(), priority, bufferConfig);
        pos = base + mel;
        continue;
      }

      if (mel == 5) {
        if (!shortMessageFlagSet) {
          Serial.printf("UECP MEC 0x46 (AID 0x%04X) from %s: MEL=5 but short-message flag not "
                        "set, proceeding anyway (MEL is authoritative)\n", aid, clientIp.c_str());
        }
        uint8_t group3AIndex = ffGroupIndex(3, false);
        if (bufferConfig == 0b11) {
          // Narrower than MEC 0x24/0x40's own Group 3A clear: only this AID's queued entries.
          ffClearMatchingBlock4(ffPool, group3AIndex, aid);
          Serial.printf("UECP applied MEC 0x46 (AID 0x%04X) queued Group 3A entries cleared from %s\n",
                        aid, clientIp.c_str());
        } else {
          uint8_t mappedGroupIndex;
          if (!odaLiveDirectoryGet(odaLiveDir, aid, &mappedGroupIndex)) {
            Serial.printf("UECP MEC 0x46 (AID 0x%04X) from %s: no known Group 3A mapping, dropped\n",
                          aid, clientIp.c_str());
          } else {
            uint16_t messageBytes = ((uint16_t)finalBytes[0] << 8) | finalBytes[1];
            bool cyclic = (bufferConfig == 0b10);
            applyGroup3ADefinition(ffPool, odaLiveDir, mappedGroupIndex, aid, messageBytes, cyclic,
                                   flags, immediate, pendingImmediateGroupIndex, state, clientIp,
                                   "MEC 0x46");
          }
        }
      } else if (mel == 8 || mel == 6) {
        bool wantVersionB = (mel == 6);
        if (shortMessageFlagSet) {
          Serial.printf("UECP MEC 0x46 (AID 0x%04X) from %s: MEL=%u but short-message flag set, "
                        "proceeding anyway (MEL is authoritative)\n", aid, clientIp.c_str(), mel);
        }
        uint8_t  block2Last5 = finalBytes[0] & 0x1F;
        uint16_t block3 = wantVersionB ? 0 : (((uint16_t)finalBytes[1] << 8) | finalBytes[2]);
        uint16_t block4 = wantVersionB ? (((uint16_t)finalBytes[1] << 8) | finalBytes[2])
                                        : (((uint16_t)finalBytes[3] << 8) | finalBytes[4]);

        if (bufferConfig == 0b11) {
          uint8_t mappedGroupIndex;
          if (!odaLiveDirectoryGet(odaLiveDir, aid, &mappedGroupIndex) ||
              ffGroupIsVersionB(mappedGroupIndex) != wantVersionB) {
            Serial.printf("UECP MEC 0x46 (AID 0x%04X) from %s: no matching %s-group mapping to "
                          "clear, dropped\n", aid, clientIp.c_str(), wantVersionB ? "B" : "A");
          } else {
            ffClearGroup(ffPool, mappedGroupIndex);
            Serial.printf("UECP applied MEC 0x46 (AID 0x%04X -> Group %u%c) buffer cleared from %s\n",
                          aid, ffGroupType(mappedGroupIndex), wantVersionB ? 'B' : 'A', clientIp.c_str());
          }
        } else {
          applyOdaDirectGroupData(ffPool, odaLiveDir, aid, block2Last5, block3, block4, bufferConfig,
                                  flags, immediate, wantVersionB, pendingImmediateGroupIndex, state,
                                  clientIp);
        }
      } else {
        Serial.printf("UECP MEC 0x46 (AID 0x%04X) from %s: unrecognised MEL=%u (expected 5, 6, or "
                      "8), ignored\n", aid, clientIp.c_str(), mel);
      }

      pos = base + mel;
      continue;
    }

    if (mec == 0x30) {
      // TMC (Traffic Message Channel) - legacy MEC, always targets RDS Group 8A (not carried in
      // the message; there's no Group byte here unlike MEC 0x24). No DSN/PSN either. Variable
      // length via its own MEL byte: MEC[1] MEL[1] Config[1] Unit[5]xN, where MEL counts
      // Config+Units (not MEC/MEL themselves) so N = (MEL-1)/5 - almost always 1 (MEL=6), but the
      // spec allows more; all units share the one Config byte. Each unit is Block2's last 5 bits
      // (1 byte) + Block3 (2 bytes) + Block4 (2 bytes).
      if ((uint16_t)(pos + 2) > msgEnd) {
        Serial.printf("UECP TMC element truncated (no MEL byte) from %s\n", clientIp.c_str());
        break;
      }
      uint8_t  mel  = destuffed[pos + 1];
      uint16_t base = pos + 2;
      if (mel < 1 || (uint16_t)(base + mel) > msgEnd) {
        Serial.printf("UECP TMC element (MEL=%u) truncated from %s\n", mel, clientIp.c_str());
        break;
      }

      uint8_t  configByte  = destuffed[base];
      uint16_t unitsLen    = mel - 1;
      uint16_t unitCount   = unitsLen / 5; // a non-multiple-of-5 remainder (malformed) is ignored,
                                             // not read as a partial unit

      // Config byte: bit7 urgency, bits6-5 buffer config, bits4-1 number of transmissions
      // (0-15), bit0 unused/ignored.
      bool    urgent           = (configByte & 0x80) != 0;
      uint8_t bufferConfig     = (configByte >> 5) & 0x03;
      uint8_t numTransmissions = (configByte >> 1) & 0x0F;

      uint8_t groupIndex = ffGroupIndex(8, false); // TMC is always Group 8A

      if (bufferConfig == 0b11) {
        ffClearGroup(ffPool, groupIndex);
        Serial.printf("UECP applied MEC 0x30 TMC (Group 8A) buffer cleared from %s\n", clientIp.c_str());
      } else if (!rdsSequenceHasGroup(state, groupIndex)) {
        // See rdsSequenceHasGroup()'s own comment - Group 8A has no struct fallback of its own
        // (unlike 0A/1A/2A/10A), so if it's not in the current sequence at all, nothing would ever
        // dequeue this element; TMC has no "immediate" bypass either, so this is unconditional.
        Serial.printf("UECP MEC 0x30 TMC from %s dropped: Group 8A isn't in the current group "
                      "sequence, so it would never be sent\n", clientIp.c_str());
      } else {
        bool cyclic = (bufferConfig == 0b10);
        // "Urgent" (transmitted next, via FF_FLAG_PRIORITY - see ffEnqueue()) is only honored for
        // a genuinely single-transmission message, per spec; anything else enqueues normally.
        uint8_t flags = (urgent && numTransmissions == 1) ? FF_FLAG_PRIORITY : 0;
        // The pool has no native repeat-count field - that's MEC 0x30-specific legacy behaviour -
        // so a one-shot message's "number of transmissions" is worked around by enqueuing that
        // many separate one-shot copies instead. Cyclic ignores the count entirely (it loops
        // forever regardless), same as MEC 0x24's cyclic mode.
        uint8_t copies = cyclic ? 1 : numTransmissions;

        uint16_t inserted = 0, wanted = (uint16_t)unitCount * copies;
        bool anyCleared = false;
        for (uint16_t u = 0; u < unitCount; u++) {
          const uint8_t* unit = &destuffed[base + 1 + u * 5];
          uint8_t  block2Last5 = unit[0] & 0x1F;
          uint16_t block3      = ((uint16_t)unit[1] << 8) | unit[2];
          uint16_t block4      = ((uint16_t)unit[3] << 8) | unit[4];
          for (uint8_t c = 0; c < copies; c++) {
            FfEnqueueResult result = ffEnqueue(ffPool, groupIndex, block2Last5, block3, block4,
                                               cyclic, flags);
            if (result != FF_ENQUEUE_FAILED) inserted++;
            if (result == FF_ENQUEUE_OK_CLEARED) anyCleared = true;
          }
        }
        if (anyCleared) {
          // Logged once per MEC 0x30 element even if the clear happened more than once while
          // queuing it (e.g. this message's own unit count exceeds FF_GROUP_MAX_SLOTS) - repeating
          // it per-unit would just be noise once the point ("Group 8A can't keep up") is made.
          Serial.printf("UECP MEC 0x30 TMC (Group 8A): buffer was full at least once while queuing "
                        "this message - cleared to keep TMC data current instead of dropping it; "
                        "consider giving Group 8A more airtime in the group sequence if this keeps "
                        "happening\n");
        }
        Serial.printf("UECP applied MEC 0x30 TMC (Group 8A, %u unit(s) x%u cop%s, %s%s) from %s\n",
                      (unsigned)unitCount, (unsigned)copies, copies == 1 ? "y" : "ies",
                      cyclic ? "cyclic" : "one-shot", (flags & FF_FLAG_PRIORITY) ? ", urgent" : "",
                      clientIp.c_str());
        if (inserted < wanted) {
          Serial.printf("UECP MEC 0x30 TMC from %s: %u of %u queue insert(s) dropped (free-format "
                        "pool full)\n", clientIp.c_str(), (unsigned)(wanted - inserted),
                        (unsigned)wanted);
        }
      }
      pos = base + mel;
      continue;
    }

    uint8_t dataLen = uecpMecDataLen(mec);
    if (dataLen == 0) {
      Serial.printf("UECP MEC 0x%02X from %s not yet handled, stopping element walk\n", mec, clientIp.c_str());
      break;
    }
    if ((uint16_t)(pos + 3 + dataLen) > msgEnd) {
      Serial.printf("UECP element (MEC 0x%02X) truncated from %s\n", mec, clientIp.c_str());
      break;
    }

    uint8_t        dsn  = destuffed[pos + 1];
    uint8_t        psn  = destuffed[pos + 2];
    const uint8_t* data = &destuffed[pos + 3];

    bool dsnMatches = uecpDsnMatches(dsn, state);
    bool psnMatches = (psn == 0 || state.psn == 0 || psn == state.psn);
    if (dsnMatches && psnMatches) {
      uecpApplyMec(mec, data, state);
      Serial.printf("UECP applied MEC 0x%02X (DSN=%u PSN=%u) from %s\n", mec, dsn, psn, clientIp.c_str());
    } else {
      Serial.printf("UECP MEC 0x%02X from %s ignored (DSN=%u PSN=%u, ours is %u/%u)\n",
                    mec, clientIp.c_str(), dsn, psn, state.dsn, state.psn);
    }

    pos += 3 + dataLen;
  }
}
