#include "rds_scheduler.h"
#include "rds_group_blocks.h"
#include <string.h>
#include <time.h>
#include <sys/time.h> // for settimeofday(), applying a UECP MEC 0x0D Time update

// Packs a generic RDS Block 2 value: 4-bit group type, 1-bit version (A=0/B=1), TP, 5-bit PTY,
// and the 5 group-specific bits. Bit positions match the IEC 62106 spec.
static uint16_t makeBlock2(uint8_t groupType, bool versionB, bool tp, uint8_t pty, uint8_t data5) {
  return ((uint16_t)(groupType & 0x0F) << 12)
       | ((uint16_t)(versionB ? 1 : 0) << 11)
       | ((uint16_t)(tp ? 1 : 0) << 10)
       | ((uint16_t)(pty & 0x1F) << 5)
       | (data5 & 0x1F);
}

// RdsGroupBlocks is queued here between being built (content/scheduling logic, buildNextGroup()
// below - pure computation, never blocks) and actually sent (timing-critical chip handshake,
// serviceRdsTx() below). Depth 2 is enough to fully decouple the two - more would only add latency
// to the Group 4A Clock Time group, whose whole point is to land as close as possible to a real
// UTC minute boundary.
#define RDS_GROUP_BUFFER_DEPTH 2
static RdsGroupBlocks rdsGroupBuffer[RDS_GROUP_BUFFER_DEPTH];
static uint8_t rdsBufHead  = 0; // next slot buildNextGroup() will fill
static uint8_t rdsBufTail  = 0; // next slot serviceRdsTx() will send
static uint8_t rdsBufCount = 0;

static inline bool rdsBufferFull()  { return rdsBufCount >= RDS_GROUP_BUFFER_DEPTH; }
static inline bool rdsBufferEmpty() { return rdsBufCount == 0; }

static void rdsBufferPush(const RdsGroupBlocks& g) {
  rdsGroupBuffer[rdsBufHead] = g;
  rdsBufHead = (rdsBufHead + 1) % RDS_GROUP_BUFFER_DEPTH;
  rdsBufCount++;
}

static RdsGroupBlocks rdsBufferPop() {
  RdsGroupBlocks g = rdsGroupBuffer[rdsBufTail];
  rdsBufTail = (rdsBufTail + 1) % RDS_GROUP_BUFFER_DEPTH;
  rdsBufCount--;
  return g;
}

// Feeder state (serviceRdsTx() below).
static bool     rdsSendPending        = false;
static uint32_t rdsPendingSinceMillis = 0;

// Epoch-minute index of the last Clock Time group actually built; -1 = never - latches so
// buildNextGroup()'s per-call :00 check can't fire twice for the same minute.
static time_t lastCtMinuteSent = -1;

// TA-toggle Group 15B burst: some receivers expect a full 8-group Group 15B burst (two 0-3 DI
// sweeps) immediately after any Group 0A that changes the TA flag while TP=1 - see buildGroup0A()
// (which detects the toggle and arms taBurstRemaining) and buildNextGroup() (which drains it,
// breaking into the normal sequence, ahead of everything but Clock Time/MEC 0x46 immediate).
// taLastSent/taLastSentValid latch the previously-*transmitted* TA bit so the comparison is against
// what actually went on air, not just the live state.tatp (which can change between sends without a
// struct-backed Group 0A ever having carried the old value) - taLastSentValid stays false until the
// first real Group 0A build, so boot never looks like a toggle from some arbitrary default.
static bool    taLastSent      = false;
static bool    taLastSentValid = false;
static uint8_t taBurstRemaining = 0;

void rdsSchedulerStart(QN8066& tx) {
  tx.rdsTxEnable(true);
  delay(200);
  // rdsSetSyncTime() only affects the library's blocking rdsSendGroup() - this scheduler itself
  // never calls it (see serviceRdsTx() below, which drives the non-blocking rdsSendGroupAsync()/
  // rdsIsGroupSent() pair instead, and doesn't consult rdsSyncTime at all) - but a sketch with its
  // own raw-RDS-group-testing feature that DOES call the blocking API directly (bypassing this
  // scheduler entirely while active) still needs it configured, so it's set here unconditionally
  // rather than left as a per-sketch afterthought; harmless for any sketch that never touches the
  // blocking path at all.
  tx.rdsSetSyncTime(60);
}

void rdsSchedulerReset() {
  rdsBufHead = rdsBufTail = rdsBufCount = 0;
  rdsSendPending = false;
  taBurstRemaining = 0; // don't resume a stale burst into whatever gets queued next
}

// Shared by every builder below: Block 1 is always the system PI, and TP/PTY come from the same
// place regardless of which group is being built.
static void fillCommonBlock1(RdsGroupBlocks& g, const RdsTxState& state) {
  g.b1.pi = ((uint16_t)state.pi[0] << 8) | state.pi[1];
}
static inline bool    currentTp(const RdsTxState& state)  { return (state.tatp & 0x02) != 0; }
static inline uint8_t currentPty(const RdsTxState& state) { return state.pty; }

// Group 0A - Programme Service name. This project's struct-backed fallback for group index 0.
static RdsGroupBlocks buildGroup0A(RdsTxState& state) {
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  bool ta = (state.tatp & 0x01) != 0;
  bool ms = (state.ms   & 0x01) != 0;
  // Arm the Group 15B TA-toggle burst: only once we have a previously-sent TA to compare against,
  // only on an actual change, and only with TP=1 (see taBurstRemaining's own comment above).
  if (taLastSentValid && ta != taLastSent && currentTp(state)) {
    taBurstRemaining = 8;
  }
  taLastSent = ta;
  taLastSentValid = true;
  uint8_t diBit = (state.diPtyi >> (3 - state.psSegment)) & 0x01;
  uint8_t data5 = ((ta ? 1 : 0) << 4) | ((ms ? 1 : 0) << 3) | (diBit << 2) | (state.psSegment & 0x03);
  g.b2.raw = makeBlock2(0, false, currentTp(state), currentPty(state), data5);
  if (state.afLen == 0) {
    g.b3.raw = 0xE0E0; // no AF list - filler ("no AF" code, repeated)
  } else {
    uint8_t numPairs = state.afLen / 2;
    g.b3.raw = ((uint16_t)state.af[state.afSegment * 2] << 8)
                        | state.af[state.afSegment * 2 + 1];
    state.afSegment = (state.afSegment + 1) % numPairs;
  }
  g.b4.raw = ((uint16_t)state.ps[state.psSegment * 2] << 8) | state.ps[state.psSegment * 2 + 1];
  state.psSegment = (state.psSegment + 1) & 0x03;
  if (state.psSegment == 0 && state.psPendingValid) {
    memcpy(state.ps, state.psPending, RDS_PS_LEN);
    state.psPendingValid = false;
  }
  return g;
}

// Group 15A - Long PS (MEC 0x21), an up-to-RDS_LONG_PS_MAX_LEN-byte UTF-8 companion to the regular
// PS above (no EBU conversion happens anywhere for it - see uecp_handler.cpp's uecpApplyLongPs()).
// Group index 30. Only ever reached if state.longPsLen > 0 or a long PS update is staged and
// waiting (state.longPsPendingValid) - see buildNextGroup()'s own dispatch check and
// rds_state.h's longPsLen comment; there's no boot-default content to fall back on the way
// ps[]/rt[] have, so with neither of those true, this group is skipped even when scheduled.
static RdsGroupBlocks buildGroup15A(RdsTxState& state) {
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);

  // Bootstrap: buildNextGroup()'s dispatch check (above) is what actually lets this function run
  // at all once something is staged, but ordinarily the swap itself only happens at the
  // end-of-cycle wrap point below - which never gets a chance to fire the very first time, since
  // longPsLen starts at 0 and this function was never being called at all before now. Apply the
  // swap immediately in that specific case, so the very first group actually sent already carries
  // the real content instead of one throwaway blank segment.
  if (state.longPsLen == 0 && state.longPsPendingValid) {
    memcpy(state.longPs, state.longPsPending, RDS_LONG_PS_MAX_LEN);
    state.longPsLen          = state.longPsPendingLen;
    state.longPsSegment      = 0;
    state.longPsPendingValid = false;
  }

  bool ta = (state.tatp & 0x01) != 0;

  // Same "does the text already end in a CR, or does one need appending" convention as RT's own
  // buildGroup2A() - a long PS shorter than the max is terminated with 0x0D so a receiver knows
  // where it ends, and only as many segments as that actually requires are sent, not always the
  // full 8.
  bool hasTrailingCr = state.longPsLen > 0 && state.longPs[state.longPsLen - 1] == 0x0D;
  uint8_t lpsBytes[4];
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t pos = state.longPsSegment * 4 + i;
    if (pos < state.longPsLen)                         lpsBytes[i] = state.longPs[pos];
    else if (pos == state.longPsLen && !hasTrailingCr)  lpsBytes[i] = 0x0D;
    else                                                 lpsBytes[i] = 0x20;
  }

  // data5: bit4 = TA (as Group 0A), bit3 reserved (0), bits2-0 = 3-bit segment counter (0-7,
  // matching RDS_LONG_PS_MAX_LEN/4 = 8 segments max).
  uint8_t data5 = ((ta ? 1 : 0) << 4) | (state.longPsSegment & 0x07);
  g.b2.raw = makeBlock2(15, false, currentTp(state), currentPty(state), data5);
  g.b3.raw = ((uint16_t)lpsBytes[0] << 8) | lpsBytes[1];
  g.b4.raw = ((uint16_t)lpsBytes[2] << 8) | lpsBytes[3];

  uint8_t totalSegs;
  if (state.longPsLen >= RDS_LONG_PS_MAX_LEN) totalSegs = RDS_LONG_PS_MAX_LEN / 4;
  else if (hasTrailingCr)                     totalSegs = ((state.longPsLen - 1) / 4) + 1;
  else                                         totalSegs = (state.longPsLen / 4) + 1;
  if (++state.longPsSegment >= totalSegs) {
    state.longPsSegment = 0;
    if (state.longPsPendingValid) {
      memcpy(state.longPs, state.longPsPending, RDS_LONG_PS_MAX_LEN);
      state.longPsLen = state.longPsPendingLen;
      state.longPsPendingValid = false;
    }
  }
  return g;
}

// Group 15B - basic tuning info extra signalling. Group index 31. Uses state.b15Segment rather
// than state.psSegment for the DI segment cycle - see that field's own comment in rds_state.h -
// so it stays correct whether it's reached via a scheduled sequence slot or (its main use today)
// the TA-toggle burst in buildNextGroup(), neither of which is in step with 0A's own cycling.
static RdsGroupBlocks buildGroup15B(RdsTxState& state) {
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  bool ta = (state.tatp & 0x01) != 0;
  bool ms = (state.ms   & 0x01) != 0;
  uint8_t diBit = (state.diPtyi >> (3 - state.b15Segment)) & 0x01;
  uint8_t data5 = ((ta ? 1 : 0) << 4) | ((ms ? 1 : 0) << 3) | (diBit << 2) | (state.b15Segment & 0x03);
  g.b2.raw = makeBlock2(15, true, currentTp(state), currentPty(state), data5);
  g.b3.raw = g.b1.pi;
  g.b4.raw = g.b2.raw;
  state.b15Segment = (state.b15Segment + 1) & 0x03;
  return g;
}

// Group 1A - Slow Labelling Codes (PIN) / Radio Paging. Struct-backed fallback for group index 2.
static RdsGroupBlocks buildGroup1A(RdsTxState& state) {
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  uint8_t data5 = 0; // radio paging codes field - a legacy feature this project doesn't implement
  g.b2.raw = makeBlock2(1, false, currentTp(state), currentPty(state), data5);
  uint8_t slcByteIdx = (state.slcSeq[state.slcCurrent] & 0x07) * 2;
  g.b3.raw = ((uint16_t)state.slc[slcByteIdx] << 8) | state.slc[slcByteIdx + 1];
  g.b4.raw = ((uint16_t)state.pin[0] << 8) | state.pin[1];
  if (state.slcSeqLen > 0) {
    state.slcCurrent = (state.slcCurrent + 1) % state.slcSeqLen;
  }
  return g;
}

// Group 1B - PIN - group index 3.
static RdsGroupBlocks buildGroup1B(RdsTxState& state) {
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  uint8_t data5 = 0;
  g.b2.raw = makeBlock2(1, true, currentTp(state), currentPty(state), data5);
  g.b3.raw = g.b1.pi;
  g.b4.raw = ((uint16_t)state.pin[0] << 8) | state.pin[1];
  return g;
}

// Group 2A - Radio Text. Struct-backed fallback for group index 4.
static RdsGroupBlocks buildGroup2A(RdsTxState& state) {
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  RtMessage& msg = state.rt[state.rtCurrent];
  uint8_t data5 = ((state.rtABFlag ? 1 : 0) << 4) | (state.rtSegment & 0x0F);
  g.b2.raw = makeBlock2(2, false, currentTp(state), currentPty(state), data5);

  bool hasTrailingCr = msg.textLen > 0 && msg.text[msg.textLen - 1] == 0x0D;
  uint8_t rtBytes[4];
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t pos = state.rtSegment * 4 + i;
    if (pos < msg.textLen)                         rtBytes[i] = msg.text[pos];
    else if (pos == msg.textLen && !hasTrailingCr)  rtBytes[i] = 0x0D;
    else                                            rtBytes[i] = 0x20;
  }
  g.b3.raw = ((uint16_t)rtBytes[0] << 8) | rtBytes[1];
  g.b4.raw = ((uint16_t)rtBytes[2] << 8) | rtBytes[3];

  uint8_t totalSegs;
  if (msg.textLen >= 64)  totalSegs = 16;
  else if (hasTrailingCr) totalSegs = ((msg.textLen - 1) / 4) + 1;
  else                    totalSegs = (msg.textLen / 4) + 1;
  if (++state.rtSegment >= totalSegs) {
    state.rtSegment = 0;
    if (state.rtPendingValid) {
      // "Buffer RT" swap (see rds_state.h's rtBufferMode comment) - the message that was playing
      // has now shown one full pass, so it's safe to swap in the staged content without tearing
      // anything mid-message. Bypasses the ordinary repeat/rotation bookkeeping below entirely:
      // rtCurrent/rtRepeatsDone are reset fresh for the swapped-in content, and falling through to
      // that bookkeeping afterwards would immediately re-advance them before message 0 of the new
      // content has even been shown once.
      memcpy(state.rt, state.rtPending, sizeof(state.rt));
      state.rtCount       = state.rtPendingCount;
      state.rtCurrent     = 0;
      state.rtRepeatsDone = 0;
      // Same toggleAB/rtSeeded convention as a live, non-buffered MEC 0x0A arrival - see
      // uecpApplyRt()'s own comment on why the very first real message is exempt.
      if (!state.rtSeeded) {
        state.rtSeeded = true;
      } else if (state.rt[0].toggleAB) {
        state.rtABFlag = !state.rtABFlag;
      }
      state.rtPendingValid = false;
    } else if (state.rtCount <= 1) {
      if (msg.repeatCount != 0 && state.rtRepeatsDone < msg.repeatCount) {
        state.rtRepeatsDone++;
      }
    } else {
      state.rtRepeatsDone++;
      if (msg.repeatCount != 0 && state.rtRepeatsDone >= msg.repeatCount) {
        state.rtRepeatsDone = 0;
        state.rtCurrent = (state.rtCurrent + 1) % state.rtCount;
        if (state.rt[state.rtCurrent].toggleAB) state.rtABFlag = !state.rtABFlag;
      }
    }
  }
  return g;
}

// Group 3A - ODA (Open Data Application) AID/group announcements, cycled from this project's own
// manually-configured table (state.odaManual[] - see rds_state.h). Only used as struct fallback
// when nothing is queued in the free-format pool for Group 3A (index 6 - see buildNextGroup()'s
// dispatch below); if odaManualCount is 0, this is never called at all. Block 3 (the ODA short
// message) is always 0 here - there's no live ODA data source behind a manually-configured entry
// to supply one.
static RdsGroupBlocks buildGroup3A(RdsTxState& state) {
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  const OdaManualEntry& entry = state.odaManual[state.odaManualCurrent];
  g.b2.raw = makeBlock2(3, false, currentTp(state), currentPty(state), entry.groupIndex & 0x1F);
  g.b3.raw = 0;
  g.b4.raw = entry.aid;
  state.odaManualCurrent = (state.odaManualCurrent + 1) % state.odaManualCount;
  return g;
}

// Group 10A - Programme Type Name (PTYN, MEC 0x3E). Struct-backed fallback for group index 20
// (ffGroupIndex(10, false)) - like ps[]/rt[], this is a single dedicated field, not a free-format
// queue, since there's only ever one current PTYN value. Block2 last5: bit4 = ptynABflag, bits3-0
// = ptynSegment (always 0b0000 or 0b0001 - only the LSB varies, but the field is 4 bits wide per
// the RDS spec's own Group 10A layout). Block3+block4 carry the 4-character segment itself, MSB
// first, same convention as every other text group here.
static RdsGroupBlocks buildGroup10A(RdsTxState& state) {
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  uint8_t data5 = ((state.ptynABflag ? 1 : 0) << 4) | (state.ptynSegment & 0x0F);
  g.b2.raw = makeBlock2(10, false, currentTp(state), currentPty(state), data5);
  uint8_t base = state.ptynSegment * 4;
  g.b3.raw = ((uint16_t)state.ptyn[base]     << 8) | state.ptyn[base + 1];
  g.b4.raw = ((uint16_t)state.ptyn[base + 2] << 8) | state.ptyn[base + 3];
  state.ptynSegment = (state.ptynSegment + 1) & 0x01;
  return g;
}

// Builds one group from a UECP MEC 0x24 (Free-Format Group) queued message - see
// free_format_groups.h. Block 1 is always the system PI (same as every struct-backed builder
// above); Block 2 is the standard group-type/version/TP/PTY bits plus the message's own last-5
// data bits; Block 3 is the message's block3 for version A, but the PI code again for version B,
// per the RDS spec's own Block 3 convention for B-type groups; Block 4 is the message's block4
// unconditionally.
static RdsGroupBlocks buildGenericGroup(const RdsTxState& state, uint8_t groupType, bool isVersionB,
                                         uint8_t block2Last5, uint16_t block3, uint16_t block4) {
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  g.b2.raw = makeBlock2(groupType, isVersionB, currentTp(state), currentPty(state), block2Last5);
  g.b3.raw = isVersionB ? g.b1.pi : block3;
  g.b4.raw = block4;
  return g;
}

// Builds exactly one RDS group's content and returns it - pure computation, no I2C, never blocks.
//
// Dispatch order: Clock Time (Group 4A) first, whenever it's due - it's not schedulable/
// overridable via MEC 0x24 (see rdsSchedulerApplyPendingTime()'s comment on why it's built
// directly from the system clock rather than from state like everything else). Otherwise MEC 0x46
// "immediate" priority, if one's pending - breaks into the schedule on the very next group, the
// same way Group 4A's own clock-tick check above does, rather than waiting for its turn in
// state.rdsSequence; guarded by ffHasContent() defensively (it should always still be there - this
// runs well within microseconds of the enqueue that set it - but never trust that blindly).
// Otherwise walk state.rdsSequence: for whichever group index comes up, free-format content (if
// any is queued for it) always wins over this project's own struct-backed groups; 0/2/4/20 (0A/1A/
// 2A/10A) always have struct fallback content, and 6 (3A) and 30 (15A) have it conditionally (3A
// only once at least one manual ODA mapping is configured - see buildGroup3A(); 15A only once a
// long PS has actually been set OR one is staged and waiting - see buildGroup15A() - the latter
// so the very first long PS ever received isn't stuck waiting on a chicken-and-egg dispatch gate
// that only buildGroup15A() itself, once reachable, can clear); any *other* index with nothing
// queued is quietly skipped - the loop tries the next sequence slot immediately rather than
// transmitting nothing, keeping the on-air group rate constant. Bounded to one full lap of the
// sequence so a pathological all-empty sequence still terminates rather than looping forever
// (falls back to Group 0A, which - being in every sketch's own default sequence - shouldn't
// normally be reachable).
static RdsGroupBlocks buildNextGroup(QN8066& tx, RdsTxState& state, FreeFormatPool& ffPool,
                                     uint8_t& pendingImmediateGroupIndex, uint8_t fallbackOffsetByte) {
  time_t now       = time(nullptr);
  time_t curMinute = now / 60;
  if (now >= 1600000000 && now % 60 == 0 && curMinute != lastCtMinuteSent) {
    lastCtMinuteSent = curMinute;
    RdsGroupBlocks g;
    fillCommonBlock1(g, state);
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    int32_t mjd = tx.calculateMJD(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
    uint8_t hour = timeinfo.tm_hour;
    uint8_t min  = timeinfo.tm_min;
    uint8_t offsetByte = state.ctFromUecp ? state.ctOffset : fallbackOffsetByte;
    uint8_t data5 = (uint8_t)((mjd >> 15) & 0x0F);
    g.b2.raw = makeBlock2(4, false, currentTp(state), currentPty(state), data5);
    g.b3.raw = (uint16_t)(((mjd & 0x7FFF) << 1) | ((hour >> 4) & 0x01));
    g.b4.raw = (uint16_t)(((min & 0x3F) << 6) | ((hour & 0x0F) << 12) | (offsetByte & 0x3F));
    return g;
  }

  if (pendingImmediateGroupIndex != FF_INDEX_NONE) {
    uint8_t groupIndex = pendingImmediateGroupIndex;
    pendingImmediateGroupIndex = FF_INDEX_NONE;
    if (ffHasContent(ffPool, groupIndex)) {
      FreeFormatSlot msg = ffDequeueForSend(ffPool, groupIndex);
      return buildGenericGroup(state, ffGroupType(groupIndex), ffGroupIsVersionB(groupIndex),
                                msg.block2Last5, msg.block3, msg.block4);
    }
  }

  // TA-toggle Group 15B burst (armed by buildGroup0A() above) - breaks into the schedule for 8
  // groups straight after the 0A that changed TA, same way MEC 0x46 immediate does above, so it
  // doesn't wait for 15B's own turn (if any) in state.rdsSequence. Checked after Clock Time/
  // immediate so those still preempt it same as they would any other slot.
  if (taBurstRemaining > 0) {
    taBurstRemaining--;
    return buildGroup15B(state);
  }

  for (uint8_t attempt = 0; attempt < state.rdsSequenceLen; attempt++) {
    uint8_t groupIndex = state.rdsSequence[state.rdsSeqPos];
    state.rdsSeqPos = (state.rdsSeqPos + 1) % state.rdsSequenceLen;

    if (ffHasContent(ffPool, groupIndex)) {
      FreeFormatSlot msg = ffDequeueForSend(ffPool, groupIndex);
      return buildGenericGroup(state, ffGroupType(groupIndex), ffGroupIsVersionB(groupIndex),
                                msg.block2Last5, msg.block3, msg.block4);
    }

    if (groupIndex == 0)  return buildGroup0A(state);
    if (groupIndex == 2)  return buildGroup1A(state);
    if (groupIndex == 4)  return buildGroup2A(state);
    if (groupIndex == 6 && state.odaManualCount > 0) return buildGroup3A(state);
    if (groupIndex == 20) return buildGroup10A(state);
    if (groupIndex == 30 && (state.longPsLen > 0 || state.longPsPendingValid)) return buildGroup15A(state);
    // Some other group index with nothing queued for it right now - skip, try the next slot.
  }

  return buildGroup0A(state); // shouldn't be reachable - see this function's own comment above
}

// Non-blocking feeder: pulls from rdsGroupBuffer and drives the chip via the async API, waiting
// for the chip's own confirmation (not an arbitrary timeout) before sending the next one - no
// delay()/blocking at all. rdsSendTimeoutMs is a generous sanity ceiling for detecting a genuinely
// stuck/faulty chip, not a normal-path timing budget - the actual pace is set entirely by the
// chip's own confirmation.
#define RDS_SEND_TIMEOUT_MS 300
static void serviceRdsTx(QN8066& tx) {
  if (rdsSendPending) {
    if (!tx.rdsIsGroupSent()) {
      if (millis() - rdsPendingSinceMillis > RDS_SEND_TIMEOUT_MS) {
        Serial.println("RDS: no confirmation from chip within 300ms - forcing on, check wiring/I2C");
        rdsSendPending = false; // fall through and send the next one anyway rather than stalling forever
      } else {
        return; // still waiting - nothing else to do this call
      }
    } else {
      rdsSendPending = false; // confirmed - safe to send the next group
    }
  }

  if (!rdsSendPending && !rdsBufferEmpty()) {
    RdsGroupBlocks g = rdsBufferPop();
    tx.rdsSendGroupAsync(g.b1, g.b2, g.b3, g.b4);
    rdsSendPending = true;
    rdsPendingSinceMillis = millis();
  }
}

void rdsSchedulerTick(QN8066& tx, RdsTxState& state, FreeFormatPool& ffPool,
                      uint8_t& pendingImmediateGroupIndex, uint8_t fallbackOffsetByte) {
  if (!rdsBufferFull()) {
    rdsBufferPush(buildNextGroup(tx, state, ffPool, pendingImmediateGroupIndex, fallbackOffsetByte));
  }
  serviceRdsTx(tx);
}

// Converts a UTC calendar date/time to seconds-since-epoch without relying on timegm() (a GNU/BSD
// extension not guaranteed present on every ESP32 Arduino core/newlib build).
static time_t utcMktime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  int64_t y   = (int64_t)year - (month <= 2 ? 1 : 0);
  int64_t era = (y >= 0 ? y : y - 399) / 400;
  int64_t yoe = y - era * 400;
  int64_t doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  int64_t days = era * 146097 + doe - 719468;
  return (time_t)(days * 86400LL + hour * 3600L + minute * 60L + second);
}

void rdsSchedulerApplyPendingTime(RdsTxState& state) {
  if (!state.ctPending) return;

  struct timeval tv;
  tv.tv_sec  = utcMktime(state.ctYear, state.ctMonth, state.ctDay,
                         state.ctHour, state.ctMinute, state.ctSecond);
  tv.tv_usec = state.ctCentisecond * 10000;
  settimeofday(&tv, nullptr);

  state.ctPending = false;
  Serial.println("System clock updated from UECP MEC 0x0D Time message");
}
