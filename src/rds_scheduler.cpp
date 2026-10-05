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
// what actually went on air, not just the live main-service tatp (which can change between sends without a
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

// Shared by every builder below: Block 1 is always the on-air (active data set's main service) PI,
// and TP/PTY come from the same place regardless of which group is being built.
static void fillCommonBlock1(RdsGroupBlocks& g, const RdsEncoderState& state) {
  const RdsMainService& main = rdsActiveMain(state);
  g.b1.pi = ((uint16_t)main.pi[0] << 8) | main.pi[1];
}
static inline bool    currentTp(const RdsEncoderState& state)  { return (rdsActiveMain(state).tatp & 0x02) != 0; }
static inline uint8_t currentPty(const RdsEncoderState& state) { return rdsActiveMain(state).pty; }

// Group 0A - Programme Service name. This project's struct-backed fallback for group index 0.
static RdsGroupBlocks buildGroup0A(RdsEncoderState& state) {
  RdsMainService& main = rdsActiveMain(state);
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  bool ta = (main.tatp & 0x01) != 0;
  bool ms = (main.ms   & 0x01) != 0;
  // Arm the Group 15B TA-toggle burst: only once we have a previously-sent TA to compare against,
  // only on an actual change, and only with TP=1 (see taBurstRemaining's own comment above).
  if (taLastSentValid && ta != taLastSent && currentTp(state)) {
    taBurstRemaining = 8;
  }
  taLastSent = ta;
  taLastSentValid = true;
  uint8_t diBit = (main.diPtyi >> (3 - state.psSegment)) & 0x01;
  uint8_t data5 = ((ta ? 1 : 0) << 4) | ((ms ? 1 : 0) << 3) | (diBit << 2) | (state.psSegment & 0x03);
  g.b2.raw = makeBlock2(0, false, currentTp(state), currentPty(state), data5);
  if (main.afLen == 0) {
    g.b3.raw = 0xE0E0; // no AF list - filler ("no AF" code, repeated)
  } else {
    uint8_t numPairs = main.afLen / 2;
    if (state.afSegment >= numPairs) state.afSegment = 0; // defensive - list may have shrunk
    g.b3.raw = ((uint16_t)main.af[state.afSegment * 2] << 8)
                        | main.af[state.afSegment * 2 + 1];
    state.afSegment = (state.afSegment + 1) % numPairs;
  }
  g.b4.raw = ((uint16_t)main.ps[state.psSegment * 2] << 8) | main.ps[state.psSegment * 2 + 1];
  state.psSegment = (state.psSegment + 1) & 0x03;
  if (state.psSegment == 0 && main.psPendingValid) {
    memcpy(main.ps, main.psPending, RDS_PS_LEN);
    main.psPendingValid = false;
  }
  return g;
}

// Group 15A - Long PS (MEC 0x21), an up-to-RDS_LONG_PS_MAX_LEN-byte UTF-8 companion to the regular
// PS above (no EBU conversion happens anywhere for it - see uecp_handler.cpp's uecpApplyLongPs()).
// Group index 30. Only ever reached if the main service's longPsLen > 0 or a long PS update is
// staged and waiting (longPsPendingValid) - see buildNextGroup()'s own dispatch check and
// rds_state.h's longPsLen comment; there's no boot-default content to fall back on the way
// ps[]/rt[] have, so with neither of those true, this group is skipped even when scheduled.
static RdsGroupBlocks buildGroup15A(RdsEncoderState& state) {
  RdsMainService& main = rdsActiveMain(state);
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);

  // Bootstrap: buildNextGroup()'s dispatch check (above) is what actually lets this function run
  // at all once something is staged, but ordinarily the swap itself only happens at the
  // end-of-cycle wrap point below - which never gets a chance to fire the very first time, since
  // longPsLen starts at 0 and this function was never being called at all before now. Apply the
  // swap immediately in that specific case, so the very first group actually sent already carries
  // the real content instead of one throwaway blank segment.
  if (main.longPsLen == 0 && main.longPsPendingValid) {
    memcpy(main.longPs, main.longPsPending, RDS_LONG_PS_MAX_LEN);
    main.longPsLen          = main.longPsPendingLen;
    state.longPsSegment     = 0;
    main.longPsPendingValid = false;
  }

  bool ta = (main.tatp & 0x01) != 0;

  // Same "does the text already end in a CR, or does one need appending" convention as RT's own
  // buildGroup2A() - a long PS shorter than the max is terminated with 0x0D so a receiver knows
  // where it ends, and only as many segments as that actually requires are sent, not always the
  // full 8.
  bool hasTrailingCr = main.longPsLen > 0 && main.longPs[main.longPsLen - 1] == 0x0D;
  uint8_t lpsBytes[4];
  for (uint8_t i = 0; i < 4; i++) {
    uint16_t pos = state.longPsSegment * 4 + i;
    if (pos < main.longPsLen)                         lpsBytes[i] = main.longPs[pos];
    else if (pos == main.longPsLen && !hasTrailingCr)  lpsBytes[i] = 0x0D;
    else                                                lpsBytes[i] = 0x20;
  }

  // data5: bit4 = TA (as Group 0A), bit3 reserved (0), bits2-0 = 3-bit segment counter (0-7,
  // matching RDS_LONG_PS_MAX_LEN/4 = 8 segments max).
  uint8_t data5 = ((ta ? 1 : 0) << 4) | (state.longPsSegment & 0x07);
  g.b2.raw = makeBlock2(15, false, currentTp(state), currentPty(state), data5);
  g.b3.raw = ((uint16_t)lpsBytes[0] << 8) | lpsBytes[1];
  g.b4.raw = ((uint16_t)lpsBytes[2] << 8) | lpsBytes[3];

  uint8_t totalSegs;
  if (main.longPsLen >= RDS_LONG_PS_MAX_LEN) totalSegs = RDS_LONG_PS_MAX_LEN / 4;
  else if (hasTrailingCr)                    totalSegs = ((main.longPsLen - 1) / 4) + 1;
  else                                        totalSegs = (main.longPsLen / 4) + 1;
  if (++state.longPsSegment >= totalSegs) {
    state.longPsSegment = 0;
    if (main.longPsPendingValid) {
      memcpy(main.longPs, main.longPsPending, RDS_LONG_PS_MAX_LEN);
      main.longPsLen = main.longPsPendingLen;
      main.longPsPendingValid = false;
    }
  }
  return g;
}

// Group 15B - basic tuning info extra signalling. Group index 31. Uses state.b15Segment rather
// than state.psSegment for the DI segment cycle - see that field's own comment in rds_state.h -
// so it stays correct whether it's reached via a scheduled sequence slot or (its main use today)
// the TA-toggle burst in buildNextGroup(), neither of which is in step with 0A's own cycling.
static RdsGroupBlocks buildGroup15B(RdsEncoderState& state) {
  const RdsMainService& main = rdsActiveMain(state);
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  bool ta = (main.tatp & 0x01) != 0;
  bool ms = (main.ms   & 0x01) != 0;
  uint8_t diBit = (main.diPtyi >> (3 - state.b15Segment)) & 0x01;
  uint8_t data5 = ((ta ? 1 : 0) << 4) | ((ms ? 1 : 0) << 3) | (diBit << 2) | (state.b15Segment & 0x03);
  g.b2.raw = makeBlock2(15, true, currentTp(state), currentPty(state), data5);
  g.b3.raw = g.b1.pi;
  g.b4.raw = g.b2.raw;
  state.b15Segment = (state.b15Segment + 1) & 0x03;
  return g;
}

// Group 1A - Slow Labelling Codes (PIN) / Radio Paging. Struct-backed fallback for group index 2.
static RdsGroupBlocks buildGroup1A(RdsEncoderState& state) {
  const RdsDataSet&     ds   = rdsActiveDataSet(state);
  const RdsMainService& main = ds.main;
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  uint8_t data5 = 0; // radio paging codes field - a legacy feature this project doesn't implement
  g.b2.raw = makeBlock2(1, false, currentTp(state), currentPty(state), data5);
  if (state.slcCurrent >= ds.slcSeqLen) state.slcCurrent = 0; // defensive - sequence may have shrunk
  uint8_t slcByteIdx = (ds.slcSeq[state.slcCurrent] & 0x07) * 2;
  g.b3.raw = ((uint16_t)ds.slc[slcByteIdx] << 8) | ds.slc[slcByteIdx + 1];
  g.b4.raw = ((uint16_t)main.pin[0] << 8) | main.pin[1];
  if (ds.slcSeqLen > 0) {
    state.slcCurrent = (state.slcCurrent + 1) % ds.slcSeqLen;
  }
  return g;
}

// Group 1B - PIN - group index 3.
static RdsGroupBlocks buildGroup1B(RdsEncoderState& state) {
  const RdsMainService& main = rdsActiveMain(state);
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  uint8_t data5 = 0;
  g.b2.raw = makeBlock2(1, true, currentTp(state), currentPty(state), data5);
  g.b3.raw = g.b1.pi;
  g.b4.raw = ((uint16_t)main.pin[0] << 8) | main.pin[1];
  return g;
}

// Group 2A - Radio Text. Struct-backed fallback for group index 4; only called when
// buildNextGroup()'s own dispatch gate has already confirmed the main service's rtCount > 0, so
// main.rt[state.rtCurrent] below is always a real message - never reached while genuinely nothing
// has ever been configured.
static RdsGroupBlocks buildGroup2A(RdsEncoderState& state) {
  RdsMainService& main = rdsActiveMain(state);
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  if (state.rtCurrent >= main.rtCount) state.rtCurrent = 0; // defensive - buffer may have shrunk
  RtMessage& msg = main.rt[state.rtCurrent];
  uint8_t data5 = ((main.rtABFlag ? 1 : 0) << 4) | (state.rtSegment & 0x0F);
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
    if (main.rtCount <= 1) {
      if (msg.repeatCount != 0 && state.rtRepeatsDone < msg.repeatCount) {
        state.rtRepeatsDone++;
      }
    } else {
      state.rtRepeatsDone++;
      if (msg.repeatCount != 0 && state.rtRepeatsDone >= msg.repeatCount) {
        state.rtRepeatsDone = 0;
        state.rtCurrent = (state.rtCurrent + 1) % main.rtCount;
        if (main.rt[state.rtCurrent].toggleAB) main.rtABFlag = !main.rtABFlag;
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
static RdsGroupBlocks buildGroup3A(RdsEncoderState& state) {
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
static RdsGroupBlocks buildGroup10A(RdsEncoderState& state) {
  const RdsMainService& main = rdsActiveMain(state);
  RdsGroupBlocks g;
  fillCommonBlock1(g, state);
  uint8_t data5 = ((main.ptynABflag ? 1 : 0) << 4) | (state.ptynSegment & 0x0F);
  g.b2.raw = makeBlock2(10, false, currentTp(state), currentPty(state), data5);
  uint8_t base = state.ptynSegment * 4;
  g.b3.raw = ((uint16_t)main.ptyn[base]     << 8) | main.ptyn[base + 1];
  g.b4.raw = ((uint16_t)main.ptyn[base + 2] << 8) | main.ptyn[base + 3];
  state.ptynSegment = (state.ptynSegment + 1) & 0x01;
  return g;
}

// Group 14A index (ffGroupIndex(14, false)) - also the gate for Group 14B, since without 14A in
// the sequence a receiver has no EON services to apply a 14B TA flag to.
static const uint8_t RDS_GROUP_14A = 28;

// Group 14A - Enhanced Other Networks, one variant of one EON service per group. Block 2's last 5
// bits are that service's TP plus the 4-bit variant code; block 4 is always its PI. Block 3 depends
// on the variant:
//   0-3  PS characters 2v, 2v+1          4   two AF codes (Method A list, filler-padded)
//   5-9  {tuning freq, mapped freq}      12  linkage information
//   13   PTY (bits 15-11) + TA (bit 0)   14  PIN
//   10, 11, 15 (unallocated / broadcaster-specific - no data held): 0x0000
static void fillGroup14A(const RdsEncoderState& state, const RdsEonService& eon, uint8_t variant,
                         uint8_t afPos, RdsGroupBlocks& g) {
  fillCommonBlock1(g, state);
  bool tpOn = (eon.tatp & 0x02) != 0;
  g.b2.raw = makeBlock2(14, false, currentTp(state), currentPty(state),
                        ((tpOn ? 1 : 0) << 4) | (variant & 0x0F));
  uint16_t b3 = 0;
  if (variant <= 3) {
    b3 = ((uint16_t)eon.ps[variant * 2] << 8) | eon.ps[variant * 2 + 1];
  } else if (variant == 4) {
    uint8_t i  = afPos * 2;
    uint8_t lo = (i + 1 < eon.afLen) ? eon.af[i + 1] : 0xCD; // filler code pads an odd-length list
    b3 = ((uint16_t)eon.af[i] << 8) | lo;
  } else if (variant <= 9) {
    b3 = ((uint16_t)eon.mapped[variant - 5][0] << 8) | eon.mapped[variant - 5][1];
  } else if (variant == 12) {
    b3 = ((uint16_t)eon.linkage[0] << 8) | eon.linkage[1];
  } else if (variant == 13) {
    b3 = ((uint16_t)(eon.pty & 0x1F) << 11) | (eon.tatp & 0x01);
  } else if (variant == 14) {
    b3 = ((uint16_t)eon.pin[0] << 8) | eon.pin[1];
  }
  g.b3.raw = b3;
  g.b4.raw = ((uint16_t)eon.pi[0] << 8) | eon.pi[1];
}

// Builds the next Group 14A in the walk: every enabled EON service of the active data set in turn
// (list order) gets the whole eonVariantSeq before the walk moves on to the next. Variant 4 sends
// the service's whole AF list as consecutive groups (one AF pair each) before stepping to the next
// variant. The only variants ever skipped are the AF ones with no data for that service: 4 when it
// has no AF list, and 5-9 when that variant's mapped pair is still unset (00 00). Every other
// variant is sent as stored, zeros included; choosing a sequence that doesn't send unwanted
// variants is the configuration's job. Disabled services are skipped. Returns false (nothing
// built) if there's nothing to send - no enabled EON services, or none with any sendable variant -
// so buildNextGroup() moves on to the next sequence slot.
static bool buildGroup14A(RdsEncoderState& state, RdsGroupBlocks& g) {
  const RdsDataSet& ds = rdsActiveDataSet(state);
  if (ds.eonCount == 0 || ds.eonVariantSeqLen == 0) return false;
  if (state.eonCurrent >= ds.eonCount) {
    state.eonCurrent = 0;
    state.eonSeqPos  = 0;
    state.eonAfPos   = 0;
  }

  // Bounded: at most one full sequence pass per service, plus one step onto each service.
  uint16_t guard = (uint16_t)(ds.eonCount + 1) * (ds.eonVariantSeqLen + 1);
  while (guard-- > 0) {
    const RdsEonService& eon = ds.eon[state.eonCurrent];
    if (!eon.enabled || state.eonSeqPos >= ds.eonVariantSeqLen) {
      state.eonCurrent = (state.eonCurrent + 1) % ds.eonCount; // on to the next service
      state.eonSeqPos  = 0;
      state.eonAfPos   = 0;
      continue;
    }
    uint8_t variant = ds.eonVariantSeq[state.eonSeqPos] & 0x0F;
    bool emptyAf     = (variant == 4 && eon.afLen == 0);
    bool emptyMapped = (variant >= 5 && variant <= 9 &&
                        eon.mapped[variant - 5][0] == 0 && eon.mapped[variant - 5][1] == 0);
    if (emptyAf || emptyMapped) {
      state.eonSeqPos++;
      continue;
    }

    fillGroup14A(state, eon, variant, state.eonAfPos, g);

    if (variant == 4 && (uint16_t)(state.eonAfPos + 1) * 2 < eon.afLen) {
      state.eonAfPos++; // more of this AF list still to go
    } else {
      state.eonAfPos = 0;
      state.eonSeqPos++;
    }
    return true;
  }
  return false;
}

// Group 14B - EON TA signal for one EON service, sent as an 8-group burst whenever its TA flag
// changes (see buildNextGroup() and RdsEncoderState::eonTaBurstRemaining). Block 2's last 5 bits:
// that service's TP (bit 4), its TA (bit 3), reserved (bits 2-0). Block 3 is the on-air PI, as for
// any B group; block 4 is the EON service's PI. Returns false if the burst no longer applies (the
// service has gone, been disabled, or 14A has left the group sequence).
static bool buildGroup14B(const RdsEncoderState& state, RdsGroupBlocks& g) {
  const RdsDataSet& ds = rdsActiveDataSet(state);
  if (state.eonTaBurstIndex >= ds.eonCount) return false;
  const RdsEonService& eon = ds.eon[state.eonTaBurstIndex];
  if (!eon.enabled || !rdsSequenceHasGroup(state, RDS_GROUP_14A)) return false;

  fillCommonBlock1(g, state);
  uint8_t data5 = (((eon.tatp & 0x02) ? 1 : 0) << 4) | (((eon.tatp & 0x01) ? 1 : 0) << 3);
  g.b2.raw = makeBlock2(14, true, currentTp(state), currentPty(state), data5);
  g.b3.raw = g.b1.pi;
  g.b4.raw = ((uint16_t)eon.pi[0] << 8) | eon.pi[1];
  return true;
}

// Builds one group from a UECP MEC 0x24 (Free-Format Group) queued message - see
// free_format_groups.h. Block 1 is always the system PI (same as every struct-backed builder
// above); Block 2 is the standard group-type/version/TP/PTY bits plus the message's own last-5
// data bits; Block 3 is the message's block3 for version A, but the PI code again for version B,
// per the RDS spec's own Block 3 convention for B-type groups; Block 4 is the message's block4
// unconditionally.
static RdsGroupBlocks buildGenericGroup(const RdsEncoderState& state, uint8_t groupType, bool isVersionB,
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
// the active rdsSequence; guarded by ffHasContent() defensively (it should always still be there - this
// runs well within microseconds of the enqueue that set it - but never trust that blindly).
// Otherwise walk the active data set's rdsSequence: for whichever group index comes up, free-format content (if
// any is queued for it) always wins over this project's own struct-backed groups; 0/2/20 (0A/1A/
// 10A) always have struct fallback content, and 4 (2A), 6 (3A), 28 (14A) and 30 (15A) have it conditionally
// (2A only once the main service's rt[] actually holds a message - see buildGroup2A() and rtCount's own comment
// in rds_state.h; 3A only once at least one manual ODA mapping is configured - see buildGroup3A();
// 14A only while the active data set has an enabled EON service - see buildGroup14A();
// 15A only once a long PS has actually been set OR one is staged and waiting - see
// buildGroup15A()) - for 15A, the "staged and waiting" half of the check exists so the very first
// real update ever received isn't stuck waiting on a chicken-and-egg dispatch gate that only the
// group's own build function, once reachable, can clear; any *other* index with nothing queued is
// quietly skipped - the loop tries
// the next sequence slot immediately rather than transmitting nothing, keeping the on-air group
// rate constant. Bounded to one full lap of the sequence so a pathological all-empty sequence
// still terminates rather than looping forever (falls back to Group 0A, which - being in every
// sketch's own default sequence - shouldn't normally be reachable).
static RdsGroupBlocks buildNextGroup(QN8066& tx, RdsEncoderState& state, FreeFormatPool& ffPool,
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
  // doesn't wait for 15B's own turn (if any) in the active rdsSequence. Checked after Clock Time/
  // immediate so those still preempt it same as they would any other slot.
  if (taBurstRemaining > 0) {
    taBurstRemaining--;
    return buildGroup15B(state);
  }

  // EON TA Group 14B burst (armed by uecp_handler.cpp when a MEC 0x03 changes an EON service's TA)
  // - breaks into the schedule the same way, straight after any main-service 15B burst, rather than
  // waiting for that service's next turn in the 14A walk. If the service no longer qualifies (see
  // buildGroup14B()), the rest of the burst is dropped.
  if (state.eonTaBurstRemaining > 0) {
    state.eonTaBurstRemaining--;
    RdsGroupBlocks g;
    if (buildGroup14B(state, g)) return g;
    state.eonTaBurstRemaining = 0;
  }

  const RdsDataSet&     ds   = rdsActiveDataSet(state);
  const RdsMainService& main = ds.main;
  if (state.rdsSeqPos >= ds.rdsSequenceLen) state.rdsSeqPos = 0; // defensive - sequence may have shrunk
  for (uint8_t attempt = 0; attempt < ds.rdsSequenceLen; attempt++) {
    uint8_t groupIndex = ds.rdsSequence[state.rdsSeqPos];
    state.rdsSeqPos = (state.rdsSeqPos + 1) % ds.rdsSequenceLen;

    if (ffHasContent(ffPool, groupIndex)) {
      FreeFormatSlot msg = ffDequeueForSend(ffPool, groupIndex);
      return buildGenericGroup(state, ffGroupType(groupIndex), ffGroupIsVersionB(groupIndex),
                                msg.block2Last5, msg.block3, msg.block4);
    }

    if (groupIndex == 0)  return buildGroup0A(state);
    if (groupIndex == 2)  return buildGroup1A(state);
    if (groupIndex == 4 && main.rtCount > 0) return buildGroup2A(state);
    if (groupIndex == 6 && state.odaManualCount > 0) return buildGroup3A(state);
    if (groupIndex == 20) return buildGroup10A(state);
    if (groupIndex == 30 && (main.longPsLen > 0 || main.longPsPendingValid)) return buildGroup15A(state);
    if (groupIndex == RDS_GROUP_14A) {
      RdsGroupBlocks g;
      if (buildGroup14A(state, g)) return g; // no enabled EON services - skip, like any empty slot
    }
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

void rdsSchedulerTick(QN8066& tx, RdsEncoderState& state, FreeFormatPool& ffPool,
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

void rdsSchedulerApplyPendingTime(RdsEncoderState& state) {
  if (!state.ctPending) return;

  struct timeval tv;
  tv.tv_sec  = utcMktime(state.ctYear, state.ctMonth, state.ctDay,
                         state.ctHour, state.ctMinute, state.ctSecond);
  tv.tv_usec = state.ctCentisecond * 10000;
  settimeofday(&tv, nullptr);

  state.ctPending = false;
  Serial.println("System clock updated from UECP MEC 0x0D Time message");
}
