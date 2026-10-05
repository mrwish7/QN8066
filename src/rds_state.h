#pragma once
#include <stdint.h>

#define RDS_RT_BUFFER_SIZE  6
#define RDS_RT_MAX_LEN     64
#define RDS_PS_LEN          8
#define RDS_LONG_PS_MAX_LEN 32 // Group 15A sends it in 4-byte segments, 3-bit segment counter (0-7)
#define RDS_SLC_LEN        16
#define RDS_AF_MAX_LEN     48
#define RDS_ODA_MANUAL_MAX  8
#define RDS_PTYN_LEN        8
#define RDS_SEQUENCE_MAX_LEN 0xFC // UECP MEC 0x16's own LEN field is one byte, capped at 0xFC by
                                    // spec - matches this array's own capacity exactly

// Data sets (UECP DSNs) are fixed at 1..RDS_DSN_COUNT, as is common encoder behaviour - DSN n lives
// at RdsEncoderState::dataSets[n-1]. Each holds one main service plus up to RDS_EON_PER_DSN_MAX
// other-network (EON) services, for RDS_EON_PER_DSN_MAX + 1 (18) PSNs per DSN in total.
#define RDS_DSN_COUNT        6
#define RDS_EON_PER_DSN_MAX 17
#define RDS_EON_MAPPED_COUNT 5   // Group 14A variants 5-9 (mapped frequencies), one pair each
#define RDS_EON_SEQ_MAX     16   // Group 14A variant codes are 4 bits, so 16 distinct entries

struct RtMessage {
  uint8_t text[RDS_RT_MAX_LEN];
  uint8_t textLen;      // actual character count (0-64)
  uint8_t repeatCount;  // full passes to send before advancing; 0 = loop forever
  bool    toggleAB;     // flip the shared A/B flag when this message starts
};

// One manually-configured ODA (Open Data Application) AID -> group mapping - see
// RdsEncoderState::odaManual[] below.
struct OdaManualEntry {
  uint16_t aid;
  uint8_t  groupIndex; // ffGroupIndex() packing (see free_format_groups.h) - which group this
                         // AID's ODA data is transmitted on
};

// A data set's main service - the one actually on air (Groups 0A/1A/2A/10A/15A/15B) while its data
// set is the active one. Holds the content of every DSN+PSN-addressed MEC. Transmission positions
// (psSegment, rtSegment etc.) aren't here - they only ever matter for the one service currently on
// air, so they live once in RdsEncoderState instead.
struct RdsMainService {
  uint8_t psnNumber;      // this service's UECP PSN (1-255); a MEC addressed to PSN 0 or to this
                            // number resolves to this service - see uecp_handler.cpp

  uint8_t pi[2];

  uint8_t ps[RDS_PS_LEN];
  uint8_t psPending[RDS_PS_LEN]; // staged new PS (MEC 0x02); only meaningful when psPendingValid
  bool    psPendingValid;        // true = swap psPending into ps at the next clean psSegment==0
                                  // wrap, so a receiver never sees a torn mix of old/new PS chars
                                  // mid-cycle (set back to false by buildGroup0A() once applied)

  // Long PS (MEC 0x21), transmitted on Group 15A - a longer, free-length companion to the regular
  // 8-character PS above, UTF-8 rather than EBU (no charset conversion happens for it anywhere -
  // see uecp_handler.cpp's uecpApplyLongPs()). longPsLen == 0 means "not configured": unlike PS,
  // 15A has no boot-default content, so rds_scheduler.cpp's buildNextGroup() skips it entirely
  // (same "no default, don't send" treatment buildGroup3A() gets when odaManualCount == 0) even if
  // 15A is scheduled - there's nothing sensible to show.
  uint8_t longPs[RDS_LONG_PS_MAX_LEN];
  uint8_t longPsLen;      // 0-32; 0 = not configured, see above

  uint8_t longPsPending[RDS_LONG_PS_MAX_LEN]; // staged new long PS (MEC 0x21); only meaningful
                                                // when longPsPendingValid
  uint8_t longPsPendingLen;                    // length of the staged value above
  bool    longPsPendingValid;                  // true = swap longPsPending/longPsPendingLen into
                                                 // longPs/longPsLen at the next clean
                                                 // longPsSegment==0 wrap - same clean-swap
                                                 // convention as psPending/psPendingValid above

  uint8_t pty;            // Programme Type (MEC 0x07)

  uint8_t tatp;           // bit 0 = TA, bit 1 = TP (MEC 0x03)
  uint8_t diPtyi;         // bit0=Mono/Stereo(1=Stereo), bit1=Artificial Head(1=yes), bit2=Compressed(1=yes), bit3=Dynamic PTY(1=dynamic) (MEC 0x04, single combined byte per UECP spec)
  uint8_t ms;             // bit 0 = Music/Speech, 1=Music (MEC 0x05)

  uint8_t pin[2];         // raw PIN day/hour/minute, MSB first (MEC 0x06); 0x0000 = not yet received
  uint8_t linkage[2];     // linkage information (MEC 0x2E), MSB first - stored only for now, not
                            // yet transmitted (Group 1A block 4 linkage bits)

  RtMessage rt[RDS_RT_BUFFER_SIZE];
  uint8_t   rtCount;        // number of populated messages (0-6); 0 = no RT configured at all (the
                             // boot default - see rds_defaults.cpp) or explicitly cleared with no
                             // replacement text (MEC 0x0A with MEL==0, or MEL==1 and clear-bits with
                             // no text byte - see uecpApplyRt() in uecp_handler.cpp), in which case
                             // rds_scheduler.cpp's buildNextGroup() stops building Group 2A entirely
                             // until real RT text arrives, the same way it skips any other group
                             // with nothing to send rather than transmitting empty content
  bool      rtABFlag;       // A/B flag value used in every Group 2A group for this service
  bool      rtSeeded;       // true once real (UECP/web) content has been stored in rt[] since the
                             // last time RT was paused (boot, or a clear-with-no-text that leaves
                             // rtCount at 0 - see uecpApplyRt(), which resets this back to false
                             // the moment RT goes empty). While
                             // false, there's nothing meaningful yet to flip relative to, so the
                             // first real message after the pause forces rtABFlag to false (the A
                             // position) outright, regardless of its own toggleAB bit, instead of
                             // flipping it against a value that was never part of a genuine
                             // broadcast toggle lineage - every following message's toggleAB bit is
                             // then respected normally until the next pause

  uint8_t af[RDS_AF_MAX_LEN]; // raw AF list bytes as received (Method A/B per UECP MEC 0x13, unparsed)
  uint8_t afLen;              // actual used length within af[] (0 = no AF list yet)

  // Programme Type Name (MEC 0x3E) - an 8-character string transmitted on Group 10A, split into
  // two 4-character segments per RDS Group 10A's own layout (unlike ps[]'s four 2-character
  // segments). Applied immediately (no staging like psPending/psPendingValid) since MEC 0x3E is
  // DSN/PSN-addressed and fixed-length like PS, but there's no existing receiver convention for
  // "PTYN is mid-update" that a torn transmission would violate - same reasoning as RT/AF.
  uint8_t ptyn[RDS_PTYN_LEN];
  bool    ptynABflag;  // flipped every time a MEC 0x3E element is applied (see uecpApplyMec() in
                         // uecp_handler.cpp) - unconditionally, unlike RT's toggleAB (which is a
                         // bit the source itself sets); MEC 0x3E carries no such bit of its own
};

// One other-network (EON) service, carried on Group 14A while its data set is active - only the
// fields 14A can actually transmit. No PS staging: an EON PS change tearing mid-cycle matters far
// less than the main service's Group 0A PS, and which PS segment goes out when is driven entirely by
// the data set's eonVariantSeq. Created (with default content, disabled) by a UECP MEC 0x28 PSN
// list - see rds_defaults.h's rdsDefinePsnList() - then enabled/disabled by MEC 0x0B and filled by
// the DSN+PSN-addressed MECs (see uecp_handler.cpp's uecpForEachService()). While its data set is
// on air and it's enabled, it's transmitted on Group 14A (rds_scheduler.cpp's buildGroup14A()),
// plus a Group 14B burst whenever its TA flag changes.
struct RdsEonService {
  uint8_t psnNumber;      // this service's UECP PSN (1-255)
  bool    enabled;        // UECP MEC 0x0B - only enabled services are transmitted on Group 14A.
                            // Off when created by MEC 0x28; content can still be stored while off
  uint8_t pi[2];
  uint8_t ps[RDS_PS_LEN];                       // 14A variants 0-3
  uint8_t pty;                                  // 14A variant 13
  uint8_t tatp;                                 // bit 0 = TA, bit 1 = TP (variant 13 / block 2)
  uint8_t pin[2];                               // 14A variant 14
  uint8_t af[RDS_AF_MAX_LEN];                   // 14A variant 4 (AF Method A), MEC 0x14
  uint8_t afLen;                                // 0 = no list, and variant 4 is skipped
  uint8_t mapped[RDS_EON_MAPPED_COUNT][2];      // 14A variants 5-9: [variant-5] = {tuning freq,
                                                  // mapped freq}, MEC 0x14; 00 00 = unset, and
                                                  // that variant is skipped
  uint8_t linkage[2];                           // 14A variant 12, MEC 0x2E
};

// One UECP data set (DSN): its main service, its EON services, and the content of every DSN-only
// MEC (no PSN field).
struct RdsDataSet {
  RdsMainService main;
  RdsEonService  eon[RDS_EON_PER_DSN_MAX];
  uint8_t        eonCount; // number of eon[] entries in use (0-17), in MEC 0x28 list order

  // The RDS group scheduling order - rds_scheduler.cpp's buildNextGroup() walks the active data
  // set's copy from RdsEncoderState::rdsSeqPos, wrapping at rdsSequenceLen, to decide which group to
  // build next; see its own comment for the full dispatch rules. Seeded at boot from each sketch's
  // own RDS_SEQUENCE/RDS_SEQUENCE_LEN constants (setup() copies them in once), then reconfigurable at
  // runtime via UECP MEC 0x16 (uecp_handler.cpp's processUecpFrame()) or per-sketch - the serial
  // sketch's "SEQ" CFG line key, or the WiFi sketches' "Group Sequence" web form field - both
  // replace the whole sequence and reset rdsSeqPos to 0 so the new order is read from its own
  // beginning rather than wherever the old one left off. Entries are real RDS group indices -
  // ffGroupIndex(groupType, versionB) packing, same as free_format_groups.h's own indices.
  uint8_t rdsSequence[RDS_SEQUENCE_MAX_LEN];
  uint8_t rdsSequenceLen; // 1..RDS_SEQUENCE_MAX_LEN entries actually in use

  uint8_t slc[RDS_SLC_LEN]; // SLC bytes (MEC 0x1A)
  uint8_t slcSeq[16];       // Group 1A variant order (MEC 0x29, group 0x02)
  uint8_t slcSeqLen;

  // Group 14A variant order (MEC 0x29, group 0x1C) - walked in full for each EON service in turn
  // (see rds_scheduler.cpp's buildGroup14A()). Boot default 0-9, 12, 13, 14.
  uint8_t eonVariantSeq[RDS_EON_SEQ_MAX];
  uint8_t eonVariantSeqLen;
};

struct RdsEncoderState {
  RdsDataSet dataSets[RDS_DSN_COUNT];
  uint8_t    activeDsn;   // 1..RDS_DSN_COUNT - the data set currently on air (boot default 1)

  // --- On-air transmission positions - always refer to the active data set and its main service,
  // and are reset whenever the active data set changes (see rdsSelectDataSet() below). ---
  uint8_t psSegment;      // 0-3: next 2-byte pair index for Group 0A
  uint8_t b15Segment;     // 0-3: next DI segment index for Group 15B - deliberately separate from
                            // psSegment. 15B isn't scheduled in lockstep with 0A (in particular the
                            // 8-group TA-toggle burst - see rds_scheduler.cpp's buildNextGroup() -
                            // fires between 0A sends and needs two full 0-3 sweeps of its own),
                            // so it can't just borrow 0A's position in the DI cycle.
  uint8_t longPsSegment;  // 0-7: next 4-byte chunk index for Group 15A - its own independent
                            // counter, same reasoning as b15Segment above: 15A's segment count is
                            // content-length-dependent (see buildGroup15A()) rather than a fixed
                            // 4-segment cycle like PS's, so it can't share psSegment either
  uint8_t rtCurrent;      // index of the RT message currently being transmitted
  uint8_t rtSegment;      // 0-15: which 4-byte chunk within current message
  uint8_t rtRepeatsDone;  // full passes of current message completed this cycle
  uint8_t afSegment;      // next 2-byte AF pair index to send, cycles like psSegment based on afLen
  uint8_t ptynSegment;    // 0-1: next 4-char PTYN segment index for Group 10A
  uint8_t slcCurrent;     // position within the active data set's slcSeq[]
  uint8_t rdsSeqPos;      // buildNextGroup()'s current position within the active rdsSequence[]

  // Group 14A walk (see rds_scheduler.cpp's buildGroup14A()): each enabled EON service in turn gets
  // the whole eonVariantSeq, then the walk moves on to the next one.
  uint8_t eonCurrent;     // index into the active data set's eon[] of the service being sent
  uint8_t eonSeqPos;      // position within eonVariantSeq for that service
  uint8_t eonAfPos;       // AF pair index while sending variant 4 (the whole list goes out as
                            // consecutive groups before the walk moves to the next variant)

  // Group 14B EON TA burst: armed by uecp_handler.cpp when a MEC 0x03 changes the TA flag of an
  // enabled EON service in the active data set (while 14A is in its group sequence), then drained
  // by buildNextGroup() as 8 consecutive 14B groups - the EON counterpart of the main service's
  // 15B TA burst. A second TA change before the burst finishes replaces it.
  uint8_t eonTaBurstRemaining;
  uint8_t eonTaBurstIndex; // index into the active data set's eon[] of the service it signals

  // --- Global (not per data set) ---

  // The content every freshly created service starts from - each main and EON service a UECP MEC
  // 0x28 PSN list definition creates is initialised from this (EON services take just the fields
  // they have: PI, PS, PTY, TA/TP, PIN). Captured from DSN 1's main service by
  // rdsCopyDataSet1ToAll() at boot, so it's the sketch's own boot station identity, not just the
  // library's generic placeholders.
  RdsMainService serviceDefaults;

  // Manually-configured ODA AID -> group mappings, transmitted as Group 3A "master data" (the
  // same role ps[]/rt[] play for 0A/2A - see rds_scheduler.cpp's buildGroup3A()) whenever nothing
  // is queued in the free-format pool for Group 3A. Lets an operator declare known ODA app
  // placements up front - useful for a UECP source that only ever sends MEC 0x46 ODA data and
  // never its own Group 3A/MEC 0x40 definitions, which would otherwise leave receivers (and this
  // device's own MEC 0x46 handling) with no way to know which group that AID's data is even on.
  // Set per-sketch - the serial sketch's "ODA" CFG line key (applyConfigLine() in its .ino), or
  // the WiFi sketches' "Add ODA Mapping" web form field (handleUpdate()'s "oda_add" branch) - which
  // both, alongside populating this fixed-config table, also seed the same mapping into the live,
  // UECP-learned OdaLiveDirectory (oda_directory.h) that incoming MEC 0x46 elements actually
  // resolve against; a later live MEC 0x24/0x40/0x46 definition for the same AID overwrites that
  // seed normally.
  OdaManualEntry odaManual[RDS_ODA_MANUAL_MAX];
  uint8_t        odaManualCount;   // number of entries actually populated (0 = none configured,
                                     // so buildGroup3A() is never reachable at all)
  uint8_t        odaManualCurrent; // which entry buildGroup3A() sends next, cycling round-robin

  // MEC 0x0D (Time) - unlike every other MEC handled here, this one carries no DSN/PSN at all
  // (see uecp_handler.cpp's processUecpFrame()): it's a single global "set the clock" message,
  // not addressed to a specific dataset/service. Date/time is UTC. Staged here rather than pushed
  // straight to the system clock, since this struct (and the UECP parsing that populates it) is
  // also compiled into the Linux test harness, which must never touch the host machine's real
  // system clock - only rds_scheduler.cpp's rdsSchedulerApplyPendingTime() (called from each
  // sketch's own loop(), which checks ctPending) is allowed to actually call settimeofday().
  uint16_t ctYear;        // full year (e.g. 2026); only meaningful once ctFromUecp is true
  uint8_t  ctMonth;       // 1-12
  uint8_t  ctDay;         // 1-31
  uint8_t  ctHour;        // 0-23 (UTC)
  uint8_t  ctMinute;      // 0-59 (UTC)
  uint8_t  ctSecond;      // 0-59 (UTC)
  uint8_t  ctCentisecond; // 0-99
  uint8_t  ctOffset;      // raw 6-bit UECP-format local time offset: bit5=sign (0=+, 1=-),
                           // bits4-0=magnitude in half-hours - matches RDS Group 4A block4's own
                           // offset/offset_sign bit layout exactly, so it's used unmodified
  bool     ctFromUecp;    // true once a MEC 0x0D message has ever been applied - once set, Group
                           // 4A uses ctOffset (and the system clock, kept in sync from ctYear..
                           // ctCentisecond by rdsSchedulerApplyPendingTime()) instead of the
                           // platform's own fallback
  bool     ctPending;     // true from the moment a fresh MEC 0x0D update is staged until
                           // rdsSchedulerApplyPendingTime() applies it to the system clock and
                           // clears this flag
};

inline RdsDataSet& rdsActiveDataSet(RdsEncoderState& state) {
  return state.dataSets[state.activeDsn - 1];
}
inline const RdsDataSet& rdsActiveDataSet(const RdsEncoderState& state) {
  return state.dataSets[state.activeDsn - 1];
}
inline RdsMainService& rdsActiveMain(RdsEncoderState& state) {
  return rdsActiveDataSet(state).main;
}
inline const RdsMainService& rdsActiveMain(const RdsEncoderState& state) {
  return rdsActiveDataSet(state).main;
}

// Resets every on-air transmission position (see RdsEncoderState) back to the start - called
// whenever the content they index into is swapped out wholesale (a data set switch).
inline void rdsResetOnAirPositions(RdsEncoderState& state) {
  state.psSegment     = 0;
  state.b15Segment    = 0;
  state.longPsSegment = 0;
  state.rtCurrent     = 0;
  state.rtSegment     = 0;
  state.rtRepeatsDone = 0;
  state.afSegment     = 0;
  state.ptynSegment   = 0;
  state.slcCurrent    = 0;
  state.rdsSeqPos     = 0;
  state.eonCurrent    = 0;
  state.eonSeqPos     = 0;
  state.eonAfPos      = 0;
  state.eonTaBurstRemaining = 0; // the burst was about the old data set's EON services
}

// Makes data set dsn (1..RDS_DSN_COUNT) the one on air - its main service immediately replaces the
// previous one in every main-service group. Returns false (nothing changed) if dsn is out of range.
// Selecting the already-active data set is a no-op that still returns true.
inline bool rdsSelectDataSet(RdsEncoderState& state, uint8_t dsn) {
  if (dsn < 1 || dsn > RDS_DSN_COUNT) return false;
  if (dsn == state.activeDsn) return true;
  state.activeDsn = dsn;
  rdsResetOnAirPositions(state);
  return true;
}

// True if groupIndex appears anywhere in the active data set's group sequence. A free-format
// enqueue (MEC 0x24/0x30/0x40/0x46 - see uecp_handler.cpp) for a group that isn't scheduled at all
// would just sit in the pool forever: rds_scheduler.cpp's buildNextGroup() walk over this same
// rdsSequence is the only thing that ever calls ffDequeueForSend() for a given group index, so an
// unscheduled group's queue can never drain on its own, only fill up until it hits
// FF_GROUP_MAX_SLOTS. Callers use this to reject such enqueues up front instead - except for MEC
// 0x46 "immediate" priority messages, which bypass rdsSequence entirely via
// pendingImmediateGroupIndex (see buildNextGroup()'s own check), so they're never gated on
// sequence membership at all.
inline bool rdsSequenceHasGroup(const RdsEncoderState& state, uint8_t groupIndex) {
  const RdsDataSet& ds = rdsActiveDataSet(state);
  for (uint8_t i = 0; i < ds.rdsSequenceLen; i++) {
    if (ds.rdsSequence[i] == groupIndex) return true;
  }
  return false;
}
