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

struct RtMessage {
  uint8_t text[RDS_RT_MAX_LEN];
  uint8_t textLen;      // actual character count (0-64)
  uint8_t repeatCount;  // full passes to send before advancing; 0 = loop forever
  bool    toggleAB;     // flip the shared A/B flag when this message starts
};

// One manually-configured ODA (Open Data Application) AID -> group mapping - see
// RdsTxState::odaManual[] below.
struct OdaManualEntry {
  uint16_t aid;
  uint8_t  groupIndex; // ffGroupIndex() packing (see free_format_groups.h) - which group this
                         // AID's ODA data is transmitted on
};

struct RdsTxState {
  uint8_t dsn;            // Data Set Number - which encoder/dataset this state belongs to (UECP address field)
  uint8_t psn;            // Programme Service Number - which service within that dataset this state belongs to (UECP address field)

  uint8_t pi[2];

  uint8_t ps[RDS_PS_LEN];
  uint8_t psSegment;      // 0-3: next 2-byte pair index for Group 0A

  uint8_t b15Segment;     // 0-3: next DI segment index for Group 15B - deliberately separate from
                            // psSegment. 15B isn't scheduled in lockstep with 0A (in particular the
                            // 8-group TA-toggle burst - see rds_scheduler.cpp's buildNextGroup() -
                            // fires between 0A sends and needs two full 0-3 sweeps of its own),
                            // so it can't just borrow 0A's position in the DI cycle.

  uint8_t psPending[RDS_PS_LEN]; // staged new PS (MEC 0x02); only meaningful when psPendingValid
  bool    psPendingValid;        // true = swap psPending into ps at the next clean psSegment==0
                                  // wrap, so a receiver never sees a torn mix of old/new PS chars
                                  // mid-cycle (set by sendRDS() back to false once applied)

  // Long PS (MEC 0x21), transmitted on Group 15A - a longer, free-length companion to the regular
  // 8-character PS above, UTF-8 rather than EBU (no charset conversion happens for it anywhere -
  // see uecp_handler.cpp's uecpApplyLongPs()). longPsLen == 0 means "not configured": unlike PS,
  // 15A has no boot-default content, so rds_scheduler.cpp's buildNextGroup() skips it entirely
  // (same "no default, don't send" treatment buildGroup3A() gets when odaManualCount == 0) even if
  // 15A is scheduled - there's nothing sensible to show. longPsSegment is its own independent 0-7
  // counter, same reasoning as b15Segment above: 15A's segment count is content-length-dependent
  // (see buildGroup15A()) rather than a fixed 4-segment cycle like PS's, so it can't share psSegment
  // either.
  uint8_t longPs[RDS_LONG_PS_MAX_LEN];
  uint8_t longPsLen;      // 0-32; 0 = not configured, see above
  uint8_t longPsSegment;  // 0-7: next 4-byte chunk index for Group 15A

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
  uint8_t slc[RDS_SLC_LEN]; // SLC bytes
  uint8_t slcSeq[16];
  uint8_t slcSeqLen;
  uint8_t slcCurrent;

  RtMessage rt[RDS_RT_BUFFER_SIZE];
  uint8_t   rtCount;        // number of populated messages (0-6); 0 = no RT configured at all (the
                             // boot default - see rds_defaults.cpp) or explicitly cleared with no
                             // replacement text (MEC 0x0A with MEL==0, or MEL==1 and clear-bits with
                             // no text byte - see uecpApplyRt() in uecp_handler.cpp), in which case
                             // rds_scheduler.cpp's buildNextGroup() stops building Group 2A entirely
                             // until real RT text arrives, the same way it skips any other group
                             // with nothing to send rather than transmitting empty content
  uint8_t   rtCurrent;      // index of message currently being transmitted
  uint8_t   rtSegment;      // 0-15: which 4-byte chunk within current message
  uint8_t   rtRepeatsDone;  // full passes of current message completed this cycle
  bool      rtABFlag;       // live A/B flag value used in every Group 2A group
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
  uint8_t afSegment;          // next 2-byte pair index to send, cycles like psSegment based on afLen

  // Programme Type Name (MEC 0x3E) - an 8-character string transmitted on Group 10A, split into
  // two 4-character segments per RDS Group 10A's own layout (unlike ps[]'s four 2-character
  // segments). Applied immediately (no staging like psPending/psPendingValid) since MEC 0x3E is
  // DSN/PSN-addressed and fixed-length like PS, but there's no existing receiver convention for
  // "PTYN is mid-update" that a torn transmission would violate - same reasoning as RT/AF.
  uint8_t ptyn[RDS_PTYN_LEN];
  uint8_t ptynSegment; // 0-1: next 4-char segment index for Group 10A
  bool    ptynABflag;  // flipped every time a MEC 0x3E element is applied (see uecpApplyMec() in
                         // uecp_handler.cpp) - unconditionally, unlike RT's toggleAB (which is a
                         // bit the source itself sets); MEC 0x3E carries no such bit of its own

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

  // The RDS group scheduling order - rds_scheduler.cpp's buildNextGroup() walks this array from
  // rdsSeqPos, wrapping at rdsSequenceLen, to decide which group to build next; see its own
  // comment for the full dispatch rules. Seeded at boot from each sketch's own RDS_SEQUENCE/
  // RDS_SEQUENCE_LEN constants (setup() copies them in once), then reconfigurable at runtime via
  // UECP MEC 0x16 (uecp_handler.cpp's processUecpFrame()) or per-sketch - the serial sketch's "SEQ"
  // CFG line key, or the WiFi sketches' "Group Sequence" web form field - both replace the whole
  // sequence and reset rdsSeqPos to 0 so the new order is read from its own beginning rather than
  // wherever the old one left off. Entries are real RDS group indices - ffGroupIndex(groupType,
  // versionB) packing, same as free_format_groups.h's own indices.
  uint8_t rdsSequence[RDS_SEQUENCE_MAX_LEN];
  uint8_t rdsSequenceLen; // 1..RDS_SEQUENCE_MAX_LEN entries actually in use
  uint8_t rdsSeqPos;      // buildNextGroup()'s current position within rdsSequence[]

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

// True if groupIndex appears anywhere in the first rdsSequenceLen entries of state.rdsSequence.
// A free-format
// enqueue (MEC 0x24/0x30/0x40/0x46 - see uecp_handler.cpp) for a group that isn't scheduled at all
// would just sit in the pool forever: rds_scheduler.cpp's buildNextGroup() walk over this same
// rdsSequence is the only thing that ever calls ffDequeueForSend() for a given group index, so an
// unscheduled group's queue can never drain on its own, only fill up until it hits
// FF_GROUP_MAX_SLOTS. Callers use this to reject such enqueues up front instead - except for MEC
// 0x46 "immediate" priority messages, which bypass rdsSequence entirely via
// pendingImmediateGroupIndex (see buildNextGroup()'s own check), so they're never gated on
// sequence membership at all.
inline bool rdsSequenceHasGroup(const RdsTxState& state, uint8_t groupIndex) {
  for (uint8_t i = 0; i < state.rdsSequenceLen; i++) {
    if (state.rdsSequence[i] == groupIndex) return true;
  }
  return false;
}
