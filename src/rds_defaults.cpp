#include "rds_defaults.h"
#include "ebu_charset.h"
#include <string.h>

// Group 14A default variant order: PS (0-3), AF (4), mapped frequencies (5-9), linkage (12),
// PTY/TA (13), PIN (14). 10 and 11 are unused by the spec; 15 is broadcaster-specific.
static const uint8_t RDS_EON_DEFAULT_SEQUENCE[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 14};

static void rdsMainServiceSetDefaults(RdsMainService& main) {
  memset(&main, 0, sizeof(main));

  main.psnNumber = 1;

  // A placeholder, not a real assigned PI code - every sketch overwrites this the moment any real
  // UECP MEC 0x01/CFG "PI="/web "PI Code" source sets one. If a particular deployment wants a
  // different boot placeholder, override rdsActiveMain(state).pi[] right after
  // rdsEncoderStateSetDefaults(), in that sketch's own setup() - same pattern as its own
  // currentFrequency/currentPower boot constants.
  main.pi[0] = 0xCC;
  main.pi[1] = 0x66;

  utf8ToEbu("*QN8066*", main.ps, RDS_PS_LEN);

  main.pty = 0;

  main.tatp   = 0x02; // TA=0, TP=1
  main.diPtyi = 0x01; // Mono/Stereo=1 (Stereo), Artificial Head/Compressed/Dynamic PTY=0
  main.ms     = 0x01; // Music

  // pin[]/linkage[] stay 0x0000 (not yet set) from the memset above.

  // No boot default RT content - rtCount stays 0 (from the memset above), so
  // rds_scheduler.cpp's buildNextGroup() won't build Group 2A at all until real RT text actually
  // arrives (MEC 0x0A, a CFG "RT=" line, or a web "rds_rt" field - see rtCount's own comment in
  // rds_state.h). A sketch that wants a non-empty boot RT message can still set one right after
  // this call, in its own setup() - same pattern as pi[] above.
  main.rtABFlag = false;

  main.afLen = 0;

  utf8ToEbu("        ", main.ptyn, RDS_PTYN_LEN); // 8 spaces - printable placeholder until a
                                                     // real MEC 0x3E arrives
  main.ptynABflag = false;
}

static void rdsDataSetSetDefaults(RdsDataSet& ds) {
  memset(&ds, 0, sizeof(ds));
  rdsMainServiceSetDefaults(ds.main);
  ds.eonCount = 0;

  // The group sequence is left empty here - it has no sketch-independent default; each sketch
  // seeds it from its own RDS_SEQUENCE constant in setup().

  ds.slcSeqLen = 1; // one slot, defaulting to slc[0]=0 (a blank/generic code) - Group 1A still
                     // gets sent (carrying PIN) even before any real SLC has been configured

  memcpy(ds.eonVariantSeq, RDS_EON_DEFAULT_SEQUENCE, sizeof(RDS_EON_DEFAULT_SEQUENCE));
  ds.eonVariantSeqLen = sizeof(RDS_EON_DEFAULT_SEQUENCE);
}

void rdsEncoderStateSetDefaults(RdsEncoderState& state) {
  memset(&state, 0, sizeof(state));
  for (uint8_t i = 0; i < RDS_DSN_COUNT; i++) {
    rdsDataSetSetDefaults(state.dataSets[i]);
  }
  state.activeDsn = 1;
  rdsResetOnAirPositions(state);
  state.serviceDefaults = state.dataSets[0].main; // until rdsCopyDataSet1ToAll() captures the
                                                    // sketch's own boot overrides
}

void rdsCopyDataSet1ToAll(RdsEncoderState& state) {
  for (uint8_t i = 1; i < RDS_DSN_COUNT; i++) {
    state.dataSets[i] = state.dataSets[0];
  }
  state.serviceDefaults = state.dataSets[0].main;
}

// An EON service gets the subset of fields it has from the same template a main service is
// created from; AF, mapped frequencies and linkage start empty.
static void rdsEonServiceSetDefaults(RdsEonService& eon, const RdsMainService& defaults) {
  memset(&eon, 0, sizeof(eon));
  memcpy(eon.pi, defaults.pi, sizeof(eon.pi));
  memcpy(eon.ps, defaults.ps, sizeof(eon.ps));
  eon.pty  = defaults.pty;
  eon.tatp = defaults.tatp;
  memcpy(eon.pin, defaults.pin, sizeof(eon.pin));
}

RdsPsnListResult rdsDefinePsnList(RdsEncoderState& state, uint8_t dsn, const uint8_t* psns,
                                  uint8_t count) {
  if (dsn < 1 || dsn > RDS_DSN_COUNT)              return RDS_PSN_LIST_BAD_DSN;
  if (dsn == state.activeDsn)                      return RDS_PSN_LIST_ACTIVE_DSN;
  if (count == 0 || count > RDS_EON_PER_DSN_MAX + 1) return RDS_PSN_LIST_BAD_COUNT;
  for (uint8_t i = 0; i < count; i++) {
    if (psns[i] == 0) return RDS_PSN_LIST_BAD_PSN;
    for (uint8_t j = 0; j < i; j++) {
      if (psns[j] == psns[i]) return RDS_PSN_LIST_DUPLICATE;
    }
  }

  RdsDataSet& ds = state.dataSets[dsn - 1];
  ds.main           = state.serviceDefaults;
  ds.main.psnNumber = psns[0];
  for (uint8_t i = 0; i < RDS_EON_PER_DSN_MAX; i++) {
    rdsEonServiceSetDefaults(ds.eon[i], state.serviceDefaults);
  }
  ds.eonCount = count - 1;
  for (uint8_t i = 0; i < ds.eonCount; i++) {
    ds.eon[i].psnNumber = psns[i + 1];
  }
  return RDS_PSN_LIST_OK;
}
