#include "rds_defaults.h"
#include "ebu_charset.h"
#include <string.h>

void rdsTxStateSetDefaults(RdsTxState& state) {
  memset(&state, 0, sizeof(state));

  state.dsn = 1;
  state.psn = 1;

  // A placeholder, not a real assigned PI code - every sketch overwrites this the moment any real
  // UECP MEC 0x01/CFG "PI="/web "PI Code" source sets one. If a particular deployment wants a
  // different boot placeholder, override state.pi[] right after this call, in that sketch's own
  // setup() - same pattern as its own currentFrequency/currentPower boot constants.
  state.pi[0] = 0xCC;
  state.pi[1] = 0x66;

  utf8ToEbu("*QN8066*", state.ps, RDS_PS_LEN);
  state.psSegment = 0;

  state.pty = 0;

  state.tatp   = 0x02; // TA=0, TP=1
  state.diPtyi = 0x01; // Mono/Stereo=1 (Stereo), Artificial Head/Compressed/Dynamic PTY=0
  state.ms     = 0x01; // Music

  state.pin[0] = 0x00;
  state.pin[1] = 0x00; // not yet set

  state.slcSeqLen  = 1; // one slot, defaulting to slc[0]=0 (a blank/generic code) - Group 1A still
                         // gets sent (carrying PIN) even before any real SLC has been configured
  state.slcCurrent = 0;

  // No boot default RT content - rtCount stays 0 (from the memset above), so
  // rds_scheduler.cpp's buildNextGroup() won't build Group 2A at all until real RT text actually
  // arrives (MEC 0x0A, a CFG "RT=" line, or a web "rds_rt" field - see rtCount's own comment in
  // rds_state.h). A sketch that wants a non-empty boot RT message can still set one right after
  // this call, in its own setup() - same pattern as state.pi[] above.
  state.rtSegment     = 0;
  state.rtRepeatsDone = 0;
  state.rtABFlag      = false;

  state.afLen     = 0;
  state.afSegment = 0;

  utf8ToEbu("        ", state.ptyn, RDS_PTYN_LEN); // 8 spaces - printable placeholder until a
                                                      // real MEC 0x3E arrives
  state.ptynSegment = 0;
  state.ptynABflag   = false;
}
