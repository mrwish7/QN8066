/*
  ESP32 Dev Module / ESP32-C3 version - SERIAL_UECP variant.

  No WiFi, no web interface. This sketch feeds the QN8066 from an external analogue audio source
  (line-in) and gets ALL of its RDS content from a single source: UECP frames arriving over the
  USB/Serial port (same 0xFE ... 0xFF framing, same MEC parsing as the WiFi variants' TCP UECP
  server). There is no raw RDS group input mode here (that existed only to test against the RDS
  group scheduler directly and has been dropped in this variant) - Serial carries UECP only.

  All of the actual RDS/UECP logic - UECP frame parsing/dispatch (uecp_handler.h/.cpp), the
  free-format group pool (free_format_groups.h/.cpp), the live ODA AID->group directory
  (oda_directory.h/.cpp), the RDS session state struct and its boot defaults (rds_state.h,
  rds_defaults.h/.cpp), EBU/UTF-8 text conversion (ebu_charset.h/.cpp), and the RDS group builder/
  scheduler/transmitter (rds_scheduler.h/.cpp) - lives in the QN8066 Arduino library itself
  (installed under Documents/Arduino/libraries/QN8066/src/), not in this sketch's own folder. All
  three sketches in this project (this one, ESP32_WEB_QN8066_NO_DAC, ESP32_WEB_QN8066) #include the
  exact same copy from there - there is nothing left to keep manually in sync between them for any
  of that. What remains here is genuinely serial-specific: the CFG text-line protocol
  (applyConfigLine() below) and Serial-vs-UECP-frame byte routing (handleSerialInput()) - the WiFi
  sketches have their own equivalents (web form fields, the TCP UECP server) instead.

  Because there's no WiFi, there's no NTP either: the system clock stays unset (Group 4A / Clock
  Time is suppressed - see rds_scheduler.cpp's buildNextGroup()) until the first UECP MEC 0x0D
  (Time) message arrives and sets it. From then on Group 4A tracks the system clock as usual.

  Frequency/power/audio parameters (frequency, power, stereo/mono, pre-emphasis, frequency
  deviation, input impedance, buffer gain, soft clip) have no web form to change them any more -
  edit the constants below and re-flash for a permanent change, or override them at runtime with
  a CFG line (see below). All RDS content (PI/PS/RT/PTY/TA/TP/MS/DI/PIN/AF/SLC/PTYN/Time) is set via
  UECP as usual, same MECs as the WiFi variants.

  1. To connect: open a serial connection to this device at 115200 baud and send UECP frames
     (0xFE <SQC> <DSN/PSN elements...> 0xFF), one straight after another. No connection handshake
     is required - frames are parsed as they arrive.
  2. UECP Site/Encoder address filtering: this device's own address defaults to 0/0 (accept every
     frame regardless of address, per the UECP "all sites/all encoders" convention) - override at
     runtime with a CFG line's SITE/ENC keys (see below), or edit uecpOurSite/uecpOurEncoder below
     for a permanent default.
  3. Pre-config line: some UECP sources never send every field (e.g. no MEC 0x01 for PI, or no
     DSN/PSN datasets/PIC assigned), and some values (this device's own DSN/PSN/site/encoder
     identity, and the boot frequency/power/audio parameters) have no UECP MEC of their own at
     all. A single text line - "CFG KEY=VALUE KEY=VALUE ...\n" - sent over the same serial
     connection lets a companion tool (see tcp2serial.py's --config option) push those in once,
     any time before or between UECP frames, without needing a UECP frame or a web form. Keys not
     present in the line are left untouched. A value containing a space must be quoted (e.g.
     PS="MY RADIO") - see nextCfgToken() below; unquoted values (most keys, which never contain a
     space anyway) work exactly as a bare KEY=VALUE token always has. Recognised keys: PI (hex),
     DSN, PSN, SITE, ENC, POWER, FREQ, PTY, STEREO, PREEMPH, DERIV, IMPED, GAIN, SOFTCLIP (all
     decimal except PI), and:
       - PS=text (e.g. PS="MY RADIO") - for a UECP source that never sends its own MEC 0x02.
         Applied exactly like a real MEC 0x02 (staged into psPending, not written to ps[] directly
         - see rds_state.h's psPending/psPendingValid comment).
       - RT=text (e.g. RT="MY RADIOTEXT STRING") - for a UECP source that never sends its own MEC
         0x0A. Replaces the whole RT buffer with this as the sole message 0, same as the WiFi
         sketches' own "rds_rt" web form field (see ESP32_WEB_QN8066.ino's handleUpdate()). Applied
         immediately by default - unlike PS, RT's own A/B flag is what tells a receiver "content
         changed", so an immediate, possibly mid-message switch is normally the spec-compliant way
         RT updates propagate - unless RTBUF=1 (below) has turned on buffered mode, in which case
         this stages instead and waits for a clean swap point.
       - RTBUF=0|1 - "buffer RT" mode (see rds_state.h's rtBufferMode comment). 0 (default) is the
         plain immediate RT replace described above. 1 defers any RT= (or MEC 0x0A clearing
         replace) instead: staged into rtPending until the message currently playing has shown one
         full pass through its own text, then swapped in cleanly by rds_scheduler.cpp's
         buildGroup2A() - no receiver ever sees a message torn mid-transmission, at the cost of the
         new text taking up to one full message cycle to actually appear.
       - ODA=AIDHEX:GROUPNUM[A|B] (e.g. ODA=CD46:8A) - manually maps an ODA AID to a group, both
         for Group 3A to announce as "master data" (see rds_state.h's odaManual[]/
         rds_scheduler.cpp's buildGroup3A()) and, via the same call, as a seed entry in the live
         AID->group directory (oda_directory.h) that incoming MEC 0x46 elements resolve against -
         for UECP sources that only ever send MEC 0x46 ODA data and never their own Group 3A/MEC
         0x40 definitions; repeat the key to configure multiple mappings in one line.
       - SEQ=GROUPNUM[A|B],GROUPNUM[A|B],... (e.g. SEQ=0A,1A,2A,0A,3A,2A) - replaces the whole RDS
         group schedule wholesale, restarting it from position 0 (see rds_state.h's rdsSequence[]
         comment); the same thing UECP MEC 0x16 sets.
     See applyConfigLine() below.
  4. This sketch was written for both the ESP32 Dev Module and ESP32-C3 (see the I2C pin #ifdef
     below) - if you're using a different ESP32 model, check/adjust the pin connections.

  Wire up (same as the WiFi variant's NO_DAC sketch):

  | QN8066 Pin          | ESP32 Dev Module | ESP32-C3 |
  | --------------------|------------------|----------|
  | VCC                 | 3.3V             | 3.3V     |
  | GND                 | GND              | GND      |
  | SDIO / SDA (pin 2)  | GPIO21           | GPIO8    |
  | SCLK (pin 1)        | GPIO22           | GPIO9    |

  GPIO4 (via ledcAttach/ledcWrite in setup()) is NOT optional and is not a PWM amplifier
  controller, despite what earlier comments in this file's ancestry claimed - it generates the
  32768 Hz reference clock the QN8066 module's XCLK pin needs to run at all (matches
  tx.setup()'s xtalDiv=1 argument: xtal_div = round(freq_of_xtal / 32.768kHz), so xtalDiv=1 means
  a 32768 Hz reference is expected). On this board, ESP32 GPIO4 (labelled "SCK" on its pinout) is
  wired directly to the QN8066 module's XCLK pin - there is no separate crystal on the QN8066
  side. One consequence: since the QN8066's entire internal RDS group timing is derived from this
  same ESP32-generated clock, there is exactly one clock domain in this system, not two
  independent free-running ones - see rds_scheduler.cpp's serviceRdsTx(), which relies on that.

  Prototype documentation: https://pu2clr.github.io/QN8066/
  PU2CLR QN8066 API documentation: https://pu2clr.github.io/QN8066/extras/apidoc/html/

  By PU2CLR, Ricardo, Feb 2024.
*/

#include <QN8066.h>
#include <rds_state.h>
#include <uecp_handler.h>
#include <rds_defaults.h>
#include <free_format_groups.h>
#include <oda_directory.h>
#include <rds_scheduler.h>
#include <ebu_charset.h>

// I2C bus pins on ESP32 or ESP32-C3
#ifdef ARDUINO_ESP32C3_DEV
  #define ESP32_I2C_SDA 8
  #define ESP32_I2C_SCL 9
  #warning "ESP32C3 Dev Module"
#else
  #define ESP32_I2C_SDA 21
  #define ESP32_I2C_SCL 22
  #warning "ESP32 Dev Module"
#endif

// --- Fixed boot-time transmitter parameters (no web form in this variant - edit and re-flash) ---
uint16_t currentFrequency     = 1071; // 106.9 MHz (100 kHz steps: 1071 = 107.1 MHz)
uint16_t previousFrequency    = currentFrequency; // tracks the last value actually pushed to the
                                                    // chip, so a CFG FREQ= that repeats the
                                                    // current frequency doesn't re-tune it
uint8_t  currentPower         = 40;
uint8_t  currentStereoMono    = 0;
uint8_t  currentPreEmphasis   = 0;
uint8_t  currentFreqDeviation = 87;
uint8_t  currentRdsDeviation  = 20; // was a hardcoded literal in setup()'s rdsSetFrequencyDeviation() call
uint8_t  currentPilotGain     = 11; // was a hardcoded literal in setup()'s setTxPilotGain() call
uint8_t  currentInputImpedance = 1;
uint8_t  currentBufferGain    = 1;
uint8_t  currentSoftClip      = 0;

// Local Time setup - only used as a fallback Group 4A offset until a UECP MEC 0x0D Time message
// has ever set uecpRdsState.ctOffset (see rds_scheduler.cpp's buildNextGroup()).
const long gmtOffset_sec     = 7200;
const int  daylightOffset_sec = 0;

// Converts the constants above into the same 6-bit format UECP MEC 0x0D and RDS Group 4A's own
// offset/offset_sign bits share (bit5 = sign, bits4-0 = magnitude in half-hours). Passed into
// rdsSchedulerTick() (see loop() below) every call - a deployment/location property this sketch
// owns, not something the shared scheduler should have a single hardcoded value for.
uint8_t computeLocalOffsetByte() {
  long totalOffsetSec     = gmtOffset_sec + daylightOffset_sec;
  long magnitudeHalfHours = labs(totalOffsetSec) / 1800;
  if (magnitudeHalfHours > 0x1F) magnitudeHalfHours = 0x1F; // clamp - the field can't hold more
  uint8_t sign = (totalOffsetSec < 0) ? 1 : 0;
  return (uint8_t)((sign << 5) | (magnitudeHalfHours & 0x1F));
}

QN8066 tx;

// RDS state fed exclusively by UECP frames arriving over Serial. Single source of truth for
// everything rds_scheduler.cpp's buildNextGroup() transmits.
RdsTxState uecpRdsState;

// Per-group free-format queues fed by UECP MEC 0x24/0x30/0x40/0x46 - see free_format_groups.h.
// Independent of uecpRdsState: this is purely additional groups/content layered on top of this
// project's own fixed struct-backed ones (0A/1A/2A/3A-manual/4A/10A), not a replacement for any of it.
FreeFormatPool ffPool;

// Persistent ODA AID -> group directory, learned from live Group 3A traffic - see
// oda_directory.h and uecp_handler.cpp's isGroup3A handling. Outlives ffPool's own Group 3A
// queue entries (which get freed after a one-shot transmission); only a Group 3A "wipe all"
// clears this.
OdaLiveDirectory odaLiveDir;

// Set (to a group index) by uecp_handler.cpp when a MEC 0x46 "immediate" priority message is
// applied - rds_scheduler.cpp's buildNextGroup() checks this before walking uecpRdsState's own
// group sequence and, if set, builds that group next regardless of whose scheduled turn it
// actually is (breaking into the order, the same way Group 4A's own clock-tick check does), then
// clears it back to FF_INDEX_NONE. Only one pending group at a time - if a second "immediate"
// message arrives before the first is served, it simply overwrites this, so the newer one wins
// and the earlier one falls back to being served at its own group's next regular turn (still with
// FF_FLAG_PRIORITY, so still first in line within that group's own queue).
uint8_t pendingImmediateGroupIndex = FF_INDEX_NONE;

// This device's own UECP address (site: 10 bits, encoder: 6 bits). 0/0 = accept every UECP frame
// regardless of its address, per the UECP spec's "all sites"/"all encoders" convention.
uint16_t uecpOurSite    = 0;
uint8_t  uecpOurEncoder = 0;

// Boot-time default group sequence, copied into uecpRdsState.rdsSequence[]/rdsSequenceLen once in
// setup() (see rdsTxStateSetDefaults()'s call site below). Editing these two constants and
// re-flashing changes only the *default* - the live, actually-scheduled copy in uecpRdsState is
// what rds_scheduler.cpp's buildNextGroup() reads, and is independently reconfigurable at runtime
// (UECP MEC 0x16, or the "SEQ" CFG key) without touching either of these.
//
// Entries are real RDS group indices - ffGroupIndex(groupType, versionB), i.e. (groupType<<1)|
// versionB - NOT the old 0/1/2 shorthand this array used before free-format support existed.
// 0=Group 0A (PS), 2=Group 1A (PIN), 4=Group 2A (RT), 20=Group 10A (PTYN) - this project's own
// struct-backed fallback content always exists for those four (see rds_scheduler.cpp's
// buildNextGroup()). 6= Group 3A (ODA) also has struct fallback, but only once at least one
// manual ODA mapping is configured (see buildGroup3A()). Add any other group index (e.g. 22 =
// Group 11A) to reserve it a scheduled slot for UECP MEC 0x24 free-format/ODA content instead -
// buildNextGroup() checks the free-format queue for whatever index is up first regardless of
// which group that is, and only falls back to struct content for 0/2/4/6/20; an added slot with
// nothing queued for it is silently skipped rather than transmitting nothing that cycle, so
// listing extra indices "just in case" costs nothing when they're empty. 16 (Group 8A) is
// reserved here purely so MEC 0x30 TMC content actually gets transmitted - it has no struct
// fallback of its own, so it only ever carries free-format content, but it still needs a slot in
// this sequence to be checked at all (an omitted index is never dequeued, no matter what's queued
// for it).
// Gives ~0.9s full PS cycle and ~2.4s full RT cycle at 11.4 groups/sec (15-slot lap).
const uint8_t RDS_SEQUENCE[]   = {0, 2, 4, 0, 6, 4, 0, 16, 4, 0, 24, 4, 0, 20, 4};
const uint8_t RDS_SEQUENCE_LEN = 15;

// --- Single-connection UECP frame framing state, fed by bytes arriving on Serial (USB) ---
static bool     uecpInPacket = false;
static uint8_t  uecpBuf[UECP_MAX_FRAME];
static uint16_t uecpBufLen   = 0;

// --- CFG line accumulator - only fed bytes that arrive outside a UECP frame (see
// handleSerialInput() below), so plain text config lines and binary UECP frames can share the
// same serial connection without either one needing to escape the other's framing bytes.
#define CFG_LINE_MAX 160
static char    cfgLine[CFG_LINE_MAX];
static uint8_t cfgLineLen = 0;

// Like strtok(str, " "), but a space inside a "..." pair doesn't end the token, and the quotes
// themselves are stripped from the returned token - so a value can carry an embedded literal
// space, e.g. PS="MY RADIO" or RT="MY RADIOTEXT STRING". Unlike strtok, there's no separate "first
// call passes the string, later calls pass nullptr" convention: pass the same cursor (pointing at
// line initially) into every call within one line, and this advances it past whatever it consumed.
// Compacts the token in place as quotes are stripped, exactly like strtok does with delimiters, so
// the returned pointer is still into the original line buffer.
//
// If a quote is opened but never closed before the line ends, returns nullptr (same as genuinely
// running out of tokens) but also sets *unterminated - the caller should treat the rest of the line
// as unparsed rather than silently accept whatever partial token this would otherwise have
// produced (which could otherwise swallow every following KEY=VALUE on the line into one value).
static char* nextCfgToken(char** cursor, bool* unterminated) {
  char* p = *cursor;
  while (*p == ' ') p++;
  if (*p == '\0') return nullptr;
  char* tokenStart = p;
  char* out = p;
  bool inQuotes = false;
  while (*p != '\0' && (inQuotes || *p != ' ')) {
    if (*p == '"') { inQuotes = !inQuotes; p++; continue; }
    *out++ = *p++;
  }
  if (inQuotes) { *unterminated = true; return nullptr; }
  if (*p != '\0') p++;
  *out = '\0';
  *cursor = p;
  return tokenStart;
}

// Applies one "CFG KEY=VALUE KEY=VALUE ...\n" line - a lightweight way to preset values that
// either have no UECP MEC of their own (this device's own DSN/PSN/site/encoder identity, and the
// boot-time frequency/power/audio parameters) or that a particular UECP source might simply never
// send (e.g. no MEC 0x01 for PI). Unrecognised keys and a line that doesn't start with "CFG" are
// ignored; keys omitted from the line leave that setting untouched. line is modified in place
// (nextCfgToken()-style, see above) and must be NUL-terminated.
void applyConfigLine(char* line) {
  char* cursor = line;
  bool  unterminated = false;
  char* tok = nextCfgToken(&cursor, &unterminated);
  if (tok == nullptr || strcmp(tok, "CFG") != 0) return; // not a config line - ignore silently

  bool freqChanged = false;

  while ((tok = nextCfgToken(&cursor, &unterminated)) != nullptr) {
    char* eq = strchr(tok, '=');
    if (eq == nullptr) continue;
    *eq = '\0';
    const char* key = tok;
    const char* val = eq + 1;

    if (strcmp(key, "PI") == 0) {
      uint16_t pi = (uint16_t) strtol(val, nullptr, 16);
      uecpRdsState.pi[0] = (uint8_t)(pi >> 8);
      uecpRdsState.pi[1] = (uint8_t)(pi & 0xFF);
    } else if (strcmp(key, "DSN") == 0) {
      uecpRdsState.dsn = (uint8_t) atoi(val);
    } else if (strcmp(key, "PSN") == 0) {
      uecpRdsState.psn = (uint8_t) atoi(val);
    } else if (strcmp(key, "SITE") == 0) {
      uecpOurSite = (uint16_t) atoi(val) & 0x03FF;
    } else if (strcmp(key, "ENC") == 0) {
      uecpOurEncoder = (uint8_t) atoi(val) & 0x3F;
    } else if (strcmp(key, "PTY") == 0) {
      uecpRdsState.pty = (uint8_t) atoi(val);
    } else if (strcmp(key, "POWER") == 0) {
      currentPower = (uint8_t) atoi(val);
      // setPAC() toggles the chip's SYSTEM1.recal bit, which per the QN8066 library's own comment
      // "resets the state to initial states and recalibrates all blocks" - a real on-chip reset
      // (not an ESP32 reset), which drops RDS transmission until it's re-armed below, and
      // invalidates whatever the scheduler's own buffer/pending-send bookkeeping thought was true.
      tx.setPAC(currentPower);
      rdsSchedulerStart(tx);
      rdsSchedulerReset();
    } else if (strcmp(key, "FREQ") == 0) {
      currentFrequency = (uint16_t) atoi(val);
      freqChanged = true;
    } else if (strcmp(key, "STEREO") == 0) {
      currentStereoMono = (uint8_t) atoi(val);
      tx.setTxMono(currentStereoMono);
    } else if (strcmp(key, "PREEMPH") == 0) {
      currentPreEmphasis = (uint8_t) atoi(val);
      tx.setPreEmphasis(currentPreEmphasis);
    } else if (strcmp(key, "DERIV") == 0) {
      currentFreqDeviation = (uint8_t) atoi(val);
      tx.setTxFrequencyDeviation(currentFreqDeviation);
    } else if (strcmp(key, "IMPED") == 0) {
      currentInputImpedance = (uint8_t) atoi(val);
      tx.setTxInputImpedance(currentInputImpedance);
    } else if (strcmp(key, "GAIN") == 0) {
      currentBufferGain = (uint8_t) atoi(val);
      tx.setTxInputBufferGain(currentBufferGain);
    } else if (strcmp(key, "SOFTCLIP") == 0) {
      currentSoftClip = (uint8_t) atoi(val);
      tx.setTxSoftClippingEnable(currentSoftClip);
    } else if (strcmp(key, "PS") == 0) {
      // For a UECP source that never sends its own MEC 0x02. Quote the value to include an
      // embedded space, e.g. PS="MY RADIO" - nextCfgToken() above already stripped the quotes and
      // left any space they enclosed intact by the time val reaches here.
      uint8_t psEbu[RDS_PS_LEN];
      memset(psEbu, ' ', RDS_PS_LEN); // pad short values with spaces - utf8ToEbu() only writes as
                                        // many bytes as the input actually converts to, it never
                                        // pads the rest of out[] itself
      utf8ToEbu(val, psEbu, RDS_PS_LEN);
      // Staged into psPending exactly like a real MEC 0x02 element (see uecpApplyMec() in
      // uecp_handler.cpp), not written to state.ps[] directly - sendRDS() swaps it in at the next
      // clean psSegment==0 wrap so a receiver never sees old/new characters torn mid-cycle (see
      // rds_state.h's psPending/psPendingValid comment).
      memcpy(uecpRdsState.psPending, psEbu, RDS_PS_LEN);
      uecpRdsState.psPendingValid = true;
      Serial.printf("CFG PS=\"%s\" applied (staged, takes effect within one PS cycle)\n", val);
      continue;
    } else if (strcmp(key, "RT") == 0) {
      // For a UECP source that never sends its own MEC 0x0A. Quote the value to include embedded
      // spaces, e.g. RT="MY RADIOTEXT STRING". A clearing replace of the whole RT buffer, becoming
      // the sole message 0 (any other slots are UECP-only territory) - mirrors the WiFi sketches'
      // own "rds_rt" web form field (see ESP32_WEB_QN8066.ino's handleUpdate()) and
      // uecp_handler.cpp's uecpApplyRt(), including respecting rtBufferMode (see rds_state.h's own
      // comment, and the RTBUF key below): buffered, this stages into rtPending instead of
      // touching the live buffer, and rds_scheduler.cpp's buildGroup2A() swaps it in once the
      // message currently playing has shown a full pass - unbuffered (the default), it's applied
      // immediately, same as always, since RT's own A/B flag is the receiver-facing "content
      // changed" signal and a torn mid-cycle transition is the spec-compliant way it's expected to
      // propagate, unlike PS which has no such flag of its own.
      if (uecpRdsState.rtBufferMode) {
        memset(uecpRdsState.rtPending, 0, sizeof(uecpRdsState.rtPending));
        uecpRdsState.rtPending[0].textLen     = (uint8_t) utf8ToEbu(val, uecpRdsState.rtPending[0].text, RDS_RT_MAX_LEN);
        uecpRdsState.rtPending[0].repeatCount = 0; // infinite
        uecpRdsState.rtPending[0].toggleAB    = true;
        uecpRdsState.rtPendingCount           = 1;
        uecpRdsState.rtPendingValid           = true;
        Serial.printf("CFG RT=\"%s\" applied (staged, takes effect once the current message finishes)\n", val);
      } else {
        memset(uecpRdsState.rt, 0, sizeof(uecpRdsState.rt));
        uecpRdsState.rt[0].textLen     = (uint8_t) utf8ToEbu(val, uecpRdsState.rt[0].text, RDS_RT_MAX_LEN);
        uecpRdsState.rt[0].repeatCount = 0; // infinite
        uecpRdsState.rt[0].toggleAB    = true;
        uecpRdsState.rtCount           = 1;
        uecpRdsState.rtCurrent         = 0;
        uecpRdsState.rtSegment         = 0;
        uecpRdsState.rtRepeatsDone     = 0;
        uecpRdsState.rtABFlag          = !uecpRdsState.rtABFlag;
        Serial.printf("CFG RT=\"%s\" applied\n", val);
      }
      continue;
    } else if (strcmp(key, "RTBUF") == 0) {
      // Toggles rtBufferMode (see rds_state.h's own comment) - 0 (default) is the original instant
      // RT replace, 1 is the deferred "clean" swap. Falls through to the generic "applied" log
      // below rather than doing its own, same as every other plain on/off or numeric key here.
      uecpRdsState.rtBufferMode = (atoi(val) != 0);
    } else if (strcmp(key, "ODA") == 0) {
      // Manually configure one ODA AID -> group mapping (see rds_state.h's odaManual[] and
      // buildGroup3A()), and seed the same mapping into the live directory (oda_directory.h) so
      // incoming MEC 0x46 elements can resolve it too - see the odaLiveDirectorySet() call below.
      // Value is "<AID hex>:<group number><A|B>", e.g. ODA=CD46:8A maps AID 0xCD46 to Group 8A.
      // Appends rather than replaces - repeat this key within one CFG line to configure several
      // mappings at once. Does its own logging (the value's shape doesn't fit the generic
      // "applied" message below), so it skips straight to the next token afterwards.
      char* mutableVal = eq + 1;
      char* colon = strchr(mutableVal, ':');
      if (colon == nullptr) {
        Serial.println("CFG: ODA value missing ':' (expected AIDHEX:GROUPNUM[A|B]), ignored");
      } else {
        *colon = '\0';
        uint16_t   aid       = (uint16_t) strtol(mutableVal, nullptr, 16);
        const char* groupStr = colon + 1;
        uint8_t    groupType = (uint8_t) atoi(groupStr); // stops at the trailing A/B letter
        bool versionB = false;
        for (const char* p = groupStr; *p; p++) {
          if (*p == 'B' || *p == 'b') { versionB = true; break; }
        }
        if (uecpRdsState.odaManualCount < RDS_ODA_MANUAL_MAX) {
          OdaManualEntry& entry = uecpRdsState.odaManual[uecpRdsState.odaManualCount];
          entry.aid        = aid;
          entry.groupIndex = ffGroupIndex(groupType, versionB);
          uecpRdsState.odaManualCount++;
          // Also seed the live directory with this mapping, not just odaManual[] - odaManual[]
          // only feeds buildGroup3A()'s own outgoing Group 3A announcements, but incoming MEC
          // 0x46 elements (mel==5/6/8) resolve their target group via odaLiveDirectoryGet(), not
          // odaManual[]. Without this, a UECP source that only ever sends MEC 0x46 - exactly the
          // case this CFG key exists for, per the comment above - would have every element
          // dropped as "no known group mapping" even with odaManual[] fully configured. A later
          // live MEC 0x24/0x40/0x46 definition for the same AID still overwrites this normally
          // (odaLiveDirectorySet() replaces by AID), and a Group 3A "wipe all" still clears it.
          odaLiveDirectorySet(odaLiveDir, entry.aid, entry.groupIndex);
          Serial.printf("CFG ODA=0x%04X:%u%c applied (%u manual mapping(s) configured)\n",
                        aid, groupType, versionB ? 'B' : 'A', uecpRdsState.odaManualCount);
        } else {
          Serial.println("CFG: ODA manual mapping table full, entry ignored");
        }
      }
      continue;
    } else if (strcmp(key, "SEQ") == 0) {
      // Reconfigures the RDS group sequence wholesale (see rds_state.h's rdsSequence[]/
      // rdsSequenceLen/rdsSeqPos comment and rds_scheduler.cpp's buildNextGroup() dispatch) - the
      // same thing UECP MEC 0x16 sets, just from a CFG line instead. Value is a comma-separated
      // list of "<group number><A|B>" entries (same mini-format as ODA='s own group specifier),
      // e.g. SEQ=0A,1A,2A,0A,3A,2A - replaces the whole sequence and restarts it from position 0.
      // Capped at RDS_SEQUENCE_MAX_LEN entries; extras are silently dropped, same as MEC 0x16's
      // own spec-level LEN cap. Does its own logging, so it skips straight to the next token
      // afterwards.
      char*   p      = eq + 1;
      uint8_t newLen = 0;
      while (*p != '\0' && newLen < RDS_SEQUENCE_MAX_LEN) {
        char* comma = strchr(p, ',');
        if (comma != nullptr) *comma = '\0';
        uint8_t groupType = (uint8_t) atoi(p); // stops at the trailing A/B letter
        bool versionB = false;
        for (const char* q = p; *q; q++) {
          if (*q == 'B' || *q == 'b') { versionB = true; break; }
        }
        uecpRdsState.rdsSequence[newLen++] = ffGroupIndex(groupType, versionB);
        if (comma == nullptr) break;
        p = comma + 1;
      }
      if (newLen == 0) {
        Serial.println("CFG: SEQ value had no valid entries, ignored");
      } else {
        uecpRdsState.rdsSequenceLen = newLen;
        uecpRdsState.rdsSeqPos      = 0; // restart from the new sequence's own beginning
        Serial.printf("CFG SEQ applied (%u entries)\n", newLen);
      }
      continue;
    } else {
      Serial.printf("CFG: unrecognised key '%s', ignored\n", key);
      continue;
    }
    Serial.printf("CFG %s=%s applied\n", key, val);
  }

  if (unterminated) {
    Serial.println("CFG: unterminated quote, rest of line ignored");
  }

  // Only re-tune if FREQ actually asked for a different frequency than what's already live -
  // same guard the old web-form handler used, avoiding a needless re-tune glitch.
  if (freqChanged && currentFrequency != previousFrequency) {
    tx.setTX(currentFrequency);
    previousFrequency = currentFrequency;
  }
}

// Reads every byte currently available on Serial and routes it to whichever parser owns it: a
// UECP frame (0xFE ... 0xFF, handled byte-for-byte the same way the WiFi variants' TCP server
// does) or, for anything arriving outside a frame, a CFG text line (see applyConfigLine() above).
// Arriving inside a UECP frame always wins - 0xFE immediately abandons any partial CFG line, so a
// station's binary UECP traffic can never be misread as config text.
void handleSerialInput() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    if (b == 0xFE) {
      uecpInPacket = true;
      uecpBufLen   = 0;
      cfgLineLen   = 0; // abandon any partial CFG line - a UECP frame is starting
    } else if (b == 0xFF && uecpInPacket) {
      uecpInPacket = false;
      processUecpFrame(uecpBuf, uecpBufLen, "USB", uecpRdsState, ffPool, odaLiveDir,
                       pendingImmediateGroupIndex, uecpOurSite, uecpOurEncoder);
      uecpBufLen = 0;
    } else if (uecpInPacket) {
      if (uecpBufLen < UECP_MAX_FRAME) {
        uecpBuf[uecpBufLen++] = b;
      } else {
        // Frame too large - discard and resync
        uecpInPacket = false;
        uecpBufLen   = 0;
      }
    } else if (b == '\n' || b == '\r') {
      if (cfgLineLen > 0) {
        cfgLine[cfgLineLen] = '\0';
        applyConfigLine(cfgLine);
        cfgLineLen = 0;
      }
    } else if (cfgLineLen < CFG_LINE_MAX - 1) {
      cfgLine[cfgLineLen++] = (char)b;
    } else {
      cfgLineLen = 0; // line too long - drop and resync on the next newline
    }
  }
}

void setup() {
  Serial.begin(115200);

  rdsTxStateSetDefaults(uecpRdsState);
  // Seed the live, runtime-mutable group sequence from this sketch's own boot default - see
  // RDS_SEQUENCE's own comment above. rdsTxStateSetDefaults() itself leaves these zeroed (it has
  // no sketch-specific sequence of its own to default to), so this is the .ino's own job, same as
  // ffPoolInit()/odaLiveDirectoryInit() just below.
  memcpy(uecpRdsState.rdsSequence, RDS_SEQUENCE, RDS_SEQUENCE_LEN);
  uecpRdsState.rdsSequenceLen = RDS_SEQUENCE_LEN;
  uecpRdsState.rdsSeqPos      = 0;
  ffPoolInit(ffPool);
  odaLiveDirectoryInit(odaLiveDir);

  ledcAttach(4, 32768, 1);
  ledcWrite(4, 1);
  // The line below may be necessary to setup I2C pins on ESP32
  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL);

  tx.setup(
    1,
    currentStereoMono,
    false,
    0,
    1
  );
  tx.setTX(currentFrequency);
  delay(500);
  // Push the remaining boot defaults to the chip - none of these apply themselves at startup
  // otherwise.
  tx.setTxFrequencyDeviation(currentFreqDeviation);
  tx.setTxPilotGain(currentPilotGain);
  tx.rdsSetFrequencyDeviation(currentRdsDeviation);
  tx.setPAC(currentPower);
  tx.setTxInputImpedance(currentInputImpedance);
  tx.setTxInputBufferGain(currentBufferGain);
  tx.setPreEmphasis(currentPreEmphasis);
  tx.setTxSoftClippingEnable(currentSoftClip);
  rdsSchedulerStart(tx);

  Serial.println("Transmitting (analogue audio in). Waiting for UECP frames on Serial (115200 baud)...");
}

void loop() {
  handleSerialInput();
  rdsSchedulerApplyPendingTime(uecpRdsState);
  rdsSchedulerTick(tx, uecpRdsState, ffPool, pendingImmediateGroupIndex, computeLocalOffsetByte());
}
