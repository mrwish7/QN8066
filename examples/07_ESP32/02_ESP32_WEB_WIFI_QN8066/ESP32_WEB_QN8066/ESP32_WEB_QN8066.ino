/*
  ESP32 Dev Modeule version - NO_DAC variant.

  This variant is for setups feeding the QN8066 from an external analogue audio source (line-in),
  not from an internally decoded/streamed digital source - so it drops the ESP32-audioI2S
  streaming/I2S-DAC path entirely (no Audio.h, no I2S pins, no stream URL/volume web fields). RDS
  is still fully driven by UECP, but with only one UECP source now: the TCP UECP server on port
  8067 (handleUecpServer() below) - there is no audio-embedded (AAC-DSE/MP2-ancillary) UECP path
  here, since there's no audio decoder in this sketch to extract it from.

1. This sketch uses HTML form to get and process commands from external browser applications to configure the
   FM transmitter (see the Python example `esp32_qn8066.py`).
2. It also utilizes the internal Real Time Clock (RTC) of the ESP32 to send RDS messages with
   this information.
3. Sets the Date and Time based on your network/Internet information.
4. You need to modify the sketch to set the local time for your location.
5. To connect to your Wi-Fi network, you must provide the SSID and password of your router.
Otherwise, this application will not function.
6. This sketch was compiled and tested on the ESP32 Der Module.
If you are using a different ESP32 model,
   consider reviewing the pin connections.
7. In the Arduino IDE, during application execution, check the Serial Monitor for the IP address
   assigned to the ESP32 by your router's DHCP service.
8. RDS message updates such as PTY, PS, RT, and Time are not executed immediately.
This may depend on
   the receiver's update timing as well as the distribution of each message's timing programmed in this sketch.
ESP32 Dev Module Wire up

  | Device name               |
QN8066 Pin           |  ESP32 Dev Module |
  | --------------------------|
-------------------- | ----------------- |
  | QN8066                    |
|                   | 
  |                           | VCC                  |
3.3V         |
  |                           |
GND                  |
GND          |    
  |                           |
SDIO / SDA (pin 2)   |      GPIO21 [1]   |
  |                           |
SCLK (pin 1)         |      GPIO22 [1]   |
  | --------------------------| ---------------------|
----------------- |
  | PWM Power Controller [2]  |                      |                   |
  |                           |                      |
GPIO12       | 

  1. It can change if you are not using the ESP32 Dev Module.
Check you ESP32 board pinout 
  2. A suggestion if you intend to use PWM to control the RF output power of an amplifier.
Prototype documentation: https://pu2clr.github.io/QN8066/
  PU2CLR QN8066 API documentation: https://pu2clr.github.io/QN8066/extras/apidoc/html/

  ESP32 Internal RTC: https://github.com/fbiego/ESP32Time

  By PU2CLR, Ricardo,  Feb  2024.
*/


#include <WiFi.h>
#include <WebServer.h>
#include <time.h>    // Using internal RTC of ESP32
#include <QN8066.h>
#include <rds_state.h>
#include <uecp_handler.h>
#include <rds_defaults.h>
#include <ebu_charset.h>
#include <free_format_groups.h>
#include <oda_directory.h>
#include <rds_scheduler.h>
#include <stdarg.h>   // for bufAppend()'s va_list

// I2C bus pin on ESP32 or ESP32C3
#ifdef ARDUINO_ESP32C3_DEV
  #define ESP32_I2C_SDA 8     // GPIO4
  #define ESP32_I2C_SCL 9     // GPIO5 
  #warning "ESP32C3 Dev Module"
#else 
  #define ESP32_I2C_SDA 8    // GPI21
  #define ESP32_I2C_SCL 9    // GPI22 
  #warning "ESP32 Dev Module"
#endif

// Clock Time (Group 4A) is sent exactly once per UTC minute, aligned to real wall-clock :00
// boundaries - see buildNextGroup()'s Group 4A branch. No fixed millis()-based interval is used any more.

uint16_t currentFrequency = 1061; // 106.9 MHz
uint16_t previousFrequency = currentFrequency;

uint8_t currentPower = 28;
uint8_t currentStereoMono     = 0;
uint8_t currentPreEmphasis    = 0;
uint8_t currentFreqDeviation = 83;
uint8_t currentRdsDeviation = 21;
uint8_t currentPilotGain = 11;
uint8_t currentInputImpedance = 1;
uint8_t currentBufferGain     = 1;
uint8_t currentSoftClip       = 0;
// Wi-Fi setup
const char* ssid = "ssid";
// Change to your WIFI SSID
const char* password = "password";
// Change to your password
const char* hostname = "qn8066";
// Network (DHCP/DNS) hostname this device registers as

// --- Boot-time RDS station-identity defaults - edit these and re-flash to set a station's identity
// the moment the board boots, with no web form or UECP frame needed first. Applied in setup(),
// right after rdsEncoderStateSetDefaults(uecpRdsState), overriding the generic placeholder values
// that shared library call sets (rds_defaults.cpp), to every data set's main service alike (DSN
// 1-6, PSN 1). Still fully reconfigurable afterwards at runtime: the web form's RDS fields, or a
// UECP frame (MEC 0x01=PI, 0x02=PS, 0x0A=RT, 0x07=PTY, 0x03=TA/TP, 0x04=DI, 0x05=MS) - but a
// reset or power cycle always comes back to these.
uint16_t rdsDefaultPI    = 0xCC66;     // placeholder - set your station's real assigned PI code here
const char* rdsDefaultPS = "*QN8066*"; // up to 8 chars; shorter values are space-padded
const char* rdsDefaultRT = "";         // up to 64 chars; "" means no boot RT message at all -
                                         // rds_scheduler.cpp's buildNextGroup() then won't build
                                         // Group 2A until a real one arrives
uint8_t  rdsDefaultPTY   = 0;          // Programme Type code (0 = undefined/none)
bool     rdsDefaultTA    = false;      // Traffic Announcement
bool     rdsDefaultTP    = true;       // Traffic Programme
bool     rdsDefaultMusic = true;       // Music/Speech: true = Music, false = Speech
bool     rdsDefaultStereo         = true;  // RDS DI Mono/Stereo flag - independent of
                                             // currentStereoMono above, which is the chip's own
                                             // actual audio encoding, not this RDS-advertised flag
bool     rdsDefaultArtificialHead = false; // RDS DI Artificial Head flag
bool     rdsDefaultCompressed     = false; // RDS DI Compressed flag
bool     rdsDefaultDynamicPTY     = false; // RDS DI Dynamic PTY flag

// Local Time setup
const long gmtOffset_sec = 7200;
// Brasilia local date and time offset (-3h)
const int daylightOffset_sec = 0;
// There is some confusion about this here in Brazil.
// As of now, there is no daylight saving time in effect in Brazil.

// Converts the constants above into the same 6-bit format UECP MEC 0x0D and RDS Group 4A's own
// offset/offset_sign bits share (bit5 = sign, bits4-0 = magnitude in half-hours) - used as the
// Group 4A offset whenever no UECP Time message has ever set uecpRdsState.ctOffset (see
// buildNextGroup()'s Group 4A branch).
uint8_t computeLocalOffsetByte() {
  long totalOffsetSec     = gmtOffset_sec + daylightOffset_sec;
  long magnitudeHalfHours = labs(totalOffsetSec) / 1800;
  if (magnitudeHalfHours > 0x1F) magnitudeHalfHours = 0x1F; // clamp - the field can't hold more
  uint8_t sign = (totalOffsetSec < 0) ? 1 : 0;
  return (uint8_t)((sign << 5) | (magnitudeHalfHours & 0x1F));
}


// Web server
WebServer server(80);

// UECP TCP server (UECP_MAX_FRAME is defined in uecp_handler.h) - the only source of UECP frames
// in this NO_DAC variant (no audio decoder here to carry an embedded AAC-DSE/MP2-ancillary path).
#define UECP_TCP_PORT    8067
#define UECP_MAX_CLIENTS 4
WiFiServer uecp_server(UECP_TCP_PORT);

struct UecpClient {
  WiFiClient client;
  bool inPacket;
  uint8_t buf[UECP_MAX_FRAME];
  uint16_t bufLen;
};
UecpClient uecp_clients[UECP_MAX_CLIENTS];

QN8066 tx;

bool rawGroupMode = false;

// RDS state fed by UECP frames from the TCP server below and by the web interface. This is the
// single source of truth for everything buildNextGroup() transmits.
RdsEncoderState uecpRdsState;

// Persistent ODA AID -> group directory, learned from live Group 3A traffic - see
// oda_directory.h and uecp_handler.cpp's isGroup3A handling. Outlives ffPool's own Group 3A
// queue entries (which get freed after a one-shot transmission); only a Group 3A "wipe all"
// clears this.
OdaLiveDirectory odaLiveDir;

// Per-group free-format queues fed by UECP MEC 0x24/0x30/0x40/0x46 - see free_format_groups.h.
// Independent of uecpRdsState: this is purely additional groups/content layered on top of this
// project's own fixed struct-backed ones (0A/1A/2A/3A/4A/10A), not a replacement for any of it.
FreeFormatPool ffPool;

// Set (to a group index) by uecp_handler.cpp when a MEC 0x46 "immediate" priority message is
// applied - see buildNextGroup()'s own check of this below. FF_INDEX_NONE (its usual value) means
// nothing is pending.
uint8_t pendingImmediateGroupIndex = FF_INDEX_NONE;

// This device's own UECP address(es) - see UecpAddressConfig's own comment in uecp_handler.h for
// its default (a single 0/0 entry = accept every UECP frame regardless of its address) and how the
// web interface's UECP Site/Encoder Address fields populate it. The UECP spec allows an encoder
// more than one site address and more than one encoder address, up to
// UECP_MAX_SITE_ADDRESSES/UECP_MAX_ENCODER_ADDRESSES (4/8) each - each field takes a comma-
// separated hex list (e.g. "01A,02B"), replacing the whole respective list each time it's submitted.
UecpAddressConfig uecpOurAddress;

// Group scheduler state. The sequence position itself (rdsSeqPos) now lives in uecpRdsState
// (rds_state.h) alongside the sequence it walks, not here - see that struct's rdsSequence[]/
// rdsSequenceLen/rdsSeqPos comment - so UECP MEC 0x16 and the web form's "Group Sequence" field
// (uecp_handler.cpp/handleUpdate()'s "rds_seq" branch respectively) can reconfigure both without
// any extra plumbing back into the .ino.
time_t lastCtMinuteSent = -1;
// epoch-minute index (unix time / 60) of the last Clock Time group actually sent; -1 = never -
// latches so buildNextGroup()'s per-call :00 check can't fire twice for the same minute

// Boot-time default group sequence, copied into every data set's rdsSequence[]/rdsSequenceLen once
// in setup() (see rdsEncoderStateSetDefaults()'s call site below). Editing these two constants and
// re-flashing changes only the *default* - the live, actually-scheduled copy in uecpRdsState is
// what buildNextGroup() reads, and is independently reconfigurable at runtime (UECP MEC 0x16, or
// the web form's "Group Sequence" field) without touching either of these.
//
// Entries are real RDS group indices - ffGroupIndex(groupType, versionB), i.e. (groupType<<1)|
// versionB - NOT the old 0/1/2 shorthand this array used before free-format support existed.
// 0=Group 0A (PS), 2=Group 1A (PIN), 4=Group 2A (RT), 20=Group 10A (PTYN) - this project's own
// struct-backed fallback content always exists for those four (see buildNextGroup() below). 6=
// Group 3A (ODA) also has struct fallback, but only once at least one manual ODA mapping is
// configured (see buildGroup3A()). Add any other group index (e.g. 22 = Group 11A) to reserve it
// a scheduled slot for UECP MEC 0x24 free-format/ODA content instead - buildNextGroup() checks the
// free-format queue for whatever index is up first regardless of which group that is, and only
// falls back to struct content for 0/2/4/6/20; an added slot with nothing queued for it is
// silently skipped (see buildNextGroup()'s retry loop) rather than transmitting nothing that
// cycle, so listing extra indices "just in case" costs nothing when they're empty. 16 (Group 8A)
// is reserved here purely so MEC 0x30 TMC content actually gets transmitted - it has no struct
// fallback of its own, so it only ever carries free-format content, but it still needs a slot in
// this sequence to be checked at all (an omitted index is never dequeued, no matter what's queued
// for it).
const uint8_t RDS_SEQUENCE[]   = {0, 4, 0, 4, 0, 4, 0, 4, 0, 6, 0, 4, 0, 24, 0, 4, 0, 2, 0, 16, 0, 20};
const uint8_t RDS_SEQUENCE_LEN = 22;
#include "index_html.h"

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// Escapes '"' and '\' for JSON string embedding, writing into `out` (size `outSize`, including
// the NUL terminator). Truncates (never overflows) if `out` is too small for the escaped result.
void jsonEscape(const char* s, char* out, size_t outSize) {
  size_t o = 0;
  for (size_t i = 0; s[i] && o + 1 < outSize; i++) {
    char c = s[i];
    if (c == '"' || c == '\\') {
      if (o + 2 >= outSize) break; // no room left for the escape pair
      out[o++] = '\\';
    }
    out[o++] = c;
  }
  out[o] = '\0';
}

// Appends a printf-style formatted fragment to a fixed buffer, advancing *pos - used instead of
// repeated String "+=" concatenation (each of which can trigger a heap realloc+copy) so
// handleStatus()/handleFormSubmit() never touch the heap while building their response. Safe
// against truncation: never writes past bufSize, and clamps *pos so a later call's
// "bufSize - *pos" can't underflow even if an earlier fragment got truncated.
static void bufAppend(char* buf, size_t bufSize, size_t* pos, const char* fmt, ...) {
  if (*pos >= bufSize) return;
  va_list args;
  va_start(args, fmt);
  int written = vsnprintf(buf + *pos, bufSize - *pos, fmt, args);
  va_end(args);
  if (written > 0) {
    *pos += (size_t)written;
    if (*pos > bufSize) *pos = bufSize; // clamp - vsnprintf reports the untruncated length
  }
}

void handleStatus() {
  const RdsMainService& main = rdsActiveMain(uecpRdsState);
  char piBuf[5];
  snprintf(piBuf, sizeof(piBuf), "%02X%02X", main.pi[0], main.pi[1]);
  char pinBuf[5];
  snprintf(pinBuf, sizeof(pinBuf), "%02X%02X", main.pin[0], main.pin[1]);
  // Comma-separated hex lists (e.g. "01A,02B") - the display counterpart of parseUecpAddressList()
  // (uecp_handler.h), which the "uecp_site"/"uecp_enc" form fields parse back on submit.
  char   uecpSiteBuf[UECP_MAX_SITE_ADDRESSES * 4 + 1] = "";
  size_t uecpSitePos = 0;
  for (uint8_t i = 0; i < uecpOurAddress.siteCount; i++) {
    int written = snprintf(uecpSiteBuf + uecpSitePos, sizeof(uecpSiteBuf) - uecpSitePos,
                           i == 0 ? "%03X" : ",%03X", uecpOurAddress.sites[i]);
    if (written > 0) uecpSitePos += (size_t) written;
  }
  char   uecpEncBuf[UECP_MAX_ENCODER_ADDRESSES * 3 + 1] = "";
  size_t uecpEncPos = 0;
  for (uint8_t i = 0; i < uecpOurAddress.encoderCount; i++) {
    int written = snprintf(uecpEncBuf + uecpEncPos, sizeof(uecpEncBuf) - uecpEncPos,
                           i == 0 ? "%02X" : ",%02X", uecpOurAddress.encoders[i]);
    if (written > 0) uecpEncPos += (size_t) written;
  }
  char psUtf8[RDS_PS_LEN * 3 + 1];
  ebuToUtf8(main.ps, RDS_PS_LEN, psUtf8, sizeof(psUtf8));
  char rtUtf8[RDS_RT_MAX_LEN * 3 + 1];
  ebuToUtf8(main.rt[0].text, main.rt[0].textLen, rtUtf8, sizeof(rtUtf8));

  char psEscaped[sizeof(psUtf8) * 2];
  jsonEscape(psUtf8, psEscaped, sizeof(psEscaped));
  char rtEscaped[sizeof(rtUtf8) * 2];
  jsonEscape(rtUtf8, rtEscaped, sizeof(rtEscaped));

  static char json[1536]; // static: keeps this off the (much more limited) task stack
  size_t pos = 0;
  bufAppend(json, sizeof(json), &pos, "{\"frequency\":%.1f", (float)currentFrequency / 10.0f);
  bufAppend(json, sizeof(json), &pos, ",\"power\":%u", currentPower);
  bufAppend(json, sizeof(json), &pos, ",\"stereo_mono\":%u", currentStereoMono);
  bufAppend(json, sizeof(json), &pos, ",\"pre_emphasis\":%u", currentPreEmphasis);
  bufAppend(json, sizeof(json), &pos, ",\"frequency_deviation\":%u", currentFreqDeviation);
  bufAppend(json, sizeof(json), &pos, ",\"input_impedance\":%u", currentInputImpedance);
  bufAppend(json, sizeof(json), &pos, ",\"buffer_gain\":%u", currentBufferGain);
  bufAppend(json, sizeof(json), &pos, ",\"soft_clip\":%u", currentSoftClip);
  bufAppend(json, sizeof(json), &pos, ",\"raw_mode\":%u", rawGroupMode ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_pi\":\"%s\"", piBuf);
  bufAppend(json, sizeof(json), &pos, ",\"rds_pty\":%u", main.pty);
  bufAppend(json, sizeof(json), &pos, ",\"rds_ps\":\"%s\"", psEscaped);
  bufAppend(json, sizeof(json), &pos, ",\"rds_rt\":\"%s\"", rtEscaped);
  bufAppend(json, sizeof(json), &pos, ",\"rds_dsn\":%u", uecpRdsState.activeDsn);
  bufAppend(json, sizeof(json), &pos, ",\"rds_psn\":%u", main.psnNumber);
  bufAppend(json, sizeof(json), &pos, ",\"uecp_site\":\"%s\"", uecpSiteBuf);
  bufAppend(json, sizeof(json), &pos, ",\"uecp_enc\":\"%s\"", uecpEncBuf);
  bufAppend(json, sizeof(json), &pos, ",\"rds_pin\":\"%s\"", pinBuf);
  bufAppend(json, sizeof(json), &pos, ",\"rds_ta\":%u", (main.tatp & 0x01) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_tp\":%u", (main.tatp & 0x02) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_ms\":%u", (main.ms & 0x01) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_di_stereo\":%u", (main.diPtyi & 0x01) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_di_artifhead\":%u", (main.diPtyi & 0x02) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_di_compressed\":%u", (main.diPtyi & 0x04) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_ptyi\":%u", (main.diPtyi & 0x08) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_rt_repeat\":%u", main.rt[0].repeatCount);
  bufAppend(json, sizeof(json), &pos, "}");
  server.send(200, "application/json", json);
}

void handleUpdate() {
  // Every RDS field below edits the active data set (the one on air) - see "rds_dsn".
  RdsDataSet&     ds   = rdsActiveDataSet(uecpRdsState);
  RdsMainService& main = ds.main;
  char field[24];
  server.argName(0).toCharArray(field, sizeof(field));
  // Processa e aplica o valor do campo correspondente

  if (strcmp(field, "frequency") == 0) {
    char frequency[16];
    server.arg("frequency").toCharArray(frequency, sizeof(frequency));
    currentFrequency = (uint16_t) (atof(frequency) * 10);
    if (currentFrequency != previousFrequency) {
      tx.setTX(currentFrequency);
      previousFrequency = currentFrequency;
    }
    Serial.printf("Frequency updated to: %s MHz\n", frequency);
  } else if (strcmp(field, "power") == 0) {
    char power[8];
    server.arg("power").toCharArray(power, sizeof(power));
    currentPower = (uint8_t) atoi(power);
    // setPAC() toggles the chip's SYSTEM1.recal bit ("resets the state to initial states and
    // recalibrates all blocks", per the QN8066 library's own comment) - a real on-chip reset that
    // drops RDS transmission until it's re-armed, and invalidates the scheduler's own buffer/
    // pending-send bookkeeping.
    tx.setPAC(currentPower);
    rdsSchedulerStart(tx);
    rdsSchedulerReset();
    Serial.printf("Power updated to: %u\n", currentPower);
  } else if (strcmp(field, "rds_pi") == 0) {
    char rds_pi[8];
    server.arg("rds_pi").toCharArray(rds_pi, sizeof(rds_pi));
    uint16_t pi = (uint16_t) strtol(rds_pi, NULL, 16);
    main.pi[0] = (uint8_t)(pi >> 8);
    main.pi[1] = (uint8_t)(pi & 0xFF);
    Serial.printf("RDS PI updated to: 0x%04X\n", pi);
  } else if (strcmp(field, "rds_pty") == 0) {
    main.pty = (uint8_t) server.arg("rds_pty").toInt();
    Serial.printf("RDS PTY updated to: %u\n", main.pty);
  } else if (strcmp(field, "rds_ps") == 0) {
    char rds_ps[40];
    server.arg("rds_ps").toCharArray(rds_ps, sizeof(rds_ps));
    size_t len = strlen(rds_ps);
    while (len < 8 && len + 1 < sizeof(rds_ps)) rds_ps[len++] = ' ';
    rds_ps[len] = '\0';
    // Staged, not applied to ps[] directly - see buildNextGroup()'s psSegment==0 wrap check, same
    // clean-swap guarantee as a UECP MEC 0x02 update.
    utf8ToEbu(rds_ps, main.psPending, RDS_PS_LEN);
    main.psPendingValid = true;
    Serial.printf("RDS PS updated to: %s\n", rds_ps);
  } else if (strcmp(field, "rds_rt") == 0) {
    // Small headroom over RDS_RT_MAX_LEN for a stray/oversized submission - toCharArray() below
    // truncates safely to sizeof-1 regardless, matching the old .substring(0,64) behavior.
    char rds_rt[RDS_RT_MAX_LEN + 8];
    server.arg("rds_rt").toCharArray(rds_rt, sizeof(rds_rt));
    // The web interface only ever controls a single RT message: submitting new text clears the
    // whole buffer (any other slots are UECP-only territory) and becomes the sole message 0.
    memset(main.rt, 0, sizeof(main.rt));
    main.rt[0].textLen     = (uint8_t) utf8ToEbu(rds_rt, main.rt[0].text, RDS_RT_MAX_LEN);
    main.rt[0].repeatCount = 0;  // infinite
    main.rt[0].toggleAB    = true;
    main.rtCount           = 1;
    uecpRdsState.rtCurrent         = 0;
    uecpRdsState.rtSegment         = 0;
    uecpRdsState.rtRepeatsDone     = 0;
    main.rtABFlag          = !main.rtABFlag;
    Serial.printf("RDS RT updated to: %s\n", rds_rt);
  } else if (strcmp(field, "rds_rt_repeat") == 0) {
    main.rt[0].repeatCount = (uint8_t) server.arg("rds_rt_repeat").toInt();
    Serial.printf("RDS RT repeat count updated to: %u\n", main.rt[0].repeatCount);
  } else if (strcmp(field, "uecp_site") == 0) {
    // Comma-separated hex list (e.g. "01A,02B") - replaces the whole site address list wholesale,
    // same "replace, don't append" convention as the "Group Sequence" field below; up to
    // UECP_MAX_SITE_ADDRESSES (4) entries, extras silently dropped. A single "0" (the default)
    // restores "accept every UECP frame regardless of its site address" - see UecpAddressConfig's
    // own comment in uecp_handler.h and uecpAddressMatches() in uecp_handler.cpp for the exact rule
    // once more than one address is configured.
    char uecp_site[24];
    server.arg("uecp_site").toCharArray(uecp_site, sizeof(uecp_site));
    uint16_t parsed[UECP_MAX_SITE_ADDRESSES];
    uint8_t  n = parseUecpAddressList(uecp_site, parsed, UECP_MAX_SITE_ADDRESSES, 0x03FF, 16);
    if (n > 0) {
      memcpy(uecpOurAddress.sites, parsed, n * sizeof(uint16_t));
      uecpOurAddress.siteCount = n;
    }
    Serial.printf("UECP Site Address(es) updated to: %s (%u entries)\n", uecp_site, n);
  } else if (strcmp(field, "uecp_enc") == 0) {
    // Comma-separated hex list (e.g. "00,01") - same "replace wholesale" convention as uecp_site
    // above; up to UECP_MAX_ENCODER_ADDRESSES (8) entries.
    char uecp_enc[32];
    server.arg("uecp_enc").toCharArray(uecp_enc, sizeof(uecp_enc));
    uint16_t parsed[UECP_MAX_ENCODER_ADDRESSES];
    uint8_t  n = parseUecpAddressList(uecp_enc, parsed, UECP_MAX_ENCODER_ADDRESSES, 0x3F, 16);
    if (n > 0) {
      for (uint8_t i = 0; i < n; i++) uecpOurAddress.encoders[i] = (uint8_t) parsed[i];
      uecpOurAddress.encoderCount = n;
    }
    Serial.printf("UECP Encoder Address(es) updated to: %s (%u entries)\n", uecp_enc, n);
  } else if (strcmp(field, "oda_add") == 0) {
    // Manually configure one ODA AID -> group mapping (see rds_state.h's odaManual[] and
    // buildGroup3A()), and seed the same mapping into the live directory (oda_directory.h) so
    // incoming MEC 0x46 elements can resolve it too - see the odaLiveDirectorySet() call below.
    // Value is "<AID hex>:<group number><A|B>", e.g. CD46:8A maps AID 0xCD46 to Group 8A. Appends
    // rather than replaces - submitting this field again adds another mapping, the web counterpart
    // to the serial sketch's repeatable "ODA=" CFG line key.
    char odaVal[24];
    server.arg("oda_add").toCharArray(odaVal, sizeof(odaVal));
    char* colon = strchr(odaVal, ':');
    if (colon == nullptr) {
      Serial.println("ODA mapping value missing ':' (expected AIDHEX:GROUPNUM[A|B]), ignored");
    } else {
      *colon = '\0';
      uint16_t    aid       = (uint16_t) strtol(odaVal, nullptr, 16);
      const char* groupStr  = colon + 1;
      uint8_t     groupType = (uint8_t) atoi(groupStr); // stops at the trailing A/B letter
      bool versionB = false;
      for (const char* p = groupStr; *p; p++) {
        if (*p == 'B' || *p == 'b') { versionB = true; break; }
      }
      if (uecpRdsState.odaManualCount < RDS_ODA_MANUAL_MAX) {
        OdaManualEntry& entry = uecpRdsState.odaManual[uecpRdsState.odaManualCount];
        entry.aid        = aid;
        entry.groupIndex = ffGroupIndex(groupType, versionB);
        uecpRdsState.odaManualCount++;
        // Also seed the live directory with this mapping, not just odaManual[] - odaManual[] only
        // feeds buildGroup3A()'s own outgoing Group 3A announcements, but incoming MEC 0x46
        // elements (mel==5/6/8) resolve their target group via odaLiveDirectoryGet(), not
        // odaManual[]. A later live MEC 0x24/0x40/0x46 definition for the same AID still overwrites
        // this normally, and a Group 3A "wipe all" still clears it.
        odaLiveDirectorySet(odaLiveDir, entry.aid, entry.groupIndex);
        Serial.printf("ODA mapping added: 0x%04X -> Group %u%c (%u manual mapping(s) configured)\n",
                      aid, groupType, versionB ? 'B' : 'A', uecpRdsState.odaManualCount);
      } else {
        Serial.println("ODA manual mapping table full, entry ignored");
      }
    }
  } else if (strcmp(field, "rds_seq") == 0) {
    // Reconfigures the RDS group sequence wholesale (see rds_state.h's rdsSequence[]/
    // rdsSequenceLen/rdsSeqPos comment and buildNextGroup()'s own dispatch) - the web counterpart
    // to the serial sketch's "SEQ" CFG line key, and to UECP MEC 0x16, both of which set the exact
    // same state. Value is a comma-separated list of "<group number><A|B>" entries (same mini-
    // format as the ODA mapping field above), e.g. 0A,1A,2A,0A,3A,2A - replaces the whole sequence
    // and restarts it from position 0. Capped at RDS_SEQUENCE_MAX_LEN entries; extras are
    // silently dropped, same as MEC 0x16's own spec-level LEN cap.
    char seqVal[512]; // generous headroom for "<num><A|B>," per entry - more than enough for any
                        // sequence typed by hand; a UECP MEC 0x16 frame remains the way to push a
                        // sequence long enough to actually need the full RDS_SEQUENCE_MAX_LEN
    server.arg("rds_seq").toCharArray(seqVal, sizeof(seqVal));
    char*   p      = seqVal;
    uint8_t newLen = 0;
    while (*p != '\0' && newLen < RDS_SEQUENCE_MAX_LEN) {
      char* comma = strchr(p, ',');
      if (comma != nullptr) *comma = '\0';
      uint8_t groupType = (uint8_t) atoi(p); // stops at the trailing A/B letter
      bool versionB = false;
      for (const char* q = p; *q; q++) {
        if (*q == 'B' || *q == 'b') { versionB = true; break; }
      }
      ds.rdsSequence[newLen++] = ffGroupIndex(groupType, versionB);
      if (comma == nullptr) break;
      p = comma + 1;
    }
    if (newLen == 0) {
      Serial.println("RDS group sequence value had no valid entries, ignored");
    } else {
      ds.rdsSequenceLen = newLen;
      uecpRdsState.rdsSeqPos      = 0; // restart from the new sequence's own beginning
      Serial.printf("RDS group sequence updated (%u entries)\n", newLen);
    }
  } else if (strcmp(field, "rds_dsn") == 0) {
    // Switches the active data set (1-6) - the same thing UECP MEC 0x1C will do.
    uint8_t dsn = (uint8_t) server.arg("rds_dsn").toInt();
    if (rdsSelectDataSet(uecpRdsState, dsn)) {
      Serial.printf("RDS active DSN switched to: %u\n", dsn);
    } else {
      Serial.printf("RDS DSN %u out of range (1-%u), ignored\n", dsn, RDS_DSN_COUNT);
    }
  } else if (strcmp(field, "rds_psn") == 0) {
    // Renumbers the active data set's main service (1-255) - the PSN UECP elements must address it
    // by (PSN 0 always reaches it regardless).
    uint8_t psn = (uint8_t) server.arg("rds_psn").toInt();
    if (psn != 0) {
      main.psnNumber = psn;
      Serial.printf("RDS main service PSN updated to: %u\n", psn);
    } else {
      Serial.println("RDS PSN 0 isn't a valid service number (1-255), ignored");
    }
  } else if (strcmp(field, "rds_pin") == 0) {
    char rds_pin[8];
    server.arg("rds_pin").toCharArray(rds_pin, sizeof(rds_pin));
    uint16_t pin = (uint16_t) strtol(rds_pin, NULL, 16);
    main.pin[0] = (uint8_t)(pin >> 8);
    main.pin[1] = (uint8_t)(pin & 0xFF);
    Serial.printf("RDS PIN updated to: 0x%04X\n", pin);
  } else if (strcmp(field, "rds_ta") == 0) {
    bool on = (server.arg("rds_ta") == "1");
    main.tatp = on ? (main.tatp | 0x01) : (main.tatp & (uint8_t)~0x01);
    Serial.printf("RDS TA updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_tp") == 0) {
    bool on = (server.arg("rds_tp") == "1");
    main.tatp = on ? (main.tatp | 0x02) : (main.tatp & (uint8_t)~0x02);
    Serial.printf("RDS TP updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_ms") == 0) {
    bool on = (server.arg("rds_ms") == "1");
    main.ms = on ? (main.ms | 0x01) : (main.ms & (uint8_t)~0x01);
    Serial.printf("RDS MS updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_di_stereo") == 0) {
    bool on = (server.arg("rds_di_stereo") == "1");
    main.diPtyi = on ? (main.diPtyi | 0x01) : (main.diPtyi & (uint8_t)~0x01);
    Serial.printf("RDS DI Mono/Stereo updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_di_artifhead") == 0) {
    bool on = (server.arg("rds_di_artifhead") == "1");
    main.diPtyi = on ? (main.diPtyi | 0x02) : (main.diPtyi & (uint8_t)~0x02);
    Serial.printf("RDS DI Artificial Head updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_di_compressed") == 0) {
    bool on = (server.arg("rds_di_compressed") == "1");
    main.diPtyi = on ? (main.diPtyi | 0x04) : (main.diPtyi & (uint8_t)~0x04);
    Serial.printf("RDS DI Compressed updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_ptyi") == 0) {
    bool on = (server.arg("rds_ptyi") == "1");
    main.diPtyi = on ? (main.diPtyi | 0x08) : (main.diPtyi & (uint8_t)~0x08);
    Serial.printf("RDS Dynamic PTY updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "frequency_deviation") == 0) {
    currentFreqDeviation = server.arg("frequency_deviation").toInt();
    tx.setTxFrequencyDeviation(currentFreqDeviation);
    Serial.printf("Frequency Deviation updated to: %u\n", currentFreqDeviation);
  } else if (strcmp(field, "input_impedance") == 0) {
    currentInputImpedance = server.arg("input_impedance").toInt();
    tx.setTxInputImpedance(currentInputImpedance);
    Serial.printf("Input Impedance updated to: %u\n", currentInputImpedance);
  } else if (strcmp(field, "stereo_mono") == 0) {
    currentStereoMono = server.arg("stereo_mono").toInt();
    tx.setTxMono(currentStereoMono);
    Serial.printf("Stereo/Mono updated to: %u\n", currentStereoMono);
  } else if (strcmp(field, "buffer_gain") == 0) {
    currentBufferGain = server.arg("buffer_gain").toInt();
    tx.setTxInputBufferGain(currentBufferGain);
    Serial.printf("Buffer Gain updated to: %u\n", currentBufferGain);
  } else if (strcmp(field, "pre_emphasis") == 0) {
    currentPreEmphasis = server.arg("pre_emphasis").toInt();
    tx.setPreEmphasis(currentPreEmphasis);
    Serial.printf("Pre-Emphasis updated to: %u\n", currentPreEmphasis);
  } else if (strcmp(field, "soft_clip") == 0) {
    currentSoftClip = server.arg("soft_clip").toInt();
    tx.setTxSoftClippingEnable(currentSoftClip);
    Serial.printf("Soft Clip updated to: %u\n", currentSoftClip);
  } else if (strcmp(field, "raw_mode") == 0) {
    bool wasRaw = rawGroupMode;
    rawGroupMode = (server.arg("raw_mode") == "1");
    if (rawGroupMode) {
      while (Serial.available()) Serial.read();
      // flush stale bytes
    } else if (wasRaw) {
      // Re-arm the chip after raw mode: its RDS state machine was driven by
      // the external sender;
      // reinitialise to the same state as startup so the
      // local scheduler can take over cleanly.
      rdsSchedulerStart(tx);
      rdsSchedulerReset();
      uecpRdsState.psSegment = 0;
      uecpRdsState.rtSegment = 0;
      uecpRdsState.rdsSeqPos = 0;
      // No CT-group deferral needed here any more: buildNextGroup()'s Group 4A check reads the
      // system clock directly (non-blocking) and only ever fires right at a real minute boundary.
    }
    Serial.println(rawGroupMode ? "RAW_MODE_ON" : "RAW_MODE_OFF");
  }

  // Enviar resposta ao cliente confirmando o recebimento do campo
  char responseMsg[48];
  snprintf(responseMsg, sizeof(responseMsg), "%s has been updated.", field);
  server.send(200, "text/plain", responseMsg);
}


// Função para tratar o envio do formulário
void handleFormSubmit() {
  // The submitted RDS values are saved to the data set active when the form was submitted; a changed
  // DSN is applied last (see below), switching to that data set afterwards.
  RdsMainService& main = rdsActiveMain(uecpRdsState);
  char frequency[16];
  server.arg("frequency").toCharArray(frequency, sizeof(frequency));
  char power[8];
  server.arg("power").toCharArray(power, sizeof(power));
  char rds_pi[8];
  server.arg("rds_pi").toCharArray(rds_pi, sizeof(rds_pi));
  char rds_pty[8];
  server.arg("rds_pty").toCharArray(rds_pty, sizeof(rds_pty));
  char rds_ps[40];
  server.arg("rds_ps").toCharArray(rds_ps, sizeof(rds_ps));
  // Small headroom over RDS_RT_MAX_LEN for a stray/oversized submission - toCharArray() below
  // truncates safely to sizeof-1 regardless, matching the old .substring(0,64) behavior.
  char rds_rt[RDS_RT_MAX_LEN + 8];
  server.arg("rds_rt").toCharArray(rds_rt, sizeof(rds_rt));
  char rds_rt_repeat[8];
  server.arg("rds_rt_repeat").toCharArray(rds_rt_repeat, sizeof(rds_rt_repeat));
  char rds_dsn[8];
  server.arg("rds_dsn").toCharArray(rds_dsn, sizeof(rds_dsn));
  char rds_psn[8];
  server.arg("rds_psn").toCharArray(rds_psn, sizeof(rds_psn));
  char uecp_site[24]; // comma-separated hex list, up to UECP_MAX_SITE_ADDRESSES entries
  server.arg("uecp_site").toCharArray(uecp_site, sizeof(uecp_site));
  char uecp_enc[32]; // comma-separated hex list, up to UECP_MAX_ENCODER_ADDRESSES entries
  server.arg("uecp_enc").toCharArray(uecp_enc, sizeof(uecp_enc));
  char rds_pin[8];
  server.arg("rds_pin").toCharArray(rds_pin, sizeof(rds_pin));
  char rds_ta[4];
  server.arg("rds_ta").toCharArray(rds_ta, sizeof(rds_ta));
  char rds_tp[4];
  server.arg("rds_tp").toCharArray(rds_tp, sizeof(rds_tp));
  char rds_ms[4];
  server.arg("rds_ms").toCharArray(rds_ms, sizeof(rds_ms));
  char rds_di_stereo[4];
  server.arg("rds_di_stereo").toCharArray(rds_di_stereo, sizeof(rds_di_stereo));
  char rds_di_artifhead[4];
  server.arg("rds_di_artifhead").toCharArray(rds_di_artifhead, sizeof(rds_di_artifhead));
  char rds_di_compressed[4];
  server.arg("rds_di_compressed").toCharArray(rds_di_compressed, sizeof(rds_di_compressed));
  char rds_ptyi[4];
  server.arg("rds_ptyi").toCharArray(rds_ptyi, sizeof(rds_ptyi));
  char frequency_deviation[8];
  server.arg("frequency_deviation").toCharArray(frequency_deviation, sizeof(frequency_deviation));
  char input_impedance[8];
  server.arg("input_impedance").toCharArray(input_impedance, sizeof(input_impedance));
  char stereo_mono[8];
  server.arg("stereo_mono").toCharArray(stereo_mono, sizeof(stereo_mono));
  char buffer_gain[8];
  server.arg("buffer_gain").toCharArray(buffer_gain, sizeof(buffer_gain));
  char pre_emphasis[8];
  server.arg("pre_emphasis").toCharArray(pre_emphasis, sizeof(pre_emphasis));
  char soft_clip[8];
  server.arg("soft_clip").toCharArray(soft_clip, sizeof(soft_clip));
  // Aqui você pode adicionar o código para enviar essas configurações para o QN8066
  // Como por exemplo, configurar a frequência de transmissão, potência, e os parâmetros RDS.
  char   response[512];
  size_t rpos = 0;
  bufAppend(response, sizeof(response), &rpos, "<html><body><h1>Settings Received</h1>");
  bufAppend(response, sizeof(response), &rpos, "<p>Frequency: %s MHz</p>", frequency);
  bufAppend(response, sizeof(response), &rpos, "<p>Power%%: %s</p>", power);
  bufAppend(response, sizeof(response), &rpos, "<p>RDS PTY: %s</p>", rds_pty);
  bufAppend(response, sizeof(response), &rpos, "<p>RDS PS: %s</p>", rds_ps);
  bufAppend(response, sizeof(response), &rpos, "<p>RDS RT: %s</p>", rds_rt);
  bufAppend(response, sizeof(response), &rpos, "</body></html>");

  if (frequency[0] != '\0') {
    currentFrequency = (uint16_t) (atof(frequency) * 10);
    if (currentFrequency != previousFrequency) {
      tx.setTX(currentFrequency);
      previousFrequency = currentFrequency;
    }
  }

  if (power[0] != '\0') {
    currentPower = (uint8_t) atoi(power);
    // See handleUpdate()'s "power" branch: setPAC() triggers an on-chip recalibration that drops
    // RDS transmission until re-armed, and invalidates the scheduler's own buffer/pending-send state.
    tx.setPAC(currentPower);
    rdsSchedulerStart(tx);
    rdsSchedulerReset();
  }

  if (rds_ps[0] != '\0') {
    size_t len = strlen(rds_ps);
    while (len < 8 && len + 1 < sizeof(rds_ps)) rds_ps[len++] = ' ';
    rds_ps[len] = '\0';
    utf8ToEbu(rds_ps, main.psPending, RDS_PS_LEN);
    main.psPendingValid = true;
  }
  if (rds_rt[0] != '\0') {
    // See handleUpdate()'s "rds_rt" branch: the web interface only ever controls a single RT
    // message, so a new submission clears the whole buffer and becomes the sole message 0.
    memset(main.rt, 0, sizeof(main.rt));
    main.rt[0].textLen     = (uint8_t) utf8ToEbu(rds_rt, main.rt[0].text, RDS_RT_MAX_LEN);
    main.rt[0].repeatCount = 0;  // infinite
    main.rt[0].toggleAB    = true;
    main.rtCount           = 1;
    uecpRdsState.rtCurrent         = 0;
    uecpRdsState.rtSegment         = 0;
    uecpRdsState.rtRepeatsDone     = 0;
    main.rtABFlag          = !main.rtABFlag;
  }
  if (rds_rt_repeat[0] != '\0') main.rt[0].repeatCount = (uint8_t) atoi(rds_rt_repeat);

  if (rds_pi[0] != '\0') {
    uint16_t pi = (uint16_t) strtol(rds_pi, NULL, 16);
    main.pi[0] = (uint8_t)(pi >> 8);
    main.pi[1] = (uint8_t)(pi & 0xFF);
  }
  main.pty = (uint8_t) atoi(rds_pty);

  if (rds_psn[0]   != '\0' && atoi(rds_psn) != 0) main.psnNumber = (uint8_t) atoi(rds_psn);
  if (uecp_site[0] != '\0') {
    uint16_t parsed[UECP_MAX_SITE_ADDRESSES];
    uint8_t  n = parseUecpAddressList(uecp_site, parsed, UECP_MAX_SITE_ADDRESSES, 0x03FF, 16);
    if (n > 0) {
      memcpy(uecpOurAddress.sites, parsed, n * sizeof(uint16_t));
      uecpOurAddress.siteCount = n;
    }
  }
  if (uecp_enc[0] != '\0') {
    uint16_t parsed[UECP_MAX_ENCODER_ADDRESSES];
    uint8_t  n = parseUecpAddressList(uecp_enc, parsed, UECP_MAX_ENCODER_ADDRESSES, 0x3F, 16);
    if (n > 0) {
      for (uint8_t i = 0; i < n; i++) uecpOurAddress.encoders[i] = (uint8_t) parsed[i];
      uecpOurAddress.encoderCount = n;
    }
  }
  if (rds_pin[0]   != '\0') {
    uint16_t pin = (uint16_t) strtol(rds_pin, NULL, 16);
    main.pin[0] = (uint8_t)(pin >> 8);
    main.pin[1] = (uint8_t)(pin & 0xFF);
  }

  // The flag selects are always present in a full-form submit (unlike native checkboxes,
  // which browsers omit from FormData when unchecked), so these are applied unconditionally.
  main.tatp = (strcmp(rds_ta, "1") == 0) ? (main.tatp | 0x01) : (main.tatp & (uint8_t)~0x01);
  main.tatp = (strcmp(rds_tp, "1") == 0) ? (main.tatp | 0x02) : (main.tatp & (uint8_t)~0x02);
  main.ms   = (strcmp(rds_ms, "1") == 0) ? (main.ms   | 0x01) : (main.ms   & (uint8_t)~0x01);
  main.diPtyi = (strcmp(rds_di_stereo,     "1") == 0) ? (main.diPtyi | 0x01) : (main.diPtyi & (uint8_t)~0x01);
  main.diPtyi = (strcmp(rds_di_artifhead,  "1") == 0) ? (main.diPtyi | 0x02) : (main.diPtyi & (uint8_t)~0x02);
  main.diPtyi = (strcmp(rds_di_compressed, "1") == 0) ? (main.diPtyi | 0x04) : (main.diPtyi & (uint8_t)~0x04);
  main.diPtyi = (strcmp(rds_ptyi,          "1") == 0) ? (main.diPtyi | 0x08) : (main.diPtyi & (uint8_t)~0x08);

  currentFreqDeviation = atoi(frequency_deviation);
  currentInputImpedance = atoi(input_impedance);
  currentStereoMono     = atoi(stereo_mono);
  currentBufferGain     = atoi(buffer_gain);
  currentPreEmphasis    = atoi(pre_emphasis);
  currentSoftClip       = atoi(soft_clip);
  tx.setTxFrequencyDeviation(currentFreqDeviation);
  tx.setTxInputImpedance(currentInputImpedance);
  tx.setTxMono(currentStereoMono);
  tx.setTxInputBufferGain(currentBufferGain);
  tx.setPreEmphasis(currentPreEmphasis);
  tx.setTxSoftClippingEnable(currentSoftClip);

  // Last, so everything above lands in the data set that was active when the form was submitted.
  if (rds_dsn[0]   != '\0') rdsSelectDataSet(uecpRdsState, (uint8_t) atoi(rds_dsn));

  server.send(200, "text/html", response);
}

void processSerialGroups() {
  if (Serial.available() < 8) return;

  uint8_t buf[8];
  Serial.readBytes(buf, 8);

  RDS_BLOCK1 b1;
  RDS_BLOCK2 b2;
  RDS_BLOCK3 b3;
  RDS_BLOCK4 b4;

  b1.byteContent[1] = buf[0]; b1.byteContent[0] = buf[1];
  b2.byteContent[1] = buf[2]; b2.byteContent[0] = buf[3];
  b3.byteContent[1] = buf[4]; b3.byteContent[0] = buf[5];
  b4.byteContent[1] = buf[6]; b4.byteContent[0] = buf[7];

  tx.rdsSendGroup(b1, b2, b3, b4);

  Serial.write(0xAC);
  // ACK: group accepted and transmitted
}

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Local Time fails");
    return;
  }
  Serial.println(&timeinfo, "%Y/%m/%d %H:%M:%S");
}


void handleUecpServer() {
  WiFiClient newClient = uecp_server.accept();
  if (newClient) {
    bool accepted = false;
    for (int i = 0; i < UECP_MAX_CLIENTS; i++) {
      if (!uecp_clients[i].client.connected()) {
        uecp_clients[i].client = newClient;
        uecp_clients[i].inPacket = false;
        uecp_clients[i].bufLen = 0;
        accepted = true;
        Serial.printf("UECP client connected from %s\n", newClient.remoteIP().toString().c_str());
        break;
      }
    }
    if (!accepted) {
      newClient.stop();
    }
  }

  for (int i = 0; i < UECP_MAX_CLIENTS; i++) {
    WiFiClient& c = uecp_clients[i].client;
    if (!c.connected()) continue;
    while (c.available()) {
      uint8_t b = (uint8_t)c.read();
      if (b == 0xFE) {
        uecp_clients[i].inPacket = true;
        uecp_clients[i].bufLen = 0;
      } else if (b == 0xFF && uecp_clients[i].inPacket) {
        uecp_clients[i].inPacket = false;
        processUecpFrame(uecp_clients[i].buf, uecp_clients[i].bufLen,
                         c.remoteIP().toString(), uecpRdsState, ffPool, odaLiveDir,
                         pendingImmediateGroupIndex, uecpOurAddress);
        uecp_clients[i].bufLen = 0;
      } else if (uecp_clients[i].inPacket) {
        if (uecp_clients[i].bufLen < UECP_MAX_FRAME) {
          uecp_clients[i].buf[uecp_clients[i].bufLen++] = b;
        } else {
          // Frame too large — discard and resync
          uecp_clients[i].inPacket = false;
          uecp_clients[i].bufLen = 0;
        }
      }
    }
  }
}

void setup() {
  // Inicializa a comunicação serial
  Serial.begin(115200);

  rdsEncoderStateSetDefaults(uecpRdsState);
  // Apply this sketch's own RDS station-identity defaults (see the rdsDefault* constants above) to
  // DSN 1's main service, overriding the generic library placeholders rdsEncoderStateSetDefaults()
  // just set - rdsCopyDataSet1ToAll() below then gives every other data set the same starting
  // point. Written directly into ps[] (not the psPending staging area MEC 0x02 and the web form's
  // PS field use) since nothing has been transmitted yet - there's no "torn mid-cycle" receiver to
  // protect against.
  RdsDataSet&     ds   = rdsActiveDataSet(uecpRdsState);
  RdsMainService& main = ds.main;
  main.pi[0] = (uint8_t)(rdsDefaultPI >> 8);
  main.pi[1] = (uint8_t)(rdsDefaultPI & 0xFF);
  {
    uint8_t psEbu[RDS_PS_LEN];
    memset(psEbu, ' ', RDS_PS_LEN); // pad short values with spaces - utf8ToEbu() never pads itself
    utf8ToEbu(rdsDefaultPS, psEbu, RDS_PS_LEN);
    memcpy(main.ps, psEbu, RDS_PS_LEN);
  }
  {
    // rtCount is only set to 1 if rdsDefaultRT actually converts to some real text - an empty
    // string leaves it at 0, so Group 2A isn't built until a real RT message arrives.
    RtMessage& msg  = main.rt[0];
    msg.textLen     = (uint8_t) utf8ToEbu(rdsDefaultRT, msg.text, RDS_RT_MAX_LEN);
    msg.repeatCount = 0; // loop forever
    msg.toggleAB    = false;
    main.rtCount    = (msg.textLen > 0) ? 1 : 0;
  }
  main.pty    = rdsDefaultPTY;
  main.tatp   = (rdsDefaultTA ? 0x01 : 0x00) | (rdsDefaultTP ? 0x02 : 0x00);
  main.ms     = rdsDefaultMusic ? 0x01 : 0x00;
  main.diPtyi = (rdsDefaultStereo ? 0x01 : 0x00) | (rdsDefaultArtificialHead ? 0x02 : 0x00) |
                (rdsDefaultCompressed ? 0x04 : 0x00) | (rdsDefaultDynamicPTY ? 0x08 : 0x00);

  // Seed the live, runtime-mutable group sequence from this sketch's own boot default - see
  // RDS_SEQUENCE's own comment above. rdsEncoderStateSetDefaults() itself leaves it empty (it has
  // no sketch-specific sequence of its own to default to), so this is the .ino's own job, same as
  // ffPoolInit()/odaLiveDirectoryInit() just below.
  memcpy(ds.rdsSequence, RDS_SEQUENCE, RDS_SEQUENCE_LEN);
  ds.rdsSequenceLen = RDS_SEQUENCE_LEN;
  // Every data set (DSN 1-6) starts out identical to DSN 1 as just configured.
  rdsCopyDataSet1ToAll(uecpRdsState);
  ffPoolInit(ffPool);
  odaLiveDirectoryInit(odaLiveDir);

  ledcAttach(4, 32768, 1);
  ledcWrite(4, 1);
  // The line below may be necessary to setup I2C pins on ESP32
  Wire.begin(ESP32_I2C_SDA, ESP32_I2C_SCL);
  // Conecta na rede Wi-Fi
  WiFi.setHostname(hostname); // both of these must come before WiFi.begin() to take effect
  WiFi.enableIPv6();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
  }
  Serial.println("Wi-Fi connected!");

  // Web server setup
  server.on("/", handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/setParameters", HTTP_POST, handleFormSubmit);
  server.on("/update", HTTP_POST, handleUpdate);
  server.begin();
  Serial.println("\nServidor HTTP started.");
  uecp_server.begin();
  Serial.printf("UECP TCP server started on port %d\n", UECP_TCP_PORT);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  tx.setup(
    1,
    currentStereoMono,
    false,
    0,
    1
  );
  tx.setTX(currentFrequency);   // Sets frequency to 106.9 MHz
  delay(500);
  // Push the remaining boot defaults to the chip - none of these apply themselves at startup
  // otherwise, only when a value is later submitted via the web interface.
  tx.setTxFrequencyDeviation(currentFreqDeviation);
  tx.setTxPilotGain(currentPilotGain);
  tx.rdsSetFrequencyDeviation(currentRdsDeviation);
  tx.setPAC(currentPower);
  rdsSchedulerStart(tx);
  // Sets the current RTC based on Network if it is available
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");

  Serial.println("Transmitting (analogue audio in)...");
}

void loop() {
  server.handleClient();
  handleUecpServer();
  rdsSchedulerApplyPendingTime(uecpRdsState);
  if (rawGroupMode) {
    processSerialGroups();
  } else {
    rdsSchedulerTick(tx, uecpRdsState, ffPool, pendingImmediateGroupIndex, computeLocalOffsetByte());
  }
}
