/*
  ESP32 Dev Modeule version.


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
#include "Audio.h"   // ESP32-audioI2S library
#include <rds_state.h>
#include <uecp_handler.h>
#include <rds_defaults.h>
#include <ebu_charset.h>
#include <free_format_groups.h>
#include <oda_directory.h>
#include <rds_scheduler.h>
#include <stdarg.h>   // for bufAppend()'s va_list

// --- Audio I2S Pins ---
#define I2S_BCLK      5
#define I2S_LRC       6
#define I2S_DOUT      7

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

uint16_t currentFrequency = 1069; // 106.9 MHz
uint16_t previousFrequency = 1069; // 106.9 MHz

uint8_t currentPower = 25;
uint8_t currentStereoMono     = 0;
uint8_t currentPreEmphasis    = 1;
// 83 = 75 kHz - the standard/nominal FM deviation reference; see index_html.h's corrected
// "MPX Deviation" dropdown for the rest of the register-value-to-kHz mapping.
uint8_t currentFreqDeviation  = 83;
uint8_t currentRdsDeviation   = 20; // was a hardcoded literal in setup()'s rdsSetFrequencyDeviation() call
uint8_t currentPilotGain      = 11; // was a hardcoded literal in setup()'s setTxPilotGain() call
uint8_t currentInputImpedance = 1;
uint8_t currentBufferGain     = 1;
uint8_t currentSoftClip       = 0;
// Wi-Fi setup
const char* ssid = "ssid";
// Change to your WIFI SSID
const char* password = "pass";
// Change to your password

// Local Time setup
const long gmtOffset_sec = -10800;
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

// UECP TCP server (UECP_MAX_FRAME is defined in uecp_handler.h) - a second source of UECP frames
// alongside the AAC/MP2 audio-embedded path below; both feed the same uecpRdsState.
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
Audio audio; // Audio object for streaming
// Fixed-size buffer rather than String: this is a long-lived global, updated repeatedly over the
// device's uptime by the web interface, and a growable String here would be a heap-fragmentation
// risk over a 24/7 run (see the other String -> char[] conversions below for the same reasoning).
char currentStreamUrl[128] = "http://192.168.1.101:8000/stream.mp3";
uint8_t currentVolume = 21; // Max volume for I2S audio is 21

bool rawGroupMode = false;

// RDS state fed by UECP frames from either source (the audio stream's DSE/ancillary data via
// audio_process_uecp(), or the TCP UECP server below) and by the web interface. This is now the
// single source of truth for everything buildNextGroup() transmits.
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
// applied - see buildNextGroup()'s own check of this below. FF_INDEX_NONE (its usual value) means
// nothing is pending.
uint8_t pendingImmediateGroupIndex = FF_INDEX_NONE;

// This device's own UECP address (site: 10 bits, encoder: 6 bits). 0/0 = accept every UECP frame
// regardless of its address, per the UECP spec's "all sites"/"all encoders" convention. Set via
// the web interface's UECP Site/Encoder Address fields.
uint16_t uecpOurSite    = 0;
uint8_t  uecpOurEncoder = 0;

void audio_process_uecp(const uint8_t* data, size_t len) {
    processUecpFrame(data, (uint16_t)len, "AAC-DSE", uecpRdsState, ffPool, odaLiveDir,
                     pendingImmediateGroupIndex, uecpOurSite, uecpOurEncoder);
}

// Group scheduler state. The sequence position itself (rdsSeqPos) now lives in uecpRdsState
// (rds_state.h) alongside the sequence it walks, not here - see that struct's rdsSequence[]/
// rdsSequenceLen/rdsSeqPos comment - so UECP MEC 0x16 and the web form's "Group Sequence" field
// (uecp_handler.cpp/handleUpdate()'s "rds_seq" branch respectively) can reconfigure both without
// any extra plumbing back into the .ino.
time_t lastCtMinuteSent = -1;
// epoch-minute index (unix time / 60) of the last Clock Time group actually sent; -1 = never -
// latches so buildNextGroup()'s per-call :00 check can't fire twice for the same minute

// Boot-time default group sequence, copied into uecpRdsState.rdsSequence[]/rdsSequenceLen once in
// setup() (see rdsTxStateSetDefaults()'s call site below). Editing these two constants and
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
// cycle, so listing extra indices "just in case" costs nothing when they're empty.
// Sequence: 5 × Group 0A (PS), 1 × Group 1A (PIN/SLC), 3 × Group 2A (RT), plus one reserved slot
// each for Group 3A (ODA AID/group announcements - free-format or manual "master data", see
// buildGroup3A()), Group 8A (16 - reserved purely so MEC 0x30 TMC content actually gets
// transmitted; it has no struct fallback of its own, so an omitted index would leave any queued
// TMC content never dequeued regardless of what's in it), and Group 10A (PTYN) per 12-group
// cycle. Gives ~950ms full PS cycle and ~2.5s full RT cycle at 11.4 groups/sec.
const uint8_t RDS_SEQUENCE[]   = {0, 0, 4, 0, 0, 4, 0, 4, 2, 6, 16, 20};
const uint8_t RDS_SEQUENCE_LEN = 12;
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
  char piBuf[5];
  snprintf(piBuf, sizeof(piBuf), "%02X%02X", uecpRdsState.pi[0], uecpRdsState.pi[1]);
  char pinBuf[5];
  snprintf(pinBuf, sizeof(pinBuf), "%02X%02X", uecpRdsState.pin[0], uecpRdsState.pin[1]);
  char uecpSiteBuf[4];
  snprintf(uecpSiteBuf, sizeof(uecpSiteBuf), "%03X", uecpOurSite);
  char uecpEncBuf[3];
  snprintf(uecpEncBuf, sizeof(uecpEncBuf), "%02X", uecpOurEncoder);
  char psUtf8[RDS_PS_LEN * 3 + 1];
  ebuToUtf8(uecpRdsState.ps, RDS_PS_LEN, psUtf8, sizeof(psUtf8));
  char rtUtf8[RDS_RT_MAX_LEN * 3 + 1];
  ebuToUtf8(uecpRdsState.rt[0].text, uecpRdsState.rt[0].textLen, rtUtf8, sizeof(rtUtf8));

  char psEscaped[sizeof(psUtf8) * 2];
  jsonEscape(psUtf8, psEscaped, sizeof(psEscaped));
  char rtEscaped[sizeof(rtUtf8) * 2];
  jsonEscape(rtUtf8, rtEscaped, sizeof(rtEscaped));
  char urlEscaped[sizeof(currentStreamUrl) * 2];
  jsonEscape(currentStreamUrl, urlEscaped, sizeof(urlEscaped));

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
  bufAppend(json, sizeof(json), &pos, ",\"rds_pty\":%u", uecpRdsState.pty);
  bufAppend(json, sizeof(json), &pos, ",\"rds_ps\":\"%s\"", psEscaped);
  bufAppend(json, sizeof(json), &pos, ",\"rds_rt\":\"%s\"", rtEscaped);
  bufAppend(json, sizeof(json), &pos, ",\"stream_url\":\"%s\"", urlEscaped);
  bufAppend(json, sizeof(json), &pos, ",\"stream_volume\":%u", currentVolume);
  bufAppend(json, sizeof(json), &pos, ",\"rds_dsn\":%u", uecpRdsState.dsn);
  bufAppend(json, sizeof(json), &pos, ",\"rds_psn\":%u", uecpRdsState.psn);
  bufAppend(json, sizeof(json), &pos, ",\"uecp_site\":\"%s\"", uecpSiteBuf);
  bufAppend(json, sizeof(json), &pos, ",\"uecp_enc\":\"%s\"", uecpEncBuf);
  bufAppend(json, sizeof(json), &pos, ",\"rds_pin\":\"%s\"", pinBuf);
  bufAppend(json, sizeof(json), &pos, ",\"rds_ta\":%u", (uecpRdsState.tatp & 0x01) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_tp\":%u", (uecpRdsState.tatp & 0x02) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_ms\":%u", (uecpRdsState.ms & 0x01) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_di_stereo\":%u", (uecpRdsState.diPtyi & 0x01) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_di_artifhead\":%u", (uecpRdsState.diPtyi & 0x02) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_di_compressed\":%u", (uecpRdsState.diPtyi & 0x04) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_ptyi\":%u", (uecpRdsState.diPtyi & 0x08) ? 1 : 0);
  bufAppend(json, sizeof(json), &pos, ",\"rds_rt_repeat\":%u", uecpRdsState.rt[0].repeatCount);
  bufAppend(json, sizeof(json), &pos, "}");
  server.send(200, "application/json", json);
}

void handleUpdate() {
  char field[24];
  server.argName(0).toCharArray(field, sizeof(field));
  // Processa e aplica o valor do campo correspondente

  if (strcmp(field, "stream_url") == 0) {
    server.arg("stream_url").toCharArray(currentStreamUrl, sizeof(currentStreamUrl));
    audio.stopSong();
    audio.connecttohost(currentStreamUrl);
    Serial.printf("Stream URL updated to: %s\n", currentStreamUrl);
  } else if (strcmp(field, "stream_volume") == 0) {
    currentVolume = server.arg("stream_volume").toInt();
    if (currentVolume > 21) currentVolume = 21;
    audio.setVolume(currentVolume);
    Serial.printf("Stream Volume updated to: %u\n", currentVolume);
  } else if (strcmp(field, "frequency") == 0) {
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
    uecpRdsState.pi[0] = (uint8_t)(pi >> 8);
    uecpRdsState.pi[1] = (uint8_t)(pi & 0xFF);
    Serial.printf("RDS PI updated to: 0x%04X\n", pi);
  } else if (strcmp(field, "rds_pty") == 0) {
    uecpRdsState.pty = (uint8_t) server.arg("rds_pty").toInt();
    Serial.printf("RDS PTY updated to: %u\n", uecpRdsState.pty);
  } else if (strcmp(field, "rds_ps") == 0) {
    char rds_ps[40];
    server.arg("rds_ps").toCharArray(rds_ps, sizeof(rds_ps));
    size_t len = strlen(rds_ps);
    while (len < 8 && len + 1 < sizeof(rds_ps)) rds_ps[len++] = ' ';
    rds_ps[len] = '\0';
    // Staged, not applied to ps[] directly - see buildNextGroup()'s psSegment==0 wrap check, same
    // clean-swap guarantee as a UECP MEC 0x02 update.
    utf8ToEbu(rds_ps, uecpRdsState.psPending, RDS_PS_LEN);
    uecpRdsState.psPendingValid = true;
    Serial.printf("RDS PS updated to: %s\n", rds_ps);
  } else if (strcmp(field, "rds_rt") == 0) {
    // Small headroom over RDS_RT_MAX_LEN for a stray/oversized submission - toCharArray() below
    // truncates safely to sizeof-1 regardless, matching the old .substring(0,64) behavior.
    char rds_rt[RDS_RT_MAX_LEN + 8];
    server.arg("rds_rt").toCharArray(rds_rt, sizeof(rds_rt));
    // The web interface only ever controls a single RT message: submitting new text clears the
    // whole buffer (any other slots are UECP-only territory) and becomes the sole message 0.
    memset(uecpRdsState.rt, 0, sizeof(uecpRdsState.rt));
    uecpRdsState.rt[0].textLen     = (uint8_t) utf8ToEbu(rds_rt, uecpRdsState.rt[0].text, RDS_RT_MAX_LEN);
    uecpRdsState.rt[0].repeatCount = 0;  // infinite
    uecpRdsState.rt[0].toggleAB    = true;
    uecpRdsState.rtCount           = 1;
    uecpRdsState.rtCurrent         = 0;
    uecpRdsState.rtSegment         = 0;
    uecpRdsState.rtRepeatsDone     = 0;
    uecpRdsState.rtABFlag          = !uecpRdsState.rtABFlag;
    Serial.printf("RDS RT updated to: %s\n", rds_rt);
  } else if (strcmp(field, "rds_rt_repeat") == 0) {
    uecpRdsState.rt[0].repeatCount = (uint8_t) server.arg("rds_rt_repeat").toInt();
    Serial.printf("RDS RT repeat count updated to: %u\n", uecpRdsState.rt[0].repeatCount);
  } else if (strcmp(field, "uecp_site") == 0) {
    char uecp_site[8];
    server.arg("uecp_site").toCharArray(uecp_site, sizeof(uecp_site));
    uecpOurSite = (uint16_t) strtol(uecp_site, NULL, 16) & 0x03FF;
    Serial.printf("UECP Site Address updated to: 0x%X\n", uecpOurSite);
  } else if (strcmp(field, "uecp_enc") == 0) {
    char uecp_enc[8];
    server.arg("uecp_enc").toCharArray(uecp_enc, sizeof(uecp_enc));
    uecpOurEncoder = (uint8_t) strtol(uecp_enc, NULL, 16) & 0x3F;
    Serial.printf("UECP Encoder Address updated to: 0x%X\n", uecpOurEncoder);
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
      uecpRdsState.rdsSequence[newLen++] = ffGroupIndex(groupType, versionB);
      if (comma == nullptr) break;
      p = comma + 1;
    }
    if (newLen == 0) {
      Serial.println("RDS group sequence value had no valid entries, ignored");
    } else {
      uecpRdsState.rdsSequenceLen = newLen;
      uecpRdsState.rdsSeqPos      = 0; // restart from the new sequence's own beginning
      Serial.printf("RDS group sequence updated (%u entries)\n", newLen);
    }
  } else if (strcmp(field, "rds_dsn") == 0) {
    uecpRdsState.dsn = (uint8_t) server.arg("rds_dsn").toInt();
    Serial.printf("RDS DSN updated to: %u\n", uecpRdsState.dsn);
  } else if (strcmp(field, "rds_psn") == 0) {
    uecpRdsState.psn = (uint8_t) server.arg("rds_psn").toInt();
    Serial.printf("RDS PSN updated to: %u\n", uecpRdsState.psn);
  } else if (strcmp(field, "rds_pin") == 0) {
    char rds_pin[8];
    server.arg("rds_pin").toCharArray(rds_pin, sizeof(rds_pin));
    uint16_t pin = (uint16_t) strtol(rds_pin, NULL, 16);
    uecpRdsState.pin[0] = (uint8_t)(pin >> 8);
    uecpRdsState.pin[1] = (uint8_t)(pin & 0xFF);
    Serial.printf("RDS PIN updated to: 0x%04X\n", pin);
  } else if (strcmp(field, "rds_ta") == 0) {
    bool on = (server.arg("rds_ta") == "1");
    uecpRdsState.tatp = on ? (uecpRdsState.tatp | 0x01) : (uecpRdsState.tatp & (uint8_t)~0x01);
    Serial.printf("RDS TA updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_tp") == 0) {
    bool on = (server.arg("rds_tp") == "1");
    uecpRdsState.tatp = on ? (uecpRdsState.tatp | 0x02) : (uecpRdsState.tatp & (uint8_t)~0x02);
    Serial.printf("RDS TP updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_ms") == 0) {
    bool on = (server.arg("rds_ms") == "1");
    uecpRdsState.ms = on ? (uecpRdsState.ms | 0x01) : (uecpRdsState.ms & (uint8_t)~0x01);
    Serial.printf("RDS MS updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_di_stereo") == 0) {
    bool on = (server.arg("rds_di_stereo") == "1");
    uecpRdsState.diPtyi = on ? (uecpRdsState.diPtyi | 0x01) : (uecpRdsState.diPtyi & (uint8_t)~0x01);
    Serial.printf("RDS DI Mono/Stereo updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_di_artifhead") == 0) {
    bool on = (server.arg("rds_di_artifhead") == "1");
    uecpRdsState.diPtyi = on ? (uecpRdsState.diPtyi | 0x02) : (uecpRdsState.diPtyi & (uint8_t)~0x02);
    Serial.printf("RDS DI Artificial Head updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_di_compressed") == 0) {
    bool on = (server.arg("rds_di_compressed") == "1");
    uecpRdsState.diPtyi = on ? (uecpRdsState.diPtyi | 0x04) : (uecpRdsState.diPtyi & (uint8_t)~0x04);
    Serial.printf("RDS DI Compressed updated to: %s\n", on ? "1" : "0");
  } else if (strcmp(field, "rds_ptyi") == 0) {
    bool on = (server.arg("rds_ptyi") == "1");
    uecpRdsState.diPtyi = on ? (uecpRdsState.diPtyi | 0x08) : (uecpRdsState.diPtyi & (uint8_t)~0x08);
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
  char stream_url[128];
  server.arg("stream_url").toCharArray(stream_url, sizeof(stream_url));
  char stream_volume[8];
  server.arg("stream_volume").toCharArray(stream_volume, sizeof(stream_volume));
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
  char uecp_site[8];
  server.arg("uecp_site").toCharArray(uecp_site, sizeof(uecp_site));
  char uecp_enc[8];
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
  bufAppend(response, sizeof(response), &rpos, "<p>Stream URL: %s</p>", stream_url);
  bufAppend(response, sizeof(response), &rpos, "<p>Stream Volume: %s</p>", stream_volume);
  bufAppend(response, sizeof(response), &rpos, "<p>Frequency: %s MHz</p>", frequency);
  bufAppend(response, sizeof(response), &rpos, "<p>Power%%: %s</p>", power);
  bufAppend(response, sizeof(response), &rpos, "<p>RDS PTY: %s</p>", rds_pty);
  bufAppend(response, sizeof(response), &rpos, "<p>RDS PS: %s</p>", rds_ps);
  bufAppend(response, sizeof(response), &rpos, "<p>RDS RT: %s</p>", rds_rt);
  bufAppend(response, sizeof(response), &rpos, "</body></html>");

  if (stream_url[0] != '\0' && strcmp(stream_url, currentStreamUrl) != 0) {
    snprintf(currentStreamUrl, sizeof(currentStreamUrl), "%s", stream_url);
    audio.stopSong();
    audio.connecttohost(currentStreamUrl);
  }

  if (stream_volume[0] != '\0') {
    currentVolume = atoi(stream_volume);
    if (currentVolume > 21) currentVolume = 21;
    audio.setVolume(currentVolume);
  }

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
    utf8ToEbu(rds_ps, uecpRdsState.psPending, RDS_PS_LEN);
    uecpRdsState.psPendingValid = true;
  }
  if (rds_rt[0] != '\0') {
    // See handleUpdate()'s "rds_rt" branch: the web interface only ever controls a single RT
    // message, so a new submission clears the whole buffer and becomes the sole message 0.
    memset(uecpRdsState.rt, 0, sizeof(uecpRdsState.rt));
    uecpRdsState.rt[0].textLen     = (uint8_t) utf8ToEbu(rds_rt, uecpRdsState.rt[0].text, RDS_RT_MAX_LEN);
    uecpRdsState.rt[0].repeatCount = 0;  // infinite
    uecpRdsState.rt[0].toggleAB    = true;
    uecpRdsState.rtCount           = 1;
    uecpRdsState.rtCurrent         = 0;
    uecpRdsState.rtSegment         = 0;
    uecpRdsState.rtRepeatsDone     = 0;
    uecpRdsState.rtABFlag          = !uecpRdsState.rtABFlag;
  }
  if (rds_rt_repeat[0] != '\0') uecpRdsState.rt[0].repeatCount = (uint8_t) atoi(rds_rt_repeat);

  if (rds_pi[0] != '\0') {
    uint16_t pi = (uint16_t) strtol(rds_pi, NULL, 16);
    uecpRdsState.pi[0] = (uint8_t)(pi >> 8);
    uecpRdsState.pi[1] = (uint8_t)(pi & 0xFF);
  }
  uecpRdsState.pty = (uint8_t) atoi(rds_pty);

  if (rds_dsn[0]   != '\0') uecpRdsState.dsn = (uint8_t) atoi(rds_dsn);
  if (rds_psn[0]   != '\0') uecpRdsState.psn = (uint8_t) atoi(rds_psn);
  if (uecp_site[0] != '\0') uecpOurSite    = (uint16_t) strtol(uecp_site, NULL, 16) & 0x03FF;
  if (uecp_enc[0]  != '\0') uecpOurEncoder = (uint8_t)  strtol(uecp_enc, NULL, 16) & 0x3F;
  if (rds_pin[0]   != '\0') {
    uint16_t pin = (uint16_t) strtol(rds_pin, NULL, 16);
    uecpRdsState.pin[0] = (uint8_t)(pin >> 8);
    uecpRdsState.pin[1] = (uint8_t)(pin & 0xFF);
  }

  // The flag selects are always present in a full-form submit (unlike native checkboxes,
  // which browsers omit from FormData when unchecked), so these are applied unconditionally.
  uecpRdsState.tatp = (strcmp(rds_ta, "1") == 0) ? (uecpRdsState.tatp | 0x01) : (uecpRdsState.tatp & (uint8_t)~0x01);
  uecpRdsState.tatp = (strcmp(rds_tp, "1") == 0) ? (uecpRdsState.tatp | 0x02) : (uecpRdsState.tatp & (uint8_t)~0x02);
  uecpRdsState.ms   = (strcmp(rds_ms, "1") == 0) ? (uecpRdsState.ms   | 0x01) : (uecpRdsState.ms   & (uint8_t)~0x01);
  uecpRdsState.diPtyi = (strcmp(rds_di_stereo,     "1") == 0) ? (uecpRdsState.diPtyi | 0x01) : (uecpRdsState.diPtyi & (uint8_t)~0x01);
  uecpRdsState.diPtyi = (strcmp(rds_di_artifhead,  "1") == 0) ? (uecpRdsState.diPtyi | 0x02) : (uecpRdsState.diPtyi & (uint8_t)~0x02);
  uecpRdsState.diPtyi = (strcmp(rds_di_compressed, "1") == 0) ? (uecpRdsState.diPtyi | 0x04) : (uecpRdsState.diPtyi & (uint8_t)~0x04);
  uecpRdsState.diPtyi = (strcmp(rds_ptyi,          "1") == 0) ? (uecpRdsState.diPtyi | 0x08) : (uecpRdsState.diPtyi & (uint8_t)~0x08);

  currentFreqDeviation  = atoi(frequency_deviation);
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
                         pendingImmediateGroupIndex, uecpOurSite, uecpOurEncoder);
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
  // Conecta na rede Wi-Fi
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

  // --- AUDIO SETUP ---
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(currentVolume);
  audio.connecttohost(currentStreamUrl);

  Serial.println("Trasmitting & Streaming...");
}

void loop() {
  audio.loop(); // Required to process the audio stream buffer continuously

  // 24/7 Watchdog: If the stream stops abruptly, try to reconnect every 10 seconds
  static unsigned long lastAudioCheck = 0;
  if (millis() - lastAudioCheck > 10000) {
    if (!audio.isRunning() && currentStreamUrl[0] != '\0') {
      Serial.println("Stream dropped. Reconnecting...");
      audio.connecttohost(currentStreamUrl);
    }
    lastAudioCheck = millis();
  }

  server.handleClient();
  handleUecpServer();
  rdsSchedulerApplyPendingTime(uecpRdsState);
  if (rawGroupMode) {
    processSerialGroups();
  } else {
    rdsSchedulerTick(tx, uecpRdsState, ffPool, pendingImmediateGroupIndex, computeLocalOffsetByte());
  }
}
