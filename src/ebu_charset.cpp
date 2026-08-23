#include "ebu_charset.h"

// Unicode code points for EBU bytes 0x80-0xFF, in order (index 0 = byte 0x80).
static const uint16_t EBU_CODEPOINTS[128] = {
  0x00E1, 0x00E0, 0x00E9, 0x00E8, 0x00ED, 0x00EC, 0x00F3, 0x00F2, 0x00FA, 0x00F9, 0x00D1, 0x00C7, 0x015E, 0x00DF, 0x00A1, 0x0132,
  0x00E2, 0x00E4, 0x00EA, 0x00EB, 0x00EE, 0x00EF, 0x00F4, 0x00F6, 0x00FB, 0x00FC, 0x00F1, 0x00E7, 0x015F, 0x011F, 0x0131, 0x0133,
  0x00AA, 0x03B1, 0x00A9, 0x2030, 0x011E, 0x011B, 0x0148, 0x0151, 0x03C0, 0x20AC, 0x00A3, 0x0024, 0x2190, 0x2191, 0x2192, 0x2193,
  0x2070, 0x00B9, 0x00B2, 0x00B3, 0x00B1, 0x0130, 0x0144, 0x0171, 0x03BC, 0x00BF, 0x00F7, 0x00B0, 0x00BC, 0x00BD, 0x00BE, 0x00A7,
  0x00C1, 0x00C0, 0x00C9, 0x00C8, 0x00CD, 0x00CC, 0x00D3, 0x00D2, 0x00DA, 0x00D9, 0x0158, 0x010C, 0x0160, 0x017D, 0x0110, 0x013F,
  0x00C2, 0x00C4, 0x00CA, 0x00CB, 0x00CE, 0x00CF, 0x00D4, 0x00D6, 0x00DB, 0x00DC, 0x0159, 0x010D, 0x0161, 0x017E, 0x0111, 0x006C,
  0x00C3, 0x00C5, 0x00C6, 0x0152, 0x0177, 0x00DD, 0x00D5, 0x00D8, 0x00DE, 0x014A, 0x0154, 0x0106, 0x015A, 0x0179, 0x0166, 0x00F0,
  0x00E3, 0x00E5, 0x00E6, 0x0153, 0x0175, 0x00FD, 0x00F5, 0x00F8, 0x00FE, 0x014B, 0x0155, 0x0107, 0x015B, 0x017A, 0x0167, 0x00FF
};

static int ebuByteForCodepoint(uint32_t cp) {
  for (int i = 0; i < 128; i++) {
    if (EBU_CODEPOINTS[i] == cp) return 0x80 + i;
  }
  return -1;
}

// Decodes one UTF-8 code point starting at *p and advances *p past it.
// Returns 0xFFFFFFFF (and advances by 1 byte) on a malformed/truncated lead byte.
static uint32_t utf8Decode(const char** p) {
  const uint8_t* s = (const uint8_t*)*p;
  uint8_t b0 = s[0];
  if (b0 < 0x80) { *p += 1; return b0; }
  if ((b0 & 0xE0) == 0xC0 && s[1]) {
    uint32_t cp = ((uint32_t)(b0 & 0x1F) << 6) | (s[1] & 0x3F);
    *p += 2; return cp;
  }
  if ((b0 & 0xF0) == 0xE0 && s[1] && s[2]) {
    uint32_t cp = ((uint32_t)(b0 & 0x0F) << 12) | ((uint32_t)(s[1] & 0x3F) << 6) | (s[2] & 0x3F);
    *p += 3; return cp;
  }
  if ((b0 & 0xF8) == 0xF0 && s[1] && s[2] && s[3]) {
    uint32_t cp = ((uint32_t)(b0 & 0x07) << 18) | ((uint32_t)(s[1] & 0x3F) << 12) | ((uint32_t)(s[2] & 0x3F) << 6) | (s[3] & 0x3F);
    *p += 4; return cp;
  }
  *p += 1;
  return 0xFFFFFFFF;
}

size_t utf8ToEbu(const char* utf8, uint8_t* out, size_t outMax) {
  size_t n = 0;
  const char* p = utf8;
  while (*p && n < outMax) {
    uint32_t cp = utf8Decode(&p);
    if (cp < 0x80) {
      out[n++] = (uint8_t)cp;
    } else {
      int b = ebuByteForCodepoint(cp);
      out[n++] = (b >= 0) ? (uint8_t)b : '?';
    }
  }
  return n;
}

static size_t utf8Encode(uint32_t cp, char* out) {
  if (cp < 0x80) { out[0] = (char)cp; return 1; }
  if (cp < 0x800) {
    out[0] = (char)(0xC0 | (cp >> 6));
    out[1] = (char)(0x80 | (cp & 0x3F));
    return 2;
  }
  out[0] = (char)(0xE0 | (cp >> 12));
  out[1] = (char)(0x80 | ((cp >> 6) & 0x3F));
  out[2] = (char)(0x80 | (cp & 0x3F));
  return 3;
}

size_t ebuToUtf8(const uint8_t* ebu, size_t len, char* outUtf8, size_t outMax) {
  size_t n = 0;
  for (size_t i = 0; i < len; i++) {
    if (n + 4 > outMax) break; // leave room for worst case 3 bytes + NUL
    uint8_t b = ebu[i];
    uint32_t cp = (b < 0x80) ? b : EBU_CODEPOINTS[b - 0x80];
    n += utf8Encode(cp, outUtf8 + n);
  }
  outUtf8[n] = '\0';
  return n;
}
