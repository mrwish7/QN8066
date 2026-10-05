#pragma once
#include <stdint.h>
#include <stddef.h>

// EBU/RDS extended Latin character set used for RDS PS/RT text (UECP "basic" character table).
// Bytes 0x00-0x7F map identically to ASCII/UTF-8 code points below 0x80.
// Bytes 0x80-0xFF map to the extended Latin/Greek/symbol table in ebu_charset.cpp.

// Converts a NUL-terminated UTF-8 string to EBU bytes. Writes at most outMax bytes into out
// (not NUL-terminated - out is a fixed-width byte buffer, e.g. RdsMainService::ps or RtMessage::text).
// Unrepresentable code points map to '?' (0x3F). Returns the number of bytes written.
size_t utf8ToEbu(const char* utf8, uint8_t* out, size_t outMax);

// Converts len EBU bytes to a NUL-terminated UTF-8 string. Writes at most outMax bytes into
// outUtf8 including the terminating NUL. Returns the number of bytes written, excluding the NUL.
size_t ebuToUtf8(const uint8_t* ebu, size_t len, char* outUtf8, size_t outMax);
