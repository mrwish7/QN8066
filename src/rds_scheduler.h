#pragma once
#include <QN8066.h>
#include "rds_state.h"
#include "free_format_groups.h"

// Non-blocking RDS group scheduler + transmitter. Extracted from what used to be duplicated
// (nearly verbatim) inline in each of this project's three ESP32 sketches - see any sketch's own
// setup()/loop() for how these are wired in. Everything except the four functions below (group
// building, the build/send buffer, chip-confirmation bookkeeping, the Clock Time minute latch) is
// a file-scope static inside rds_scheduler.cpp, not exposed here - each sketch is a completely
// separate compiled binary, so those statics are never shared *across* sketches despite the source
// being one shared copy; they're just hidden from sketches that have no business touching them.
//
// This owns none of the actual protocol/session state (RdsEncoderState, FreeFormatPool) or the chip
// handle (QN8066) - those stay declared in each sketch's own .ino (as they must: each sketch has
// its own hardware wiring and its own single instance of the RDS session) and are passed in by
// reference, the same convention uecp_handler.h's processUecpFrame() already uses.

// Enables RDS transmission on the chip and gives it its own settling delay - call once from
// setup(), and again anywhere rdsSchedulerReset() is called (see that function's own comment).
// Also configures rdsSetSyncTime(), which only affects the QN8066 library's own blocking
// rdsSendGroup() call - this scheduler never uses it, but a sketch with its own raw-RDS-group-
// testing feature that calls the blocking API directly still needs it set; harmless otherwise.
void rdsSchedulerStart(QN8066& tx);

// Discards anything queued in the internal build/send buffer and any in-flight chip-confirmation
// wait. Call this alongside rdsSchedulerStart() after tx.setPAC() (or anything else that resets the
// chip's own RDS state out from under this scheduler) - setPAC() toggles the chip's SYSTEM1.recal
// bit, a real on-chip reset that invalidates whatever this scheduler's buffer/pending-send
// bookkeeping thought was true.
void rdsSchedulerReset();

// Applies a UECP MEC 0x0D (Time) update that's been staged in state by uecp_handler.cpp's
// uecpApplyTime() - the only place allowed to call settimeofday(), since uecp_handler.cpp itself
// (also ported into this project's Linux UECP test harness, which must never touch a real host
// system clock) never touches the system clock directly. Call every loop() iteration; near-instant
// once state.ctPending is set, a no-op otherwise.
void rdsSchedulerApplyPendingTime(RdsEncoderState& state);

// The whole per-loop-iteration RDS pipeline in one call: builds the next group if there's room in
// the internal buffer - deciding what to build exactly like this project's old per-sketch
// buildNextGroup() did (Group 4A on a real UTC minute boundary first, then a pending MEC 0x46
// "immediate" message if one's set, then the active data set's rdsSequence walk - free-format content always
// winning over struct-backed fallbacks for whichever group comes up) - then services the chip via
// the non-blocking async API, waiting for the chip's own confirmation (never an arbitrary timeout
// on the normal path) before sending the next one. Never delay()/blocks, so loop() stays fully
// responsive to serial/UECP/web input throughout.
//
// fallbackOffsetByte: the Group 4A local-time-offset byte to use until a real UECP MEC 0x0D Time
// message has ever set state.ctOffset (state.ctFromUecp) - each sketch computes this from its own
// hardcoded local-time constants (see each .ino's computeLocalOffsetByte()), since that's a
// deployment/location property, not something this shared scheduler should own a single value for.
void rdsSchedulerTick(QN8066& tx, RdsEncoderState& state, FreeFormatPool& ffPool,
                      uint8_t& pendingImmediateGroupIndex, uint8_t fallbackOffsetByte);
