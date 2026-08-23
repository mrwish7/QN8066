#pragma once
#include <stdint.h>

// Tracks which RDS group each ODA (Open Data Application) AID is currently mapped to, as
// announced by live UECP MEC 0x24 messages for Group 3A - see uecp_handler.cpp's mec==0x24
// handling. Kept separate from FreeFormatPool's own Group 3A on-air queue: a one-shot 3A
// definition gets dequeued and freed from that queue after its single transmission, but the AID
// mapping it established must still be findable by a later MEC 0x46 (ODA data, implemented
// separately) message for the same AID - this directory is that persistent record, independent
// of the on-air queue's lifecycle. Entries are kept indefinitely once learned - the only thing
// that clears them is a Group 3A "wipe all" (MED mode 0b11) arriving, alongside
// ffClearGroup()'s own wipe of the on-air queue; nothing here expires or gets evicted on its own.
// Fixed-capacity, no heap - same rationale as FreeFormatPool, but small and simple enough (a
// handful of ODA apps in practice) that a flat linear-scan array is the right tool, not a pool.
#define ODA_LIVE_DIRECTORY_SIZE 16

struct OdaLiveEntry {
  uint16_t aid;
  uint8_t  groupIndex; // ffGroupIndex() packing - which group this AID's ODA data is on
  bool     inUse;
};

struct OdaLiveDirectory {
  OdaLiveEntry entries[ODA_LIVE_DIRECTORY_SIZE];
};

void odaLiveDirectoryInit(OdaLiveDirectory& dir);

// Upserts aid's mapping to groupIndex - called on every live Group 3A MEC 0x24 arrival (one-shot
// or cyclic alike, and regardless of whether that particular message also gets queued for on-air
// resend - see ffContainsExact()'s use in uecp_handler.cpp), since the mapping itself matters
// independently of the on-air queue's own dedup/repeat bookkeeping. If aid isn't already tracked
// and the table is full, the update is dropped (logged by the caller) rather than evicting an
// existing entry - keeping a possibly-stale mapping is safer than guessing which one to discard.
bool odaLiveDirectorySet(OdaLiveDirectory& dir, uint16_t aid, uint8_t groupIndex);

// Removes every tracked mapping - called only when a Group 3A MEC 0x24 "wipe all" (MED mode
// 0b11) arrives, alongside ffClearGroup() wiping the on-air queue. Nothing else clears entries.
void odaLiveDirectoryClear(OdaLiveDirectory& dir);

// Looks up aid's current mapping. Returns false (outGroupIndex untouched) if aid has never been
// seen (or was seen before the last wipe-all). For future MEC 0x46 use.
bool odaLiveDirectoryGet(const OdaLiveDirectory& dir, uint16_t aid, uint8_t* outGroupIndex);
