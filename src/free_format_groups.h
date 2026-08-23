#pragma once
#include <stdint.h>

// Pool-backed, per-group queues for UECP MEC 0x24 (Free-Format Group) content, and the framework
// this is meant to generalize into for Open Data Applications / future MEC 0x4n commands.
//
// Design: a single fixed-capacity slab pool (FF_POOL_SIZE slots) shared across all 32 possible
// RDS groups (group type 0-15 x version A/B), with each group's queued messages threaded through
// the pool via intrusive "next" indices rather than 32 separate pre-allocated arrays. A 32-entry
// head-index table (groupHead[]) says where (if anywhere) each group's list starts. Allocating a
// slot is an O(1) pop off a free list threaded the same way; freeing is an O(1) push back onto it.
// No heap: the whole pool is one static array, sized once at compile time - RAM cost is
// FF_POOL_SIZE * sizeof(FreeFormatSlot) regardless of how many groups are actually in use, which
// matters here because in practice only a handful of groups (if any) ever carry free-format/ODA
// content at once - a 32-groups x N-slots-each design would mostly sit empty.

// 32 possible RDS groups: group type (0-15) x version (A/B). See ffGroupIndex()/ffGroupType()/
// ffGroupIsVersionB() below for the packing.
#define FF_GROUP_COUNT 32

// Total message slots shared across all 32 groups - tune to taste.
#define FF_POOL_SIZE 128

// Per-group cap, independent of overall pool occupancy - without this, one group that's never
// cleared (especially via cyclic entries, which never free themselves on their own - see
// FreeFormatSlot::cyclic) could otherwise monopolise the entire shared pool and starve every
// other group's ffEnqueue() calls too. 128/32 means at most 4 groups can be simultaneously maxed
// out before the pool is fully committed, which is an accepted tradeoff of sharing one pool
// across 32 groups rather than a guarantee that unrelated groups are never affected by heavy use
// of a few - it just stops a single misbehaving/never-cleared group from doing so alone. Reaching
// this cap doesn't drop the new message (see ffEnqueue()) - it wipes the group's existing queue
// to make room, on the theory that a group filling faster than it drains is better served by
// fresh content than by whatever happened to arrive first.
#define FF_GROUP_MAX_SLOTS 32
#if FF_GROUP_MAX_SLOTS > FF_POOL_SIZE
#error "FF_GROUP_MAX_SLOTS can't exceed FF_POOL_SIZE - a single group could never reach its own cap otherwise"
#endif

// "No slot" / "end of list" sentinel - FF_POOL_SIZE must stay below this.
#define FF_INDEX_NONE 0xFF

// FreeFormatSlot::flags bit(s). FF_FLAG_PRIORITY makes ffEnqueue() insert at the head of the
// group's queue instead of the tail, so the very next ffDequeueForSend() for that group returns
// it - used by MEC 0x30 TMC's "urgent" bit (see uecp_handler.cpp). Available to any future MEC
// 0x4n that needs the same "transmit this next" semantics.
#define FF_FLAG_PRIORITY 0x01

// One queued free-format message. Kept flat (pool-relative index, not a pointer) so the whole
// pool can be one contiguous static array.
struct FreeFormatSlot {
  uint8_t  block2Last5; // Block 2 bits4-0 (the MED byte's own bits4-0, passed through as-is)
  uint16_t block3;
  uint16_t block4;
  bool     cyclic;      // true = stays queued and rotates to the back of the line after being
                         // sent (MED mode 0b10); false = one-shot, freed immediately after its
                         // one transmission (MED mode 0b00)
  uint8_t  flags;        // FF_FLAG_* - reserved, unused today
  uint8_t  next;          // next slot in this group's queue, or in the free list; FF_INDEX_NONE
                           // marks the end of either
};

struct FreeFormatPool {
  FreeFormatSlot slots[FF_POOL_SIZE];
  uint8_t        groupHead[FF_GROUP_COUNT];  // per-group queue head, or FF_INDEX_NONE if empty
  uint8_t        groupCount[FF_GROUP_COUNT]; // per-group slot count, enforcing FF_GROUP_MAX_SLOTS
                                               // independently of overall pool occupancy
  uint8_t        freeHead;                    // free-slot list head, or FF_INDEX_NONE if full
};

// Packs/unpacks the UECP MEC 0x24 Group byte's (groupType, versionB) pair into/from the single
// 0-31 index groupHead[]/ffHasContent()/etc. use to identify a group.
inline uint8_t ffGroupIndex(uint8_t groupType, bool versionB) {
  return (uint8_t)((groupType << 1) | (versionB ? 1 : 0));
}
inline uint8_t ffGroupType(uint8_t groupIndex)     { return groupIndex >> 1; }
inline bool    ffGroupIsVersionB(uint8_t groupIndex) { return (groupIndex & 0x01) != 0; }

void ffPoolInit(FreeFormatPool& pool);

// MEC 0x24 MED mode 0b11 - clears every queued message for one group, returning its slots to the
// free list.
void ffClearGroup(FreeFormatPool& pool, uint8_t groupIndex);

// Result of ffEnqueue() - distinguishes a plain success from one that had to clear groupIndex's
// existing queue first to make room, so callers can log that distinctly (an operator seeing
// FF_ENQUEUE_OK_CLEARED repeatedly for one group knows that group needs more airtime in the
// schedule, not just that "something happened").
enum FfEnqueueResult {
  FF_ENQUEUE_OK,          // queued normally - groupIndex had room already
  FF_ENQUEUE_OK_CLEARED,  // groupIndex was already at FF_GROUP_MAX_SLOTS, so its whole existing
                           // queue was dropped to make room before this message was queued
  FF_ENQUEUE_FAILED       // nothing queued - the shared pool has no free slots left even after
                           // clearing groupIndex's own queue (i.e. other groups collectively hold
                           // every remaining slot)
};

// MEC 0x24 MED mode 0b00 (cyclic=false, one-shot) or 0b10 (cyclic=true) - queues one message,
// normally at the tail of groupIndex's list (arrival order), or at the head if flags has
// FF_FLAG_PRIORITY set (the next ffDequeueForSend() for this group returns it immediately). If
// groupIndex is already at FF_GROUP_MAX_SLOTS, its entire existing queue is cleared first (see
// FF_GROUP_MAX_SLOTS's own comment) so this message can still go in - the data stays current
// instead of getting stuck. Only returns FF_ENQUEUE_FAILED (nothing queued) if the shared pool
// itself is exhausted even after that clear. Never blocks, never heap-allocates.
FfEnqueueResult ffEnqueue(FreeFormatPool& pool, uint8_t groupIndex, uint8_t block2Last5,
                          uint16_t block3, uint16_t block4, bool cyclic, uint8_t flags = 0);

bool ffHasContent(const FreeFormatPool& pool, uint8_t groupIndex);

// True if groupIndex's queue already contains a slot with this exact (block2Last5, block3,
// block4) combination. Used by Group 3A's dedup-before-enqueue policy (see uecp_handler.cpp) to
// avoid piling up duplicate entries when a UECP source periodically resends an unchanged ODA
// definition - generically useful, not 3A-specific itself.
bool ffContainsExact(const FreeFormatPool& pool, uint8_t groupIndex, uint8_t block2Last5,
                     uint16_t block3, uint16_t block4);

// Removes every queued message in groupIndex's queue whose block4 equals the given value,
// returning their slots to the free list - a narrower counterpart to ffClearGroup()'s whole-group
// wipe. Used by MEC 0x40's AID-scoped clear (block4 happens to carry the ODA AID for Group 3A
// messages, but this itself is generic - matches any block4 value in any group).
void ffClearMatchingBlock4(FreeFormatPool& pool, uint8_t groupIndex, uint16_t block4);

// True once groupIndex already holds FF_GROUP_MAX_SLOTS messages - i.e. the next ffEnqueue() for
// this group will clear it first (see FF_ENQUEUE_OK_CLEARED). Doesn't itself enforce or change
// anything; a plain read-only query for callers that want to know ahead of time.
bool ffGroupIsFull(const FreeFormatPool& pool, uint8_t groupIndex);

// Removes and returns the message at the head of groupIndex's queue. A cyclic message is
// re-queued at the tail (so a multi-message cycle advances to the next one next time instead of
// repeating); a one-shot message is freed instead. Caller must have already checked
// ffHasContent() - calling this on an empty group is undefined (there's no slot to return).
FreeFormatSlot ffDequeueForSend(FreeFormatPool& pool, uint8_t groupIndex);
