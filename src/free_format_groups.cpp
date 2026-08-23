#include "free_format_groups.h"

static uint8_t ffAlloc(FreeFormatPool& pool) {
  uint8_t idx = pool.freeHead;
  if (idx == FF_INDEX_NONE) return FF_INDEX_NONE; // pool full
  pool.freeHead = pool.slots[idx].next;
  return idx;
}

static void ffFree(FreeFormatPool& pool, uint8_t idx) {
  pool.slots[idx].next = pool.freeHead;
  pool.freeHead = idx;
}

// Appends idx onto the tail of groupIndex's list - the normal case (arrival order preserved).
static void ffLinkAtTail(FreeFormatPool& pool, uint8_t groupIndex, uint8_t idx) {
  pool.slots[idx].next = FF_INDEX_NONE;
  uint8_t* linkFrom = &pool.groupHead[groupIndex];
  while (*linkFrom != FF_INDEX_NONE) linkFrom = &pool.slots[*linkFrom].next;
  *linkFrom = idx;
}

// Inserts idx at the head of groupIndex's list - used for FF_FLAG_PRIORITY (MEC 0x30 TMC's
// "urgent" bit): the next ffDequeueForSend() for this group picks this slot up immediately,
// ahead of whatever was already queued.
static void ffLinkAtHead(FreeFormatPool& pool, uint8_t groupIndex, uint8_t idx) {
  pool.slots[idx].next = pool.groupHead[groupIndex];
  pool.groupHead[groupIndex] = idx;
}

void ffPoolInit(FreeFormatPool& pool) {
  for (uint8_t i = 0; i < FF_GROUP_COUNT; i++) {
    pool.groupHead[i]  = FF_INDEX_NONE;
    pool.groupCount[i] = 0;
  }
  for (uint8_t i = 0; i < FF_POOL_SIZE; i++) {
    pool.slots[i].next = (uint8_t)((i + 1 < FF_POOL_SIZE) ? (i + 1) : FF_INDEX_NONE);
  }
  pool.freeHead = 0;
}

void ffClearGroup(FreeFormatPool& pool, uint8_t groupIndex) {
  uint8_t idx = pool.groupHead[groupIndex];
  while (idx != FF_INDEX_NONE) {
    uint8_t next = pool.slots[idx].next;
    ffFree(pool, idx);
    idx = next;
  }
  pool.groupHead[groupIndex]  = FF_INDEX_NONE;
  pool.groupCount[groupIndex] = 0;
}

FfEnqueueResult ffEnqueue(FreeFormatPool& pool, uint8_t groupIndex, uint8_t block2Last5,
                          uint16_t block3, uint16_t block4, bool cyclic, uint8_t flags) {
  bool cleared = false;
  if (pool.groupCount[groupIndex] >= FF_GROUP_MAX_SLOTS) {
    // groupIndex's own cap is already reached - rather than drop this new message and leave the
    // group cycling whatever old content happened to fill it first, wipe the whole queue and
    // start fresh with just this one. Caller logs FF_ENQUEUE_OK_CLEARED distinctly so an operator
    // can tell this group's own airtime (in the RDS group sequence) isn't keeping up with its
    // incoming message rate.
    ffClearGroup(pool, groupIndex);
    cleared = true;
  }

  uint8_t idx = ffAlloc(pool);
  if (idx == FF_INDEX_NONE) return FF_ENQUEUE_FAILED; // shared pool itself exhausted - caller logs

  FreeFormatSlot& slot = pool.slots[idx];
  slot.block2Last5 = block2Last5 & 0x1F;
  slot.block3      = block3;
  slot.block4      = block4;
  slot.cyclic      = cyclic;
  slot.flags       = flags;

  if (flags & FF_FLAG_PRIORITY) {
    ffLinkAtHead(pool, groupIndex, idx);
  } else {
    ffLinkAtTail(pool, groupIndex, idx);
  }
  pool.groupCount[groupIndex]++;
  return cleared ? FF_ENQUEUE_OK_CLEARED : FF_ENQUEUE_OK;
}

bool ffHasContent(const FreeFormatPool& pool, uint8_t groupIndex) {
  return pool.groupHead[groupIndex] != FF_INDEX_NONE;
}

bool ffGroupIsFull(const FreeFormatPool& pool, uint8_t groupIndex) {
  return pool.groupCount[groupIndex] >= FF_GROUP_MAX_SLOTS;
}

bool ffContainsExact(const FreeFormatPool& pool, uint8_t groupIndex, uint8_t block2Last5,
                     uint16_t block3, uint16_t block4) {
  uint8_t want = block2Last5 & 0x1F; // match ffEnqueue()'s own masking
  uint8_t idx = pool.groupHead[groupIndex];
  while (idx != FF_INDEX_NONE) {
    const FreeFormatSlot& slot = pool.slots[idx];
    if (slot.block2Last5 == want && slot.block3 == block3 && slot.block4 == block4) return true;
    idx = slot.next;
  }
  return false;
}

void ffClearMatchingBlock4(FreeFormatPool& pool, uint8_t groupIndex, uint16_t block4) {
  uint8_t* linkFrom = &pool.groupHead[groupIndex];
  while (*linkFrom != FF_INDEX_NONE) {
    uint8_t idx = *linkFrom;
    if (pool.slots[idx].block4 == block4) {
      *linkFrom = pool.slots[idx].next; // splice idx out - linkFrom now points at its old
                                          // successor, so don't advance; re-examine what's here
      ffFree(pool, idx);
      pool.groupCount[groupIndex]--;
    } else {
      linkFrom = &pool.slots[idx].next; // no match - move on to idx's own successor
    }
  }
}

FreeFormatSlot ffDequeueForSend(FreeFormatPool& pool, uint8_t groupIndex) {
  uint8_t idx = pool.groupHead[groupIndex];
  FreeFormatSlot result = pool.slots[idx]; // copy out - the slot may be freed/relinked below

  pool.groupHead[groupIndex] = pool.slots[idx].next; // unlink from the head either way

  if (result.cyclic) {
    ffLinkAtTail(pool, groupIndex, idx); // rotate to the back of this group's line - still
                                           // occupies a slot, so groupCount is unchanged
  } else {
    ffFree(pool, idx);
    pool.groupCount[groupIndex]--;
  }
  return result;
}
