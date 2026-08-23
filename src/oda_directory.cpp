#include "oda_directory.h"

void odaLiveDirectoryInit(OdaLiveDirectory& dir) {
  for (uint8_t i = 0; i < ODA_LIVE_DIRECTORY_SIZE; i++) {
    dir.entries[i].inUse = false;
  }
}

bool odaLiveDirectorySet(OdaLiveDirectory& dir, uint16_t aid, uint8_t groupIndex) {
  int16_t freeSlot = -1;
  for (uint8_t i = 0; i < ODA_LIVE_DIRECTORY_SIZE; i++) {
    if (dir.entries[i].inUse && dir.entries[i].aid == aid) {
      dir.entries[i].groupIndex = groupIndex; // already tracked - just refresh the mapping
      return true;
    }
    if (!dir.entries[i].inUse && freeSlot < 0) freeSlot = (int16_t)i;
  }
  if (freeSlot < 0) return false; // full, and aid isn't already in it - drop (caller logs this)

  OdaLiveEntry& entry = dir.entries[freeSlot];
  entry.aid        = aid;
  entry.groupIndex = groupIndex;
  entry.inUse      = true;
  return true;
}

void odaLiveDirectoryClear(OdaLiveDirectory& dir) {
  odaLiveDirectoryInit(dir);
}

bool odaLiveDirectoryGet(const OdaLiveDirectory& dir, uint16_t aid, uint8_t* outGroupIndex) {
  for (uint8_t i = 0; i < ODA_LIVE_DIRECTORY_SIZE; i++) {
    if (dir.entries[i].inUse && dir.entries[i].aid == aid) {
      *outGroupIndex = dir.entries[i].groupIndex;
      return true;
    }
  }
  return false;
}
