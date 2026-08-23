#pragma once
#include <QN8066.h>

// One RDS group's worth of blocks, queued (in rds_scheduler.cpp) between being built (content/
// scheduling logic - buildNextGroup()) and actually sent (timing-critical chip handshake -
// serviceRdsTx()). Originally split into its own header specifically to dodge an Arduino IDE
// quirk (auto-generated function prototypes get hoisted above all user code in a .ino, including
// any struct defined inline there, which then fails "does not name a type" for whichever function
// used it first) - now living in a plain library .cpp rather than a .ino, that quirk no longer
// applies here at all, but the struct stays in its own small header regardless: it's a clean,
// reusable type with nothing else it needs to live next to.
struct RdsGroupBlocks {
  RDS_BLOCK1 b1;
  RDS_BLOCK2 b2;
  RDS_BLOCK3 b3;
  RDS_BLOCK4 b4;
};
