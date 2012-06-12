#ifndef DATAWRAPPERCURRENT_H
#define DATAWRAPPERCURRENT_H

#ifdef TARGET_IS_PC
    #include "Vision/VisionWrapper/datawrapperpc.h"
#elif TARGET_IS_NUVIEW
    #include "Vision/VisionWrapper/datawrappernuview.h"
#else
    #include "Vision/VisionWrapper/datawrapperdarwin.h"
#endif

#endif // DATAWRAPPERCURRENT_H
