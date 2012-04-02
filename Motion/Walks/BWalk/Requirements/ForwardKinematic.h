/**
 * @file ForwardKinematic.h
 * @author <A href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</A>
 * @author <a href="mailto:afabisch@tzi.de">Alexander Fabisch</a>
 */

#pragma once

#include "Autoconfig/motionconfig.h"

#if defined(USE_MODEL_DARWIN)
#include "DarwinForwardKinematic.h"
typedef DarwinForwardKinematic ForwardKinematic;
#elif defined(USE_MODEL_NAO)
#include "NAOForwardKinematic.h"
typedef NAOForwardKinematic ForwardKinematic;
#endif
