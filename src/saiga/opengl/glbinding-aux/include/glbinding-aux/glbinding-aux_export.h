﻿
#ifndef GLBINDING_AUX_API_H
#define GLBINDING_AUX_API_H

// Modified the orignal file here to use the saiga export macros
#include "saiga/export.h"
#define GLBINDING_AUX_API SAIGA_GLOBAL
#define GLBINDING_AUX_NO_EXPORT SAIGA_LOCAL

#ifndef GLBINDING_AUX_DEPRECATED
#  define GLBINDING_AUX_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef GLBINDING_AUX_DEPRECATED_EXPORT
#  define GLBINDING_AUX_DEPRECATED_EXPORT GLBINDING_AUX_API GLBINDING_AUX_DEPRECATED
#endif

#ifndef GLBINDING_AUX_DEPRECATED_NO_EXPORT
#  define GLBINDING_AUX_DEPRECATED_NO_EXPORT GLBINDING_AUX_NO_EXPORT GLBINDING_AUX_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef GLBINDING_AUX_NO_DEPRECATED
#    define GLBINDING_AUX_NO_DEPRECATED
#  endif
#endif

#endif
