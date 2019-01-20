#ifndef DOBOTDLL_GLOBAL_H
#define DOBOTDLL_GLOBAL_H

//#include <QtCore/qglobal.h>

#if defined(DOBOTDLL_LIBRARY)
#  define DOBOTDLLSHARED_EXPORT __attribute__((visibility("default")))
#else
#  define DOBOTDLLSHARED_EXPORT __attribute__((visibility("default")))
#endif

#endif // DOBOTDLL_GLOBAL_H
