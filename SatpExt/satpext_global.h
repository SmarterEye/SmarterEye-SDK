#ifndef SATPEXT_GLOBAL_H
#define SATPEXT_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(SATPEXT_LIBRARY)
#  define SATPEXTSHARED_EXPORT Q_DECL_EXPORT
#else
#  define SATPEXTSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // SATPEXT_GLOBAL_H
