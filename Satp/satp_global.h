#ifndef SATP_GLOBAL_H
#define SATP_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(SATP_LIBRARY)
#  define SATPSHARED_EXPORT Q_DECL_EXPORT
#else
#  define SATPSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // SATP_GLOBAL_H
