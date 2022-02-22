﻿#ifndef QASIOSOCKET_H
#define QASIOSOCKET_H
#include <QtGlobal>
#include <QObject>
#include <QString>
#include <QByteArray>
#include <QDebug>

#if  defined(QASIOSOCKET_NOLIB)

#define QASIOSOCKET_EXPORT

#elif defined(QASIOSOCKET_LIBRARY)

#define QASIOSOCKET_EXPORT Q_DECL_EXPORT

#else

#define QASIOSOCKET_EXPORT Q_DECL_IMPORT

#endif


//#define QASIOSOCKET_EXPORT Q_CORE_EXPORT

#endif
