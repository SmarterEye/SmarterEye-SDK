﻿#ifdef _MSC_VER
#pragma execution_character_set("utf-8")
#endif

#include <QCoreApplication>
#include <QDebug>
#include "testserver.h"
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <iostream>

void customMessageHandler(QtMsgType type, const QMessageLogContext &context,const QString & msg)
{
        QString txt;
        switch (type) {
        //调试信息提示
        case QtDebugMsg:
//            txt = QString("%1: Debug: at:%2,%3 on %4; %5").arg(QTime::currentTime().toString("hh:mm:ss.zzz"))
//                    .arg(context.file).arg(context.line).arg(context.function).arg(msg);
            txt = msg;
                break;

        //一般的warning提示
        case QtWarningMsg:
                txt = QString("%1:Warning: at:%2,%3 on %4; %5").arg(QTime::currentTime().toString("hh:mm:ss.zzz"))
                        .arg(context.file).arg(context.line).arg(context.function).arg(msg);
        break;
        //严重错误提示
        case QtCriticalMsg:
                txt = QString("%1:Critical: at:%2,%3 on %4; %5").arg(QTime::currentTime().toString("hh:mm:ss.zzz"))
                        .arg(context.file).arg(context.line).arg(context.function).arg(msg);
        break;
        //致命错误提示
        case QtFatalMsg:
                txt = QString("%1:Fatal: at:%2,%3 on %4; %5").arg(QTime::currentTime().toString("hh:mm:ss.zzz"))
                        .arg(context.file).arg(context.line).arg(context.function).arg(msg);
                abort();
        }
        QFile outFile(QString("%1/%2.txt").arg(QDir::currentPath()).arg(QDate::currentDate().toString("yyyy-MM-dd")));
        outFile.open(QIODevice::WriteOnly | QIODevice::Append);
        QTextStream ts(&outFile);
        ts << txt << endl;
}


int main(int argc, char *argv[])
{
//    qInstallMessageHandler(customMessageHandler);
    QCoreApplication a(argc, argv);
    
    TestServer server;
    std::cout << "Begin Listen,The log >> file. The File Name is Date: yyyy-MM-dd.txt" << std::endl;
    server.listen();
    return a.exec();
}
