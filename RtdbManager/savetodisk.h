#ifndef SAVETODISK_H
#define SAVETODISK_H
#include <stdlib.h>
inline void saveToDisk(){
#ifdef Q_OS_LINUX
    system("sync");
#endif
}
#endif // SAVETODISK_H
