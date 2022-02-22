#!/bin/bash
cd ..
SDK_PATH=`pwd`
VERSION=`cat $SDK_PATH/sdkVersion | sed -e 's/^[ \t]*//g' -e 's/[ \t]*$//g'`
#build sdk release

DEST_QT_PATH="/opt/Qt5.12.2/5.12.2/gcc_64"


if [ ! -d $DEST_QT_PATH ];then
    exit 1
fi
rm SdkRelease_v*  -rf
mkdir SdkRelease_v$VERSION
rm build -rf
mkdir build 
cd build

$DEST_QT_PATH/bin/qmake ../Release.pro "CONFIG+=release"
if [ $? -ne 0 ]; then
    exit 1
fi

make -f Makefile qmake_all
if [ $? -ne 0 ]; then
    exit 1
fi

make -f Makefile all -j4
if [ $? -ne 0 ]; then
    exit 1
fi

cd $SDK_PATH/Examples
rm build  -rf
mkdir build
cd build
cmake ..
make -j 4

cd $SDK_PATH
mv Runtime  SdkRelease_v$VERSION
tar -zvcf SdkRelease_v$VERSION.tar.gz SdkRelease_v$VERSION
