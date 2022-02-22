#include "rtdbareceiver.h"
#include <QTimer>
#include "message.h"
#include "rtdbkeys.h"
#include "realtimedatabase.h"
#include "protocol.h"
#include "satpext.h"

RtdbReceiver::RtdbReceiver(SATP::Protocol *protocol, QObject *parent)
    : QObject(parent),
      mProtocol(protocol)
{
    mRtdb = new RealtimeDatabase(false);
    protocol->registerBlockHandler(this);
    mNeedClear = false;
}

RtdbReceiver::~RtdbReceiver()
{
    mProtocol->unregisterBlockHandler(this);
}

bool RtdbReceiver::handleReceiveBlock(uint32_t dataType, const char *block, int size)
{
    UniqueKey key(dataType);

    switch (key.lowWord()) {
    case DataUnitTypeExt::Rtdb:
        handleRtdb(block, size);
        return true;
    default:
        break;
    }
    return false;
}

void RtdbReceiver::handleReady()
{
    QTimer::singleShot(0, this, [this](){
        mNeedClear = true;
        request();
    });
}

void RtdbReceiver::handleRtdb(const char *block, int size)
{
    if (mNeedClear) {
        mRtdb->clearData();
    }
    if (mRtdb->load(block, size)) {
        if (mNeedClear) {
            mNameKeyMap.clear();
            mNeedClear = false;
        }
        emit rtdbLoaded();
    }
}

void RtdbReceiver::handleMessage(int type, const char *message, int size)
{
    Q_UNUSED(message)
    Q_UNUSED(size)

    switch (type) {
    case MessageType::RtdbChanged:
        request();
        break;
    default:
        break;
    }
}

void RtdbReceiver::request()
{
    mProtocol->sendBlock(DataUnitTypeExt::RequestRtdb, nullptr, 0, SATP::Protocol::EnqueueForcedly);
}

void RtdbReceiver::clear()
{
    mNeedClear = true;
}

bool RtdbReceiver::isLoaded()
{
    return mRtdb->isLoaded();
}

QList<int> RtdbReceiver::getRtdbKeys()
{
    QList<quint16> keys = mRtdb->keys();

    QList<int> retKeys;

    for(quint16 key : keys) {
        retKeys.append((int)key);
    }

    return retKeys;
}

QString RtdbReceiver::getRtdbItemName(int key)
{
    RealtimeDatabase::RtdbItem rtdbItem;
    if (mRtdb->keys().contains(key) && mRtdb->getItem(key, rtdbItem)) {
        return rtdbItem.name;
    }
    return "";
}

quint16 RtdbReceiver::getKeyByName(const QString &name)
{
    if (mNameKeyMap.isEmpty()) {
        QList<quint16> keys = mRtdb->keys();
        for(quint16 key : keys) {
            RealtimeDatabase::RtdbItem rtdbItem;
            if (mRtdb->getItem(key, rtdbItem)) {
                mNameKeyMap[rtdbItem.name.toLower()] = key;
            }
        }
    }

    return mNameKeyMap[name.toLower()];
}

QVariant RtdbReceiver::getRtdbItemValue(int key)
{
    RealtimeDatabase::RtdbItem rtdbItem;
    QVariant variant;
    if (mRtdb->keys().contains(key) && mRtdb->getItem(key, rtdbItem)) {
        return rtdbItem.value;
    }
    return variant;
}

bool RtdbReceiver::getRtdbItemPersistent(int key)
{
    RealtimeDatabase::RtdbItem rtdbItem;
    if (mRtdb->getItem(key, rtdbItem)) {
        return rtdbItem.persistent;
    }
    return false;
}

QVariant RtdbReceiver::getRtdbValueByName(const QString &name)
{
    quint16 key = getKeyByName(name);
    return getRtdbItemValue(key);
}

static const int kMaxRtdbItemSize = 256;
void RtdbReceiver::saveRtdbItem(int key, QVariant value)
{
    static char buffer[kMaxRtdbItemSize];
    quint16 len = 0;
    mRtdb->buildItemData(key, value, buffer, &len);
    mProtocol->sendBlock(UniqueKey(DataUnitTypeExt::ChangeRtdbItem, key).longKey,
        buffer,
        len,
        SATP::Protocol::EnqueueForcedly);
}

void RtdbReceiver::saveRtdbItemByName(const QString &name, QVariant value)
{
    quint16 key = getKeyByName(name);
    if(getRtdbItemValue(key) != value){
        saveRtdbItem(key, value);
    }
}

