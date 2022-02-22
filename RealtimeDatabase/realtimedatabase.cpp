#include <QDateTime>
#include <QSharedMemory>
#include "realtimedatabase.h"

static const int kMaxRtdbStringLength = 126;
static const char *kRealtimeDatabaseKey = "MyRealTimeDataBaseObject";

#pragma pack(push, 1)

struct IndexNode
{
    quint16 key;
    quint16 offset;
};

struct MemoryString
{
    quint16 length;
    QChar data[kMaxRtdbStringLength];
};

struct MemoryItemBase
{
    quint16 type;
    bool persistent;
    MemoryString name;
};

struct MemoryValueItem : public MemoryItemBase
{
    char data[1];
};

struct MemoryStringItem : public MemoryItemBase
{
    MemoryString mstring;
};


#pragma pack(pop)

RealtimeDatabase::RealtimeDatabase(bool isServer)
{
    mIsServer = isServer;
    if (!mIsServer) {
        mData = nullptr;
        mSize = 0;
    } else {
        mSharedMemory = new QSharedMemory(kRealtimeDatabaseKey);
        mTemporaryCache = new QByteArray();
    }
}

RealtimeDatabase::~RealtimeDatabase()
{
    finish();
    if (mIsServer) {
        delete mSharedMemory;
        delete mTemporaryCache;
    }
}

void RealtimeDatabase::finish()
{
    if (mIsServer) {
        mSharedMemory->detach();
    }
}

bool RealtimeDatabase::isLoaded()
{
    if (!mIsServer) {
        return nullptr != mData;
    } else {
        return mSharedMemory->isAttached();
    }
}

void RealtimeDatabase::addItem(const RtdbItem &item)
{
    switch (item.type) {
    case RealtimeDatabase::BOOL:
    case RealtimeDatabase::UINT16:
    case RealtimeDatabase::INT16:
    case RealtimeDatabase::INT32:
    case RealtimeDatabase::INT64:
        {
            qint64 val = item.value.toLongLong();
            addValueItem(item.key, item.name, RealtimeDatabase::INT64, item.persistent, val);
        }
        break;

    case RealtimeDatabase::FLOAT:
    case RealtimeDatabase::DOUBLE:
        {
            double val = item.value.toDouble();
            addValueItem(item.key, item.name, RealtimeDatabase::DOUBLE, item.persistent, val);
        }
        break;

    case RealtimeDatabase::STRING:
        {
            QString val = item.value.toString();
            addStringItem(item.key, item.name, item.persistent, val);
        }
        break;

    default:
        Q_ASSERT(false);
        break;
    }
}

static const int kMaxValueItemDataSize = 8;

void RealtimeDatabase::addInternalItem(quint16 key, const QString name, quint16 type, bool persistent, const char *data, int size)
{
    if (mItemOffsets.contains(key)) return;

    int itemSize = sizeof(MemoryValueItem) - 1 + size;
    static MemoryValueItem *item = (MemoryValueItem *)
            (new char[sizeof(MemoryValueItem) -1 + kMaxValueItemDataSize]);
    item->type = type;
    item->persistent = persistent;
    copyQStringToMString(name, item->name);
    memcpy(item->data, data, size);
    mItemOffsets.insert(key, mTemporaryCache->size());
    mTemporaryCache->append((const char *)item,itemSize);
}

void RealtimeDatabase::addStringItem(quint16 key, const QString name, bool persistent, const QString &string)
{
    if (mItemOffsets.contains(key)) return;

    static MemoryStringItem *item = new MemoryStringItem;
    item->type = RealtimeDatabase::STRING;
    item->persistent = persistent;
    copyQStringToMString(name, item->name);
    copyQStringToMString(string, item->mstring);
    mItemOffsets.insert(key, mTemporaryCache->size());
    mTemporaryCache->append((const char *)item, sizeof(MemoryStringItem));
}

void RealtimeDatabase::putItemData(quint16 key, const char *data, quint16 len)
{
    QMutexLocker locker(&mAccessMutex);
    if (!isLoaded() && !load()) return;

    MemoryItemBase *item = (MemoryItemBase *)getInternalItem(key);
    MemoryValueItem *valueItem = (MemoryValueItem *)item;
    memcpy(valueItem->data, data, len);
}

void RealtimeDatabase::putItemImpl(quint16 key, const Dummy<QString> &dummy)
{
    QMutexLocker locker(&mAccessMutex);
    if (!isLoaded() && !load()) return;

    MemoryStringItem *item = (MemoryStringItem *)getInternalItem(key);
    copyQStringToMString(dummy.value, item->mstring);
}

void RealtimeDatabase::buildItemData(quint16 key, const QVariant &value, char *data, quint16 *len)
{
    QMutexLocker locker(&mAccessMutex);
    if (!isLoaded() && !load()) {
        return;
    }

    MemoryItemBase *item = (MemoryItemBase *)getInternalItem(key);
    Q_ASSERT(item);
    switch (item->type) {
    case RealtimeDatabase::BOOL:
    case RealtimeDatabase::UINT16:
    case RealtimeDatabase::INT16:
    case RealtimeDatabase::INT32:
    case RealtimeDatabase::INT64:
        {
            if (data) {
                *len = sizeof(qint64);
                putValueItemData(data, value.toLongLong());
            }
        }
        break;
    case RealtimeDatabase::FLOAT:
    case RealtimeDatabase::DOUBLE:
        {
            if (data) {
                *len = sizeof(double);
                putValueItemData(data, value.toDouble());
            }
        }
        break;
    case RealtimeDatabase::STRING:
        {
            if (data) {
                MemoryString & mstr = *(MemoryString*)data;
                copyQStringToMString(value.toString(), mstr);
                *len = mstr.length * 2 + 2;
            }
        }
        break;
    default:
        Q_ASSERT(false);
        break;
    }
}

bool RealtimeDatabase::getItem(quint16 key, RtdbItem &rtdbItem)
{
    QMutexLocker locker(&mAccessMutex);
    if (!isLoaded() && !load()) {
        return false;
    }

    MemoryItemBase *item = (MemoryItemBase *)getInternalItem(key);

    rtdbItem.key = key;
    rtdbItem.name = copyMStringToQString(item->name);
    rtdbItem.type = item->type;
    rtdbItem.persistent = item->persistent;

    switch (item->type) {
    case RealtimeDatabase::BOOL:
    case RealtimeDatabase::UINT16:
    case RealtimeDatabase::INT16:
    case RealtimeDatabase::INT32:
    case RealtimeDatabase::INT64:
        {
            MemoryValueItem *valueItem = (MemoryValueItem *)item;
            rtdbItem.value = getValueItemData(valueItem->data, (qint64)0);
        }
        break;
    case RealtimeDatabase::FLOAT:
    case RealtimeDatabase::DOUBLE:
        {
            MemoryValueItem *valueItem = (MemoryValueItem *)item;
            rtdbItem.value = getValueItemData(valueItem->data, (double)0.0);
        }
        break;

    case RealtimeDatabase::STRING:
        {
            MemoryStringItem *stringItem = (MemoryStringItem *)item;
            rtdbItem.value = copyMStringToQString(stringItem->mstring);
        }
        break;
    default:
        Q_ASSERT(false);
        rtdbItem.value = QVariant(QVariant::Invalid);
        break;
    }
    return true;
}

MemoryItemBase* RealtimeDatabase::getInternalItem(quint16 key)
{
    char *headData = (char *)(data());
    quint16 curOffset = mItemOffsets.value(key);
    headData += curOffset;

    return (MemoryItemBase *)headData;
}

char *RealtimeDatabase::getInternalItemData(quint16 key)
{
    MemoryValueItem *item = (MemoryValueItem *)getInternalItem(key);
    return item->data;
}

QString RealtimeDatabase::getStringItem(quint16 key)
{
    QMutexLocker locker(&mAccessMutex);
    if (!isLoaded() && !load()) {
        return "";
    }

    MemoryStringItem *item = (MemoryStringItem *)getInternalItem(key);
    return copyMStringToQString(item->mstring);
}

bool RealtimeDatabase::create()
{
    if (!mIsServer) return true;

    if (mSharedMemory->attach()) {
        return load();
    }

    int headSize = sizeof(quint16);
    int indexSize = mItemOffsets.size() * sizeof(IndexNode);
    int itemSize = mTemporaryCache->size();
    mSharedMemory->create(headSize + indexSize + itemSize);

    mSharedMemory->lock();

    char *head = (char *)(mSharedMemory->data());
    qint16 *itemCount = (qint16*)head;
    *itemCount = mItemOffsets.size();

    char *indexSection = head + sizeof(quint16);
    char *itemSection = indexSection + mItemOffsets.size()*sizeof(IndexNode);

    QHashIterator<quint16, quint16> i(mItemOffsets);
    while (i.hasNext()) {
        i.next();

        IndexNode *indexNode = (IndexNode*)indexSection;

        indexNode->key = i.key();
        indexNode->offset = i.value() + sizeof(quint16) + mItemOffsets.size()*sizeof(IndexNode);;

        mItemOffsets.insert(indexNode->key,indexNode->offset);

        indexSection += sizeof(IndexNode);
    }

    memcpy(itemSection, mTemporaryCache->data(), mTemporaryCache->size());
    mTemporaryCache->clear();
    mSharedMemory->unlock();
    return true;
}

void RealtimeDatabase::restoreFromMemory()
{
    char *head = (char *)(data());
    char *indexSection = head + sizeof(quint16);
    quint16 itemCount = *((quint16*)head);

    for (int i = 0; i < itemCount; i++) {
        IndexNode *indexNode = (IndexNode *)indexSection;
        mItemOffsets.insert(indexNode->key,indexNode->offset);
        indexSection += sizeof(IndexNode);
    }
}

bool RealtimeDatabase::load()
{
    if (!mIsServer) {
        return nullptr != mData;
    } else {
        if (!mSharedMemory->isAttached() && !mSharedMemory->attach()) {
            return false;
        }
        mSharedMemory->lock();
        restoreFromMemory();
        mSharedMemory->unlock();
        return true;
    }
}

bool RealtimeDatabase::load(const char *data, int size)
{
    if (!mIsServer) {
        if (nullptr == mData || mSize != size) {
            mData = new char[size];
            mSize = size;
            memcpy(mData, data, size);
            restoreFromMemory();
        }
        memcpy(mData, data, size);
        return true;
    } else {
        if(!isLoaded()) {
            if (!mSharedMemory->isAttached() && !mSharedMemory->attach()) {
                bool ret  = mSharedMemory->create(size);
                if (!ret){
                    return false;
                }
            }
            memcpy(mSharedMemory->data(), data, size);
            restoreFromMemory();
            return true;
        }
        memcpy(mSharedMemory->data(), data, size);
        return true;
    }
}

void RealtimeDatabase::copyQStringToMString(const QString &qstring, MemoryString &mstring)
{
    int len = qstring.length();
    if (len  > kMaxRtdbStringLength) len = kMaxRtdbStringLength;
    memcpy(mstring.data, qstring.data(), len * sizeof(QChar));
    mstring.length = len;
}

QString RealtimeDatabase::copyMStringToQString(MemoryString &mstring)
{
    return QString::fromRawData(mstring.data, mstring.length);
}

static const qint64 kUpdateRtdbIntervalMs = 300;
static const double kUpdateRtdbAccurate = 0.01;

RealtimeItemUpdater::RealtimeItemUpdater(RealtimeDatabase *rtdb, quint16 key):
    mRtdb(rtdb),
    mKey(key)
{
    mUpdatedTime = -1;
}

void RealtimeItemUpdater::update(qint64 val)
{
    qint64 now = QDateTime::currentDateTime().currentMSecsSinceEpoch();
    if (-1 == mUpdatedTime) {
        mIntVal = val;
        mRtdb->putItem(mKey, val);
        mUpdatedTime = now;
    } else if ((now - mUpdatedTime) > kUpdateRtdbIntervalMs) {
        if (mIntVal != val) {
            mIntVal = val;
            mRtdb->putItem(mKey, val);
            mUpdatedTime = now;
        }
    }
}

void RealtimeItemUpdater::update(double val)
{
    qint64 now = QDateTime::currentDateTime().currentMSecsSinceEpoch();
    if (-1 == mUpdatedTime) {
        mDoubleVal = val;
        mRtdb->putItem(mKey, val);
        mUpdatedTime = now;
    } else if ((now - mUpdatedTime) > kUpdateRtdbIntervalMs) {
        if (qAbs(mDoubleVal - val) > kUpdateRtdbAccurate) {
            mDoubleVal = val;
            mRtdb->putItem(mKey, val);
            mUpdatedTime = now;
        }
    }
}


const char *RealtimeDatabase::data()
{
    if (!mIsServer) {
        return mData;
    } else {
        if (!mSharedMemory->isAttached() && !load()) {
            return nullptr;
        }
        return (const char *)(mSharedMemory->data());
    }
}

int RealtimeDatabase::size()
{
    if (!mIsServer) {
        return mSize;
    } else {
        return mSharedMemory->size();
    }
}

void RealtimeDatabase::clearData()
{
    QMutexLocker locker(&mAccessMutex);
    if (!mIsServer) {
        if (mData != nullptr) {
            delete[] mData;
            mData = nullptr;
            mItemOffsets.clear();
        }
    }
}
