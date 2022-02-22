#ifndef REALTIMEDATABASE_H
#define REALTIMEDATABASE_H

#include <qglobal.h>
#include <QHash>
#include <QByteArray>
#include <QVariant>
#include <QMutex>
#include "realtimedatabase_global.h"

class QSharedMemory;
struct IndexNode;
struct MemoryString;
struct MemoryItemBase;

class REALTIMEDATABASESHARED_EXPORT RealtimeDatabase
{
public:
    enum RtdbType
    {
        UINT16,
        INT16,
        INT32,
        INT64,
        BOOL,
        FLOAT,
        DOUBLE,
        STRING,
    };

    struct RtdbItem
    {
        quint16 key;
        quint16 type;
        QString name;
        QVariant value;
        bool persistent;
    };

    template <typename T>
    struct Dummy {
        typedef T type;
        T value;
    };

    RealtimeDatabase(bool isServer = true);
    ~RealtimeDatabase();
    //add
    void addItem(const RtdbItem &item);

    //put
    void buildItemData(quint16 key, const QVariant &value, char *data = 0, quint16 *len = 0);
    void putItemData(quint16 key, const char *data, quint16 len);

    template <typename ItemT>
    void putItem(quint16 key, const ItemT &value)
    {
        Dummy<ItemT> dummy;
        dummy.value = value;
        putItemImpl(key, dummy);
    }

    //get
    bool getItem(quint16 key, RtdbItem &rtdbItem);
    template <typename ItemT>
    const ItemT getValueItem(quint16 key,const ItemT defaultValue)
    {
        if (!isLoaded()) return defaultValue;
        char *data = getInternalItemData(key);
        Q_ASSERT(data);
        return getValueItemData(data, defaultValue);
    }
    QString getStringItem(quint16 key);
    inline QList<quint16> keys(){return mItemOffsets.keys();}
    //
    bool create();
    bool load(const char *data, int size);
    bool load();
    bool isLoaded();
    void finish();
    const char *data();
    int size();
    void clearData();

protected:
    template <typename ItemT>
    void putItemImpl(quint16 key, const Dummy<ItemT> &dummy) {
        char *data = getInternalItemData(key);
        Q_ASSERT(data);
        putValueItemData(data, dummy.value);
    }

    void putItemImpl(quint16 key, const Dummy<QString> &dummy);

    template <typename ItemT>
    void addValueItem(quint16 key, const QString name, quint16 type, bool persistent, const ItemT &value)
    {
        addInternalItem(key, name, type, persistent, (char*)&value, sizeof(ItemT));
    }
    void addStringItem(quint16 key,  const QString name, bool persistent, const QString &string);
    void addInternalItem(quint16 key, const QString name, quint16 type, bool persistent, const char *data, int size);

    MemoryItemBase* getInternalItem(quint16 key);
    char *getInternalItemData(quint16 key);

    template <typename ItemT>
    void putValueItemData(char *data, const ItemT &value)
    {
        Q_ASSERT(data);
        memcpy(data, &value, sizeof(ItemT));
    }
    template <typename ItemT>
    ItemT getValueItemData(char *data, ItemT defaultValue)
    {
        ItemT value = defaultValue;
        Q_ASSERT(data);
        memcpy(&value, data, sizeof(ItemT));
        return value;
    }

    void restoreFromMemory();
    static void copyQStringToMString(const QString &qstring, MemoryString &mstring);
    static QString copyMStringToQString(MemoryString &mstring);

private:
    char *mData;
    int mSize;
    QSharedMemory *mSharedMemory;
    QHash<quint16, quint16> mItemOffsets;
    QByteArray *mTemporaryCache;
    bool mIsServer;
    QMutex mAccessMutex;
};

class REALTIMEDATABASESHARED_EXPORT RealtimeItemUpdater
{
public:
    RealtimeItemUpdater(RealtimeDatabase *rtdb, quint16 key);
    void update(qint64 val);
    void update(double val);
private:
    RealtimeDatabase *mRtdb;
    quint16 mKey;
    qint64 mIntVal;
    double mDoubleVal;
    qint64 mUpdatedTime;
};

#endif // REALTIMEDATABASE_H
