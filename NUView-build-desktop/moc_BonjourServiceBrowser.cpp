/****************************************************************************
** Meta object code from reading C++ file 'BonjourServiceBrowser.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/ConnectionManager/BonjourServiceBrowser.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'BonjourServiceBrowser.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_BonjourServiceBrowser[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      23,   22,   22,   22, 0x05,

 // slots: signature, parameters, type, tag, flags
      47,   22,   22,   22, 0x08,
      71,   62,   22,   22, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_BonjourServiceBrowser[] = {
    "BonjourServiceBrowser\0\0newBrowserInformation()\0"
    "onSocketRead()\0resolver\0"
    "onResolveCompleted(BonjourServiceResolver*)\0"
};

const QMetaObject BonjourServiceBrowser::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_BonjourServiceBrowser,
      qt_meta_data_BonjourServiceBrowser, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &BonjourServiceBrowser::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *BonjourServiceBrowser::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *BonjourServiceBrowser::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_BonjourServiceBrowser))
        return static_cast<void*>(const_cast< BonjourServiceBrowser*>(this));
    return QObject::qt_metacast(_clname);
}

int BonjourServiceBrowser::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: newBrowserInformation(); break;
        case 1: onSocketRead(); break;
        case 2: onResolveCompleted((*reinterpret_cast< BonjourServiceResolver*(*)>(_a[1]))); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void BonjourServiceBrowser::newBrowserInformation()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
