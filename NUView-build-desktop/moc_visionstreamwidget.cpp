/****************************************************************************
** Meta object code from reading C++ file 'visionstreamwidget.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/visionstreamwidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'visionstreamwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_visionStreamWidget[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      29,   20,   19,   19, 0x05,
      54,   19,   19,   19, 0x05,
      86,   19,   19,   19, 0x05,
     141,  121,   19,   19, 0x05,

 // slots: signature, parameters, type, tag, flags
     200,   19,   19,   19, 0x0a,
     217,   19,   19,   19, 0x0a,
     244,  239,   19,   19, 0x0a,
     269,   19,   19,   19, 0x0a,
     287,   19,   19,   19, 0x0a,
     309,   19,   19,   19, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_visionStreamWidget[] = {
    "visionStreamWidget\0\0datagram\0"
    "PacketReady(QByteArray*)\0"
    "rawImageChanged(const NUImage*)\0"
    "sensorsDataChanged(NUSensorsData*)\0"
    "joint,balance,touch\0"
    "sensorsDataChanged(const float*,const float*,const float*)\0"
    "connectToRobot()\0disconnectFromRobot()\0"
    "name\0updateRobotName(QString)\0"
    "readPendingData()\0sendRequestForImage()\0"
    "sendDataToRobot()\0"
};

const QMetaObject visionStreamWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_visionStreamWidget,
      qt_meta_data_visionStreamWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &visionStreamWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *visionStreamWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *visionStreamWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_visionStreamWidget))
        return static_cast<void*>(const_cast< visionStreamWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int visionStreamWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: PacketReady((*reinterpret_cast< QByteArray*(*)>(_a[1]))); break;
        case 1: rawImageChanged((*reinterpret_cast< const NUImage*(*)>(_a[1]))); break;
        case 2: sensorsDataChanged((*reinterpret_cast< NUSensorsData*(*)>(_a[1]))); break;
        case 3: sensorsDataChanged((*reinterpret_cast< const float*(*)>(_a[1])),(*reinterpret_cast< const float*(*)>(_a[2])),(*reinterpret_cast< const float*(*)>(_a[3]))); break;
        case 4: connectToRobot(); break;
        case 5: disconnectFromRobot(); break;
        case 6: updateRobotName((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 7: readPendingData(); break;
        case 8: sendRequestForImage(); break;
        case 9: sendDataToRobot(); break;
        default: ;
        }
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void visionStreamWidget::PacketReady(QByteArray * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void visionStreamWidget::rawImageChanged(const NUImage * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void visionStreamWidget::sensorsDataChanged(NUSensorsData * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void visionStreamWidget::sensorsDataChanged(const float * _t1, const float * _t2, const float * _t3)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
