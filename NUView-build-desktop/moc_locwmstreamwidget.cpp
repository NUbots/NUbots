/****************************************************************************
** Meta object code from reading C++ file 'locwmstreamwidget.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/locwmstreamwidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'locwmstreamwidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_locwmStreamWidget[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: signature, parameters, type, tag, flags
      28,   19,   18,   18, 0x05,
      53,   18,   18,   18, 0x05,
      91,   18,   18,   18, 0x05,

 // slots: signature, parameters, type, tag, flags
     135,   18,   18,   18, 0x0a,
     152,   18,   18,   18, 0x0a,
     179,  174,   18,   18, 0x0a,
     204,   18,   18,   18, 0x0a,
     222,   18,   18,   18, 0x0a,
     244,   18,   18,   18, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_locwmStreamWidget[] = {
    "locwmStreamWidget\0\0datagram\0"
    "PacketReady(QByteArray*)\0"
    "locwmDataChanged(const Localisation*)\0"
    "fieldObjectDataChanged(const FieldObjects*)\0"
    "connectToRobot()\0disconnectFromRobot()\0"
    "name\0updateRobotName(QString)\0"
    "readPendingData()\0sendRequestForImage()\0"
    "sendDataToRobot()\0"
};

const QMetaObject locwmStreamWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_locwmStreamWidget,
      qt_meta_data_locwmStreamWidget, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &locwmStreamWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *locwmStreamWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *locwmStreamWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_locwmStreamWidget))
        return static_cast<void*>(const_cast< locwmStreamWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int locwmStreamWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: PacketReady((*reinterpret_cast< QByteArray*(*)>(_a[1]))); break;
        case 1: locwmDataChanged((*reinterpret_cast< const Localisation*(*)>(_a[1]))); break;
        case 2: fieldObjectDataChanged((*reinterpret_cast< const FieldObjects*(*)>(_a[1]))); break;
        case 3: connectToRobot(); break;
        case 4: disconnectFromRobot(); break;
        case 5: updateRobotName((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 6: readPendingData(); break;
        case 7: sendRequestForImage(); break;
        case 8: sendDataToRobot(); break;
        default: ;
        }
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void locwmStreamWidget::PacketReady(QByteArray * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void locwmStreamWidget::locwmDataChanged(const Localisation * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void locwmStreamWidget::fieldObjectDataChanged(const FieldObjects * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
