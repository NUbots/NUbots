/****************************************************************************
** Meta object code from reading C++ file 'locWmGlDisplay.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/locWmGlDisplay.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'locWmGlDisplay.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_locWmGlDisplay[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      32,   16,   15,   15, 0x0a,
      85,   69,   15,   15, 0x0a,
     122,   15,   15,   15, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_locWmGlDisplay[] = {
    "locWmGlDisplay\0\0newLocalisation\0"
    "SetLocalisation(const Localisation*)\0"
    "newFieldObjects\0setFieldObjects(const FieldObjects*)\0"
    "snapshotToClipboard()\0"
};

const QMetaObject locWmGlDisplay::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_locWmGlDisplay,
      qt_meta_data_locWmGlDisplay, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &locWmGlDisplay::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *locWmGlDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *locWmGlDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_locWmGlDisplay))
        return static_cast<void*>(const_cast< locWmGlDisplay*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int locWmGlDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: SetLocalisation((*reinterpret_cast< const Localisation*(*)>(_a[1]))); break;
        case 1: setFieldObjects((*reinterpret_cast< const FieldObjects*(*)>(_a[1]))); break;
        case 2: snapshotToClipboard(); break;
        default: ;
        }
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
