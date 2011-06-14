/****************************************************************************
** Meta object code from reading C++ file 'LUTGlDisplay.h'
**
** Created: Tue Jun 14 12:50:31 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/LUTGlDisplay.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LUTGlDisplay.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_LUTGlDisplay[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      18,   14,   13,   13, 0x0a,
      41,   13,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_LUTGlDisplay[] = {
    "LUTGlDisplay\0\0LUT\0SetLUT(unsigned char*)\0"
    "snapshotToClipboard()\0"
};

const QMetaObject LUTGlDisplay::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_LUTGlDisplay,
      qt_meta_data_LUTGlDisplay, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &LUTGlDisplay::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *LUTGlDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *LUTGlDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_LUTGlDisplay))
        return static_cast<void*>(const_cast< LUTGlDisplay*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int LUTGlDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: SetLUT((*reinterpret_cast< unsigned char*(*)>(_a[1]))); break;
        case 1: snapshotToClipboard(); break;
        default: ;
        }
        _id -= 2;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
