/****************************************************************************
** Meta object code from reading C++ file 'GLDisplay.h'
**
** Created: Tue Jun 14 12:50:30 2011
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../NUView/GLDisplay.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GLDisplay.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GLDisplay[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      15,   11,   10,   10, 0x05,
      36,   11,   10,   10, 0x05,
      62,   11,   10,   10, 0x05,
      88,   11,   10,   10, 0x05,

 // slots: signature, parameters, type, tag, flags
     147,  113,   10,   10, 0x0a,
     192,  182,   10,   10, 0x0a,
     239,  215,   10,   10, 0x0a,
     287,  269,   10,   10, 0x0a,
     339,  315,   10,   10, 0x0a,
     405,  373,   10,   10, 0x0a,
     440,   10,   10,   10, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_GLDisplay[] = {
    "GLDisplay\0\0x,y\0selectPixel(int,int)\0"
    "rightSelectPixel(int,int)\0"
    "shiftSelectPixel(int,int)\0"
    "ctrlSelectPixel(int,int)\0"
    "displayID,newDisplay,width,height\0"
    "updatedDisplay(int,GLuint,int,int)\0"
    "displayID\0setPrimaryDisplay(int)\0"
    "displayID,drawingColour\0"
    "setPrimaryDisplay(int,QColor)\0"
    "displayID,enabled\0setOverlayDrawing(int,bool)\0"
    "displayID,enabled,alpha\0"
    "setOverlayDrawing(int,bool,float)\0"
    "displayID,enabled,drawingColour\0"
    "setOverlayDrawing(int,bool,QColor)\0"
    "snapshotToClipboard()\0"
};

const QMetaObject GLDisplay::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_GLDisplay,
      qt_meta_data_GLDisplay, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GLDisplay::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GLDisplay::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GLDisplay::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GLDisplay))
        return static_cast<void*>(const_cast< GLDisplay*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int GLDisplay::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: selectPixel((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 1: rightSelectPixel((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 2: shiftSelectPixel((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: ctrlSelectPixel((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 4: updatedDisplay((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< GLuint(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])),(*reinterpret_cast< int(*)>(_a[4]))); break;
        case 5: setPrimaryDisplay((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: setPrimaryDisplay((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< QColor(*)>(_a[2]))); break;
        case 7: setOverlayDrawing((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 8: setOverlayDrawing((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3]))); break;
        case 9: setOverlayDrawing((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2])),(*reinterpret_cast< QColor(*)>(_a[3]))); break;
        case 10: snapshotToClipboard(); break;
        default: ;
        }
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void GLDisplay::selectPixel(int _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void GLDisplay::rightSelectPixel(int _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void GLDisplay::shiftSelectPixel(int _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void GLDisplay::ctrlSelectPixel(int _t1, int _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_END_MOC_NAMESPACE
